# bms_daemon UART 转发：问题与成因分析

> 分析对象：roboparty_bms 1.6.x 的内建 UART 转发（取代 robopi-addon 的 robopi-uart-bridge）。
> 结论先行：**tap 模式下转发链路本身无竞态；standalone 模式存在一处真实数据竞争；另有 2 处双读者风险与 2 处可靠性/配置缺陷。**

## 1. 当前转发机制（简述）

配置来自 `/etc/default/bms_daemon`（`FORWARD_ENABLE` / `FORWARD_PORT_A` / `FORWARD_PORT_B` / `FORWARD_BAUD`），由 systemd `EnvironmentFile` 注入，`main()` 启动时读取一次，运行中不可热更新。

转发不是独立的"读-写"流水线，而是挂在 `bms::SerialPort` 读取路径上的 **tee（分叉）**：协议层每 `read()` 到 N 字节，就在同一次调用里把这 N 字节写向目标串口。分两种模式（`src/bms_daemon.cpp:477-497`）：

| 模式 | 触发条件 | 数据路径 | poll_sink 驱动者 |
|---|---|---|---|
| **tap** | `FORWARD_PORT_A` 与 BMS 口是同一设备（`same_device()` 比对 `st_rdev`，穿透符号链接） | 协议层 `read_teed()`/`read_raw()` → `SerialPort::tee()` → 非阻塞写 sink fd（8KB 软队列，溢出丢弃） | 主循环（`src/bms_daemon.cpp:266,317`） |
| **standalone** | port_a 与 BMS 口不同，或 GFCAN/NRF | `bms_forward` 泵线程自开 port_a，`read_raw()` 内部 tee 到 sink | 泵线程（`src/bms_daemon.cpp:191`） |

转发语义：**只镜像读到的字节**（BMS→主机方向），不镜像写出的请求帧；`tcflush` 丢弃的字节不进镜像流。

## 2. 问题清单

### 问题 1（数据竞争，中）：`poll_sink()` 被两个线程并发调用

**问题内容**：standalone 模式（FORWARD_PORT_A 与 BMS 口不同）下，`UartForwarder::poll_sink()` 同时被两个线程周期性调用：

- `bms_forward` 泵线程：`pump_loop()` 每轮调用（`src/bms_daemon.cpp:191`）；
- 主循环：`run_daemon()` 无条件调用（`src/bms_daemon.cpp:266,317`）——它只判空 `forwarder` 指针，**不区分模式**。

两者并发访问 `sink_`、`source_`、`attached_`、`tap_`、`sink_warned_`/`source_warned_`，这些成员**均无任何同步**。`bms::SerialPort::open()`（`src/protocol/serial/serial_port.cpp:20-57`）直接裸写 `fd_`，`is_open()`/`fd()` 裸读。

**原因**：设计意图写在 `poll_sink()` 的注释里（"tap 模式由 owner 周期调用；standalone 模式由泵线程驱动"），但**调用点没有做模式闸门**，`UartForwarder` 内部也不强制单一驱动者。工厂函数 `make_protocol_forwarder()` 在 `!same_port` 时既 `start()` 泵线程，又把 forwarder 交给 `run_daemon`，两路驱动就此产生。

**后果**（均为未定义行为的具体表现形式）：

- 启动/重连窗口内两个线程同时通过 `!sink_.is_open()` 检查并各自 `sink_.open()` → 同一 tty 被打开两次，`fd_` 被后者覆盖，前者 fd 泄漏；tee 可能写向被覆盖前的旧 fd；
- `attached_` 与 `sink_.fd()` 的比较-赋值交错 → tap 偶发被挂到过期 fd 或被拨到 -1，表现为下游丢字节；
- `sink_warned_` 等标志的良性数据竞争。

**触发条件**：`FORWARD_PORT_A` ≠ BMS 口的 TWS/GF/SCUD485 部署（BMS 在 CAN 或独立 UART 上、转发另走一路）。GFCAN/NRF 分支不把 forwarder 交给主循环，不受此问题影响。默认部署（ttyS3→ttyS7 且 BMS 在 ttyS3）为 tap 模式，也不受影响。

**顺带说明**：`forward_mutex_` 只保护 `forward_fd_`/`forward_pending_`/`forward_drop_logged_` 三个字段（`src/protocol/serial/serial_port.hpp:52-63`），tee 与 `set_forward_fd()` 互斥是完备的；但 `SerialPort` 自身的 `fd_` 与 `UartForwarder` 的挂接状态完全在锁外。

### 问题 2（双读者风险，中）：NRF 模式缺少 same_device 防护

**问题内容**：NRF 模式下，`NrfPmicDriver` 自己打开 argv[1] 串口（`src/drivers/nrf/nrf_pmic_driver.cpp:43-45`），而 `make_standalone_forwarder()`（`src/bms_daemon.cpp:492-497`）**不做** `same_device()` 检查，无条件开泵线程打开 `FORWARD_PORT_A`。若两者指向同一 tty，就是两个读者抢一个串口——正是内建转发要消灭的字节互抢问题（daemon 读失败、下游流残缺）。

**原因**：`src/bms_daemon.cpp:490` 的注释写 "For BMS types that own no serial port at all (GFCAN, NRF)"——该前提对 NRF **不成立**（NRF 占串口，只有 GFCAN 真正无串口）。注释错误导致防护遗漏。TWS/GF/SCUD485 走 `make_protocol_forwarder()` 有 `same_device()` 闸门，唯独 NRF 漏了。

**后果**：与旧 robopi-uart-bridge 双进程同 tty 完全相同：Modbus/命令响应被两读者分食，读失败间歇出现。

### 问题 3（TOCTOU，低）：`same_device()` 启动时一次判定，设备未就绪会误判

**问题内容**：`same_device()`（`src/bms_daemon.cpp:102-112`）只在构造 forwarder 时判定一次。若 `FORWARD_PORT_A` 用符号链接（如 `/dev/serial/by-id/...`）而启动时尚未出现，`stat()` 失败 → 返回 false → 误判为"不同口"走 standalone 模式；等设备节点出现后泵线程打开它，若它实际指向 BMS 同一 tty（如 ttyS3），即形成双读者。

**原因**：分类决策发生在设备就绪之前，且此后不再复核。字符串不等 + `stat` 失败的组合被静默当作"两个独立串口"。

**后果**：与问题 2 相同（字节互抢），但仅在符号链接/热插拔时序下触发，故严重度低。

### 问题 4（可靠性）：sink 写错误后转发永久失效

**问题内容**：`drain_forward_locked()`（`src/protocol/serial/serial_port.cpp:108-133`）遇到非 EAGAIN 写错误（EIO/EPIPE/EBADF…）时清空队列、告警一次、**保留 fd 不摘除**。而 `poll_sink()` 只在 `!sink_.is_open()` 时才重开 sink——`is_open()` 仍为真，于是**永远不会重开**。目标串口一旦消失或持续报错，转发静默死亡直至进程重启。

**原因**：两个设计决定叠加出死角——(a) 写错误不摘 fd 是为了防止"daemon 以为 tap 还武装着"的状态错位（注释 `src/protocol/serial/serial_port.cpp:121-123`）；(b) 重连逻辑只覆盖"打不开"，没覆盖"已打开但写不通"。(a) 避免了一种错位，却使 (b) 无从触发。

**后果**：sink 热拔/驱动异常后下游拿不到任何数据，且只有一条一次性日志，现场难察觉。

### 问题 5（配置陷阱）：`FORWARD_PORT_B` 与源口相同会回灌总线

**问题内容**：没有任何检查阻止 `FORWARD_PORT_B` 与 `FORWARD_PORT_A`/BMS 口指向同一设备。一旦配错，tee 把读到的字节原样写回同一 tty（`sink_` 以 O_RDWR 打开），等于把 BMS 应答**回灌到总线**，干扰 Modbus 事务。

**原因**：`same_device()` 只用于"tap 还是 standalone"的分流，没有用于 sink 侧的合法性校验；`FORWARD_BAUD` 不匹配有告警（`src/bms_daemon.cpp:548-554`），端口回环却没有对应防线。

## 3. 转发语义说明（行为特征，非缺陷）

以下特征易被误判为丢数据，实为设计语义：

- **只转发读到的字节**：请求帧不 tee（`write_raw` 无 tee 路径），下游看到的是"BMS→主机"单向流；
- **`tcflush` 丢弃的字节不转发**：TWS/GF 每次事务前 `serial_.flush()`，读满 `expected_bytes` 即停，残留在内核缓冲的多余字节被下次 flush 丢掉且从未 tee；故转发流是"协议实际消费的字节流"，**不是总线完整录音**；
- **校验失败的帧仍会转发**：字节先读后验（tee 在 read 成功路径），CRC 错帧照样出现在下游；
- **SCUD485 转发更"全"**：逐字节状态机读（`src/drivers/scud485/scud_bms_driver.cpp:124`）会把回显帧、噪声、他机帧逐字节 tee 出去（半双工 485 回显场景下，回显的请求字节因被读到而**会**进入转发流）；
- **拥塞即丢**：sink 写不动时按 8KB 上限丢弃并告警一次（`src/protocol/serial/serial_port.cpp:91-102`），只影响下游镜像，绝不阻塞 BMS 读循环——转发是监控馈送，不是可靠传输。

## 4. 已确认无竞态的部分

- `tee()` / `set_forward_fd()` / `drain_forward_locked()` 全程持 `forward_mutex_`，幂等挂接（同 fd 直接返回、不清队列）实现正确；
- `stop()` 顺序正确：`join()` 泵线程 → `detach_tap()`（传 -1）→ `sink_.close()`，不存在写悬空 fd 的窗口；
- **tap 模式**（默认部署）：读串口、Modbus 事务、转发写入、poll_sink 全部在同一调用栈顺序执行，单线程，无交错；
- `SerialPort::close()` 故意不清 `forward_fd_`，总线因读失败重开后 tap 自动存活，`poll_sink()` 幂等重挂；
- `g_running` 用 `std::atomic<bool>`，信号安全。

## 5. 修复方向（概要，供后续参考）

| 问题 | 方向 |
|---|---|
| 1 | `run_daemon` 的 `poll_sink()` 调用加模式闸门（standalone 时跳过），或 `UartForwarder` 内部用 `std::mutex` 保护挂接状态并确保单一驱动者 |
| 2 | NRF 分支与协议分支一样做 `same_device()` 检查；相同则拒绝开启转发并告警（并修正 490 行注释） |
| 3 | 泵线程打开 source 成功后用 `fstat` 与 BMS 口复核 `st_rdev`，相同则关源并告警 |
| 4 | 写错误时摘除 tap（`set_forward_fd(-1)`）并 `sink_.close()`，让 `poll_sink()` 的既有重连逻辑接管 |
| 5 | 启动时校验 `FORWARD_PORT_B` 与源口/BMS 口不同，冲突则禁用转发并告警 |
