# roboparty_bms 代码结构与功能结构

> 分析对象：roboparty_bms 1.6.x（版本号单一来源为 `debian/changelog` 首行，由 CMake 解析）。
> 一句话概括：**一套电池管理系统（BMS）软件套件** —— C++17 守护进程负责与电池硬件通讯并向本机广播数据，配套 C++/Python 客户端 SDK、UART 转发、OTA 升级与 deb 打包，目标平台为 Armbian/OrangePi 等嵌入式 Linux（arm64），支持交叉编译。许可证 GPL-3.0。

## 1. 顶层目录

```
roboparty_bms/
├── CMakeLists.txt              # 构建入口；版本从 debian/changelog 提取
├── cmake/                      # CMake package config 模板（供非本项目消费者 find_package）
├── debian/                     # deb 打包：control / rules / postinst / prerm / changelog / conffiles
├── etc/                        # 安装进目标机 /etc 的文件
│   ├── default/bms_daemon.default   # 运行配置模板（含 @BMS_PORT@ 等占位符，打包时替换）
│   └── systemd/system/              # bms.service、bms_ota.service
├── firmware/                   # 电池固件 .bin 与 CAN 矩阵 .dbc（OTA 升级材料）
├── include/                    # 唯一公共头 bms_driver.hpp（数据结构 + 抽象接口 + 协议声明）
├── scripts/bms_ota.py          # OTA 升级脚本（Modbus 固件传输协议实现）
├── src/                        # 全部 C++ 源码（见 §3）
└── docs/                       # 文档（本目录）
```

> 注：README_CN.md 的"目录结构"一节提到的 `config/`、`service/`、`udev/` 目录在当前树中并不存在（实际对应 `etc/`），以本文件为准。

## 2. 功能架构（三层）

```
 电池包硬件
   │  Modbus RTU (RS485) / 私有 485 帧 / CAN / 文本 UART
   ▼
┌─────────────────────────────────────────────┐
│  bms_daemon（守护进程，独占硬件）              │
│  · 协议事务（请求-响应）＋ 自愈重连            │
│  · UART 转发 tee（读到的字节镜像到第二串口）   │
│  · Unix Domain Socket 广播电池快照            │
└─────────────────────────────────────────────┘
   │  /tmp/bms.sock 等（BatteryStatus 定长记录）
   ▼
┌─────────────────────────────────────────────┐
│  客户端 SDK（libbms / bms_py）                │
│  · 后台线程收快照 + shared_mutex 缓存         │
│  · get_voltage()/is_connected() 等统一接口    │
└─────────────────────────────────────────────┘
   ▼
 ROS 节点 / Python 脚本 / 其他 C++ 进程
```

核心设计原则：**串口/CAN 只有一个读者**。daemon 独占硬件，客户端只连 socket；这是为了避免多进程抢同一 tty 导致字节互食（取代了旧的 robopi-addon robopi-uart-bridge 双进程方案）。

各组件职责：

| 组件 | 职责 |
|---|---|
| `bms_daemon` | 周期（约 1s）轮询电池、合并/发布状态、维护 socket 广播、UART 转发、失败自愈 |
| `libbms.a`（`bms` 目标） | 客户端静态库：`BmsDriver::create_bms()` 工厂 + 各客户端驱动 |
| `bms_py` | Python 绑定（pybind11），暴露 `BmsDriver` 与 `NrfPmicDriver` |
| `bms_ota.py` + `bms_ota.service` | 固件 OTA：停 bms → Modbus 传固件 → 电池复位 → 起 bms |
| debian 打包 | 一键产出 .deb；`postinst` 负责配置迁移与退役旧 robopi-uart-bridge |

## 3. 源码结构（src/ 与 include/）

```
include/
└── bms_driver.hpp          # bms::BatteryStatus（#pragma pack(1) 定长快照结构）
                            # BmsDriver 抽象基类（纯虚访问器 + create_bms 工厂）
                            # tws_bms::BmsProtocol / gf_bms::GfBmsProtocol / scud_bms::ScudBmsProtocol 声明

src/
├── bms_daemon.cpp          # 守护进程 main：run_daemon<Protocol> 模板、run_can_daemon、
│                           #   UartForwarder（UART 转发）、ForwardConfig、环境变量解析
├── bms_driver.cpp          # BmsDriver::create_bms() 工厂实现（按类型字符串分发）
├── pybind_module.cpp       # bms_py 模块绑定
├── protocol/               # 【传输层】与协议无关的字节搬运
│   ├── serial/serial_port.{hpp,cpp}   # SerialPort：open/poll 读写、tee 转发、8KB 软队列
│   └── can/socket_can.{hpp,cpp}       # SocketCAN 封装，can_rx 接收线程
└── drivers/                # 【协议/驱动层】每类电池一组
    ├── tws/     protocol.cpp + tws_bms_driver.{hpp,cpp}
    ├── gf485/   protocol.cpp + gf_bms_driver.{hpp,cpp}
    ├── scud485/ scud_bms_driver.{hpp,cpp}   # 协议与客户端同文件
    ├── gfcan/   gfcan_bms_driver.{hpp,cpp}  # CAN 报文解析（bms0~bms9），无独立协议层
    └── nrf/     nrf_pmic_driver.{hpp,cpp}   # nRF PMIC 电源管理（48V/5V/19V/12V、急停、看门狗）
```

**daemon 侧协议类 vs 客户端驱动类**——每个 485 家族是两套代码，注意区分：

| | daemon 侧（读硬件、发广播） | 客户端侧（收广播、供应用调用） |
|---|---|---|
| TWS | `tws_bms::BmsProtocol`（`tws/protocol.cpp`，目标 `tws_bms_protocol`） | `TwsBmsDriver`（目标 `tws_bms`） |
| GF485 | `gf_bms::GfBmsProtocol`（`gf485/protocol.cpp`，目标 `gf485_bms_protocol`） | `GfBmsDriver`（目标 `gf485_bms`） |
| SCUD485 | `scud_bms::ScudBmsProtocol` | `ScudBmsDriver`（两者同在 `scud_bms_driver.cpp`，目标 `scud485_bms`） |
| GFCAN | daemon 直接驱动 `GfCanBmsDriver` 轮询 | 同一类直连接口 `can0`（不走 socket） |
| NRF | daemon 直接驱动 `NrfPmicDriver` 轮询打印 | 同一类直连串口（pybind 暴露电源控制） |

客户端驱动（TWS/GF/SCUD）的共性模式：构造时连 socket → `reader_thread_` 后台收 `BatteryStatus` 定长记录 → `std::shared_mutex` 保护缓存 → 虚函数访问器返回缓存值；SCUD485 额外维护 `last_rx_us_` 数据新鲜度（`get_data_age_ms()`，3s 无快照即 `is_connected()==false`）。

## 4. 构建产物（CMake 目标一览）

| 目标 | 类型 | 来源 | 用途 |
|---|---|---|---|
| `bms_daemon` | 可执行 | `src/bms_daemon.cpp` | 守护进程 |
| `bms` | 静态库 | `src/bms_driver.cpp` | 客户端 SDK（工厂 + 链接全部客户端驱动） |
| `bms_py` | pybind11 模块 | `src/pybind_module.cpp` | Python SDK，装入 site-packages |
| `bms_protocol` | 静态库 | `src/protocol/**` | SerialPort / SocketCAN 传输层 |
| `tws_bms_protocol` | 静态库 | `tws/protocol.cpp` | TWS Modbus（daemon 用） |
| `gf485_bms_protocol` | 静态库 | `gf485/protocol.cpp` | GF Modbus（daemon 用） |
| `tws_bms` / `gf485_bms` / `scud485_bms` / `gfcan_bms` / `nrf_pmic` | 静态库 | 各 `*_driver.cpp` | 客户端驱动（daemon 也链接其中共用的部分） |

公共依赖：`fmt`、`spdlog`、`pthread`、`pybind11`、`Python3`；C++17、-O3、ccache；导出 `roboparty_bms::` 命名空间的 CMake 包供外部 `find_package`。

安装布局（`debian/rules` + CMake install）：`/usr/bin/bms_daemon`、`/usr/bin/bms_ota.py`、`/usr/lib/lib*.a`、`/usr/include/`、`/etc/default/bms_daemon`、`/etc/systemd/system/*`、site-packages 下 `bms_py`。

## 5. 支持的 BMS 类型

`BMS_TYPE`（argv[4] / `/etc/default/bms_daemon`）决定 daemon 走哪条链路：

| BMS_TYPE | 总线 | 通讯方式 | 广播 socket | 特点 |
|---|---|---|---|---|
| `TWS`（默认） | RS485 | Modbus RTU（addr 默认 0x01） | `/tmp/bms.sock` | 分字段合并广播（0x9000/容量/IO 三段读） |
| `GF` | RS485 | Modbus RTU（addr 默认 0x03） | `/tmp/gf_bms.sock` | 分字段合并广播 |
| `SCUD485` | RS485 | 私有帧（FE FE + CMD，CRC16-CCITT） | `/tmp/bms.sock` + `/tmp/bms.sock.fields` | **双协议**：主 socket 广播 BatteryStatus（旧协议不变），副 socket 广播 FieldMap（0x61 30B + 0x31 39B 解码，`get_field(name)` 按名查询，未知名返回 NaN）；规格 19200 8N1，实测设备也有跑 115200 的，以电池应答为准 |
| `GFCAN` | SocketCAN | CAN 报文 bms0~bms9 解析 | —（daemon 仅轮询打印） | 客户端用 `GfCanBmsDriver` 直连 `can0` |
| `NRF` | USB tty | 文本命令行（\r\n） | —（daemon 仅轮询打印） | 电源轨控制（48V/5V/19V/12V）、急停、看门狗；Python 直连 |

广播语义两类：**分字段合并**（TWS/GF，三段读各自成功才覆盖对应字段，失败字段沿用上一轮）与**整快照**（SCUD485，一条记录对全部字段负责，失败不发）。见 `bms_daemon.cpp` 的 `publishes_snapshots` 特化表。

## 6. 运行时配置与部署

**`/etc/default/bms_daemon`**（conffile，postinst 负责向已本地修改的文件补齐新键）：

| 键 | 默认 | 说明 |
|---|---|---|
| `BMS_PORT` / `BAUD_RATE` / `TIMEOUT_MS` / `BMS_TYPE` / `BMS_ADDR` | —/115200/300/—/— | 经 `bms.service` 的 `ExecStart` 传入 argv |
| `FORWARD_ENABLE` / `FORWARD_PORT_A` / `FORWARD_PORT_B` / `FORWARD_BAUD` | 1/ttyS3/ttyS7/115200 | UART 转发，经环境变量注入（详见 `docs/uart-forward-issues.md`） |

**systemd**：

- `bms.service`：`EnvironmentFile` + `ExecStartPre=/bin/sleep 10`（等电池上电应答）+ `ExecStart=/usr/bin/bms_daemon ...` + `Restart=always`；
- `bms_ota.service`：oneshot；停 bms → `python3 bms_ota.py <conf> <firmware.bin>` → 起 bms。

**debian/postinst**：`ensure_default` 迁移 conffile、stop/disable/`pkill` 退役旧 `robopi-uart-bridge`、enable 服务。CI 见 `.github/workflows/build-deb.yml`。

## 7. 运行时线程模型

| 线程 | 存在于 | 职责 |
|---|---|---|
| main（`run_daemon` / `run_can_daemon` / NRF 循环） | 始终 | 协议事务、socket accept/广播、UART 转发挂接（tap 模式） |
| `bms_forward`（`UartForwarder::pump_loop`） | 仅 standalone 转发 | 开源口、读→tee→写 sink |
| `can_rx`（`SocketCAN::receiver_loop`） | 仅 GFCAN | CAN 帧接收分发 |
| `nrf_poll`（`NrfPmicDriver::poll_loop`） | 仅 NRF | PMIC 状态轮询 |
| 客户端 `reader_thread_` | 每个 socket 客户端实例 | 收 daemon 广播、更新缓存 |

## 8. 数据结构速查

- `bms::BatteryStatus`（`include/bms_driver.hpp:27-47`）：`#pragma pack(1)` 定长结构，经 socket 原样 `write()`/`read()` 广播——daemon 与客户端**必须同版本**，结构变更即协议变更；
- `BmsDriver`（`include/bms_driver.hpp:51-97`）：统一访问器（电压/电流/温度/SoC/容量/SOH/循环/IO/电源态…）+ `is_connected()` + `get_data_age_ms()`（目前仅 SCUD485 实现）；
- `bms::SerialPort`：`open/poll/read/write_raw` + tee 转发（`set_forward_fd`/`read_teed`，详见 `docs/uart-forward-issues.md` §1）。

## 9. 典型使用方式

**C++ 客户端**：
```cpp
auto bms = BmsDriver::create_bms("SCUD485");   // 连 /tmp/bms.sock
if (bms->is_connected()) double v = bms->get_voltage();
```

**Python 客户端**：
```python
import bms_py
b = bms_py.BmsDriver.create_bms("TWS")
print(b.get_voltage(), b.get_percentage())
```

**部署/运维**：`dpkg-buildpackage` 出包 → `dpkg -i` → 改 `/etc/default/bms_daemon` → `systemctl restart bms`（注意服务固定延迟 10s 才开始读总线）。
