// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#include "gf_bms_driver.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <poll.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <thread>
#include <vector>

GfBmsDriver::GfBmsDriver(const std::string& socket_path)
    : BmsDriver(), socket_path_(socket_path),
      fields_socket_path_(socket_path + ".fields") {
    std::memset(&cached_, 0, sizeof(cached_));
    running_ = true;
    reader_thread_ = std::thread(&GfBmsDriver::reader_loop, this);
    fields_reader_thread_ =
        std::thread(&GfBmsDriver::fields_reader_loop, this);
    logger_->info("GF BMS driver started, socket={} fields={}", socket_path_,
                  fields_socket_path_);
}

GfBmsDriver::~GfBmsDriver() {
    running_ = false;
    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
    if (fields_reader_thread_.joinable()) {
        fields_reader_thread_.join();
    }
    disconnect();
    disconnect_fields();
}

bool GfBmsDriver::try_connect() {
    if (sock_fd_ >= 0) return true;

    sock_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock_fd_ < 0) return false;

    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

    if (connect(sock_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    connected_ = true;
    logger_->info("Connected to GF BMS daemon at {}", socket_path_);
    return true;
}

void GfBmsDriver::disconnect() {
    if (sock_fd_ >= 0) {
        close(sock_fd_);
        sock_fd_ = -1;
    }
    connected_ = false;
}

bool GfBmsDriver::try_connect_fields() {
    if (fields_sock_fd_ >= 0) return true;

    fields_sock_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fields_sock_fd_ < 0) return false;

    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, fields_socket_path_.c_str(),
            sizeof(addr.sun_path) - 1);

    if (connect(fields_sock_fd_, reinterpret_cast<struct sockaddr*>(&addr),
                sizeof(addr)) < 0) {
        close(fields_sock_fd_);
        fields_sock_fd_ = -1;
        return false;
    }
    return true;
}

void GfBmsDriver::disconnect_fields() {
    if (fields_sock_fd_ >= 0) {
        close(fields_sock_fd_);
        fields_sock_fd_ = -1;
    }
}

void GfBmsDriver::reader_loop() {
    pthread_setname_np(pthread_self(), "gf_bms_reader");

    while (running_) {
        if (!try_connect()) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        struct pollfd pfd {sock_fd_, POLLIN, 0};
        int ret = poll(&pfd, 1, 500);
        if (ret <= 0) continue;

        bms::BatteryStatus wire;
        ssize_t n = read(sock_fd_, &wire, sizeof(wire));
        if (n == static_cast<ssize_t>(sizeof(wire))) {
            std::unique_lock lock(data_mutex_);
            cached_ = wire;
        } else {
            logger_->warn("GF BMS daemon disconnected, reconnecting...");
            disconnect();
        }
    }
}

// .fields socket：变长 FieldMap 记录（u32 长度前缀），格式与 SCUD 相同。
void GfBmsDriver::fields_reader_loop() {
    pthread_setname_np(pthread_self(), "gf_fields_reader");

    std::vector<uint8_t> rx_buf;

    while (running_) {
        if (!try_connect_fields()) {
            // 旧 daemon 没有 fields socket：静默重试，主 socket 读取不受影响。
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        struct pollfd pfd {fields_sock_fd_, POLLIN, 0};
        int ret = poll(&pfd, 1, 500);
        if (ret < 0) {
            if (errno == EINTR) continue;
            disconnect_fields();
            rx_buf.clear();
            continue;
        }
        if (ret == 0) {
            if (!rx_buf.empty()) {
                // 半条记录无后续 = 流错位，重连回到记录边界。
                disconnect_fields();
                rx_buf.clear();
            }
            continue;
        }
        if (pfd.revents & (POLLERR | POLLHUP)) {
            disconnect_fields();
            rx_buf.clear();
            continue;
        }

        uint8_t chunk[2048];
        ssize_t n = read(fields_sock_fd_, chunk, sizeof(chunk));
        if (n < 0) {
            if (errno == EINTR) continue;
            disconnect_fields();
            rx_buf.clear();
            continue;
        }
        if (n == 0) {
            disconnect_fields();
            rx_buf.clear();
            continue;
        }
        rx_buf.insert(rx_buf.end(), chunk, chunk + n);

        while (rx_buf.size() >= sizeof(uint32_t)) {
            uint32_t payload_len = 0;
            std::memcpy(&payload_len, rx_buf.data(), sizeof(payload_len));
            if (payload_len > scud_bms::kMaxFieldRecordBytes) {
                disconnect_fields();
                rx_buf.clear();
                break;
            }
            const size_t record_len = sizeof(uint32_t) + payload_len;
            if (rx_buf.size() < record_len) break; // 等剩余字节

            scud_bms::FieldMap map;
            if (!scud_bms::deserialize_field_map(
                    rx_buf.data() + sizeof(uint32_t), payload_len, map)) {
                disconnect_fields();
                rx_buf.clear();
                break;
            }
            rx_buf.erase(rx_buf.begin(),
                         rx_buf.begin() + static_cast<long>(record_len));

            std::unique_lock lock(data_mutex_);
            fields_ = std::move(map);
        }
    }
}

double GfBmsDriver::get_field(const std::string& name) const {
    std::shared_lock lock(data_mutex_);
    auto it = fields_.find(name);
    // NaN = 字段未知或尚无记录，永不抛异常。
    return (it == fields_.end()) ? std::numeric_limits<double>::quiet_NaN()
                                 : it->second;
}

double GfBmsDriver::get_voltage() const {
    std::shared_lock lock(data_mutex_);
    return cached_.voltage;
}

double GfBmsDriver::get_current() const {
    std::shared_lock lock(data_mutex_);
    return cached_.current;
}

double GfBmsDriver::get_temperature() const {
    std::shared_lock lock(data_mutex_);
    return cached_.temperature;
}

double GfBmsDriver::get_percentage() const {
    std::shared_lock lock(data_mutex_);
    return cached_.percentage;
}

double GfBmsDriver::get_charge() const {
    std::shared_lock lock(data_mutex_);
    return cached_.charge;
}

double GfBmsDriver::get_capacity() const {
    std::shared_lock lock(data_mutex_);
    return cached_.capacity;
}

double GfBmsDriver::get_design_capacity() const {
    std::shared_lock lock(data_mutex_);
    return cached_.design_capacity;
}

uint32_t GfBmsDriver::get_protect_status() const {
    std::shared_lock lock(data_mutex_);
    return cached_.protect_status;
}

uint16_t GfBmsDriver::get_work_state() const {
    std::shared_lock lock(data_mutex_);
    return cached_.work_state;
}

double GfBmsDriver::get_max_cell_voltage() const {
    std::shared_lock lock(data_mutex_);
    return cached_.max_cell_voltage;
}

double GfBmsDriver::get_min_cell_voltage() const {
    std::shared_lock lock(data_mutex_);
    return cached_.min_cell_voltage;
}

uint16_t GfBmsDriver::get_soh() const {
    std::shared_lock lock(data_mutex_);
    return cached_.soh;
}

uint32_t GfBmsDriver::get_cycles() const {
    std::shared_lock lock(data_mutex_);
    return cached_.cycles;
}

uint32_t GfBmsDriver::get_io_state() const {
    std::shared_lock lock(data_mutex_);
    return cached_.io_state;
}

bool GfBmsDriver::is_power_on() const {
    std::shared_lock lock(data_mutex_);
    return cached_.power_on != 0;
}

bool GfBmsDriver::is_connected() const {
    return connected_.load();
}

uint16_t GfBmsDriver::get_sw_version() const {
    std::shared_lock lock(data_mutex_);
    return cached_.sw_version;
}

uint16_t GfBmsDriver::get_hw_version() const {
    std::shared_lock lock(data_mutex_);
    return cached_.hw_version;
}

std::string GfBmsDriver::get_serial_number() const {
    std::shared_lock lock(data_mutex_);
    return std::string(cached_.serial_number);
}
