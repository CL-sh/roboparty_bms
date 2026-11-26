// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#pragma once

#include <atomic>
#include <shared_mutex>
#include <string>
#include <thread>

#include "bms_driver.hpp"
#include "scud_fields.hpp"

// GF485 socket client, dual-socket (same shape as SCUD):
//
//  - main socket: the legacy BatteryStatus fixed-size records (wire protocol
//    unchanged for old clients). Feeds the BmsDriver getters and
//    is_connected().
//  - fields socket (<socket_path>.fields): serialized FieldMap records for
//    get_field(). Absent on old daemons -- the client degrades gracefully and
//    get_field() answers NaN until the daemon provides the map.
class GfBmsDriver : public BmsDriver {
public:
    explicit GfBmsDriver(const std::string& socket_path);
    ~GfBmsDriver() override;

    // 字段表按名查询（.fields socket）；未知字段/尚无记录返回 NaN。
    double get_field(const std::string& name) const override;

    double get_voltage() const override;
    double get_current() const override;
    double get_temperature() const override;
    double get_percentage() const override;
    double get_charge() const override;
    double get_capacity() const override;
    double get_design_capacity() const override;
    uint32_t get_protect_status() const override;
    uint16_t get_work_state() const override;
    double get_max_cell_voltage() const override;
    double get_min_cell_voltage() const override;
    uint16_t get_soh() const override;
    uint32_t get_cycles() const override;
    uint32_t get_io_state() const override;
    bool is_power_on() const override;
    bool is_connected() const override;
    uint16_t get_sw_version() const override;
    uint16_t get_hw_version() const override;
    std::string get_serial_number() const override;

private:
    void reader_loop();
    void fields_reader_loop();
    bool try_connect();
    bool try_connect_fields();
    void disconnect();
    void disconnect_fields();

    std::string socket_path_;
    std::string fields_socket_path_;
    int sock_fd_ = -1;
    int fields_sock_fd_ = -1;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::thread reader_thread_;
    std::thread fields_reader_thread_;
    mutable std::shared_mutex data_mutex_;
    bms::BatteryStatus cached_{};
    scud_bms::FieldMap fields_;
};
