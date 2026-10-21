// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

#pragma once

#include <atomic>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "bms_driver.hpp"
#include "scud_fields.hpp"

// SCUD485 socket client, dual-protocol:
//
//  - main socket: the legacy BatteryStatus fixed-size records (wire protocol
//    unchanged for old clients). Feeds the BmsDriver getters, is_connected()
//    and the 3 s freshness gate.
//  - fields socket (<socket_path>.fields): serialized FieldMap records for
//    get_field(). Absent on old daemons -- the client degrades gracefully and
//    get_field() answers NaN until the daemon provides the map.
class ScudBmsDriver : public BmsDriver {
public:
    explicit ScudBmsDriver(const std::string& socket_path);
    ~ScudBmsDriver() override;

    // By-name lookup into the cached field map. Returns NaN when the name is
    // unknown or no field record arrived yet -- never throws.
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
    double get_data_age_ms() const override;

private:
    void reader_loop();        // main socket: BatteryStatus records
    void fields_reader_loop(); // fields socket: FieldMap records
    bool try_connect();
    bool try_connect_fields();
    void disconnect();
    void disconnect_fields();
    // True while the last record from the daemon is still current
    bool is_data_current() const;

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
    // CLOCK_MONOTONIC microseconds of the last complete BatteryStatus record
    // received from the daemon; -1 until the first record of the current
    // session arrives (disconnect resets it, so a fresh socket must earn
    // freshness again)
    std::atomic<int64_t> last_rx_us_{-1};
    bool stale_logged_ = false;  // reader thread only
};
