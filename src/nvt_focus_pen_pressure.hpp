// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <span>

namespace nvt {

class FocusPenPressureQueue {
public:
    static constexpr uint8_t kReportId = 5;
    static constexpr size_t kMaximumQueued = 10;

    explicit FocusPenPressureQueue(int maximum_pressure)
        : maximum_pressure_(maximum_pressure) {}

    static std::optional<int> decodeReport(
        std::span<const uint8_t> report, int maximum_pressure) {
        if (report.size() < 3 || report[0] != kReportId)
            return std::nullopt;
        const int pressure = report[1] | static_cast<int>(report[2]) << 8;
        return std::min(maximum_pressure, pressure);
    }

    bool pushReport(std::span<const uint8_t> report) {
        const std::optional<int> pressure =
            decodeReport(report, maximum_pressure_);
        if (!pressure)
            return false;
        queue_.push_back(*pressure);
        if (queue_.size() > kMaximumQueued)
            queue_.pop_front();
        return true;
    }

    int consume() {
        if (!queue_.empty()) {
            last_pressure_ = queue_.front();
            queue_.pop_front();
        }
        return last_pressure_;
    }

    size_t size() const {
        return queue_.size();
    }

    void reset() {
        queue_.clear();
        last_pressure_ = 0;
    }

private:
    std::deque<int> queue_;
    int maximum_pressure_;
    int last_pressure_ = 0;
};

}  // namespace nvt
