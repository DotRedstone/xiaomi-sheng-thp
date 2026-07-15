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
    static constexpr int kMaximumPressure = 8191;
    static constexpr size_t kMaximumQueued = 10;

    static std::optional<int> decodeReport(std::span<const uint8_t> report) {
        if (report.size() < 3 || report[0] != kReportId)
            return std::nullopt;
        const int pressure = report[1] | static_cast<int>(report[2]) << 8;
        return std::min(kMaximumPressure, pressure);
    }

    bool pushReport(std::span<const uint8_t> report) {
        const std::optional<int> pressure = decodeReport(report);
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

private:
    std::deque<int> queue_;
    int last_pressure_ = 0;
};

}  // namespace nvt
