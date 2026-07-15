// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstdint>

namespace nvt {

constexpr int kFingerFilterSlots = 12;

struct FingerCoordinate {
    int x = 0;
    int y = 0;

    bool operator==(const FingerCoordinate &) const = default;
};

struct FingerFilterResult {
    FingerCoordinate coordinate;
    bool complete = true;
    bool interpolated = false;
};

class FingerFilter {
public:
    void reset();
    FingerFilterResult processPipeline(int slot, int status,
                                       FingerCoordinate coordinate,
                                       FingerCoordinate velocity_source,
                                       uint64_t timestamp_ns,
                                       bool skip = false);

private:
    struct JitterState {
        bool initialized = false;
        bool locked = false;
        bool moving = false;
        int age = 0;
        int lock_x = 0;
        int lock_y = 0;
        int center_x = 0;
        int center_y = 0;
        int frame_count = 0;
        int move_count = 0;
        FingerCoordinate output;

        void reset();
    };

    struct KalmanState {
        bool initialized = false;
        std::array<float, 4> state{};
        std::array<float, 16> covariance{};
        float covariance_scale = 1.0F;
        float residual_x = 0.0F;
        float residual_y = 0.0F;
        float process_noise = 0.01F;
        float previous_x = 0.0F;
        float previous_y = 0.0F;

        void reset();
    };

    struct VelocityState {
        bool initialized = false;
        FingerCoordinate previous_source;
        uint64_t previous_time_us = 0;
        std::array<int, 3> instantaneous{};
        std::array<int, 3> average{};
        float adaptive_x = 0.0F;
        float adaptive_y = 0.0F;
        float previous_adaptive_x = 0.0F;
        float previous_adaptive_y = 0.0F;
        int adaptive_hold = 0;

        void reset();
    };

    struct MotionState {
        bool initialized = false;
        int age = 0;
        int turn = 0;
        float blend = 0.0F;
        float anchor_x = 0.0F;
        float anchor_y = 0.0F;
        float previous_x = 0.0F;
        float previous_y = 0.0F;
        float older_x = 0.0F;
        float older_y = 0.0F;

        void reset();
    };

    struct AxisLockState {
        bool initialized = false;
        FingerCoordinate previous_source;
        int candidate_count_x = 0;
        int candidate_count_y = 0;
        int candidate_start_x = 0;
        int candidate_start_y = 0;
        bool locked_x = false;
        bool locked_y = false;
        bool releasing_x = false;
        bool releasing_y = false;
        int locked_x_value = 0;
        int locked_y_value = 0;
        int lock_count_x = 0;
        int lock_count_y = 0;
        int release_count_x = 0;
        int release_count_y = 0;

        void reset();
    };

    std::array<JitterState, kFingerFilterSlots> jitter_{};
    std::array<KalmanState, kFingerFilterSlots> kalman_{};
    std::array<VelocityState, kFingerFilterSlots> velocity_{};
    std::array<MotionState, kFingerFilterSlots> motion_{};
    std::array<AxisLockState, kFingerFilterSlots> axis_lock_{};

    static FingerFilterResult applyJitterLock(JitterState &state, int status,
                                               FingerCoordinate coordinate,
                                               bool skip_jitter);
    FingerCoordinate processKalmanStage(int slot, int status,
                                        FingerCoordinate coordinate,
                                        FingerCoordinate velocity_source,
                                        uint64_t timestamp_ns,
                                        bool skip);
    FingerCoordinate processMotionStage(int slot, int status,
                                        FingerCoordinate coordinate,
                                        bool skip);
    FingerCoordinate processAxisLockStage(int slot, int status,
                                          FingerCoordinate coordinate,
                                          FingerCoordinate velocity_source,
                                          bool skip,
                                          bool jitter_active);
    static void advanceVelocity(VelocityState &state, int status,
                                FingerCoordinate source,
                                uint64_t timestamp_ns);
    static void updateAdaptiveParameters(VelocityState &state, int status);
};

}  // namespace nvt
