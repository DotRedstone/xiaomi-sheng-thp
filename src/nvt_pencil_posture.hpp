// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace nvt {

struct PencilImuSample {
    std::array<double, 3> acceleration{};
    std::array<double, 3> angular_velocity{};
};

std::optional<PencilImuSample> decodeP81cImuReport(
    std::span<const std::uint8_t> report);

class PencilPostureDecoder {
public:
    PencilPostureDecoder();
    void reset();
    void updatePadAcceleration(float x, float y, float z);
    void updatePenState(bool active, int tilt_x, int tilt_y,
                        int x, int y, int maximum_x, int maximum_y);
    std::optional<int> processReport(std::span<const std::uint8_t> report);

    struct Vector {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Quaternion {
        double w = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Biquad {
        std::array<double, 3> numerator{};
        std::array<double, 3> denominator{};
        std::array<double, 2> input_history{};
        std::array<double, 2> output_history{};
        bool initialized = false;

        double process(double input);
        void reset();
    };

private:
    std::array<Biquad, 6> input_filters_{};
    std::array<Biquad, 2> output_filters_{};
    std::array<Vector, 6> pad_samples_{};
    std::size_t pad_sample_count_ = 0;
    std::size_t pad_sample_next_ = 0;
    Vector pad_acceleration_{};
    Quaternion orientation_{};
    Vector integral_error_{};
    bool pen_active_ = false;
    bool pad_ready_ = false;
    bool orientation_ready_ = false;
    bool angle_ready_ = false;
    bool upright_ = false;
    bool edge_suppressed_ = false;
    bool extreme_suppressed_ = false;
    int tilt_x_ = 0;
    int tilt_y_ = 0;
    int pen_x_ = 0;
    int pen_y_ = 0;
    int maximum_x_ = 0;
    int maximum_y_ = 0;
    double previous_raw_angle_ = 0.0;
    double accumulated_angle_ = 0.0;
    double last_output_angle_ = 0.0;

    void initializeFilters();
    bool initializeOrientation(const Vector &pen_acceleration);
    void updateOrientation(const PencilImuSample &sample);
    std::optional<int> calculateAngle(bool initialized_now);
    Vector penAxis() const;
    Quaternion padOrientation() const;
};

}  // namespace nvt
