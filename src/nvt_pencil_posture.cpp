// SPDX-License-Identifier: Apache-2.0

#include "nvt_pencil_posture.hpp"

#include <algorithm>
#include <cmath>

namespace nvt {
namespace {

constexpr std::uint8_t kP81cImuReportId = 6;
constexpr std::size_t kP81cImuPayloadSize = 20;
constexpr double kPi = 3.14159265358979323846;
constexpr double kAccelerationScale = 0.24400000274181366;
constexpr double kAngularVelocityScale = 0.6 * 0.00122173;
constexpr double kProportionalGain = 40.0;
constexpr double kIntegralGain = 0.1;
constexpr double kHalfSamplePeriod = 0.01;
constexpr double kVectorEpsilon = 1.0e-12;

constexpr std::array<double, 3> kInputNumerator = {
    0.029954582208092474,
    0.059909164416184948,
    0.029954582208092474,
};
constexpr std::array<double, 3> kInputDenominator = {
    1.0,
    -1.4542435862515848,
    0.57406191508395465,
};
constexpr std::array<double, 3> kOutputNumerator = {
    0.14532388387704243,
    0.29064776775408485,
    0.14532388387704243,
};
constexpr std::array<double, 3> kOutputDenominator = {
    1.0,
    -0.67102909077409623,
    0.25232462628226593,
};

int readOriginalBeI16(const std::uint8_t *data) {
    const std::uint16_t value =
        static_cast<std::uint16_t>(data[0]) << 8 |
        static_cast<std::uint16_t>(data[1]);
    return value <= 0x8000U
               ? static_cast<int>(value)
               : static_cast<int>(value) - 0x10000;
}

double truncateTenths(double value) {
    return std::trunc(value * 10.0) / 10.0;
}

double truncateHundredths(double value) {
    return std::trunc(value * 100.0) / 100.0;
}

}  // namespace

std::optional<PencilImuSample> decodeP81cImuReport(
    std::span<const std::uint8_t> report) {
    if (report.size() < kP81cImuPayloadSize + 1 ||
        report[0] != kP81cImuReportId)
        return std::nullopt;

    PencilImuSample sample;
    const std::uint8_t *payload = report.data() + 1;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        sample.acceleration[axis] =
            static_cast<double>(readOriginalBeI16(payload + axis * 2)) *
            kAccelerationScale;
        sample.angular_velocity[axis] =
            static_cast<double>(readOriginalBeI16(payload + 6 + axis * 2)) *
            kAngularVelocityScale;
    }
    sample.acceleration[1] = -sample.acceleration[1];
    sample.angular_velocity[1] = -sample.angular_velocity[1];
    return sample;
}

double PencilPostureDecoder::Biquad::process(double input) {
    if (!initialized) {
        const double output = input *
            (numerator[0] + numerator[1] + numerator[2]) /
            (denominator[0] + denominator[1] + denominator[2]);
        input_history.fill(input);
        output_history.fill(output);
        initialized = true;
        return output;
    }

    const double output =
        numerator[0] * input +
        numerator[1] * input_history[0] +
        numerator[2] * input_history[1] -
        denominator[1] * output_history[0] -
        denominator[2] * output_history[1];
    input_history[1] = input_history[0];
    input_history[0] = input;
    output_history[1] = output_history[0];
    output_history[0] = output;
    return output;
}

void PencilPostureDecoder::Biquad::reset() {
    input_history = {};
    output_history = {};
    initialized = false;
}

namespace {

using Vector = PencilPostureDecoder::Vector;
using Quaternion = PencilPostureDecoder::Quaternion;

Vector add(Vector first, Vector second) {
    return {first.x + second.x, first.y + second.y, first.z + second.z};
}

Vector subtract(Vector first, Vector second) {
    return {first.x - second.x, first.y - second.y, first.z - second.z};
}

Vector scale(Vector value, double factor) {
    return {value.x * factor, value.y * factor, value.z * factor};
}

double dot(Vector first, Vector second) {
    return first.x * second.x + first.y * second.y + first.z * second.z;
}

Vector cross(Vector first, Vector second) {
    return {
        first.y * second.z - first.z * second.y,
        first.z * second.x - first.x * second.z,
        first.x * second.y - first.y * second.x,
    };
}

double norm(Vector value) {
    return std::sqrt(dot(value, value));
}

Vector normalized(Vector value) {
    const double length = norm(value);
    if (!(length > kVectorEpsilon))
        return {};
    return scale(value, 1.0 / length);
}

Quaternion normalized(Quaternion value) {
    const double length = std::sqrt(
        value.w * value.w + value.x * value.x +
        value.y * value.y + value.z * value.z);
    if (!(length > kVectorEpsilon))
        return {};
    const double inverse = 1.0 / length;
    return {value.w * inverse, value.x * inverse,
            value.y * inverse, value.z * inverse};
}

Quaternion conjugate(Quaternion value) {
    return {value.w, -value.x, -value.y, -value.z};
}

Quaternion multiply(Quaternion first, Quaternion second) {
    return {
        first.w * second.w - first.x * second.x -
            first.y * second.y - first.z * second.z,
        first.w * second.x + first.x * second.w +
            first.y * second.z - first.z * second.y,
        first.w * second.y - first.x * second.z +
            first.y * second.w + first.z * second.x,
        first.w * second.z + first.x * second.y -
            first.y * second.x + first.z * second.w,
    };
}

Vector rotate(Quaternion orientation, Vector value) {
    const Quaternion vector{0.0, value.x, value.y, value.z};
    const Quaternion rotated = multiply(
        multiply(orientation, vector), conjugate(orientation));
    return {rotated.x, rotated.y, rotated.z};
}

Quaternion gravityToDown(Vector acceleration) {
    constexpr Vector down{0.0, 0.0, -1.0};
    acceleration = normalized(acceleration);
    const double cosine = std::clamp(dot(acceleration, down), -1.0, 1.0);
    const double angle = std::acos(cosine);
    Vector axis = normalized(cross(acceleration, down));
    double selected_angle = angle;
    if (angle >= 2.6 || norm(axis) <= kVectorEpsilon) {
        axis = std::abs(acceleration.x) <= 0.9
                   ? Vector{1.0, 0.0, 0.0}
                   : Vector{0.0, 1.0, 0.0};
        selected_angle = angle >= 2.6 ? kPi : 0.0;
    }
    const double sine = std::sin(selected_angle * 0.5);
    return normalized({std::cos(selected_angle * 0.5),
                       axis.x * sine, axis.y * sine, axis.z * sine});
}

Vector matchAzimuth(Vector predicted_axis, Vector reference_axis) {
    const double predicted_horizontal = std::hypot(
        predicted_axis.x, predicted_axis.y);
    const double reference_horizontal = std::hypot(
        reference_axis.x, reference_axis.y);
    if (!(reference_horizontal > kVectorEpsilon))
        return predicted_axis;
    const double factor = predicted_horizontal / reference_horizontal;
    return {reference_axis.x * factor,
            reference_axis.y * factor,
            predicted_axis.z};
}

}  // namespace

void PencilPostureDecoder::initializeFilters() {
    for (Biquad &filter : input_filters_) {
        filter.numerator = kInputNumerator;
        filter.denominator = kInputDenominator;
    }
    for (Biquad &filter : output_filters_) {
        filter.numerator = kOutputNumerator;
        filter.denominator = kOutputDenominator;
    }
}

PencilPostureDecoder::PencilPostureDecoder() {
    initializeFilters();
    reset();
}

void PencilPostureDecoder::reset() {
    for (Biquad &filter : input_filters_)
        filter.reset();
    for (Biquad &filter : output_filters_)
        filter.reset();
    orientation_ = {};
    integral_error_ = {};
    orientation_ready_ = false;
    angle_ready_ = false;
    upright_ = false;
    edge_suppressed_ = false;
    extreme_suppressed_ = false;
    pen_active_ = false;
    tilt_x_ = 0;
    tilt_y_ = 0;
    pen_x_ = 0;
    pen_y_ = 0;
    maximum_x_ = 0;
    maximum_y_ = 0;
    previous_raw_angle_ = 0.0;
    accumulated_angle_ = 0.0;
    last_output_angle_ = 0.0;
}

void PencilPostureDecoder::updatePadAcceleration(float x, float y, float z) {
    pad_samples_[pad_sample_next_] = {x, y, z};
    pad_sample_next_ = (pad_sample_next_ + 1) % pad_samples_.size();
    pad_sample_count_ = std::min(pad_sample_count_ + 1, pad_samples_.size());

    Vector average;
    for (std::size_t index = 0; index < pad_sample_count_; ++index)
        average = add(average, pad_samples_[index]);
    average = scale(average, 1.0 / static_cast<double>(pad_sample_count_));
    average = normalized(average);
    average.x = truncateTenths(average.x);
    average.y = truncateTenths(average.y);
    average.z = truncateTenths(average.z);
    pad_acceleration_ = normalized(average);
    pad_ready_ = norm(pad_acceleration_) > kVectorEpsilon;
}

void PencilPostureDecoder::updatePenState(bool active, int tilt_x, int tilt_y,
                                          int x, int y,
                                          int maximum_x, int maximum_y) {
    pen_active_ = active;
    tilt_x_ = tilt_x;
    tilt_y_ = tilt_y;
    pen_x_ = x;
    pen_y_ = y;
    maximum_x_ = maximum_x;
    maximum_y_ = maximum_y;
}

PencilPostureDecoder::Vector PencilPostureDecoder::penAxis() const {
    const double tilt_x = static_cast<double>(tilt_x_) * kPi / 180.0;
    const double tilt_y = static_cast<double>(tilt_y_) * kPi / 180.0;
    const double x = -std::sin(tilt_x);
    const double y = std::sin(tilt_y);
    const double z_square = std::max(0.0, 1.0 - x * x - y * y);
    return normalized(Vector{x, y, -std::sqrt(z_square)});
}

PencilPostureDecoder::Quaternion PencilPostureDecoder::padOrientation() const {
    return gravityToDown(pad_acceleration_);
}

bool PencilPostureDecoder::initializeOrientation(
    const Vector &pen_acceleration) {
    constexpr Vector body_axis{1.0, 0.0, 0.0};
    const Vector acceleration = normalized(pen_acceleration);
    if (norm(acceleration) <= kVectorEpsilon)
        return false;

    orientation_ = gravityToDown(acceleration);
    const Quaternion pad = padOrientation();
    const Vector thp_axis = rotate(pad, penAxis());
    const Vector predicted_axis = rotate(orientation_, body_axis);
    const Vector target_axis = matchAzimuth(predicted_axis, thp_axis);
    const Vector target_body = normalized(
        rotate(conjugate(orientation_), target_axis));

    Vector current_projection = subtract(
        body_axis, scale(acceleration, dot(body_axis, acceleration)));
    Vector target_projection = subtract(
        target_body, scale(acceleration, dot(target_body, acceleration)));
    current_projection = normalized(current_projection);
    target_projection = normalized(target_projection);
    if (norm(current_projection) > kVectorEpsilon &&
        norm(target_projection) > kVectorEpsilon) {
        const double angle = std::atan2(
            dot(acceleration, cross(current_projection, target_projection)),
            std::clamp(dot(current_projection, target_projection), -1.0, 1.0));
        if (std::abs(angle) > 0.01) {
            const double sine = std::sin(angle * 0.5);
            const Quaternion correction{
                std::cos(angle * 0.5),
                acceleration.x * sine,
                acceleration.y * sine,
                acceleration.z * sine,
            };
            orientation_ = normalized(multiply(orientation_, correction));
        }
    }
    orientation_ready_ =
        std::abs(orientation_.w) > kVectorEpsilon ||
        norm(Vector{orientation_.x, orientation_.y, orientation_.z}) >
            kVectorEpsilon;
    integral_error_ = {};
    return orientation_ready_;
}

void PencilPostureDecoder::updateOrientation(const PencilImuSample &sample) {
    constexpr Vector body_axis{1.0, 0.0, 0.0};
    constexpr Vector down{0.0, 0.0, -1.0};
    Vector acceleration{
        sample.acceleration[0], sample.acceleration[1], sample.acceleration[2]};
    const double acceleration_length = norm(acceleration);
    acceleration = normalized(acceleration);
    const Quaternion inverse = conjugate(orientation_);
    const Vector expected_gravity = rotate(inverse, down);
    Vector angular_velocity{
        truncateTenths(sample.angular_velocity[0]),
        truncateTenths(sample.angular_velocity[1]),
        truncateTenths(sample.angular_velocity[2]),
    };
    Vector acceleration_error;
    if (std::abs(acceleration_length - 1000.0) <= 20.0 ||
        norm(angular_velocity) >= 2.0) {
        acceleration_error = cross(acceleration, expected_gravity);
    }
    acceleration_error.x = truncateHundredths(acceleration_error.x);
    acceleration_error.y = truncateHundredths(acceleration_error.y);
    acceleration_error.z = truncateHundredths(acceleration_error.z);

    const Vector thp_axis = rotate(padOrientation(), penAxis());
    const Vector predicted_axis = rotate(orientation_, body_axis);
    const Vector target_axis = matchAzimuth(predicted_axis, thp_axis);
    const Vector desired_body_axis = rotate(inverse, target_axis);
    Vector axis_error = cross(body_axis, desired_body_axis);
    if (edge_suppressed_ || extreme_suppressed_)
        axis_error = {};
    if (upright_) {
        acceleration_error = {};
        axis_error = {};
    }

    const Vector error = add(acceleration_error, axis_error);
    if (norm(error) <= kVectorEpsilon) {
        integral_error_ = {};
    } else {
        integral_error_ = add(
            integral_error_, scale(error, kIntegralGain));
    }
    angular_velocity = add(
        angular_velocity,
        add(integral_error_, scale(error, kProportionalGain)));

    const Quaternion current = orientation_;
    orientation_.w +=
        (-angular_velocity.x * current.x -
         angular_velocity.y * current.y -
         angular_velocity.z * current.z) * kHalfSamplePeriod;
    orientation_.x +=
        (angular_velocity.x * current.w +
         angular_velocity.z * current.y -
         angular_velocity.y * current.z) * kHalfSamplePeriod;
    orientation_.y +=
        (angular_velocity.y * current.w -
         angular_velocity.z * current.x +
         angular_velocity.x * current.z) * kHalfSamplePeriod;
    orientation_.z +=
        (angular_velocity.z * current.w +
         angular_velocity.y * current.x -
         angular_velocity.x * current.y) * kHalfSamplePeriod;
    orientation_ = normalized(orientation_);
}

std::optional<int> PencilPostureDecoder::calculateAngle(bool initialized_now) {
    constexpr Vector tablet_normal{0.0, 0.0, -1.0};
    constexpr Vector body_y{0.0, 1.0, 0.0};
    const Quaternion pad = padOrientation();
    const Vector earth_axis = normalized(rotate(pad, penAxis()));
    const Vector earth_normal = normalized(rotate(pad, tablet_normal));
    const Vector reference = normalized(cross(earth_axis, earth_normal));
    const Vector pencil_y = normalized(rotate(orientation_, body_y));
    if (norm(reference) <= kVectorEpsilon ||
        norm(pencil_y) <= kVectorEpsilon)
        return std::nullopt;

    const double cosine = std::clamp(dot(reference, pencil_y), -1.0, 1.0);
    const double sine = dot(earth_axis, cross(reference, pencil_y));
    double raw_angle = std::fmod(
        -std::atan2(sine, cosine) * 180.0 / kPi + 360.0, 360.0);

    const double tilt_x = static_cast<double>(tilt_x_) * kPi / 180.0;
    const double tilt_y = static_cast<double>(tilt_y_) * kPi / 180.0;
    const double azimuth = std::atan2(-std::sin(tilt_x), std::sin(tilt_y));
    const double azimuth_offset = std::fmod(
        -azimuth * 180.0 / kPi + 360.0, 360.0);
    raw_angle = std::fmod(raw_angle - azimuth_offset, 360.0);

    if (!angle_ready_ || initialized_now) {
        previous_raw_angle_ = raw_angle;
        accumulated_angle_ = last_output_angle_;
        angle_ready_ = true;
    } else {
        accumulated_angle_ += raw_angle - previous_raw_angle_;
        previous_raw_angle_ = raw_angle;
    }
    const double radians =
        std::fmod(accumulated_angle_, 360.0) * kPi / 180.0;
    const double filtered_sine = output_filters_[0].process(std::sin(radians));
    const double filtered_cosine = output_filters_[1].process(std::cos(radians));
    const double output = std::fmod(
        std::atan2(filtered_sine, filtered_cosine) * 180.0 / kPi, 360.0);
    last_output_angle_ = output;
    return static_cast<int>(output);
}

std::optional<int> PencilPostureDecoder::processReport(
    std::span<const std::uint8_t> report) {
    const std::optional<PencilImuSample> decoded = decodeP81cImuReport(report);
    if (!decoded || !pen_active_ || !pad_ready_)
        return std::nullopt;

    PencilImuSample filtered = *decoded;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        filtered.acceleration[axis] =
            input_filters_[axis].process(filtered.acceleration[axis]);
        filtered.angular_velocity[axis] =
            input_filters_[axis + 3].process(filtered.angular_velocity[axis]);
    }
    const Vector acceleration{
        filtered.acceleration[0],
        filtered.acceleration[1],
        filtered.acceleration[2],
    };
    const Vector normalized_acceleration = normalized(acceleration);
    const double angle_from_axis = std::acos(std::clamp(
        std::abs(normalized_acceleration.x), 0.0, 1.0)) * 180.0 / kPi;
    const bool upright = angle_from_axis < 15.0 &&
        std::abs(normalized_acceleration.x) >=
            std::max(std::abs(normalized_acceleration.y),
                     std::abs(normalized_acceleration.z)) &&
        std::hypot(normalized_acceleration.y,
                   normalized_acceleration.z) < 0.3;
    const bool edge = maximum_x_ > 0 && maximum_y_ > 0 &&
        (static_cast<double>(pen_x_) < maximum_x_ * 0.15 ||
         static_cast<double>(pen_x_) > maximum_x_ * 0.85 ||
         static_cast<double>(pen_y_) < maximum_y_ * 0.15 ||
         static_cast<double>(pen_y_) > maximum_y_ * 0.85);
    const double screen_angle = std::acos(std::clamp(
        std::abs(penAxis().z), 0.0, 1.0)) * 180.0 / kPi;
    const bool extreme = screen_angle < 25.0;
    const bool needs_reinitialize =
        (upright_ && !upright) ||
        (edge_suppressed_ && !edge) ||
        (extreme_suppressed_ && !extreme);
    upright_ = upright;
    edge_suppressed_ = edge;
    extreme_suppressed_ = extreme;
    bool initialized_now = false;
    if (!orientation_ready_ || needs_reinitialize) {
        if (!initializeOrientation(acceleration))
            return std::nullopt;
        initialized_now = true;
    } else {
        updateOrientation(filtered);
    }
    return calculateAngle(initialized_now);
}

}  // namespace nvt
