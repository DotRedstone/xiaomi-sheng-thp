// SPDX-License-Identifier: Apache-2.0

#include "nvt_finger_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace nvt {
namespace {

// N81A config and 0x37b40: 24 px first lock, 18 stable frames, and a
// 3 px/frame boundary adjustment, all in super-resolution units.
constexpr int kFirstJitterLockDistance = 240;
constexpr int kFirstJitterStableFrames = 18;
constexpr int kStableBoundaryStep = 30;

int moveTowardByExcess(int center, int current, int excess) {
    if (excess <= 0)
        return center;
    return current < center ? center - excess : center + excess;
}

int interpolate(int center, int current, int numerator, int denominator) {
    const float ratio = denominator > 0
        ? static_cast<float>(std::min(numerator, denominator)) / denominator
        : 0.0F;
    return static_cast<int>(current * ratio + center * (1.0F - ratio) + 0.5F);
}

void checkSlot(int slot) {
    if (slot < 0 || slot >= kFingerFilterSlots)
        throw std::out_of_range("finger filter slot");
}

}  // namespace

void FingerFilter::JitterState::reset() {
    *this = JitterState{};
}

void FingerFilter::KalmanState::reset() {
    *this = KalmanState{};
}

void FingerFilter::VelocityState::reset() {
    *this = VelocityState{};
}

void FingerFilter::MotionState::reset() {
    *this = MotionState{};
}

void FingerFilter::AxisLockState::reset() {
    *this = AxisLockState{};
}

void FingerFilter::reset() {
    jitter_.fill(JitterState{});
    kalman_.fill(KalmanState{});
    velocity_.fill(VelocityState{});
    motion_.fill(MotionState{});
    axis_lock_.fill(AxisLockState{});
}

void FingerFilter::advanceVelocity(
    VelocityState &state, int status, FingerCoordinate source,
    uint64_t timestamp_ns) {
    const uint64_t timestamp_us = timestamp_ns / 1000;
    if (status == 1 || !state.initialized) {
        state.initialized = true;
        state.previous_source = source;
        state.previous_time_us = timestamp_us;
        state.instantaneous.fill(0);
        state.average.fill(0);
        return;
    }

    const int64_t dx = static_cast<int64_t>(source.x) -
                       state.previous_source.x;
    const int64_t dy = static_cast<int64_t>(source.y) -
                       state.previous_source.y;
    const uint64_t elapsed_us = timestamp_us >= state.previous_time_us
        ? timestamp_us - state.previous_time_us
        : state.previous_time_us - timestamp_us;
    const int distance = static_cast<int>(std::sqrt(
        static_cast<double>(dx * dx + dy * dy)));
    const int speed = elapsed_us == 0
        ? 0
        : static_cast<int>(static_cast<uint64_t>(distance) * 1000 /
                           elapsed_us);
    state.instantaneous[2] = state.instantaneous[1];
    state.instantaneous[1] = state.instantaneous[0];
    state.instantaneous[0] = speed;
    state.average[2] = state.average[1];
    state.average[1] = state.average[0];
    state.average[0] = (state.instantaneous[0] +
                        state.instantaneous[1] +
                        state.instantaneous[2]) / 3;
    state.previous_source = source;
    state.previous_time_us = timestamp_us;
}

void FingerFilter::updateAdaptiveParameters(
    VelocityState &state, int status) {
    int first = 0;
    int second = 0;
    if (status == 1) {
        state.adaptive_hold = 0;
        first = 4;
    } else {
        const int current = state.average[0];
        const int previous = state.average[1];
        const int older = state.average[2];
        first = 10;
        if (current < 101) {
            second = 20;
            if (current - previous < 11)
                second = 40;
            if (previous - current < 11)
                first = second;
            second = 0;
            if (current < 15 && previous < 15 && older < 15) {
                const bool current_mid =
                    static_cast<uint32_t>(current - 5) < 5;
                const bool previous_mid =
                    static_cast<uint32_t>(previous - 5) < 5;
                const bool rising = previous <= older && current < previous &&
                                    current_mid && previous_mid;
                const int rising_first = rising ? 20 : 0;
                const int rising_second = rising ? 0 : 10;
                first = rising_first;
                second = rising_second;
                if (current < previous) {
                    const bool stop =
                        static_cast<uint32_t>(current - 1) > 3 ||
                        older < previous;
                    first = stop ? rising_first : 50;
                    second = stop ? rising_second : 0;
                }
            }
        }
    }

    float adaptive_x = state.adaptive_x;
    float adaptive_y = state.adaptive_y;
    if (first != 0 && second == 0) {
        adaptive_y = static_cast<float>((100 * first) / 10);
        adaptive_x = static_cast<float>((25 * first) / 10);
    } else if (second != 0 && first == 0) {
        adaptive_y = static_cast<float>((20000 * 10) / second);
        adaptive_x = static_cast<float>((5000 * 10) / second);
    }

    if (status == 2) {
        if (state.previous_adaptive_y < adaptive_y &&
            state.previous_adaptive_x < adaptive_x) {
            state.adaptive_hold = state.adaptive_hold == 0
                ? 2 : state.adaptive_hold - 1;
        }
        if (adaptive_y <= state.previous_adaptive_y &&
            adaptive_x <= state.previous_adaptive_x) {
            state.adaptive_hold = 0;
        }
    }
    if (state.adaptive_hold >= 1) {
        adaptive_x = state.previous_adaptive_x;
        adaptive_y = state.previous_adaptive_y;
    }
    state.adaptive_x = adaptive_x;
    state.adaptive_y = adaptive_y;
    state.previous_adaptive_x = adaptive_x;
    state.previous_adaptive_y = adaptive_y;
}

FingerCoordinate FingerFilter::processKalmanStage(
    int slot, int status, FingerCoordinate coordinate,
    FingerCoordinate velocity_source, uint64_t timestamp_ns, bool skip) {
    checkSlot(slot);
    KalmanState &filter = kalman_[slot];
    VelocityState &velocity = velocity_[slot];
    if (status == 0) {
        filter.reset();
        velocity.reset();
        return {};
    }
    advanceVelocity(velocity, status, velocity_source, timestamp_ns);
    if (skip)
        return coordinate;
    updateAdaptiveParameters(velocity, status);
    if (status == 1 || !filter.initialized) {
        filter.reset();
        filter.initialized = true;
        filter.state = {static_cast<float>(coordinate.x), 0.0F,
                        static_cast<float>(coordinate.y), 0.0F};
        filter.previous_x = filter.state[0];
        filter.previous_y = filter.state[2];
        filter.covariance[0] = 10.0F;
        filter.covariance[5] = 2.0F;
        filter.covariance[10] = 10.0F;
        filter.covariance[15] = 2.0F;
        return coordinate;
    }

    constexpr std::array<float, 16> transition = {
        1.0F, 1.0F, 0.0F, 0.0F,
        0.0F, 1.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 1.0F, 1.0F,
        0.0F, 0.0F, 0.0F, 1.0F,
    };
    const float process_noise_used = filter.process_noise;
    const std::array<float, 4> process_noise = {
        process_noise_used, process_noise_used / 10.0F,
        process_noise_used, process_noise_used / 10.0F,
    };

    std::array<float, 4> predicted_state{};
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            predicted_state[row] += transition[row * 4 + column] *
                                    filter.state[column];

    std::array<float, 16> temporary{};
    std::array<float, 16> unscaled_covariance{};
    std::array<float, 16> predicted_covariance{};
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            for (int inner = 0; inner < 4; ++inner)
                temporary[row * 4 + column] +=
                    transition[row * 4 + inner] *
                    filter.covariance[inner * 4 + column];
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            for (int inner = 0; inner < 4; ++inner)
                unscaled_covariance[row * 4 + column] +=
                    temporary[row * 4 + inner] *
                    transition[column * 4 + inner];
            predicted_covariance[row * 4 + column] =
                unscaled_covariance[row * 4 + column] *
                filter.covariance_scale;
        }
        predicted_covariance[row * 4 + row] +=
            process_noise[row] * 0.125F;
    }

    const float residual_x = static_cast<float>(coordinate.x) -
                             predicted_state[0];
    const float residual_y = static_cast<float>(coordinate.y) -
                             predicted_state[2];
    const float inverse_x = 1.0F / (1.0F + predicted_covariance[0]);
    const float inverse_y = 1.0F / (1.0F + predicted_covariance[10]);
    const std::array<float, 4> gain = {
        predicted_covariance[0] * inverse_x,
        predicted_covariance[4] * inverse_x,
        predicted_covariance[10] * inverse_y,
        predicted_covariance[14] * inverse_y,
    };
    filter.state = predicted_state;
    filter.state[0] += gain[0] * residual_x;
    filter.state[1] += gain[1] * residual_x;
    filter.state[2] += gain[2] * residual_y;
    filter.state[3] += gain[3] * residual_y;

    std::array<float, 16> correction{};
    for (int index = 0; index < 4; ++index)
        correction[index * 4 + index] = 1.0F;
    correction[0] -= gain[0];
    correction[4] -= gain[1];
    correction[10] -= gain[2];
    correction[14] -= gain[3];
    filter.covariance.fill(0.0F);
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            for (int inner = 0; inner < 4; ++inner)
                filter.covariance[row * 4 + column] +=
                    correction[row * 4 + inner] *
                    predicted_covariance[inner * 4 + column];

    filter.residual_x = (residual_x * residual_x +
                         filter.residual_x * 0.95F) / 1.95F;
    filter.residual_y = (residual_y * residual_y +
                         filter.residual_y * 0.95F) / 1.95F;
    const float state_distance = std::sqrt(
        (filter.state[0] - filter.previous_x) *
            (filter.state[0] - filter.previous_x) +
        (filter.state[2] - filter.previous_y) *
            (filter.state[2] - filter.previous_y));
    const float innovation_distance = std::sqrt(
        residual_x * residual_x + residual_y * residual_y);
    filter.process_noise = std::sqrt(
        state_distance * innovation_distance) / velocity.adaptive_x;
    filter.previous_x = filter.state[0];
    filter.previous_y = filter.state[2];
    const float covariance_total = unscaled_covariance[0] +
                                   unscaled_covariance[10];
    float scale = 1.0F;
    if (covariance_total != 0.0F) {
        scale = (((filter.residual_x - (process_noise_used + 1.0F)) +
                  filter.residual_y) - (process_noise_used + 1.0F)) /
                covariance_total / velocity.adaptive_y;
        scale = std::clamp(scale, filter.covariance_scale - 0.1F,
                           filter.covariance_scale + 0.02F);
        scale = std::clamp(scale, 1.0F, 1.6F);
    }
    filter.covariance_scale = scale;
    return {static_cast<int>(filter.state[0]),
            static_cast<int>(filter.state[2])};
}

FingerCoordinate FingerFilter::processMotionStage(
    int slot, int status, FingerCoordinate coordinate, bool skip) {
    checkSlot(slot);
    MotionState &state = motion_[slot];
    if (status == 0) {
        state.reset();
        return {};
    }
    if (skip)
        return coordinate;
    if (status == 1 || !state.initialized) {
        state.reset();
        state.initialized = true;
        state.age = 0;
        state.anchor_x = static_cast<float>(coordinate.x);
        state.anchor_y = static_cast<float>(coordinate.y);
        state.previous_x = state.anchor_x;
        state.previous_y = state.anchor_y;
        state.older_x = state.anchor_x;
        state.older_y = state.anchor_y;
        return coordinate;
    }

    // 0x42078 copies the N81A hardware values from config offsets
    // 0xe0/0xe4 into context +0x1c/+0x20.
    constexpr int width = 20320;
    constexpr int height = 30480;
    constexpr int base_distance = 90;
    constexpr int edge_limit = 80;
    constexpr int ramp_distance = 400;
    constexpr int prediction_weight_distance = 300;
    constexpr int ramp_frames = 40;

    const float input_x = static_cast<float>(coordinate.x);
    const float input_y = static_cast<float>(coordinate.y);
    int edge_x = coordinate.x <= width / 2
        ? coordinate.x : width - coordinate.x;
    int edge_y = coordinate.y <= height / 2
        ? coordinate.y : height - coordinate.y;
    int edge = std::min({edge_x, edge_y, edge_limit});
    edge = std::max(edge, edge_limit / 2);
    const int transition_distance = base_distance - edge;

    const float previous_distance =
        std::abs(state.previous_x - state.older_x) +
        std::abs(state.previous_y - state.older_y);
    float predicted_x = input_x;
    float predicted_y = input_y;
    float predicted_dx = input_x - state.previous_x;
    float predicted_dy = input_y - state.previous_y;
    if (previous_distance != 0.0F) {
        const float input_distance =
            std::abs(input_x - state.previous_x) +
            std::abs(input_y - state.previous_y);
        predicted_x = state.previous_x +
            (state.previous_x - state.older_x) * input_distance /
            previous_distance;
        predicted_y = state.previous_y +
            (state.previous_y - state.older_y) * input_distance /
            previous_distance;
        predicted_x = std::clamp(predicted_x, 0.0F,
                                 static_cast<float>(width));
        predicted_y = std::clamp(predicted_y, 0.0F,
                                 static_cast<float>(height));
        predicted_dx = predicted_x - state.previous_x;
        predicted_dy = predicted_y - state.previous_y;
    }

    const int cross = static_cast<int>(
        (input_y - state.previous_y) * predicted_dx -
        (input_x - state.previous_x) * predicted_dy);
    state.turn += cross > 0 ? 1 : cross < 0 ? -1 : 0;
    int absolute_turn = std::abs(state.turn);
    const float prediction_distance =
        std::abs(input_x - predicted_x) +
        std::abs(input_y - predicted_y);
    float weight = 0.0F;
    if (absolute_turn < 17) {
        weight = static_cast<float>((absolute_turn * 153U >> 4) + 51U);
    } else {
        state.turn = state.turn > 0 ? 16 : -16;
        weight = 204.0F;
    }
    if (prediction_distance > prediction_weight_distance) {
        const float distance_weight = 255.0F -
            static_cast<float>(prediction_weight_distance * 255) /
            prediction_distance;
        weight = std::max(weight, distance_weight);
    }

    const float mixed_x =
        (weight * input_x + predicted_x * (255.0F - weight)) / 255.0F;
    const float mixed_y =
        (weight * input_y + predicted_y * (255.0F - weight)) / 255.0F;
    const float mixed_distance =
        std::abs(mixed_x - state.previous_x) +
        std::abs(mixed_y - state.previous_y);
    const float edge_float = static_cast<float>(edge);
    float limited_x = state.previous_x;
    float limited_y = state.previous_y;
    if (mixed_distance != 0.0F && edge_float < mixed_distance) {
        limited_x = state.previous_x +
            (mixed_x - state.previous_x) * edge_float / mixed_distance;
        limited_y = state.previous_y +
            (mixed_y - state.previous_y) * edge_float / mixed_distance;
        limited_x = std::clamp(limited_x, 0.0F,
                               static_cast<float>(width));
        limited_y = std::clamp(limited_y, 0.0F,
                               static_cast<float>(height));
    }

    float target_blend = 0.0F;
    if (base_distance != edge && edge_float <= mixed_distance) {
        const int half_transition = transition_distance < 0
            ? (transition_distance + 1) >> 1
            : transition_distance >> 1;
        const float midpoint = static_cast<float>(half_transition + edge);
        if (midpoint <= previous_distance || mixed_distance <= midpoint) {
            target_blend = (mixed_distance - edge_float) * 32.0F /
                           static_cast<float>(transition_distance);
        } else {
            target_blend = static_cast<float>(
                (half_transition << 5) / transition_distance);
        }
    }
    target_blend = std::min(target_blend, 32.0F);
    float blend = state.blend + 4.0F;
    if (target_blend <= state.blend + 4.0F) {
        blend = target_blend;
        if (target_blend + 4.0F < state.blend)
            blend = state.blend > 4.0F ? state.blend - 4.0F : 0.0F;
    }
    if (state.age < ramp_frames &&
        std::abs(input_x - state.anchor_x) +
            std::abs(input_y - state.anchor_y) < ramp_distance) {
        blend = blend * static_cast<float>(ramp_frames + state.age) /
                static_cast<float>(ramp_frames * 2);
    }
    state.age = std::min(250, state.age + 1);
    state.blend = blend;

    const float output_x =
        (state.previous_x * 32.0F + (mixed_x - limited_x) * blend) /
        32.0F;
    const float output_y =
        (state.previous_y * 32.0F + (mixed_y - limited_y) * blend) /
        32.0F;
    const int rounded_x = static_cast<int>(output_x + 0.5F);
    const int rounded_y = static_cast<int>(output_y + 0.5F);
    state.older_x = state.previous_x;
    state.older_y = state.previous_y;
    state.previous_x = static_cast<float>(rounded_x);
    state.previous_y = static_cast<float>(rounded_y);
    return {rounded_x, rounded_y};
}

FingerCoordinate FingerFilter::processAxisLockStage(
    int slot, int status, FingerCoordinate coordinate,
    FingerCoordinate velocity_source, bool skip, bool jitter_active) {
    checkSlot(slot);
    AxisLockState &state = axis_lock_[slot];
    if (status == 0) {
        state.reset();
        return {};
    }

    int raw_dx = 0;
    int raw_dy = 0;
    if (status == 1 || !state.initialized) {
        state.reset();
        state.initialized = true;
    } else {
        raw_dx = std::abs(velocity_source.x - state.previous_source.x);
        raw_dy = std::abs(velocity_source.y - state.previous_source.y);
    }
    state.previous_source = velocity_source;
    if (skip || jitter_active)
        return coordinate;

    constexpr int lock_threshold = 30;
    constexpr int stable_frames = 20;
    constexpr int candidate_limit = lock_threshold * 2;
    constexpr int adaptive_threshold = 40;

    int output_x = coordinate.x;
    int output_y = coordinate.y;

    bool free_x = true;
    if (raw_dx > lock_threshold) {
        if (!state.locked_x)
            state.candidate_count_x = 0;
        free_x = !state.locked_x;
    } else if (state.locked_x) {
        free_x = false;
    } else if (!state.releasing_x) {
        int candidate_delta = 0;
        if (state.candidate_count_x == 0)
            state.candidate_start_x = output_x;
        else
            candidate_delta = std::abs(output_x - state.candidate_start_x);
        ++state.candidate_count_x;
        if (raw_dx >= candidate_limit ||
            state.candidate_count_x < stable_frames ||
            candidate_delta >= candidate_limit) {
            if (candidate_delta >= candidate_limit) {
                state.candidate_count_x = 0;
                state.candidate_start_x = output_x;
            }
        } else {
            state.locked_x = true;
            state.locked_x_value = output_x;
            state.lock_count_x = 0;
            state.releasing_x = false;
            free_x = false;
        }
    }

    if (raw_dy > lock_threshold) {
        if (!state.locked_y)
            state.candidate_count_y = 0;
    } else if (!state.locked_y && !state.releasing_y) {
        int candidate_delta = 0;
        if (state.candidate_count_y == 0)
            state.candidate_start_y = output_y;
        else
            candidate_delta = std::abs(output_y - state.candidate_start_y);
        ++state.candidate_count_y;
        if (raw_dy >= candidate_limit ||
            state.candidate_count_y < stable_frames ||
            candidate_delta >= candidate_limit) {
            if (candidate_delta >= candidate_limit) {
                state.candidate_count_y = 0;
                state.candidate_start_y = output_y;
            }
        } else {
            state.locked_y = true;
            state.locked_y_value = output_y;
            state.lock_count_y = 0;
            state.releasing_y = false;
        }
    }

    if (!free_x) {
        const int distance = std::abs(output_x - state.locked_x_value);
        const int excess = distance - adaptive_threshold;
        if (excess <= 0) {
            ++state.lock_count_x;
            output_x = state.locked_x_value;
        } else if (distance <= lock_threshold) {
            output_x = state.locked_x_value;
        } else if (excess > lock_threshold) {
            state.locked_x = false;
            state.releasing_x = true;
            state.candidate_count_x = 0;
            state.release_count_x = 0;
        } else {
            state.locked_x_value += output_x < state.locked_x_value
                ? -excess : excess;
            output_x = state.locked_x_value;
        }
    }

    if (state.locked_y) {
        const int distance = std::abs(output_y - state.locked_y_value);
        const int excess = distance - adaptive_threshold;
        if (excess <= 0) {
            ++state.lock_count_y;
            output_y = state.locked_y_value;
        } else if (distance <= lock_threshold) {
            output_y = state.locked_y_value;
        } else if (excess > lock_threshold) {
            state.locked_y = false;
            state.releasing_y = true;
            state.candidate_count_y = 0;
            state.release_count_y = 0;
        } else {
            state.locked_y_value += output_y < state.locked_y_value
                ? -excess : excess;
            output_y = state.locked_y_value;
        }
    }

    auto release = [=](int current, int locked, int &count, bool &active) {
        ++count;
        const float ratio = static_cast<float>(std::min(count, 5)) / 5.0F;
        const int output = static_cast<int>(
            (1.0F - ratio) * locked + ratio * current + 0.5F);
        if (count >= 5 || std::abs(current - output) < lock_threshold)
            active = false;
        return output;
    };
    if (state.releasing_x)
        output_x = release(output_x, state.locked_x_value,
                           state.release_count_x, state.releasing_x);
    if (state.releasing_y)
        output_y = release(output_y, state.locked_y_value,
                           state.release_count_y, state.releasing_y);
    return {output_x, output_y};
}

FingerFilterResult FingerFilter::applyJitterLock(
    JitterState &state, int status, FingerCoordinate coordinate,
    bool skip_jitter) {
    if (status == 1 || !state.initialized) {
        state.reset();
        state.initialized = true;
        state.locked = true;
        state.age = 1;
        state.center_x = coordinate.x;
        state.center_y = coordinate.y;
        state.output = coordinate;
        return {coordinate, true};
    }
    if (status != 2 || skip_jitter)
        return {coordinate, true};

    FingerFilterResult result{coordinate, true};
    if (state.locked) {
        const int distance = kFirstJitterLockDistance >>
            (state.age >= kFirstJitterStableFrames);
        state.lock_x = distance;
        state.lock_y = distance;

        const int dx = std::abs(coordinate.x - state.center_x);
        const int dy = std::abs(coordinate.y - state.center_y);
        const int excess_x = dx - state.lock_x;
        const int excess_y = dy - state.lock_y;
        if (dx <= state.lock_x && dy <= state.lock_y) {
            result.coordinate = {state.center_x, state.center_y};
        } else if (excess_x <= kStableBoundaryStep &&
                   excess_y <= kStableBoundaryStep) {
            state.center_x = moveTowardByExcess(
                state.center_x, coordinate.x, excess_x);
            state.center_y = moveTowardByExcess(
                state.center_y, coordinate.y, excess_y);
            result.coordinate = {state.center_x, state.center_y};
        } else {
            state.locked = false;
            state.moving = true;
            state.move_count = 0;
            result.complete = false;
        }
        ++state.frame_count;
    }
    if (state.moving) {
        result.interpolated = true;
        ++state.move_count;
        const int capped_frame_count = std::min(state.frame_count, 8);
        const int interpolation_frames = capped_frame_count > 4
            ? capped_frame_count : 5;
        result.coordinate = {
            interpolate(state.center_x, coordinate.x, state.move_count,
                        interpolation_frames),
            interpolate(state.center_y, coordinate.y, state.move_count,
                        interpolation_frames),
        };
        const int remaining = static_cast<int>(std::sqrt(
            static_cast<float>(coordinate.x - result.coordinate.x) *
                (coordinate.x - result.coordinate.x) +
            static_cast<float>(coordinate.y - result.coordinate.y) *
                (coordinate.y - result.coordinate.y)));
        if (state.move_count >= interpolation_frames || remaining < 30)
            state.moving = false;
        result.complete = true;
    }
    ++state.age;
    state.output = result.coordinate;
    return result;
}

FingerFilterResult FingerFilter::processPipeline(
    int slot, int status, FingerCoordinate coordinate,
    FingerCoordinate velocity_source, uint64_t timestamp_ns, bool skip) {
    checkSlot(slot);
    if (status == 0) {
        jitter_[slot].reset();
        kalman_[slot].reset();
        velocity_[slot].reset();
        motion_[slot].reset();
        axis_lock_[slot].reset();
        return {{}, true, false};
    }

    FingerFilterResult jitter = applyJitterLock(
        jitter_[slot], status, coordinate, skip);
    FingerCoordinate filtered = processAxisLockStage(
        slot, status, jitter.coordinate, velocity_source, skip,
        jitter.interpolated);
    filtered = processKalmanStage(slot, status, filtered, velocity_source,
                                  timestamp_ns, skip);
    filtered = processMotionStage(slot, status, filtered, skip);
    return {filtered, jitter.complete, jitter.interpolated};
}

}  // namespace nvt
