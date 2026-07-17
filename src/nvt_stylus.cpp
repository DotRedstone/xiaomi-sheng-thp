// SPDX-License-Identifier: Apache-2.0

#include "nvt_stylus.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <limits>

namespace nvt {
namespace {

constexpr std::size_t kTransportSize = 257;
constexpr std::size_t kPayloadDataTypeOffset = 0x38;
constexpr std::size_t kSpecialSourceOffset = 0x40;
constexpr std::size_t kSpecialHeaderSize = 36;
constexpr std::size_t kSpecialStateOffset = 16;
constexpr std::size_t kMutualBlockIndexOffset = 11;
constexpr std::size_t kPlaneBytes = kStylusPlaneNodes * sizeof(std::int16_t);
constexpr std::size_t kPlaneOffset =
    kTransportSize + kSpecialSourceOffset + kSpecialHeaderSize;
constexpr std::size_t kMutualBlockOffset =
    kTransportSize + 0xf78;
constexpr std::size_t kMutualBlockBytes =
    kStylusMutualBlockNodes * sizeof(std::int16_t);
constexpr std::size_t kRequiredSize =
    kMutualBlockOffset + kMutualBlockBytes;
constexpr std::size_t kTip1CandidateColumns = 12;
constexpr std::size_t kTip2CandidateRows = 8;
constexpr int kSuperResolution = 10;
constexpr int kCalibrationThreshold = 40 * kSuperResolution;
constexpr int kCalibrationRate = 10;
constexpr int kTipPitch40 = 50 * kSuperResolution;
constexpr int kTipPitch60 = 50 * kSuperResolution;
constexpr int kTipSlope40 = 5;
constexpr int kTipSlope60 = 5;
constexpr int kTipEdgeParameter = 8;
constexpr std::array<int, kStylusAxis40Nodes> kAxis40Mapping = {
    26, 78, 130, 182, 234, 286, 338, 390, 442, 494,
    546, 598, 650, 702, 754, 806, 856, 906, 956, 1006,
    1056, 1106, 1156, 1206, 1256, 1306, 1356, 1406, 1456, 1506,
    1556, 1606, 1656, 1706, 1756, 1806, 1856, 1906, 1956, 2006,
};
constexpr std::array<int, kStylusAxis60Nodes> kAxis60Mapping = {
    25, 75, 125, 175, 225, 275, 325, 375, 425, 476,
    528, 580, 632, 684, 736, 788, 840, 892, 944, 996,
    1048, 1099, 1149, 1199, 1249, 1299, 1349, 1399, 1449, 1499,
    1549, 1599, 1649, 1699, 1749, 1799, 1849, 1899, 1949, 2000,
    2052, 2104, 2156, 2208, 2260, 2312, 2364, 2416, 2468, 2520,
    2572, 2623, 2673, 2723, 2773, 2823, 2873, 2923, 2973, 3023,
};
constexpr std::array<int, 6> kTiltDifferences = {0, 50, 65, 87, 101, 108};
constexpr std::array<int, 6> kTiltAngles = {0, 1500, 3000, 4500, 6000, 7000};

int wrapAdd(int first, int second) {
    const std::uint32_t value = std::bit_cast<std::uint32_t>(first) +
                                std::bit_cast<std::uint32_t>(second);
    return std::bit_cast<std::int32_t>(value);
}

int wrapSubtract(int first, int second) {
    const std::uint32_t value = std::bit_cast<std::uint32_t>(first) -
                                std::bit_cast<std::uint32_t>(second);
    return std::bit_cast<std::int32_t>(value);
}

int wrapMultiply(int first, int second) {
    const std::uint32_t value = std::bit_cast<std::uint32_t>(first) *
                                std::bit_cast<std::uint32_t>(second);
    return std::bit_cast<std::int32_t>(value);
}

struct InterferenceLine {
    float intercept = 0.0F;
    float slope = 0.0F;
};

int truncateInterferenceFloat(float value) {
    if (std::isnan(value))
        return 0;
    if (value >= static_cast<float>(std::numeric_limits<int>::max()))
        return std::numeric_limits<int>::max();
    if (value <= static_cast<float>(std::numeric_limits<int>::min()))
        return std::numeric_limits<int>::min();
    return static_cast<int>(value);
}

template <std::size_t Size>
InterferenceLine fitInterferenceLine(const std::array<int, Size> &response,
                                     const std::array<int, Size> &predictor,
                                     const std::array<int, Size> &mask) {
    int count = 0;
    int predictor_sum = 0;
    int response_sum = 0;
    int predictor_square_sum = 0;
    int product_sum = 0;
    for (std::size_t index = 0; index < Size; ++index) {
        if (mask[index] != 1)
            continue;
        ++count;
        predictor_sum = wrapAdd(predictor_sum, predictor[index]);
        response_sum = wrapAdd(response_sum, response[index]);
        predictor_square_sum = wrapAdd(
            predictor_square_sum,
            wrapMultiply(predictor[index], predictor[index]));
        product_sum = wrapAdd(
            product_sum, wrapMultiply(response[index], predictor[index]));
    }

    const int denominator_integer = wrapSubtract(
        wrapMultiply(count, predictor_square_sum),
        wrapMultiply(predictor_sum, predictor_sum));
    const float denominator = static_cast<float>(denominator_integer);
    const float intercept = static_cast<float>(wrapSubtract(
        wrapMultiply(response_sum, predictor_square_sum),
        wrapMultiply(product_sum, predictor_sum))) / denominator;
    const float slope = static_cast<float>(wrapSubtract(
        wrapMultiply(count, product_sum),
        wrapMultiply(predictor_sum, response_sum))) / denominator;
    return {intercept, slope};
}

template <std::size_t Size>
void subtractInterferenceLine(int *plane, std::size_t stride,
                              const std::array<int, Size> &projection) {
    std::array<int, Size> response{};
    std::array<int, Size> predictor = projection;
    std::array<int, Size> selected{};
    std::array<int, Size> expanded{};
    for (std::size_t index = 0; index < Size; ++index)
        response[index] = plane[index * stride];

    for (std::size_t index = 0; index < Size; ++index) {
        if (predictor[index] <= 200)
            continue;
        if ((index > 0 && predictor[index - 1] > 200) ||
            (index + 1 < Size && predictor[index + 1] > 200))
            selected[index] = 1;
    }
    for (std::size_t index = 0; index < Size; ++index) {
        if (selected[index] != 1)
            continue;
        if (index > 0)
            expanded[index - 1] = 1;
        if (index + 1 < Size)
            expanded[index + 1] = 1;
    }
    for (int &value : predictor)
        value /= 10;

    const InterferenceLine line = fitInterferenceLine(
        response, predictor, expanded);
    for (std::size_t index = 0; index < Size; ++index) {
        if (expanded[index] != 1)
            continue;
        const int estimate = truncateInterferenceFloat(
            line.intercept + line.slope * static_cast<float>(predictor[index]));
        plane[index * stride] = wrapSubtract(response[index], estimate);
    }
}

int frameIntervalFromMode(std::uint8_t mode) {
    switch (mode) {
    case 1: return 180;
    case 2: return 240;
    case 3: return 360;
    case 4: return 60;
    case 5: return 480;
    case 6: return 144;
    default: return 120;
    }
}

template <std::size_t Size>
void copyOrBlendAxis(std::array<int, Size> &axis,
                     std::array<int, Size> &history, int blend,
                     bool copy) {
    if (copy) {
        history = axis;
        return;
    }
    for (std::size_t index = 0; index < Size; ++index) {
        const int value = (history[index] * (100 - blend) +
                           axis[index] * blend) / 100;
        history[index] = value;
        axis[index] = value;
    }
}

struct PlanePeak {
    int value = 0;
    std::size_t line = 0;
    std::size_t node = 0;
};

std::int16_t readLeI16(const std::uint8_t *data) {
    const std::uint16_t value = static_cast<std::uint16_t>(data[0]) |
                                static_cast<std::uint16_t>(data[1]) << 8;
    return static_cast<std::int16_t>(value);
}

template <std::size_t Size>
void readPlane(const std::uint8_t *source, std::array<int, Size> &output) {
    for (std::size_t index = 0; index < Size; ++index)
        output[index] = readLeI16(source + index * sizeof(std::int16_t));
}

template <std::size_t AxisSize, std::size_t CandidateLines>
void selectRows(const int *plane, std::array<int, AxisSize> &axis) {
    int peak_value = 0;
    std::size_t peak_line = 0;
    std::size_t peak_node = 0;
    for (std::size_t node = 0; node < AxisSize; ++node) {
        for (std::size_t line = 0; line < CandidateLines; ++line) {
            const int value = plane[line * AxisSize + node];
            if (value > peak_value) {
                peak_value = value;
                peak_line = line;
                peak_node = node;
            }
        }
    }

    const int *selected = plane + peak_line * AxisSize;
    std::copy_n(selected, AxisSize, axis.begin());
    if (CandidateLines == 1)
        return;

    if (peak_line == 0 || peak_line + 1 == CandidateLines) {
        const std::size_t adjacent_line = peak_line == 0 ? 1 : peak_line - 1;
        const int *adjacent = plane + adjacent_line * AxisSize;
        if (adjacent[peak_node] > peak_value / 6) {
            for (std::size_t node = 0; node < AxisSize; ++node)
                axis[node] += adjacent[node] - selected[node] / 6;
        }
        return;
    }

    const int *previous = selected - AxisSize;
    const int *next = selected + AxisSize;
    if (peak_node == 0 || peak_node + 1 == AxisSize) {
        for (std::size_t node = 0; node < AxisSize; ++node)
            axis[node] += previous[node] + next[node];
        return;
    }

    const int *adjacent = previous[peak_node] > next[peak_node]
                              ? previous
                              : next;
    if (adjacent[peak_node] > peak_value / 6) {
        for (std::size_t node = 0; node < AxisSize; ++node)
            axis[node] += adjacent[node] - selected[node] / 6;
    }
}

template <std::size_t AxisSize, std::size_t CandidateLines>
PlanePeak findRowPeak(const int *plane) {
    PlanePeak peak;
    for (std::size_t node = 0; node < AxisSize; ++node) {
        for (std::size_t line = 0; line < CandidateLines; ++line) {
            const int value = plane[line * AxisSize + node];
            if (value > peak.value)
                peak = {value, line, node};
        }
    }
    return peak;
}

template <std::size_t AxisSize, std::size_t CandidateLines>
PlanePeak findColumnPeak(const int *plane) {
    PlanePeak peak;
    for (std::size_t line = 0; line < CandidateLines; ++line) {
        for (std::size_t node = 0; node < AxisSize; ++node) {
            const int value = plane[node * CandidateLines + line];
            if (value > peak.value)
                peak = {value, line, node};
        }
    }
    return peak;
}

template <std::size_t AxisSize, std::size_t CandidateLines>
void selectRingRow(const int *guide_plane, const int *ring_plane,
                   std::array<int, AxisSize> &axis) {
    const PlanePeak guide_peak =
        findRowPeak<AxisSize, CandidateLines>(guide_plane);
    const PlanePeak ring_peak =
        findRowPeak<AxisSize, CandidateLines>(ring_plane);
    const bool peaks_align =
        std::max(guide_peak.node, ring_peak.node) -
            std::min(guide_peak.node, ring_peak.node) < 3;
    const PlanePeak selected_peak = peaks_align ? ring_peak : guide_peak;
    const int *selected = ring_plane + selected_peak.line * AxisSize;
    std::copy_n(selected, AxisSize, axis.begin());

    std::size_t adjacent_line = 0;
    if (selected_peak.line == 0) {
        adjacent_line = 1;
    } else if (selected_peak.line + 1 == CandidateLines) {
        adjacent_line = selected_peak.line - 1;
    } else {
        const int *previous = selected - AxisSize;
        const int *next = selected + AxisSize;
        adjacent_line = previous[selected_peak.node] > next[selected_peak.node]
                            ? selected_peak.line - 1
                            : selected_peak.line + 1;
    }

    const int *adjacent = ring_plane + adjacent_line * AxisSize;
    if (adjacent[selected_peak.node] > ring_peak.value / 8) {
        for (std::size_t node = 0; node < AxisSize; ++node)
            axis[node] += adjacent[node] - selected[node] / 8;
    }
}

template <std::size_t AxisSize, std::size_t CandidateLines>
void selectRingColumn(const int *guide_plane, const int *ring_plane,
                      std::array<int, AxisSize> &axis) {
    const PlanePeak guide_peak =
        findColumnPeak<AxisSize, CandidateLines>(guide_plane);
    const PlanePeak ring_peak =
        findColumnPeak<AxisSize, CandidateLines>(ring_plane);
    const bool peaks_align =
        std::max(guide_peak.node, ring_peak.node) -
            std::min(guide_peak.node, ring_peak.node) < 3;
    const PlanePeak selected_peak = peaks_align ? ring_peak : guide_peak;
    const auto value = [ring_plane](std::size_t node, std::size_t line) {
        return ring_plane[node * CandidateLines + line];
    };
    for (std::size_t node = 0; node < AxisSize; ++node)
        axis[node] = value(node, selected_peak.line);

    std::size_t adjacent_line = 0;
    if (selected_peak.line == 0) {
        adjacent_line = 1;
    } else if (selected_peak.line + 1 == CandidateLines) {
        adjacent_line = selected_peak.line - 1;
    } else {
        adjacent_line = value(selected_peak.node, selected_peak.line - 1) >
                                value(selected_peak.node,
                                      selected_peak.line + 1)
                            ? selected_peak.line - 1
                            : selected_peak.line + 1;
    }

    if (value(selected_peak.node, adjacent_line) > ring_peak.value / 8) {
        for (std::size_t node = 0; node < AxisSize; ++node) {
            const int selected = value(node, selected_peak.line);
            axis[node] += value(node, adjacent_line) - selected / 8;
        }
    }
}

template <std::size_t AxisSize, std::size_t CandidateLines>
void selectColumns(const int *plane, std::array<int, AxisSize> &axis) {
    int peak_value = 0;
    std::size_t peak_line = 0;
    std::size_t peak_node = 0;
    for (std::size_t line = 0; line < CandidateLines; ++line) {
        for (std::size_t node = 0; node < AxisSize; ++node) {
            const int value = plane[node * CandidateLines + line];
            if (value > peak_value) {
                peak_value = value;
                peak_line = line;
                peak_node = node;
            }
        }
    }

    for (std::size_t node = 0; node < AxisSize; ++node)
        axis[node] = plane[node * CandidateLines + peak_line];
    if (CandidateLines == 1)
        return;

    const auto value = [plane](std::size_t node, std::size_t line) {
        return plane[node * CandidateLines + line];
    };
    if (peak_line == 0 || peak_line + 1 == CandidateLines) {
        const std::size_t adjacent_line = peak_line == 0 ? 1 : peak_line - 1;
        if (value(peak_node, adjacent_line) > peak_value / 6) {
            for (std::size_t node = 0; node < AxisSize; ++node) {
                const int selected = value(node, peak_line);
                axis[node] += value(node, adjacent_line) - selected / 6;
            }
        }
        return;
    }

    const std::size_t previous_line = peak_line - 1;
    const std::size_t next_line = peak_line + 1;
    if (peak_node == 0 || peak_node + 1 == AxisSize) {
        for (std::size_t node = 0; node < AxisSize; ++node)
            axis[node] += value(node, previous_line) + value(node, next_line);
        return;
    }

    const std::size_t adjacent_line =
        value(peak_node, previous_line) > value(peak_node, next_line)
            ? previous_line
            : next_line;
    if (value(peak_node, adjacent_line) > peak_value / 6) {
        for (std::size_t node = 0; node < AxisSize; ++node) {
            const int selected = value(node, peak_line);
            axis[node] += value(node, adjacent_line) - selected / 6;
        }
    }
}

template <std::size_t AxisSize>
int removeBackground(std::array<int, AxisSize> &axis,
                     std::size_t exclusion_radius) {
    const auto peak = std::max_element(axis.begin(), axis.end());
    const std::size_t peak_node = peak - axis.begin();
    int background_sum = 0;
    int background_nodes = 0;
    for (std::size_t node = 0; node < AxisSize; ++node) {
        if (node + exclusion_radius < peak_node ||
            node > peak_node + exclusion_radius) {
            background_sum += axis[node];
            ++background_nodes;
        }
    }
    const int background = background_nodes == 0
                               ? 0
                               : background_sum / background_nodes;
    for (int &value : axis)
        value -= background;

    int energy = axis[peak_node];
    if (peak_node > 0)
        energy += std::max(axis[peak_node - 1], 0);
    if (peak_node + 1 < AxisSize)
        energy += std::max(axis[peak_node + 1], 0);
    return energy;
}

template <std::size_t AxisSize>
std::size_t peakIndex(const std::array<int, AxisSize> &axis) {
    return std::max_element(axis.begin(), axis.end()) - axis.begin();
}

template <std::size_t AxisSize>
int ringCoordinate(const std::array<int, AxisSize> &axis,
                   const std::array<int, AxisSize> &mapping) {
    const std::size_t peak = peakIndex(axis);
    const bool edge = peak < 4 || peak >= AxisSize - 4;
    const int search_threshold = edge ? 300 : 100;
    const int weight_floor = edge ? 105 : 35;

    std::size_t first = peak;
    while (first > 0 && axis[first - 1] >= search_threshold)
        --first;
    if (first > 0 && axis[first - 1] >= weight_floor)
        --first;

    std::size_t last = peak;
    while (last + 1 < AxisSize && axis[last + 1] >= search_threshold)
        ++last;
    if (last + 1 < AxisSize && axis[last + 1] >= weight_floor)
        ++last;

    std::int64_t weighted = 0;
    std::int64_t total = 0;
    for (std::size_t node = first; node <= last; ++node) {
        const int weight = std::max(axis[node] - weight_floor, 0);
        total += weight;
        weighted += static_cast<std::int64_t>(mapping[node]) *
                    kSuperResolution * weight;
    }
    return total == 0 ? 0 : static_cast<int>(weighted / total);
}

template <std::size_t AxisSize>
int tipCoordinate(const std::array<int, AxisSize> &axis,
                  const std::array<int, AxisSize> &mapping, int pitch,
                  int slope, int left_edge, int right_edge) {
    const std::size_t peak = peakIndex(axis);
    const int peak_value = axis[peak];
    if (peak > 0 && peak + 1 < AxisSize) {
        int left = axis[peak - 1];
        const int right = axis[peak + 1];
        const int difference = left - right;
        int residual;
        if (difference == 0 || left < right) {
            int far = 0;
            if (peak + 2 < AxisSize) {
                far = axis[peak + 2];
            } else if (peak_value != 0) {
                far = right * left / peak_value;
                if (right < far && far != 0)
                    far = right * right / far;
            }
            residual = far - right;
        } else {
            int far = 0;
            if (peak >= 2) {
                far = axis[peak - 2];
            } else if (peak_value != 0) {
                far = right * left / peak_value;
                if (left < far && far != 0)
                    far = left * left / far;
            }
            residual = far - left;
            left = right;
        }
        const int denominator_raw =
            (left - peak_value) * slope + residual * (16 - slope);
        const int denominator =
            (denominator_raw < 0 ? denominator_raw + 7
                                 : denominator_raw) >> 3;
        int offset = denominator == 0
                         ? 0
                         : difference * pitch / denominator;
        offset = std::clamp(offset, -pitch / 2, pitch / 2);
        return mapping[peak] * kSuperResolution + offset;
    }

    const int neighbour = peak == 0 ? axis[1] : axis[AxisSize - 2];
    const int magnitude = std::max(std::abs(neighbour), 1);
    const int edge_parameter = peak == 0 ? left_edge : right_edge;
    const int raw = pitch * edge_parameter * (peak_value - magnitude) /
                    magnitude;
    const int correction = std::min(pitch, raw / 100);
    if (peak == 0) {
        return mapping.front() * kSuperResolution + pitch / 2 - correction;
    }
    return mapping.back() * kSuperResolution - pitch / 2 + correction;
}

int calibrateTipCoordinate(int coordinate, int difference) {
    if (std::abs(difference) <= kCalibrationThreshold)
        return coordinate;
    const int signed_threshold = difference < 0
                                     ? kCalibrationThreshold
                                     : -kCalibrationThreshold;
    return coordinate - (difference + signed_threshold) / kCalibrationRate;
}

int differenceToTilt(int difference) {
    const int sign = difference < 0 ? -1 : 1;
    int magnitude = std::abs(difference);
    const int maximum = kTiltDifferences.back() * kSuperResolution;
    if (magnitude >= maximum)
        magnitude = (kTiltDifferences.back() - 1) * kSuperResolution;

    std::size_t interval = 0;
    while (interval + 2 < kTiltDifferences.size() &&
           magnitude >= kTiltDifferences[interval + 1] * kSuperResolution)
        ++interval;
    const int difference_span =
        kTiltDifferences[interval + 1] - kTiltDifferences[interval];
    const int angle_per_unit = difference_span == 0
                                   ? 0
                                   : (kTiltAngles[interval + 1] -
                                      kTiltAngles[interval]) /
                                         difference_span;
    const int angle = kTiltAngles[interval] +
                      ((magnitude -
                        kTiltDifferences[interval] * kSuperResolution) *
                       angle_per_unit) /
                          kSuperResolution;
    return sign * (angle / 100);
}

}  // namespace

bool parseRawStylusFrame(const std::uint8_t *frame, std::size_t frame_size,
                         RawStylusFrame &output) {
    if (frame_size < kRequiredSize ||
        frame[kTransportSize + kPayloadDataTypeOffset] != 0x1d)
        return false;

    const std::uint8_t *source = frame + kPlaneOffset;
    const std::uint8_t *header = frame + kTransportSize + kSpecialSourceOffset;
    output.frame_interval = frameIntervalFromMode(
        frame[kTransportSize + 0x3d]);
    output.mutual_block = header[kMutualBlockIndexOffset];
    output.special_state = header[kSpecialStateOffset] != 0;
    readPlane(source, output.tip_x);
    readPlane(source + kPlaneBytes, output.tip_y);
    readPlane(source + 2 * kPlaneBytes, output.ring_x);
    readPlane(source + 3 * kPlaneBytes, output.ring_y);
    readPlane(frame + kMutualBlockOffset, output.mutual_block_values);
    return true;
}

void StylusMutualAssembler::setOrdinaryMatrix(
    const StylusMutualMatrix &matrix) {
    matrix_ = matrix;
    ready_ = true;
}

void StylusMutualAssembler::ingest(const RawStylusFrame &raw) {
    if (raw.mutual_block < 1 || raw.mutual_block > 4)
        return;
    block_sum_ += raw.mutual_block;
    const std::size_t first =
        (raw.mutual_block - 1) * kStylusMutualBlockNodes;
    std::copy(raw.mutual_block_values.begin(),
              raw.mutual_block_values.end(), staging_.begin() + first);
    if (raw.mutual_block != 4)
        return;
    if (block_sum_ == 10) {
        matrix_ = staging_;
        ready_ = true;
    }
    block_sum_ = 0;
}

void preprocessStylusInterference(RawStylusFrame &raw,
                                  const StylusMutualMatrix &touch_delta,
                                  bool ring_enabled) {
    constexpr std::size_t kGroupSize = 5;
    constexpr std::size_t kRowGroups =
        kStylusMutualRows / kGroupSize;
    constexpr std::size_t kColumnGroups =
        kStylusMutualColumns / kGroupSize;

    StylusMutualMatrix working_delta{};
    for (std::size_t row = 0; row < kStylusMutualRows; ++row)
        for (std::size_t column = 0; column < kStylusMutualColumns; ++column)
            working_delta[column * kStylusMutualRows + row] =
                touch_delta[row * kStylusMutualColumns + column];

    std::array<std::array<int, kStylusMutualColumns>, kRowGroups>
        row_projections{};
    std::array<bool, kRowGroups> active_rows{};
    for (std::size_t node = 0; node < kStylusMutualColumns; ++node) {
        for (std::size_t source = 0; source < kStylusMutualRows; ++source) {
            const std::size_t group = source / kGroupSize;
            int &sum = row_projections[group][node];
            sum = wrapAdd(
                sum, working_delta[node + source * kStylusMutualColumns]);
            if (sum > 300)
                active_rows[group] = true;
        }
    }
    for (std::size_t group = 0; group < kRowGroups; ++group) {
        if (!active_rows[group])
            continue;
        subtractInterferenceLine(
            raw.tip_y.data() + group * kStylusAxis60Nodes, 1,
            row_projections[group]);
        if (ring_enabled) {
            subtractInterferenceLine(
                raw.ring_y.data() + group * kStylusAxis60Nodes, 1,
                row_projections[group]);
        }
    }

    std::array<std::array<int, kStylusMutualRows>, kColumnGroups>
        column_projections{};
    std::array<bool, kColumnGroups> active_columns{};
    for (std::size_t node = 0; node < kStylusMutualRows; ++node) {
        for (std::size_t source = 0; source < kStylusMutualColumns; ++source) {
            const std::size_t group = source / kGroupSize;
            int &sum = column_projections[group][node];
            sum = wrapAdd(
                sum, working_delta[node * kStylusMutualColumns + source]);
            if (sum > 300)
                active_columns[group] = true;
        }
    }
    for (std::size_t group = 0; group < kColumnGroups; ++group) {
        if (!active_columns[group])
            continue;
        subtractInterferenceLine(
            raw.tip_x.data() + group, kColumnGroups,
            column_projections[group]);
        if (ring_enabled) {
            subtractInterferenceLine(
                raw.ring_x.data() + group, kColumnGroups,
                column_projections[group]);
        }
    }
}

bool TipGateState::accept(int energy_40, int energy_60) {
    const int threshold = require_reacquire ? 260 : 200;
    if (energy_40 < threshold || energy_60 < threshold) {
        require_reacquire = true;
        return false;
    }
    require_reacquire = false;
    return true;
}

void StylusCoordinateState::rejectFrame() {
    current_difference_x = 0;
    current_difference_y = 0;
    ++rejected_frames;
}

void StylusCoordinateState::acceptFrame() {
    rejected_frames = 0;
}

void separateTipAxes(const RawStylusFrame &raw, TipAxes &output) {
    // 0x6883c reads Tip1 as 40 nodes with 12 candidate columns and Tip2 as
    // eight candidate rows with 60 nodes.
    selectColumns<kStylusAxis40Nodes, kTip1CandidateColumns>(
        raw.tip_x.data(), output.axis_40);
    selectRows<kStylusAxis60Nodes, kTip2CandidateRows>(
        raw.tip_y.data(), output.axis_60);
}

void removeTipBackground(TipAxes &axes) {
    axes.energy_60 = removeBackground(axes.axis_60, 3);
    axes.energy_40 = removeBackground(axes.axis_40, 3);
}

void refineTipAxes(TipAxes &axes) {
    // 0x65d90 applies a second, narrower background pass after Ring
    // separation and replaces the Tip energies with the refined values.
    axes.energy_60 = removeBackground(axes.axis_60, 1);
    axes.energy_40 = removeBackground(axes.axis_40, 1);
}

bool acceptTipEdges(const TipAxes &axes) {
    constexpr int edge_threshold = 650;
    const std::size_t peak_60 = peakIndex(axes.axis_60);
    const std::size_t peak_40 = peakIndex(axes.axis_40);
    int axis_60_reject = 0;
    if (peak_60 == 0 && axes.axis_60[0] < edge_threshold) {
        axis_60_reject = axes.axis_60[1] < axes.axis_60[0] / 4 ? -1 : 0;
    } else if (peak_60 + 1 == kStylusAxis60Nodes &&
               axes.axis_60.back() < edge_threshold) {
        axis_60_reject = axes.axis_60[kStylusAxis60Nodes - 2] <
                                 axes.axis_60.back() / 4
                             ? -1
                             : 0;
    }

    if (peak_40 == 0 && axes.axis_40[0] < edge_threshold) {
        if (axis_60_reject != -1 &&
            axes.axis_40[0] / 4 <= axes.axis_40[1])
            return true;
        return false;
    }
    if (peak_40 + 1 == kStylusAxis40Nodes &&
        axes.axis_40.back() < edge_threshold) {
        if (axis_60_reject != -1 &&
            axes.axis_40.back() / 4 <=
                axes.axis_40[kStylusAxis40Nodes - 2])
            return true;
        return false;
    }
    return axis_60_reject != -1;
}

void separateRingAxes(const RawStylusFrame &raw, RingAxes &output) {
    // 0x67da4 uses the Tip planes only to guide the Ring scan-group choice.
    selectRingColumn<kStylusAxis40Nodes, kTip1CandidateColumns>(
        raw.tip_x.data(), raw.ring_x.data(), output.axis_40);
    selectRingRow<kStylusAxis60Nodes, kTip2CandidateRows>(
        raw.tip_y.data(), raw.ring_y.data(), output.axis_60);
}

void removeRingBackground(RingAxes &axes, int threshold) {
    axes.energy_60 = removeBackground(axes.axis_60, 3);
    axes.energy_40 = removeBackground(axes.axis_40, 3);
    axes.valid = axes.energy_60 > threshold && axes.energy_40 > threshold;
}

StylusCoordinates calculateStylusCoordinates(const TipAxes &tip,
                                             const RingAxes &ring,
                                             StylusCoordinateState &state) {
    StylusCoordinates output;
    const std::size_t tip_peak_40 = peakIndex(tip.axis_40);
    const std::size_t tip_peak_60 = peakIndex(tip.axis_60);
    output.tip_x = tipCoordinate(
        tip.axis_40, kAxis40Mapping, kTipPitch40, kTipSlope40,
        kTipEdgeParameter, kTipEdgeParameter);
    output.tip_y = tipCoordinate(
        tip.axis_60, kAxis60Mapping, kTipPitch60, kTipSlope60,
        kTipEdgeParameter, kTipEdgeParameter);
    if (!ring.valid)
        return output;

    output.ring_x = ringCoordinate(ring.axis_40, kAxis40Mapping);
    output.ring_y = ringCoordinate(ring.axis_60, kAxis60Mapping);
    const bool tip_on_edge = tip_peak_40 == 0 ||
                             tip_peak_40 + 1 == kStylusAxis40Nodes ||
                             tip_peak_60 == 0 ||
                             tip_peak_60 + 1 == kStylusAxis60Nodes;
    if (!tip_on_edge) {
        state.current_difference_x = output.ring_x - output.tip_x;
        state.current_difference_y = output.ring_y - output.tip_y;
        if (state.rejected_frames > 0) {
            state.previous_difference_x = state.current_difference_x;
            state.previous_difference_y = state.current_difference_y;
            state.initialized = true;
        }
    }
    if (state.initialized) {
        if (std::abs(state.current_difference_x -
                     state.previous_difference_x) >
            50 * kSuperResolution) {
            state.current_difference_x = state.previous_difference_x;
        }
        if (std::abs(state.current_difference_y -
                     state.previous_difference_y) >
            50 * kSuperResolution) {
            state.current_difference_y = state.previous_difference_y;
        }
    }
    output.difference_x = state.current_difference_x;
    output.difference_y = state.current_difference_y;
    state.previous_difference_x = state.current_difference_x;
    state.previous_difference_y = state.current_difference_y;
    state.initialized = true;
    output.tip_x = calibrateTipCoordinate(output.tip_x,
                                          output.difference_x);
    output.tip_y = calibrateTipCoordinate(output.tip_y,
                                          output.difference_y);
    output.tilt_x = differenceToTilt(output.difference_y);
    output.tilt_y = differenceToTilt(output.difference_x);
    return output;
}

void StylusDecoder::reset() {
    tip_gate_ = {};
    coordinate_state_ = {};
    axis_history_ = {};
    axis_history_.blend = 100;
    status_ = 0;
    previous_status_ = 0;
    stylus_level_ = 0;
    coordinate_kalman_.reset();
    tilt_kalman_.reset();
    coordinate_follower_.reset();
    tilt_follower_.reset();
}

void StylusDecoder::KalmanState::reset() {
    *this = {};
    covariance_scale = 1.0F;
    process_noise = 0.01F;
}

void StylusDecoder::FollowerState::reset() {
    *this = {};
}

void StylusDecoder::rejectFrame() {
    coordinate_state_.rejectFrame();
    axis_history_.blend = 100;
    status_ = 0;
    previous_status_ = 0;
    stylus_level_ = 0;
}

void StylusDecoder::smoothAxes(StylusFrameResult &result,
                                      bool special_state) {
    const int energy_units = result.tip.energy_60 / 200 +
                             result.tip.energy_40 / 200;
    stylus_level_ = energy_units < 12 ? energy_units / 2 : 5;
    if (!special_state) {
        axis_history_.blend = 100;
        return;
    }

    int blend = stylus_level_ > 0 && stylus_level_ < 4
                    ? stylus_level_ * 10
                    : 100;
    if (axis_history_.blend < blend)
        blend = axis_history_.blend + 1;

    const bool copy_tip = previous_status_ != 3;
    copyOrBlendAxis(result.tip.axis_40, axis_history_.tip_40,
                    blend, copy_tip);
    copyOrBlendAxis(result.tip.axis_60, axis_history_.tip_60,
                    blend, copy_tip);
    const bool copy_ring = previous_status_ == 0 || status_ != 3;
    copyOrBlendAxis(result.ring.axis_40, axis_history_.ring_40,
                    blend, copy_ring);
    copyOrBlendAxis(result.ring.axis_60, axis_history_.ring_60,
                    blend, copy_ring);
    axis_history_.blend = blend;
}

void StylusDecoder::advanceStatus(bool special_state) {
    if (special_state) {
        status_ = 3;
    } else if (status_ == 0 || status_ == 3) {
        status_ = 1;
    } else if (status_ == 1) {
        status_ = 2;
    }
}

StylusDecoder::FilterPoint StylusDecoder::filterKalman(
    KalmanState &filter, FilterPoint input, int process_divisor,
    int covariance_divisor) const {
    const bool initialize = status_ == 1 ||
        (status_ == 3 && previous_status_ == 0);
    if (initialize) {
        filter.reset();
        filter.state = {static_cast<float>(input.x), 0.0F,
                        static_cast<float>(input.y), 0.0F};
        filter.previous_x = filter.state[0];
        filter.previous_y = filter.state[2];
        filter.covariance[0] = 10.0F;
        filter.covariance[5] = 2.0F;
        filter.covariance[10] = 10.0F;
        filter.covariance[15] = 2.0F;
        return input;
    }
    if (status_ != 2 && status_ != 3)
        return {static_cast<int>(filter.state[0]),
                static_cast<int>(filter.state[2])};

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

    const float residual_x = static_cast<float>(input.x) -
                             predicted_state[0];
    const float residual_y = static_cast<float>(input.y) -
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
    filter.process_noise = process_divisor == 0
        ? 0.0F
        : std::sqrt(state_distance * innovation_distance) /
              static_cast<float>(process_divisor);
    filter.previous_x = filter.state[0];
    filter.previous_y = filter.state[2];

    const float covariance_total = unscaled_covariance[0] +
                                   unscaled_covariance[10];
    float scale = 1.0F;
    if (covariance_total != 0.0F) {
        scale = (((filter.residual_x - (process_noise_used + 1.0F)) +
                  filter.residual_y) - (process_noise_used + 1.0F)) /
                covariance_total / static_cast<float>(covariance_divisor);
        scale = std::clamp(scale, filter.covariance_scale - 0.1F,
                           filter.covariance_scale + 0.02F);
        scale = std::clamp(scale, 1.0F, 1.6F);
    }
    filter.covariance_scale = scale;
    return {static_cast<int>(filter.state[0]),
            static_cast<int>(filter.state[2])};
}

StylusDecoder::FilterPoint StylusDecoder::filterFollower(
    FollowerState &state, FilterPoint input, int route, bool special_state,
    int frame_interval) const {
    const bool initialize = status_ == 1 ||
        (status_ == 3 && previous_status_ == 0);
    auto route_limit = [&]() {
        if (route == 1)
            return 3;
        int limit = 10;
        if (state.active)
            limit = state.age >= 18 ? 20 : 30;
        if (special_state && stylus_level_ + 1 != 0)
            limit = limit * 10 / (stylus_level_ + 1);
        return limit;
    };

    if (initialize) {
        state.reset();
        state.initial_x = input.x;
        state.initial_y = input.y;
        state.output_x = input.x;
        state.output_y = input.y;
        state.active = true;
        state.previous_active = true;
        state.limit = route_limit();
        state.previous_limit = state.limit;
        return input;
    }
    if (status_ != 2 && status_ != 3)
        return {state.output_x, state.output_y};

    if (state.age < frame_interval)
        ++state.age;
    state.limit = route_limit();

    if (!state.active) {
        if (state.limit < state.previous_limit && state.previous_active) {
            const int ratio = state.previous_limit == 0
                                  ? 0
                                  : state.limit * 100 /
                                        state.previous_limit;
            state.offset_x -= state.offset_x * ratio / 100;
            state.offset_y -= state.offset_y * ratio / 100;
        } else if (state.limit == state.previous_limit &&
                   state.previous_active) {
            state.offset_x = 0;
            state.offset_y = 0;
        }
        constexpr int super_resolution = 10;
        const int base = super_resolution * 50;
        if (super_resolution < state.average_x) {
            state.offset_x = std::abs(state.offset_x) < 1
                ? 0
                : state.offset_x *
                      (base + super_resolution + state.average_x) /
                      (base + state.average_x * 2);
        }
        if (super_resolution < state.average_y) {
            state.offset_y = std::abs(state.offset_y) < 1
                ? 0
                : state.offset_y *
                      (base + super_resolution + state.average_y) /
                      (base + state.average_y * 2);
        }
    } else {
        state.offset_x = 0;
        state.offset_y = 0;
    }

    state.previous_active = state.active;
    if (!state.active) {
        input.x -= state.offset_x;
        input.y -= state.offset_y;
    }
    state.raw_x = input.x;
    state.raw_y = input.y;
    const int distance = static_cast<int>(std::sqrt(
        static_cast<float>(input.x - state.output_x) *
            static_cast<float>(input.x - state.output_x) +
        static_cast<float>(input.y - state.output_y) *
            static_cast<float>(input.y - state.output_y)));
    if (state.limit < distance) {
        const bool was_active = state.active;
        if (was_active && state.previous_limit < distance)
            state.limit = std::min(state.previous_limit, distance);
        const int ratio = distance == 0 ? 0 : state.limit * 100 / distance;
        const int step_x = ratio * (input.x - state.output_x) / 100;
        const int step_y = ratio * (input.y - state.output_y) / 100;
        input.x -= step_x;
        input.y -= step_y;
        if (!was_active) {
            state.average_x = (std::abs(input.x - state.output_x) +
                               state.average_x * 4) / 5;
            state.average_y = (std::abs(input.y - state.output_y) +
                               state.average_y * 4) / 5;
        } else {
            state.active = false;
            state.offset_x = step_x;
            state.offset_y = step_y;
        }
    } else {
        input.x = state.output_x;
        input.y = state.output_y;
    }
    state.output_x = input.x;
    state.output_y = input.y;
    state.previous_limit = state.limit;
    return input;
}

void StylusDecoder::applyFinalFilter(
    StylusCoordinates &coordinates, bool ring_valid, bool special_state,
    int frame_interval) {
    int process_divisor = 200;
    int covariance_divisor = 800;
    if (special_state && stylus_level_ > 0) {
        const int divisor = stylus_level_ * 2 + 2;
        process_divisor = 2500 / divisor;
        covariance_divisor = 10000 / divisor;
    }

    FilterPoint tip = filterKalman(
        coordinate_kalman_, {coordinates.tip_x, coordinates.tip_y},
        process_divisor, covariance_divisor);
    tip = filterFollower(coordinate_follower_, tip, 0, special_state,
                         frame_interval);
    coordinates.tip_x = tip.x;
    coordinates.tip_y = tip.y;
    if (!ring_valid)
        return;

    FilterPoint tilt = filterKalman(
        tilt_kalman_, {coordinates.tilt_y, coordinates.tilt_x},
        process_divisor, covariance_divisor);
    tilt = filterFollower(tilt_follower_, tilt, 1, special_state,
                          frame_interval);
    coordinates.tilt_y = tilt.x;
    coordinates.tilt_x = tilt.y;
}

StylusFrameResult StylusDecoder::process(const RawStylusFrame &raw) {
    StylusFrameResult result;
    separateTipAxes(raw, result.tip);
    removeTipBackground(result.tip);
    if (!tip_gate_.accept(result.tip.energy_40, result.tip.energy_60)) {
        rejectFrame();
        return result;
    }

    separateRingAxes(raw, result.ring);
    refineTipAxes(result.tip);
    removeRingBackground(result.ring);
    if (!acceptTipEdges(result.tip)) {
        rejectFrame();
        return result;
    }

    smoothAxes(result, raw.special_state);
    advanceStatus(raw.special_state);
    result.active = true;
    result.prefilter_coordinates = calculateStylusCoordinates(
        result.tip, result.ring, coordinate_state_);
    result.coordinates = result.prefilter_coordinates;
    applyFinalFilter(result.coordinates, result.ring.valid,
                     raw.special_state, raw.frame_interval);
    coordinate_state_.acceptFrame();
    previous_status_ = status_;
    return result;
}

}  // namespace nvt
