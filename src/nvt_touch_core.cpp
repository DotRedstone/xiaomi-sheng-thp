// SPDX-License-Identifier: Apache-2.0

#include "nvt_touch_core.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <tuple>
#include <utility>

namespace nvt {
namespace {

constexpr int kPeakThreshold = 220;
constexpr int kNoiseThreshold = 100;
constexpr int kCoordinateReduce = 25;
constexpr int kDownThreshold = 300;
constexpr int kUpThreshold = 220;
constexpr int kBaseRefreshDelay = 8;

constexpr std::array<int, kRows> kRowMap = {
    26, 78, 130, 182, 234, 286, 338, 390, 442, 494,
    546, 598, 650, 702, 754, 806, 856, 906, 956, 1006,
    1056, 1106, 1156, 1206, 1256, 1306, 1356, 1406, 1456, 1506,
    1556, 1606, 1656, 1706, 1756, 1806, 1856, 1906, 1956, 2006,
};

constexpr std::array<int, kColumns> kColumnMap = {
    25, 75, 125, 175, 225, 275, 325, 375, 425, 476,
    528, 580, 632, 684, 736, 788, 840, 892, 944, 996,
    1048, 1099, 1149, 1199, 1249, 1299, 1349, 1399, 1449, 1499,
    1549, 1599, 1649, 1699, 1749, 1799, 1849, 1899, 1949, 2000,
    2052, 2104, 2156, 2208, 2260, 2312, 2364, 2416, 2468, 2520,
    2572, 2623, 2673, 2723, 2773, 2823, 2873, 2923, 2973, 3023,
};

struct Profile {
    std::array<int, 11> value{};
};

using Plan = TouchCore::Plan;
using MergeHistory = TouchCore::MergeHistory;

template <typename Function>
void forNeighbors8(int index, Function function) {
    const int row = index / kColumns;
    const int column = index % kColumns;
    for (int row_offset = -1; row_offset <= 1; ++row_offset) {
        const int other_row = row + row_offset;
        if (other_row < 0 || other_row >= kRows)
            continue;
        for (int column_offset = -1; column_offset <= 1; ++column_offset) {
            if (row_offset == 0 && column_offset == 0)
                continue;
            const int other_column = column + column_offset;
            if (other_column >= 0 && other_column < kColumns)
                function(other_row * kColumns + other_column);
        }
    }
}

template <typename Function>
void forNeighbors4(int index, Function function) {
    const int row = index / kColumns;
    const int column = index % kColumns;
    if (row > 0)
        function(index - kColumns);
    if (column > 0)
        function(index - 1);
    if (column + 1 < kColumns)
        function(index + 1);
    if (row + 1 < kRows)
        function(index + kColumns);
}

bool overlaps(const NodeSet &first, const NodeSet &second) {
    return (first & second).any();
}

int blockTotal(const MergeHistory &history) {
    return std::accumulate(history.block_sizes.begin(),
                           history.block_sizes.end(), 0);
}

Matrix projectionNoiseFilter(const Matrix &delta) {
    Matrix filtered{};
    for (int index = 0; index < kNodes; ++index)
        filtered[index] = delta[index] >= kNoiseThreshold ? delta[index] : 0;
    return filtered;
}

Matrix strictNoiseFilter(const Matrix &delta) {
    Matrix filtered{};
    for (int index = 0; index < kNodes; ++index)
        filtered[index] = delta[index] > kNoiseThreshold ? delta[index] : 0;
    return filtered;
}

Matrix domainFrameHistoryFilter(const Matrix &delta, int matrix_maximum) {
    if (matrix_maximum < kPeakThreshold + 100)
        return delta;
    Matrix filtered = projectionNoiseFilter(delta);
    constexpr std::array<int, 8> row_offsets = {-1, -1, 0, 1, 1, 1, 0, -1};
    constexpr std::array<int, 8> column_offsets = {0, -1, -1, -1, 0, 1, 1, 1};
    for (int row = 1; row < kRows - 1; ++row) {
        for (int column = 1; column < kColumns - 1; ++column) {
            const int index = row * kColumns + column;
            const int current = delta[index];
            if (current < 1)
                continue;
            int similar_low = 0;
            int large_gap = 0;
            for (size_t offset = 0; offset < row_offsets.size(); ++offset) {
                const int neighbor = delta[(row + row_offsets[offset]) * kColumns +
                                           column + column_offsets[offset]];
                if (neighbor < 1)
                    continue;
                const int difference = std::abs(current - neighbor);
                if (difference < 38 && neighbor < 160)
                    ++similar_low;
                if (difference > kPeakThreshold)
                    ++large_gap;
            }
            if (similar_low > 2 || (similar_low > 1 && large_gap == 0))
                filtered[index] = 0;
        }
    }
    return filtered;
}

std::vector<Profile> projectionProfiles(const Matrix &filtered, int method) {
    const int line_count = method == 2 ? kColumns : kRows;
    const int line_width = method == 2 ? kRows : kColumns;
    std::vector<Profile> profiles(line_count);
    for (int line = 0; line < line_count; ++line) {
        int first = -1;
        int last = -1;
        int count = 0;
        int total = 0;
        int peak = 0;
        for (int position = 0; position < line_width; ++position) {
            const int value = method == 2
                                  ? filtered[position * kColumns + line]
                                  : filtered[line * kColumns + position];
            if (value == 0)
                continue;
            if (first < 0)
                first = position;
            last = position;
            ++count;
            total += value;
            peak = std::max(peak, value);
        }
        if (first < 0)
            continue;
        auto &value = profiles[line].value;
        value[0] = last - first;
        value[1] = count;
        value[2] = total;
        value[3] = total / count;
        value[4] = peak;
        if (method == 2) {
            value[5] = line;
            value[6] = first;
            value[8] = line;
            value[9] = last;
        } else {
            value[5] = first;
            value[6] = line;
            value[8] = last;
            value[9] = line;
        }
    }
    return profiles;
}

float f32Product(int value, float ratio) {
    volatile float first = static_cast<float>(value);
    volatile float second = ratio;
    return static_cast<float>(first * second);
}

std::optional<int> projectionValley(const std::vector<Profile> &profiles,
                                    int left, int right, int method,
                                    int split_count = 0) {
    if (right - left <= 1)
        return std::nullopt;
    const auto &left_profile = profiles[left].value;
    const auto &right_profile = profiles[right].value;
    int best_score = std::numeric_limits<int>::max();
    int best_index = -1;
    for (int index = left + 1; index < right; ++index) {
        const auto &profile = profiles[index].value;
        const int left_distance = index - left;
        const int right_distance = right - index;
        int reference_sum;
        int reference_peak;
        int other_sum;
        if (left_distance < right_distance) {
            reference_sum = left_profile[2];
            reference_peak = left_profile[4];
            other_sum = right_profile[2];
        } else if (right_distance < left_distance) {
            reference_sum = right_profile[2];
            reference_peak = right_profile[4];
            other_sum = left_profile[2];
        } else {
            reference_sum = right_profile[3] <= left_profile[3]
                                ? left_profile[2] : right_profile[2];
            reference_peak = std::max(left_profile[4], right_profile[4]);
            other_sum = right_profile[2];
        }
        const int span_limit = method == 2 ? 13 : 7;
        bool accepted = profile[0] < span_limit &&
                        profile[2] < left_profile[2] * 0.9 &&
                        profile[2] < right_profile[2] * 0.9 &&
                        profile[2] < f32Product(reference_sum, 0.75f) &&
                        profile[4] < f32Product(reference_peak, 0.75f);
        if (method == 3 && split_count > 5 &&
            other_sum < reference_peak * 0.5 &&
            profile[2] < f32Product(reference_sum, 0.75f) &&
            profile[4] < f32Product(reference_peak, 0.75f))
            accepted = true;
        if (!accepted)
            continue;
        const int score = profile[2] + profile[4];
        if (score < best_score || (score == best_score && index > best_index)) {
            best_score = score;
            best_index = index;
        }
    }
    return best_index < 0 ? std::nullopt : std::optional<int>(best_index);
}

std::optional<Plan> peakProjectionPlan(const Matrix &filtered,
                                       const Domain &domain) {
    if (domain.peaks.size() < 2)
        return std::nullopt;
    for (int method : {2, 3}) {
        std::vector<int> coordinates;
        coordinates.reserve(domain.peaks.size());
        for (const Peak &peak : domain.peaks)
            coordinates.push_back(method == 2 ? peak.index % kColumns
                                               : peak.index / kColumns);
        std::sort(coordinates.begin(), coordinates.end());
        if (std::adjacent_find(coordinates.begin(), coordinates.end()) !=
            coordinates.end())
            continue;
        const auto profiles = projectionProfiles(filtered, method);
        Plan plan{method, {}};
        bool valid = true;
        for (size_t index = 1; index < coordinates.size(); ++index) {
            const auto cut = projectionValley(profiles, coordinates[index - 1],
                                              coordinates[index], method);
            if (!cut) {
                valid = false;
                break;
            }
            plan.cuts.push_back(*cut);
        }
        if (valid && !plan.cuts.empty())
            return plan;
    }
    return std::nullopt;
}

Plan peakHistoryPlan(const Matrix &filtered, std::vector<Peak> peaks,
                     int method) {
    const auto profiles = projectionProfiles(filtered, method);
    std::vector<int> coordinates;
    for (const Peak &peak : peaks)
        coordinates.push_back(method == 2 ? peak.index % kColumns
                                           : peak.index / kColumns);
    std::sort(coordinates.begin(), coordinates.end());
    Plan plan{method, {}};
    for (size_t index = 1; index < coordinates.size(); ++index) {
        const int left = coordinates[index - 1];
        const int right = coordinates[index];
        if (right - left <= 1) {
            plan.cuts.push_back(left);
            continue;
        }
        int best_score = std::numeric_limits<int>::max();
        int best_index = -1;
        for (int candidate = left + 1; candidate < right; ++candidate) {
            const int score = profiles[candidate].value[2] +
                              profiles[candidate].value[4];
            if (score < best_score ||
                (score == best_score && candidate > best_index)) {
                best_score = score;
                best_index = candidate;
            }
        }
        plan.cuts.push_back(best_index);
    }
    return plan;
}

std::optional<std::vector<Domain>> applyPeakPlan(const Domain &domain,
                                                  const Plan &plan) {
    const size_t count = plan.cuts.size() + 1;
    std::vector<Domain> groups(count);
    for (size_t index = 0; index < count; ++index)
        groups[index].label = domain.label + static_cast<int>(index);
    for (int node = 0; node < kNodes; ++node) {
        if (!domain.nodes.test(node))
            continue;
        const int coordinate = plan.method == 2 ? node % kColumns
                                                 : node / kColumns;
        const size_t owner = std::lower_bound(plan.cuts.begin(), plan.cuts.end(),
                                              coordinate) - plan.cuts.begin();
        groups[owner].nodes.set(node);
    }
    for (const Peak &peak : domain.peaks) {
        const int coordinate = plan.method == 2 ? peak.index % kColumns
                                                 : peak.index / kColumns;
        const size_t owner = std::lower_bound(plan.cuts.begin(), plan.cuts.end(),
                                              coordinate) - plan.cuts.begin();
        groups[owner].peaks.push_back(peak);
    }
    for (const Domain &group : groups) {
        if (group.nodes.none() || group.peaks.size() != 1)
            return std::nullopt;
    }
    return groups;
}

std::optional<std::vector<Domain>> applyPreviousGroupLabels(
    const Domain &domain, const std::vector<Domain> &previous_groups) {
    std::vector<Domain> groups;
    std::set<int> covered_peaks;
    for (const Domain &previous : previous_groups) {
        NodeSet nodes = domain.nodes & previous.nodes;
        if (nodes.none())
            continue;
        std::vector<Peak> peaks;
        for (const Peak &peak : domain.peaks)
            if (nodes.test(peak.index))
                peaks.push_back(peak);
        if (peaks.size() != 1 || covered_peaks.contains(peaks[0].index))
            continue;
        covered_peaks.insert(peaks[0].index);
        groups.push_back(Domain{domain.label + static_cast<int>(groups.size()),
                                nodes, peaks});
    }
    if (groups.size() != domain.peaks.size())
        return std::nullopt;
    return groups;
}

std::vector<Domain> projectionPeakGroups(
    const Matrix &filtered, const std::vector<Domain> &domains,
    const std::vector<Plan> &previous_plans,
    const std::vector<Domain> &previous_groups) {
    std::vector<Domain> groups;
    for (const Domain &domain : domains) {
        std::optional<std::vector<Domain>> split;
        if (const auto plan = peakProjectionPlan(filtered, domain))
            split = applyPeakPlan(domain, *plan);
        if (!split) {
            std::vector<std::vector<Domain>> candidates;
            for (const Plan &plan : previous_plans)
                if (auto candidate = applyPeakPlan(domain, plan))
                    candidates.push_back(std::move(*candidate));
            if (candidates.size() == 1)
                split = std::move(candidates[0]);
        }
        if (!split && domain.peaks.size() > 1)
            split = applyPreviousGroupLabels(domain, previous_groups);
        if (split)
            groups.insert(groups.end(), split->begin(), split->end());
        else
            groups.push_back(domain);
    }
    for (size_t index = 0; index < groups.size(); ++index)
        groups[index].label = static_cast<int>(index) + 1;
    return groups;
}

std::vector<Peak> localPeaks(const Matrix &delta) {
    std::vector<Peak> peaks;
    for (int index = 0; index < kNodes; ++index) {
        const int value = delta[index];
        if (value < kPeakThreshold)
            continue;
        bool maximum = true;
        forNeighbors8(index, [&](int neighbor) {
            if (value < delta[neighbor])
                maximum = false;
        });
        if (maximum)
            peaks.push_back(Peak{index, value});
    }
    return peaks;
}

int positiveCrossAverage(const Matrix &delta, int index) {
    const int row = index / kColumns;
    const int column = index % kColumns;
    int total = delta[index];
    int count = 1;
    auto add = [&](int neighbor) {
        if (delta[neighbor] > 0) {
            total += delta[neighbor];
            ++count;
        }
    };
    if (column > 0) add(index - 1);
    if (row > 0) add(index - kColumns);
    if (column + 1 < kColumns) add(index + 1);
    if (row + 1 < kRows) add(index + kColumns);
    return total / count;
}

std::vector<Peak> filterEqualAdjacentPeaks(const Matrix &delta,
                                           std::vector<Peak> peaks) {
    std::stable_sort(peaks.begin(), peaks.end(),
                     [](const Peak &a, const Peak &b) {
                         return a.value > b.value;
                     });
    std::vector<bool> enabled(peaks.size(), true);
    for (size_t index = 0; index + 1 < peaks.size(); ++index) {
        const Peak &first = peaks[index];
        const Peak &second = peaks[index + 1];
        if (first.value != second.value)
            continue;
        const int first_row = first.index / kColumns;
        const int first_column = first.index % kColumns;
        const int second_row = second.index / kColumns;
        const int second_column = second.index % kColumns;
        if (std::abs(first_row - second_row) +
                std::abs(first_column - second_column) >= 2)
            continue;
        if (positiveCrossAverage(delta, second.index) <
            positiveCrossAverage(delta, first.index))
            enabled[index + 1] = false;
        else
            enabled[index] = false;
    }
    std::vector<Peak> filtered;
    for (size_t index = 0; index < peaks.size() && filtered.size() < 32; ++index)
        if (enabled[index])
            filtered.push_back(peaks[index]);
    return filtered;
}

std::vector<Domain> connectedDomains(const Matrix &filtered,
                                     const std::vector<Peak> &peaks,
                                     bool orthogonal = false) {
    std::array<bool, kNodes> remaining{};
    std::array<int, kNodes> peak_lookup{};
    peak_lookup.fill(-1);
    for (int index = 0; index < kNodes; ++index)
        remaining[index] = filtered[index] > 0;
    for (size_t index = 0; index < peaks.size(); ++index)
        peak_lookup[peaks[index].index] = static_cast<int>(index);
    std::vector<Domain> domains;
    std::array<int, kNodes> queue{};
    for (int first = 0; first < kNodes; ++first) {
        if (!remaining[first])
            continue;
        int head = 0;
        int tail = 0;
        queue[tail++] = first;
        remaining[first] = false;
        NodeSet nodes;
        nodes.set(first);
        while (head < tail) {
            const int current = queue[head++];
            auto visit = [&](int neighbor) {
                if (!remaining[neighbor])
                    return;
                remaining[neighbor] = false;
                nodes.set(neighbor);
                queue[tail++] = neighbor;
            };
            if (orthogonal)
                forNeighbors4(current, visit);
            else
                forNeighbors8(current, visit);
        }
        std::vector<Peak> domain_peaks;
        for (int index = 0; index < kNodes; ++index)
            if (nodes.test(index) && peak_lookup[index] >= 0)
                domain_peaks.push_back(peaks[peak_lookup[index]]);
        if (!domain_peaks.empty())
            domains.push_back(Domain{static_cast<int>(domains.size()) + 1,
                                     nodes, std::move(domain_peaks)});
    }
    return domains;
}

std::vector<Domain> peakMergeGroups(const Matrix &filtered,
                                    const std::vector<Peak> &peaks) {
    std::array<std::vector<int>, kNodes> memberships;
    for (size_t peak_index = 0; peak_index < peaks.size(); ++peak_index) {
        const Peak &peak = peaks[peak_index];
        const int peak_row = peak.index / kColumns;
        const int peak_column = peak.index % kColumns;
        std::array<bool, kNodes> visited{};
        std::array<int, kNodes> queue{};
        int head = 0;
        int tail = 0;
        queue[tail++] = peak.index;
        visited[peak.index] = true;
        while (head < tail) {
            const int index = queue[head++];
            memberships[index].push_back(static_cast<int>(peak_index));
            forNeighbors8(index, [&](int neighbor) {
                if (visited[neighbor])
                    return;
                const int row = neighbor / kColumns;
                const int column = neighbor % kColumns;
                if (std::abs(row - peak_row) > 8 ||
                    std::abs(column - peak_column) > 8)
                    return;
                const int value = filtered[neighbor];
                if (value < 1 || value > peak.value)
                    return;
                visited[neighbor] = true;
                queue[tail++] = neighbor;
            });
        }
    }
    std::vector<NodeSet> nodes_by_peak(peaks.size());
    for (int index = 0; index < kNodes; ++index) {
        const auto &owners = memberships[index];
        if (owners.empty())
            continue;
        const int row = index / kColumns;
        const int column = index % kColumns;
        int owner = owners[0];
        double best_distance = std::numeric_limits<double>::infinity();
        int best_peak = std::numeric_limits<int>::min();
        for (int candidate : owners) {
            const Peak &peak = peaks[candidate];
            const double distance = std::hypot(
                row - peak.index / kColumns,
                column - peak.index % kColumns);
            if (distance < best_distance ||
                (distance == best_distance && peak.value > best_peak)) {
                owner = candidate;
                best_distance = distance;
                best_peak = peak.value;
            }
        }
        nodes_by_peak[owner].set(index);
    }
    std::vector<Domain> groups;
    for (size_t index = 0; index < peaks.size(); ++index)
        if (nodes_by_peak[index].any())
            groups.push_back(Domain{static_cast<int>(groups.size()) + 1,
                                    nodes_by_peak[index], {peaks[index]}});
    return groups;
}

NodeSet peakSet(const Domain &domain) {
    NodeSet result;
    for (const Peak &peak : domain.peaks)
        result.set(peak.index);
    return result;
}

bool containsPeakSet(const std::vector<NodeSet> &sets, const NodeSet &value) {
    return std::find(sets.begin(), sets.end(), value) != sets.end();
}

std::vector<Domain> applyPeakMergeHistory(
    const Matrix &filtered, const std::vector<Domain> &domains,
    const std::vector<Peak> &peaks,
    const std::vector<Domain> &projection_groups,
    const std::vector<NodeSet> &previous_domain_peak_sets,
    const std::vector<MergeHistory> &previous_histories,
    const std::vector<Domain> &previous_groups, bool peak_was_filtered) {
    std::map<int, int> peak_order;
    for (size_t index = 0; index < peaks.size(); ++index)
        peak_order[peaks[index].index] = static_cast<int>(index);
    std::map<int, Domain> group_by_peak;
    for (const Domain &group : projection_groups)
        if (group.peaks.size() == 1)
            group_by_peak[group.peaks[0].index] = group;
    std::vector<Domain> selected;
    for (const Domain &domain : domains) {
        const NodeSet domain_peak_set = peakSet(domain);
        if (domain.peaks.size() < 2) {
            for (const Domain &group : projection_groups)
                for (const Peak &peak : group.peaks)
                    if (domain_peak_set.test(peak.index)) {
                        selected.push_back(group);
                        break;
                    }
            continue;
        }
        std::vector<Peak> ordered_peaks;
        for (const Peak &peak : peaks)
            if (domain_peak_set.test(peak.index))
                ordered_peaks.push_back(peak);
        if (domain.peaks.size() == 2) {
            const Peak &first = domain.peaks[0];
            const Peak &second = domain.peaks[1];
            const int row_distance = std::abs(
                first.index / kColumns - second.index / kColumns);
            const int column_distance = std::abs(
                first.index % kColumns - second.index % kColumns);
            const bool close_island_pair =
                std::max(row_distance, column_distance) <= 2 &&
                std::min(row_distance, column_distance) <= 1;
            if (close_island_pair) {
                selected.push_back(Domain{0, domain.nodes, ordered_peaks});
                continue;
            }
            if (domains.size() > 1 && peak_was_filtered) {
                bool complete = true;
                for (const Peak &peak : ordered_peaks)
                    complete &= group_by_peak.contains(peak.index);
                if (complete) {
                    for (const Peak &peak : ordered_peaks)
                        selected.push_back(group_by_peak[peak.index]);
                    continue;
                }
            }
            auto groups = peakMergeGroups(filtered, ordered_peaks);
            selected.insert(selected.end(), groups.begin(), groups.end());
            continue;
        }

        const auto plan = peakProjectionPlan(filtered, domain);
        std::vector<const MergeHistory *> history_candidates;
        for (const MergeHistory &history : previous_histories)
            if (blockTotal(history) == static_cast<int>(domain.peaks.size()) &&
                overlaps(history.nodes, domain.nodes) &&
                (!plan || history.method == plan->method))
                history_candidates.push_back(&history);
        std::optional<MergeHistory> active_history;
        if (history_candidates.size() == 1)
            active_history = *history_candidates[0];
        if (!active_history && plan &&
            !containsPeakSet(previous_domain_peak_sets, domain_peak_set)) {
            if (applyPreviousGroupLabels(domain, previous_groups))
                active_history = MergeHistory{
                    domain.nodes, plan->method,
                    std::vector<int>(domain.peaks.size(), 1)};
        }

        std::vector<Peak> spatial_peaks;
        std::vector<int> record_ranks;
        if (plan || active_history) {
            const int method = plan ? plan->method : active_history->method;
            spatial_peaks = domain.peaks;
            std::sort(spatial_peaks.begin(), spatial_peaks.end(),
                      [method](const Peak &a, const Peak &b) {
                          const int av = method == 2 ? a.index % kColumns
                                                     : a.index / kColumns;
                          const int bv = method == 2 ? b.index % kColumns
                                                     : b.index / kColumns;
                          return av < bv;
                      });
            for (const Peak &peak : spatial_peaks)
                record_ranks.push_back(peak_order[peak.index]);
        }
        bool ordered_history = !record_ranks.empty();
        for (size_t index = 1; index < record_ranks.size(); ++index)
            ordered_history &= record_ranks[index - 1] < record_ranks[index];
        bool merged_block = false;
        if (active_history)
            merged_block = std::any_of(active_history->block_sizes.begin(),
                                       active_history->block_sizes.end(),
                                       [](int size) { return size > 1; });
        const bool use_active_history = active_history &&
            (merged_block || domains.size() > 1 || ordered_history);
        if (use_active_history) {
            const Plan history_plan = peakHistoryPlan(
                filtered, spatial_peaks, active_history->method);
            const auto history_split = applyPeakPlan(domain, history_plan);
            std::map<int, Domain> history_group_by_peak = group_by_peak;
            if (history_split) {
                history_group_by_peak.clear();
                for (const Domain &group : *history_split)
                    history_group_by_peak[group.peaks[0].index] = group;
            }
            size_t offset = 0;
            std::vector<Domain> history_groups;
            bool complete = true;
            for (int size : active_history->block_sizes) {
                Domain combined;
                for (int index = 0; index < size; ++index) {
                    const Peak &peak = spatial_peaks[offset++];
                    if (!history_group_by_peak.contains(peak.index)) {
                        complete = false;
                        break;
                    }
                    const Domain &group = history_group_by_peak[peak.index];
                    combined.nodes |= group.nodes;
                    combined.peaks.insert(combined.peaks.end(),
                                          group.peaks.begin(), group.peaks.end());
                }
                if (!complete)
                    break;
                history_groups.push_back(std::move(combined));
            }
            if (complete && !history_groups.empty()) {
                selected.insert(selected.end(), history_groups.begin(),
                                history_groups.end());
                continue;
            }
        }
        if (ordered_history) {
            for (const Peak &peak : spatial_peaks)
                selected.push_back(group_by_peak[peak.index]);
            continue;
        }
        if (domains.size() == 1 && domain.peaks.size() >= 3 &&
            !record_ranks.empty() &&
            containsPeakSet(previous_domain_peak_sets, domain_peak_set)) {
            std::vector<std::pair<int, Domain>> runs;
            for (size_t index = 0; index < spatial_peaks.size(); ++index) {
                const Peak &peak = spatial_peaks[index];
                Domain group = group_by_peak[peak.index];
                const int rank = record_ranks[index];
                if (!runs.empty() && rank < runs.back().first) {
                    auto previous = std::move(runs.back());
                    runs.pop_back();
                    previous.second.nodes |= group.nodes;
                    previous.second.peaks.insert(previous.second.peaks.end(),
                                                 group.peaks.begin(), group.peaks.end());
                    previous.first = std::min(previous.first, rank);
                    runs.push_back(std::move(previous));
                } else {
                    runs.emplace_back(rank, std::move(group));
                }
            }
            for (auto &run : runs)
                selected.push_back(std::move(run.second));
            continue;
        }
        auto groups = peakMergeGroups(filtered, ordered_peaks);
        selected.insert(selected.end(), groups.begin(), groups.end());
    }
    std::sort(selected.begin(), selected.end(), [&](const Domain &a, const Domain &b) {
        auto rank = [&](const Domain &domain) {
            int result = std::numeric_limits<int>::max();
            for (const Peak &peak : domain.peaks)
                result = std::min(result, peak_order[peak.index]);
            return result;
        };
        return rank(a) < rank(b);
    });
    for (size_t index = 0; index < selected.size(); ++index)
        selected[index].label = static_cast<int>(index) + 1;
    return selected;
}

std::vector<MergeHistory> peakMergeHistories(
    const Matrix &filtered, const std::vector<Domain> &domains,
    const std::vector<Peak> &peaks, const std::vector<Domain> &groups,
    const std::vector<MergeHistory> &previous_histories,
    const std::vector<Domain> &previous_groups,
    const std::vector<NodeSet> &previous_domain_peak_sets) {
    std::map<int, int> peak_order;
    for (size_t index = 0; index < peaks.size(); ++index)
        peak_order[peaks[index].index] = static_cast<int>(index);
    std::vector<MergeHistory> histories;
    for (const Domain &domain : domains) {
        const auto plan = peakProjectionPlan(filtered, domain);
        std::vector<const MergeHistory *> candidates;
        for (const MergeHistory &history : previous_histories)
            if (blockTotal(history) == static_cast<int>(domain.peaks.size()) &&
                overlaps(history.nodes, domain.nodes) &&
                (!plan || history.method == plan->method))
                candidates.push_back(&history);
        int method = 0;
        if (plan)
            method = plan->method;
        else if (candidates.size() == 1)
            method = candidates[0]->method;
        else
            continue;
        const NodeSet domain_peaks = peakSet(domain);
        std::vector<Domain> domain_groups;
        int peak_count = 0;
        for (const Domain &group : groups) {
            bool belongs = false;
            for (const Peak &peak : group.peaks)
                belongs |= domain_peaks.test(peak.index);
            if (belongs) {
                peak_count += static_cast<int>(group.peaks.size());
                domain_groups.push_back(group);
            }
        }
        if (peak_count != static_cast<int>(domain.peaks.size()))
            continue;
        std::sort(domain_groups.begin(), domain_groups.end(),
                  [method](const Domain &a, const Domain &b) {
                      auto coordinate = [method](const Domain &domain) {
                          int result = std::numeric_limits<int>::max();
                          for (const Peak &peak : domain.peaks)
                              result = std::min(result, method == 2
                                  ? peak.index % kColumns : peak.index / kColumns);
                          return result;
                      };
                      return coordinate(a) < coordinate(b);
                  });
        const bool has_merged = std::any_of(domain_groups.begin(), domain_groups.end(),
                                            [](const Domain &group) {
                                                return group.peaks.size() > 1;
                                            });
        bool previous_match = false;
        for (const MergeHistory &history : previous_histories)
            previous_match |= history.method == method &&
                              blockTotal(history) == static_cast<int>(domain.peaks.size()) &&
                              overlaps(history.nodes, domain.nodes);
        if (!has_merged && !previous_match) {
            if (domain.peaks.size() < 3)
                continue;
            std::vector<int> ranks;
            for (const Domain &group : domain_groups)
                for (const Peak &peak : group.peaks)
                    ranks.push_back(peak_order[peak.index]);
            bool ordered = true;
            for (size_t index = 1; index < ranks.size(); ++index)
                ordered &= ranks[index - 1] < ranks[index];
            const bool recovered = applyPreviousGroupLabels(domain, previous_groups).has_value() &&
                !containsPeakSet(previous_domain_peak_sets, domain_peaks);
            if (!ordered && !recovered)
                continue;
        }
        MergeHistory history{domain.nodes, method, {}};
        for (const Domain &group : domain_groups)
            history.block_sizes.push_back(static_cast<int>(group.peaks.size()));
        histories.push_back(std::move(history));
    }
    return histories;
}

std::vector<Domain> coordinatePeakGroups(const std::vector<Domain> &groups,
                                         const Matrix &coordinate_delta,
                                         const std::vector<Peak> &peaks) {
    const auto coordinate_domains = connectedDomains(coordinate_delta, peaks, true);
    NodeSet labeled;
    for (const Domain &group : groups)
        labeled |= group.nodes;
    std::vector<NodeSet> additions(groups.size());
    for (const Domain &domain : coordinate_domains) {
        int owner = -1;
        int owner_count = 0;
        for (size_t index = 0; index < groups.size(); ++index)
            if (overlaps(groups[index].nodes, domain.nodes)) {
                owner = static_cast<int>(index);
                ++owner_count;
            }
        if (owner_count == 1)
            additions[owner] |= domain.nodes & ~labeled;
    }
    std::vector<Domain> result = groups;
    for (size_t index = 0; index < result.size(); ++index)
        result[index].nodes |= additions[index];
    return result;
}

std::vector<bool> coordinateFringeActive(const std::vector<Domain> &groups) {
    std::vector<Peak> representatives;
    for (const Domain &group : groups)
        if (!group.peaks.empty())
            representatives.push_back(*std::max_element(
                group.peaks.begin(), group.peaks.end(),
                [](const Peak &a, const Peak &b) { return a.value < b.value; }));
    std::vector<bool> active(representatives.size(), false);
    for (size_t index = 0; index < representatives.size(); ++index) {
        const int first_row = representatives[index].index / kColumns;
        const int first_column = representatives[index].index % kColumns;
        for (size_t other = 0; other < representatives.size(); ++other) {
            if (index == other)
                continue;
            const int row = representatives[other].index / kColumns;
            const int column = representatives[other].index % kColumns;
            if ((first_row - row) * (first_row - row) +
                    (first_column - column) * (first_column - column) > 49) {
                active[index] = true;
                break;
            }
        }
    }
    return active;
}

std::vector<Domain> coordinateFringeGroups(const std::vector<Domain> &groups,
                                           const Matrix &coordinate_delta,
                                           const std::vector<bool> &active) {
    if (groups.size() < 2)
        return groups;
    NodeSet labeled;
    for (const Domain &group : groups)
        labeled |= group.nodes;
    std::vector<Domain> result = groups;
    for (size_t group_index = 0; group_index < groups.size(); ++group_index) {
        if (!active[group_index])
            continue;
        for (int index = 0; index < kNodes; ++index) {
            if (labeled.test(index) || coordinate_delta[index] <= kCoordinateReduce)
                continue;
            bool adjacent = false;
            forNeighbors4(index, [&](int neighbor) {
                adjacent |= groups[group_index].nodes.test(neighbor);
            });
            if (adjacent)
                result[group_index].nodes.set(index);
        }
    }
    return result;
}

std::optional<Contact> domainContact(const Matrix &delta, const Domain &domain) {
    long long row_sum = 0;
    long long column_sum = 0;
    int denominator = 0;
    int peak = 0;
    for (const Peak &item : domain.peaks)
        peak = std::max(peak, item.value);
    for (int index = 0; index < kNodes; ++index) {
        if (!domain.nodes.test(index))
            continue;
        const int weight = std::max(delta[index] - kCoordinateReduce, 0);
        denominator += weight;
        row_sum += static_cast<long long>(weight) * kRowMap[index / kColumns];
        column_sum += static_cast<long long>(weight) * kColumnMap[index % kColumns];
    }
    if (denominator == 0)
        return std::nullopt;
    const int x = static_cast<int>(std::nearbyint(
        static_cast<double>(column_sum) * kSuperResolution / denominator));
    const int y = static_cast<int>(std::nearbyint(
        static_cast<double>(row_sum) * kSuperResolution / denominator));
    return Contact{domain.label, x, y, peak,
                   static_cast<int>(domain.nodes.count()), denominator,
                   domain.nodes};
}

int contactDistance(int first_x, int first_y, int second_x, int second_y) {
    const float dx = static_cast<float>(first_x - second_x);
    const float dy = static_cast<float>(first_y - second_y);
    const float square = static_cast<float>(static_cast<float>(dx * dx) +
                                            static_cast<float>(dy * dy));
    return static_cast<int>(static_cast<float>(std::sqrt(square)));
}

struct AssignmentResult {
    int cost = std::numeric_limits<int>::max();
    std::vector<int> selected;
};

std::vector<std::pair<int, int>> minimumDistanceAssignment(
    const std::vector<std::pair<int, int>> &predictions,
    const std::vector<std::pair<int, int>> &contacts) {
    if (predictions.empty() || contacts.empty())
        return {};
    const bool transpose = predictions.size() > contacts.size();
    const auto &rows = transpose ? contacts : predictions;
    const auto &columns = transpose ? predictions : contacts;
    std::vector<std::vector<int>> costs(rows.size(),
                                        std::vector<int>(columns.size()));
    for (size_t row = 0; row < rows.size(); ++row)
        for (size_t column = 0; column < columns.size(); ++column)
            costs[row][column] = contactDistance(
                rows[row].first, rows[row].second,
                columns[column].first, columns[column].second);
    std::map<std::pair<int, uint32_t>, AssignmentResult> memo;
    std::function<AssignmentResult(int, uint32_t)> solve =
        [&](int row, uint32_t used) -> AssignmentResult {
            if (row == static_cast<int>(rows.size()))
                return AssignmentResult{0, {}};
            const auto key = std::make_pair(row, used);
            if (memo.contains(key))
                return memo[key];
            AssignmentResult best;
            for (int column = 0; column < static_cast<int>(columns.size()); ++column) {
                if (used & (1U << column))
                    continue;
                AssignmentResult tail = solve(row + 1, used | (1U << column));
                AssignmentResult candidate;
                candidate.cost = costs[row][column] + tail.cost;
                candidate.selected.push_back(column);
                candidate.selected.insert(candidate.selected.end(),
                                          tail.selected.begin(), tail.selected.end());
                if (candidate.cost < best.cost ||
                    (candidate.cost == best.cost && candidate.selected < best.selected))
                    best = std::move(candidate);
            }
            memo[key] = best;
            return best;
        };
    const AssignmentResult result = solve(0, 0);
    std::vector<std::pair<int, int>> pairs;
    for (size_t row = 0; row < result.selected.size(); ++row)
        pairs.emplace_back(transpose ? result.selected[row] : static_cast<int>(row),
                           transpose ? static_cast<int>(row) : result.selected[row]);
    std::sort(pairs.begin(), pairs.end());
    return pairs;
}

}  // namespace

TouchCore::TouchCore() = default;

void TouchCore::reset() {
    reference_.reset();
    previous_plans_.clear();
    previous_groups_.clear();
    previous_domains_.clear();
    previous_domain_peak_sets_.clear();
    previous_histories_.clear();
    tracker_.fill(TrackerSlot{});
    next_tracking_id_ = 0;
    palm_retain_count_ = 0;
    palm_latched_ = false;
    previous_palm_domains_.clear();
    base_refresh_count_ = 0;
    last_counter_.reset();
}

void TouchCore::decayPalmNoPeaks(int matrix_maximum) {
    if (!palm_latched_)
        palm_retain_count_ = 0;
    else if (palm_retain_count_ < 1)
        palm_latched_ = false;
    else {
        const int previous = palm_retain_count_--;
        if (previous >= 7 && matrix_maximum <= 199)
            palm_retain_count_ >>= 1;
    }
    previous_palm_domains_.clear();
}

void TouchCore::processNoTouch(int matrix_maximum) {
    decayPalmNoPeaks(matrix_maximum);
}

bool TouchCore::updatePalm(const Matrix &delta,
                           const std::vector<Peak> &peaks,
                           const std::vector<Domain> &domains,
                           int matrix_maximum) {
    if (peaks.empty()) {
        decayPalmNoPeaks(matrix_maximum);
        return false;
    }

    bool detected = false;
    const int half_maximum = matrix_maximum / 2;
    const int required_nodes = matrix_maximum > 700 ? 52 : 62;
    for (const Peak &peak : peaks) {
        if (peak.value < half_maximum || peak.value < 500)
            continue;
        const int threshold = std::max(200, peak.value * 6 / 10);
        const int peak_row = peak.index / kColumns;
        const int peak_column = peak.index % kColumns;
        int count = 0;
        int minimum_column = peak_column;
        int maximum_column = peak_column;
        const auto scan_row = [&](int row) {
            int row_count = 0;
            for (int column = peak_column; column >= 0; --column) {
                const int value = delta[row * kColumns + column];
                if (value <= threshold || value > peak.value)
                    break;
                ++row_count;
                ++count;
                minimum_column = std::min(minimum_column, column);
                maximum_column = std::max(maximum_column, column);
            }
            for (int column = peak_column + 1; column < kColumns; ++column) {
                const int value = delta[row * kColumns + column];
                if (value <= threshold || value > peak.value)
                    break;
                ++row_count;
                ++count;
                minimum_column = std::min(minimum_column, column);
                maximum_column = std::max(maximum_column, column);
            }
            return row_count;
        };
        for (int row = peak_row; row >= 0; --row)
            if (scan_row(row) == 0)
                break;
        for (int row = peak_row + 1; row < kRows; ++row)
            if (scan_row(row) == 0)
                break;
        if (maximum_column - minimum_column > 3 && count >= required_nodes) {
            detected = true;
            break;
        }
    }

    int positive_area = 0;
    int negative_area = 0;
    for (int value : delta) {
        positive_area += value > 300;
        negative_area += value < -300;
    }
    const int total_area = positive_area + negative_area;
    if (positive_area > 110)
        detected |= total_area > 720 || negative_area > 30;
    else
        detected |= total_area > 720;

    std::vector<std::pair<int, int>> previous_coordinates;
    for (const Domain &domain : previous_domains_) {
        if (domain.peaks.empty())
            continue;
        const Peak &peak = *std::max_element(
            domain.peaks.begin(), domain.peaks.end(),
            [](const Peak &first, const Peak &second) {
                return first.value < second.value;
            });
        previous_coordinates.emplace_back(
            peak.index % kColumns, peak.index / kColumns);
    }

    std::vector<PalmDomain> current;
    const bool was_latched = palm_latched_;
    constexpr int stylus_adjustment = 5;
    for (const Domain &domain : domains) {
        std::vector<std::pair<int, int>> points;
        int maximum_column = 0;
        int maximum_row = 0;
        for (int index = 0; index < kNodes; ++index) {
            if (!domain.nodes.test(index))
                continue;
            const int column = index % kColumns;
            const int row = index / kColumns;
            points.emplace_back(column, row);
            maximum_column = std::max(maximum_column, column);
            maximum_row = std::max(maximum_row, row);
        }
        std::vector<std::pair<int, int>> working;
        for (auto point : points)
            if (point.first < maximum_column && point.second < maximum_row)
                working.push_back(point);
        double major_length = 0.0;
        double minor_width = 0.0;
        double shape_ratio = 1.0;
        if (working.size() >= 2) {
            auto first_endpoint = working[0];
            auto second_endpoint = working[0];
            for (auto first : working) {
                for (auto second : working) {
                    const double distance = std::hypot(
                        first.first - second.first,
                        first.second - second.second);
                    if (major_length <= distance) {
                        major_length = distance;
                        first_endpoint = first;
                        second_endpoint = second;
                    }
                }
            }
            const int center_column =
                (first_endpoint.first + second_endpoint.first) / 2;
            const int center_row =
                (first_endpoint.second + second_endpoint.second) / 2;
            const int row_difference =
                second_endpoint.second - first_endpoint.second;
            const double major_slope = row_difference
                ? static_cast<double>(first_endpoint.first - second_endpoint.first) /
                      row_difference
                : 0.0;
            const double perpendicular = major_slope ? -1.0 / major_slope : 0.0;
            const double line_scale = std::sqrt(perpendicular * perpendicular + 1.0);
            double minor_radius = 0.0;
            for (auto point : working) {
                const double line_distance = std::abs(
                    (point.first - perpendicular * point.second) -
                    (center_column - perpendicular * center_row)) / line_scale;
                if (line_distance < 0.5)
                    minor_radius = std::max(
                        minor_radius,
                        std::hypot(point.first - center_column,
                                   point.second - center_row));
            }
            minor_width = minor_radius * 2.0;
            const int area_minus_seed =
                std::max(static_cast<int>(domain.nodes.count()) - 1, 0);
            shape_ratio = major_length
                ? area_minus_seed / (major_length * major_length) : 1.0;
        }

        const int area_minus_seed =
            std::max(static_cast<int>(domain.nodes.count()) - 1, 0);
        int minimum_row = kRows;
        int minimum_column = kColumns;
        int maximum_domain_row = 0;
        int maximum_domain_column = 0;
        for (auto point : points) {
            minimum_column = std::min(minimum_column, point.first);
            maximum_domain_column = std::max(maximum_domain_column, point.first);
            minimum_row = std::min(minimum_row, point.second);
            maximum_domain_row = std::max(maximum_domain_row, point.second);
        }
        const int row_span = maximum_domain_row - minimum_row;
        const int column_span = maximum_domain_column - minimum_column;
        int coordinate_overlap = 0;
        const int expanded_minimum_row = std::max(minimum_row - 2, 0);
        const int expanded_maximum_row = std::min(maximum_domain_row + 2,
                                                   kRows - 1);
        const int expanded_minimum_column = std::max(minimum_column - 2, 0);
        const int expanded_maximum_column = std::min(
            maximum_domain_column + 2, kColumns - 1);
        for (auto [column, row] : previous_coordinates) {
            const int index = row * kColumns + column;
            if (domain.nodes.test(index) ||
                (expanded_minimum_column <= column &&
                 column <= expanded_maximum_column &&
                 expanded_minimum_row <= row && row <= expanded_maximum_row))
                ++coordinate_overlap;
        }
        int palm_count = 0;
        if (!was_latched && area_minus_seed > 9 && major_length > 7.0 &&
            minor_width > 3.9 && shape_ratio < 0.75) {
            const int overlap_threshold = area_minus_seed * 65 / 100;
            for (const PalmDomain &previous : previous_palm_domains_) {
                const int overlap = static_cast<int>(
                    (domain.nodes & previous.nodes).count());
                if (overlap < overlap_threshold)
                    continue;
                if (std::abs(major_length - previous.major_length) < 1.5 &&
                    previous.shape_ratio != 0.0)
                    palm_count = previous.palm_count + 1;
                break;
            }
        }
        current.push_back(PalmDomain{domain.nodes, major_length, minor_width,
                                     shape_ratio, palm_count});
        const bool broad_shape =
            ((row_span < 12 && column_span < 12) || matrix_maximum < 1101) &&
            coordinate_overlap < 2 && shape_ratio > 0.8 &&
            ((row_span > 7 && column_span > 7) ||
             (row_span > 6 && column_span > 6 && row_span + column_span > 14)) &&
            ((area_minus_seed >= 63 - stylus_adjustment &&
              matrix_maximum > 1100) ||
             (area_minus_seed >= 59 - stylus_adjustment &&
              matrix_maximum < 1101));
        bool filled_rectangle = false;
        if (coordinate_overlap < 2 && row_span > 8 && column_span > 8) {
            const int rectangle_area = (row_span + 1) * (column_span + 1);
            const int fill_ratio = rectangle_area
                ? area_minus_seed * 100 / rectangle_area : 0;
            filled_rectangle = fill_ratio <= 99 &&
                area_minus_seed > fill_ratio * 62 / 100 &&
                matrix_maximum <= 1049;
        }
        const bool persistent_shape = !was_latched &&
            ((area_minus_seed >= 48 - stylus_adjustment && palm_count > 1) ||
             area_minus_seed > 49 ||
             (shape_ratio > 0.5 &&
              area_minus_seed >= 53 - stylus_adjustment)) &&
            matrix_maximum < 950;
        if (coordinate_overlap < 2)
            detected |= broad_shape || filled_rectangle || persistent_shape;
    }
    if (detected) {
        palm_latched_ = true;
        palm_retain_count_ = 50;
    }
    const bool active = palm_latched_;
    previous_palm_domains_ = active ? std::vector<PalmDomain>{}
                                    : std::move(current);
    return active;
}

std::vector<Slot> TouchCore::updateTracker(
    const std::vector<Contact> &contacts,
    std::vector<TrackedSlot> *tracked_slots) {
    std::vector<int> active_numbers;
    std::vector<std::pair<int, int>> predictions;
    for (int number = 0; number < kFingerSlots; ++number) {
        if (!tracker_[number].active)
            continue;
        active_numbers.push_back(number);
        const TrackerSlot &slot = tracker_[number];
        const int distance = contactDistance(
            slot.contact.x, slot.contact.y,
            slot.previous_x, slot.previous_y);
        int x = slot.contact.x;
        int y = slot.contact.y;
        if (distance >= kSuperResolution * 50 && slot.age >= 4 &&
            active_numbers.size() <= 3) {
            auto floor_div_three = [](int value) {
                int quotient = value / 3;
                const int remainder = value % 3;
                if (remainder != 0 && value < 0)
                    --quotient;
                return quotient;
            };
            x += floor_div_three((slot.contact.x - slot.previous_x) * 2);
            y += floor_div_three((slot.contact.y - slot.previous_y) * 2);
        }
        predictions.emplace_back(x, y);
    }
    // The original gate disables prediction whenever the total active count
    // exceeds three. Recompute those entries after the complete count is known.
    if (active_numbers.size() > 3) {
        predictions.clear();
        for (int number : active_numbers)
            predictions.emplace_back(tracker_[number].contact.x,
                                     tracker_[number].contact.y);
    }
    std::vector<std::pair<int, int>> coordinates;
    for (const Contact &contact : contacts)
        coordinates.emplace_back(contact.x, contact.y);
    const auto assignments = minimumDistanceAssignment(predictions, coordinates);
    std::vector<bool> matched_contacts(contacts.size(), false);
    std::array<TrackerSlot, kFingerSlots> next{};
    for (auto [slot_index, contact_index] : assignments) {
        const int number = active_numbers[slot_index];
        const Contact &contact = contacts[contact_index];
        matched_contacts[contact_index] = true;
        if (contact.peak < kUpThreshold)
            continue;
        const TrackerSlot &previous = tracker_[number];
        next[number] = TrackerSlot{true, previous.tracking_id, contact,
                                   previous.contact.x, previous.contact.y,
                                   previous.age + 1};
    }
    std::vector<int> free_numbers;
    for (int number = 0; number < kFingerSlots; ++number)
        if (!tracker_[number].active)
            free_numbers.push_back(number);
    size_t free_index = 0;
    for (size_t contact_index = 0; contact_index < contacts.size(); ++contact_index) {
        const Contact &contact = contacts[contact_index];
        if (matched_contacts[contact_index] || contact.peak < kDownThreshold)
            continue;
        if (free_index >= free_numbers.size())
            break;
        const int number = free_numbers[free_index++];
        next[number] = TrackerSlot{true, next_tracking_id_++, contact,
                                   contact.x, contact.y, 1};
    }
    tracker_ = next;
    std::vector<Slot> visible;
    for (int number = 0; number < kFingerSlots; ++number) {
        if (!tracker_[number].active)
            continue;
        if (tracked_slots) {
            const TrackerSlot &slot = tracker_[number];
            tracked_slots->push_back(TrackedSlot{
                number, slot.tracking_id, slot.age <= 2 ? 1 : 2,
                slot.age, slot.contact});
        }
        if (tracker_[number].age >= 2)
            visible.push_back(Slot{number, tracker_[number].tracking_id,
                                   tracker_[number].contact});
    }
    return visible;
}

FrameResult TouchCore::process(const Matrix &matrix,
                                      std::optional<uint16_t> counter,
                                      uint8_t frame_type) {
    if (!reference_) {
        reference_ = matrix;
        last_counter_ = counter;
    }
    FrameResult result;
    int matrix_maximum = std::numeric_limits<int>::min();
    for (int index = 0; index < kNodes; ++index) {
        result.delta[index] = matrix[index] - (*reference_)[index];
        matrix_maximum = std::max(matrix_maximum, result.delta[index]);
    }
    const Matrix coordinate_delta =
        domainFrameHistoryFilter(result.delta, matrix_maximum);
    const Matrix coordinate_labels = matrix_maximum >= kPeakThreshold + 100
        ? coordinate_delta : projectionNoiseFilter(result.delta);
    const Matrix filtered = strictNoiseFilter(result.delta);
    result.search_peaks = localPeaks(result.delta);
    result.peaks = filterEqualAdjacentPeaks(result.delta, result.search_peaks);
    result.domains = connectedDomains(filtered, result.peaks);
    result.palm_active = updatePalm(
        result.delta, result.peaks, result.domains, matrix_maximum);
    const std::vector<Domain> empty_domains;
    const std::vector<Domain> &processing_domains = result.palm_active
        ? empty_domains : result.domains;
    const Matrix projection_filtered = projectionNoiseFilter(result.delta);
    std::vector<Plan> current_plans;
    for (const Domain &domain : processing_domains)
        if (auto plan = peakProjectionPlan(projection_filtered, domain))
            current_plans.push_back(std::move(*plan));
    std::vector<Domain> groups = projectionPeakGroups(
        projection_filtered, processing_domains,
        previous_plans_, previous_groups_);
    const bool has_multi_peak = std::any_of(
        processing_domains.begin(), processing_domains.end(),
        [](const Domain &domain) { return domain.peaks.size() > 1; });
    if (has_multi_peak)
        groups = applyPeakMergeHistory(
            projection_filtered, processing_domains, result.peaks, groups,
            previous_domain_peak_sets_, previous_histories_, previous_groups_,
            result.search_peaks.size() > result.peaks.size());
    std::vector<MergeHistory> current_histories = peakMergeHistories(
        projection_filtered, processing_domains, result.peaks, groups,
        previous_histories_, previous_groups_, previous_domain_peak_sets_);
    groups = coordinatePeakGroups(groups, coordinate_labels, result.peaks);
    const auto fringe_active = coordinateFringeActive(groups);
    if (std::any_of(fringe_active.begin(), fringe_active.end(),
                    [](bool value) { return value; }))
        groups = coordinateFringeGroups(groups, coordinate_delta, fringe_active);
    result.groups = groups;
    for (const Domain &domain : groups)
        if (auto contact = domainContact(coordinate_delta, domain))
            result.contacts.push_back(std::move(*contact));
    result.slots = updateTracker(result.contacts, &result.tracked_slots);

    if (result.palm_active) {
        previous_plans_.clear();
        previous_groups_.clear();
    } else if (!current_plans.empty()) {
        previous_plans_ = current_plans;
    } else if (result.peaks.size() < 2) {
        previous_plans_.clear();
    }
    previous_groups_ = groups;
    previous_domains_ = result.domains;
    previous_domain_peak_sets_.clear();
    for (const Domain &domain : processing_domains)
        previous_domain_peak_sets_.push_back(peakSet(domain));
    previous_histories_ = std::move(current_histories);

    const bool counter_advanced = !counter || !last_counter_ ||
                                  counter != last_counter_;
    if (frame_type == 4 && counter_advanced &&
        matrix_maximum <= kPeakThreshold) {
        if (base_refresh_count_ < kBaseRefreshDelay)
            ++base_refresh_count_;
        else {
            reference_ = matrix;
            base_refresh_count_ = 0;
        }
    } else {
        base_refresh_count_ = 0;
    }
    last_counter_ = counter;
    return result;
}

}  // namespace nvt
