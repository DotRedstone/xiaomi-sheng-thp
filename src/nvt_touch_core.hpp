// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <bitset>
#include <cstdint>
#include <optional>
#include <vector>

namespace nvt {

constexpr int kRows = 40;
constexpr int kColumns = 60;
constexpr int kNodes = kRows * kColumns;
constexpr int kSuperResolution = 10;
constexpr int kFingerSlots = 12;

using Matrix = std::array<int, kNodes>;
using NodeSet = std::bitset<kNodes>;

struct Peak {
    int index = 0;
    int value = 0;
};

struct Domain {
    int label = 0;
    NodeSet nodes;
    std::vector<Peak> peaks;
};

struct Contact {
    int label = 0;
    int x = 0;
    int y = 0;
    int peak = 0;
    int area = 0;
    int weight = 0;
    NodeSet nodes;

    bool operator==(const Contact &) const = default;
};

struct Slot {
    int number = 0;
    int tracking_id = 0;
    Contact contact;

    bool operator==(const Slot &) const = default;
};

struct TrackedSlot {
    int number = 0;
    int tracking_id = 0;
    int status = 0;
    int age = 0;
    Contact contact;
};

struct FrameResult {
    Matrix delta{};
    std::vector<Peak> search_peaks;
    std::vector<Peak> peaks;
    std::vector<Domain> domains;
    std::vector<Domain> groups;
    std::vector<Contact> contacts;
    std::vector<TrackedSlot> tracked_slots;
    std::vector<Slot> slots;
    bool palm_active = false;
};

class TouchCore {
public:
    TouchCore();

    void reset();
    void processNoTouch(int matrix_maximum = 0);
    FrameResult process(const Matrix &matrix,
                        std::optional<uint16_t> counter = std::nullopt,
                        uint8_t frame_type = 4);
    bool hasReference() const { return reference_.has_value(); }
    const Matrix &reference() const { return *reference_; }

    struct Plan {
        int method = 0;
        std::vector<int> cuts;
    };

    struct MergeHistory {
        NodeSet nodes;
        int method = 0;
        std::vector<int> block_sizes;
    };

    struct PalmDomain {
        NodeSet nodes;
        double major_length = 0;
        double minor_width = 0;
        double shape_ratio = 0;
        int palm_count = 0;
    };

    struct TrackerSlot {
        bool active = false;
        int tracking_id = 0;
        Contact contact;
        int previous_x = 0;
        int previous_y = 0;
        int age = 0;
    };

private:

    std::optional<Matrix> reference_;
    std::vector<Plan> previous_plans_;
    std::vector<Domain> previous_groups_;
    std::vector<Domain> previous_domains_;
    std::vector<NodeSet> previous_domain_peak_sets_;
    std::vector<MergeHistory> previous_histories_;
    std::array<TrackerSlot, kFingerSlots> tracker_{};
    int next_tracking_id_ = 0;
    int palm_retain_count_ = 0;
    bool palm_latched_ = false;
    std::vector<PalmDomain> previous_palm_domains_;
    int base_refresh_count_ = 0;
    std::optional<uint16_t> last_counter_;

    void decayPalmNoPeaks(int matrix_maximum);
    bool updatePalm(const Matrix &delta, const std::vector<Peak> &peaks,
                    const std::vector<Domain> &domains,
                    int matrix_maximum);
    std::vector<Slot> updateTracker(
        const std::vector<Contact> &contacts,
        std::vector<TrackedSlot> *tracked_slots = nullptr);
};

}  // namespace nvt
