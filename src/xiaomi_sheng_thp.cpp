// SPDX-License-Identifier: Apache-2.0

#include "nvt_touch_core.hpp"
#include "nvt_finger_filter.hpp"
#include "nvt_stylus.hpp"
#include "nvt_focus_pen_pressure.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/input.h>
#include <linux/uinput.h>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

constexpr const char *kControlPath = "/proc/nvt_thp_raw";
constexpr const char *kStreamPath = "/proc/nvt_thp_stream";
constexpr const char *kStylusPath = "/proc/nvt_thp_stylus";
constexpr uint32_t kStreamMagic = 0x3150544e;
constexpr size_t kTransportLength = 257;
constexpr size_t kMatrixOffset = kTransportLength + 0x40;
constexpr int kMaxX = 30479;
constexpr int kMaxY = 20319;
constexpr int kPenMaxX = 30479;
constexpr int kPenMaxY = 20319;
constexpr int kPenPressureMax = 8191;
constexpr size_t kStartupReferenceFrames = 72;
constexpr auto kStreamStallTimeout = std::chrono::milliseconds(100);
constexpr std::string_view kFocusPenName = "Xiaomi Focus Pen";

std::atomic<bool> running = true;

#pragma pack(push, 1)
struct StreamHeader {
    uint32_t magic;
    uint16_t header_length;
    uint16_t frame_length;
    uint64_t sequence;
    uint64_t timestamp_ns;
    uint16_t reserved;
    uint16_t flags;
    uint32_t checksum;
};
#pragma pack(pop)

static_assert(sizeof(StreamHeader) == 32);

void signalHandler(int) {
    running = false;
}

uint16_t readLe16(const uint8_t *data) {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(data[1]) << 8;
}

int16_t readLeI16(const uint8_t *data) {
    return static_cast<int16_t>(readLe16(data));
}

void writeControl(const char *path, int value) {
    const int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0)
        throw std::runtime_error(std::string("open ") + path + ": " +
                                 std::strerror(errno));
    const std::string text = std::to_string(value) + "\n";
    const ssize_t written = write(fd, text.data(), text.size());
    const int saved_errno = errno;
    close(fd);
    if (written != static_cast<ssize_t>(text.size()))
        throw std::runtime_error(std::string("write ") + path + ": " +
                                 std::strerror(saved_errno));
}

void setupAxis(int fd, unsigned code, int minimum, int maximum,
               int resolution = 0) {
    uinput_abs_setup setup{};
    setup.code = code;
    setup.absinfo.minimum = minimum;
    setup.absinfo.maximum = maximum;
    setup.absinfo.resolution = resolution;
    if (ioctl(fd, UI_ABS_SETUP, &setup) < 0)
        throw std::runtime_error("UI_ABS_SETUP failed");
}

class UInputTouch {
public:
    UInputTouch() {
        active_.fill(-1);
        fd_ = open("/dev/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0)
            throw std::runtime_error(std::string("open /dev/uinput: ") +
                                     std::strerror(errno));
        for (unsigned type : {EV_KEY, EV_ABS})
            checkedIoctl(UI_SET_EVBIT, type);
        checkedIoctl(UI_SET_KEYBIT, BTN_TOUCH);
        for (unsigned axis : {ABS_MT_SLOT, ABS_MT_TOUCH_MAJOR,
                              ABS_MT_POSITION_X, ABS_MT_POSITION_Y,
                              ABS_MT_TOOL_TYPE, ABS_MT_TRACKING_ID,
                              ABS_MT_PRESSURE})
            checkedIoctl(UI_SET_ABSBIT, axis);
        checkedIoctl(UI_SET_PROPBIT, INPUT_PROP_DIRECT);

        uinput_setup setup{};
        setup.id.bustype = BUS_VIRTUAL;
        setup.id.vendor = 0x2717;
        setup.id.product = 0x3653;
        setup.id.version = 1;
        std::strncpy(setup.name, "NVTCapacitiveTouchScreen",
                     sizeof(setup.name) - 1);
        if (ioctl(fd_, UI_DEV_SETUP, &setup) < 0)
            throw std::runtime_error("UI_DEV_SETUP failed");
        setupAxis(fd_, ABS_MT_SLOT, 0, nvt::kFingerSlots - 1);
        setupAxis(fd_, ABS_MT_TOUCH_MAJOR, 0, 256);
        setupAxis(fd_, ABS_MT_POSITION_X, 0, kMaxX, 113);
        setupAxis(fd_, ABS_MT_POSITION_Y, 0, kMaxY, 113);
        setupAxis(fd_, ABS_MT_TOOL_TYPE, 0, MT_TOOL_PALM);
        setupAxis(fd_, ABS_MT_TRACKING_ID, 0, 65535);
        setupAxis(fd_, ABS_MT_PRESSURE, 0, 1023);
        if (ioctl(fd_, UI_DEV_CREATE) < 0)
            throw std::runtime_error("UI_DEV_CREATE failed");
        usleep(100000);
    }

    ~UInputTouch() {
        if (fd_ < 0)
            return;
        try {
            report({});
        } catch (...) {
        }
        ioctl(fd_, UI_DEV_DESTROY);
        close(fd_);
    }

    void report(const std::vector<nvt::Slot> &slots) {
        std::array<const nvt::Slot *, nvt::kFingerSlots> visible{};
        for (const nvt::Slot &slot : slots)
            visible[slot.number] = &slot;
        for (int number = 0; number < nvt::kFingerSlots; ++number) {
            if (active_[number] >= 0 && visible[number] == nullptr) {
                event(EV_ABS, ABS_MT_SLOT, number);
                event(EV_ABS, ABS_MT_TRACKING_ID, -1);
                active_[number] = -1;
            }
        }
        for (int number = 0; number < nvt::kFingerSlots; ++number) {
            const nvt::Slot *slot = visible[number];
            if (!slot)
                continue;
            event(EV_ABS, ABS_MT_SLOT, number);
            if (active_[number] != slot->tracking_id) {
                event(EV_ABS, ABS_MT_TRACKING_ID, slot->tracking_id);
                active_[number] = slot->tracking_id;
            }
            const nvt::Contact &contact = slot->contact;
            event(EV_ABS, ABS_MT_POSITION_X, contact.x);
            event(EV_ABS, ABS_MT_POSITION_Y, kMaxY - contact.y);
            event(EV_ABS, ABS_MT_TOOL_TYPE, MT_TOOL_FINGER);
            event(EV_ABS, ABS_MT_TOUCH_MAJOR,
                  std::min(255, contact.area * 12));
            event(EV_ABS, ABS_MT_PRESSURE, std::min(1023, contact.peak));
        }
        event(EV_KEY, BTN_TOUCH, slots.empty() ? 0 : 1);
        event(EV_SYN, SYN_REPORT, 0);
    }

private:
    int fd_ = -1;
    std::array<int, nvt::kFingerSlots> active_{};

    void checkedIoctl(unsigned long request, unsigned long value) {
        if (ioctl(fd_, request, value) < 0)
            throw std::runtime_error("uinput capability ioctl failed");
    }

    void event(uint16_t type, uint16_t code, int32_t value) {
        input_event input{};
        input.type = type;
        input.code = code;
        input.value = value;
        if (write(fd_, &input, sizeof(input)) != sizeof(input))
            throw std::runtime_error("uinput event write failed");
    }
};

struct PenState {
    bool active = false;
    bool contact = false;
    int x = 0;
    int y = 0;
    int pressure = 0;
    int tilt_x = 0;
    int tilt_y = 0;
};

class UInputPen {
public:
    UInputPen() {
        fd_ = open("/dev/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0)
            throw std::runtime_error(std::string("open /dev/uinput: ") +
                                     std::strerror(errno));
        for (unsigned type : {EV_KEY, EV_ABS})
            checkedIoctl(UI_SET_EVBIT, type);
        for (unsigned key : {BTN_TOOL_PEN, BTN_TOUCH, BTN_STYLUS,
                             BTN_STYLUS2})
            checkedIoctl(UI_SET_KEYBIT, key);
        for (unsigned axis : {ABS_X, ABS_Y, ABS_PRESSURE, ABS_DISTANCE,
                              ABS_TILT_X, ABS_TILT_Y})
            checkedIoctl(UI_SET_ABSBIT, axis);
        checkedIoctl(UI_SET_PROPBIT, INPUT_PROP_DIRECT);

        uinput_setup setup{};
        setup.id.bustype = BUS_VIRTUAL;
        setup.id.vendor = 0x2717;
        setup.id.product = 0x3654;
        setup.id.version = 1;
        std::strncpy(setup.name, "NVTCapacitivePenM80p",
                     sizeof(setup.name) - 1);
        if (ioctl(fd_, UI_DEV_SETUP, &setup) < 0)
            throw std::runtime_error("UI_DEV_SETUP failed");
        setupAxis(fd_, ABS_X, 0, kPenMaxX, 113);
        setupAxis(fd_, ABS_Y, 0, kPenMaxY, 113);
        setupAxis(fd_, ABS_PRESSURE, 0, kPenPressureMax);
        setupAxis(fd_, ABS_DISTANCE, 0, 1);
        setupAxis(fd_, ABS_TILT_X, -60, 60);
        setupAxis(fd_, ABS_TILT_Y, -60, 60);
        if (ioctl(fd_, UI_DEV_CREATE) < 0)
            throw std::runtime_error("UI_DEV_CREATE failed");
        usleep(100000);
    }

    ~UInputPen() {
        if (fd_ < 0)
            return;
        try {
            report({});
        } catch (...) {
        }
        ioctl(fd_, UI_DEV_DESTROY);
        close(fd_);
    }

    void report(const PenState &state) {
        const bool contact = state.active && state.contact;
        event(EV_KEY, BTN_TOOL_PEN, state.active);
        event(EV_KEY, BTN_TOUCH, contact);
        event(EV_KEY, BTN_STYLUS, 0);
        event(EV_KEY, BTN_STYLUS2, 0);
        if (state.active) {
            event(EV_ABS, ABS_X, state.x);
            event(EV_ABS, ABS_Y, state.y);
            event(EV_ABS, ABS_PRESSURE, contact ? state.pressure : 0);
            event(EV_ABS, ABS_DISTANCE, contact ? 0 : 1);
        } else {
            event(EV_ABS, ABS_PRESSURE, 0);
            event(EV_ABS, ABS_DISTANCE, 0);
        }
        event(EV_ABS, ABS_TILT_X, state.active ? state.tilt_x : 0);
        event(EV_ABS, ABS_TILT_Y, state.active ? state.tilt_y : 0);
        event(EV_SYN, SYN_REPORT, 0);
    }

private:
    int fd_ = -1;

    void checkedIoctl(unsigned long request, unsigned long value) {
        if (ioctl(fd_, request, value) < 0)
            throw std::runtime_error("uinput capability ioctl failed");
    }

    void event(uint16_t type, uint16_t code, int32_t value) {
        input_event input{};
        input.type = type;
        input.code = code;
        input.value = value;
        if (write(fd_, &input, sizeof(input)) != sizeof(input))
            throw std::runtime_error("uinput event write failed");
    }
};

std::string readFirstLine(const std::filesystem::path &path) {
    std::ifstream input(path);
    std::string line;
    std::getline(input, line);
    return line;
}

bool ueventHasName(const std::filesystem::path &path,
                   std::string_view expected_name) {
    std::ifstream input(path);
    std::string line;
    const std::string expected = "HID_NAME=" + std::string(expected_name);
    while (std::getline(input, line)) {
        if (line == expected)
            return true;
    }
    return false;
}

class FocusPenHidReader {
public:
    ~FocusPenHidReader() {
        closeHidraw();
        closeEvent();
    }

    void service(nvt::FocusPenPressureQueue &queue) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_scan_) {
            next_scan_ = now + std::chrono::seconds(1);
            if (hidraw_fd_ < 0)
                openHidraw();
            if (event_fd_ < 0)
                grabReportEvent();
        }
        drainHidraw(queue);
        drainEvent();
    }

private:
    int hidraw_fd_ = -1;
    int event_fd_ = -1;
    std::chrono::steady_clock::time_point next_scan_{};

    void openHidraw() {
        std::error_code error;
        std::vector<std::filesystem::path> entries;
        for (const auto &entry : std::filesystem::directory_iterator(
                 "/sys/class/hidraw", error))
            entries.push_back(entry.path());
        std::sort(entries.begin(), entries.end());
        for (const auto &entry : entries) {
            if (!ueventHasName(entry / "device/uevent", kFocusPenName))
                continue;
            const std::string path = "/dev/" + entry.filename().string();
            const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
            if (fd < 0)
                continue;
            hidraw_fd_ = fd;
            std::cerr << "pen pressure transport ready (" << path
                      << ", HID report 5)\n";
            return;
        }
    }

    void grabReportEvent() {
        std::error_code error;
        std::vector<std::filesystem::path> entries;
        for (const auto &entry : std::filesystem::directory_iterator(
                 "/sys/class/input", error)) {
            if (entry.path().filename().string().starts_with("event"))
                entries.push_back(entry.path());
        }
        std::sort(entries.begin(), entries.end());
        for (const auto &entry : entries) {
            if (readFirstLine(entry / "device/name") != kFocusPenName)
                continue;
            const std::string path = "/dev/input/" + entry.filename().string();
            const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
            if (fd < 0)
                continue;
            if (ioctl(fd, EVIOCGRAB, 1) < 0) {
                close(fd);
                continue;
            }
            event_fd_ = fd;
            std::cerr << "claimed Focus Pen report-5 evdev transport ("
                      << path << "); keypad-plus events suppressed\n";
            return;
        }
    }

    void drainHidraw(nvt::FocusPenPressureQueue &queue) {
        if (hidraw_fd_ < 0)
            return;
        std::array<uint8_t, 64> report{};
        while (true) {
            const ssize_t size = read(hidraw_fd_, report.data(), report.size());
            if (size > 0) {
                queue.pushReport(std::span(report.data(),
                                           static_cast<size_t>(size)));
                continue;
            }
            if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                return;
            closeHidraw();
            return;
        }
    }

    void drainEvent() {
        if (event_fd_ < 0)
            return;
        std::array<input_event, 32> events{};
        while (true) {
            const ssize_t size = read(event_fd_, events.data(), sizeof(events));
            if (size > 0)
                continue;
            if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                return;
            closeEvent();
            return;
        }
    }

    void closeHidraw() {
        if (hidraw_fd_ >= 0)
            close(hidraw_fd_);
        hidraw_fd_ = -1;
    }

    void closeEvent() {
        if (event_fd_ >= 0) {
            ioctl(event_fd_, EVIOCGRAB, 0);
            close(event_fd_);
        }
        event_fd_ = -1;
    }
};

class LiveTouchAdapter {
public:
    struct Update {
        std::optional<nvt::FrameResult> result;
        bool baseline_ready = false;
    };

    Update feed(const nvt::Matrix &matrix, uint16_t counter,
                uint8_t frame_type, uint64_t timestamp_ns) {
        if (frame_type == 2) {
            core_.reset();
            filter_.reset();
            core_.process(matrix, counter, frame_type);
            reference_samples_.clear();
            return {std::nullopt, true};
        }
        if (frame_type != 4)
            return {};
        if (!core_.hasReference()) {
            reference_samples_.push_back(matrix);
            if (reference_samples_.size() < kStartupReferenceFrames)
                return {};
            nvt::Matrix reference{};
            std::array<int, kStartupReferenceFrames> values{};
            for (int node = 0; node < nvt::kNodes; ++node) {
                for (size_t sample = 0; sample < reference_samples_.size(); ++sample)
                    values[sample] = reference_samples_[sample][node];
                std::sort(values.begin(), values.end());
                reference[node] = (values[kStartupReferenceFrames / 2 - 1] +
                                   values[kStartupReferenceFrames / 2]) / 2;
            }
            core_.reset();
            filter_.reset();
            core_.process(reference, counter, 2);
            reference_samples_.clear();
            return {std::nullopt, true};
        }
        nvt::FrameResult result = core_.process(matrix, counter, frame_type);
        std::array<const nvt::TrackedSlot *, nvt::kFingerSlots> tracked{};
        for (const nvt::TrackedSlot &slot : result.tracked_slots)
            tracked[slot.number] = &slot;
        std::vector<nvt::Slot> visible;
        for (int number = 0; number < nvt::kFingerSlots; ++number) {
            const nvt::TrackedSlot *slot = tracked[number];
            if (!slot) {
                filter_.processPipeline(number, 0, {}, {}, timestamp_ns);
                continue;
            }
            const nvt::FingerCoordinate filter_coordinate{
                slot->contact.y, slot->contact.x};
            const auto filtered = filter_.processPipeline(
                number, slot->status, filter_coordinate,
                filter_coordinate, timestamp_ns, slot->age == 1);
            if (slot->age < 2)
                continue;
            nvt::Contact contact = slot->contact;
            // The clean core uses Linux's pre-inversion axis order, while
            // the filter smoothing state stores the two axes transposed.
            contact.x = filtered.coordinate.y;
            contact.y = filtered.coordinate.x;
            visible.push_back(nvt::Slot{
                number, slot->tracking_id, std::move(contact)});
        }
        result.slots = std::move(visible);
        return {std::move(result), false};
    }

private:
    nvt::TouchCore core_;
    nvt::FingerFilter filter_;
    std::vector<nvt::Matrix> reference_samples_;
};

class StreamReader {
public:
    explicit StreamReader(int fd) : fd_(fd) {
        buffer_.reserve(256 * 1024);
    }

    template <typename Function>
    void readAvailable(Function function) {
        std::array<uint8_t, 256 * 1024> input{};
        while (true) {
            const ssize_t size = read(fd_, input.data(), input.size());
            if (size > 0) {
                buffer_.insert(buffer_.end(), input.begin(), input.begin() + size);
                continue;
            }
            if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                break;
            if (size == 0)
                break;
            throw std::runtime_error(std::string("stream read: ") +
                                     std::strerror(errno));
        }
        size_t offset = 0;
        while (buffer_.size() - offset >= sizeof(StreamHeader)) {
            StreamHeader header{};
            std::memcpy(&header, buffer_.data() + offset, sizeof(header));
            if (header.magic != kStreamMagic ||
                header.header_length != sizeof(StreamHeader))
                throw std::runtime_error("lost THP stream framing");
            const size_t record_length = header.header_length + header.frame_length;
            if (buffer_.size() - offset < record_length)
                break;
            if (header.flags & 1)
                function(header.timestamp_ns,
                         buffer_.data() + offset + header.header_length,
                         header.frame_length);
            offset += record_length;
        }
        if (offset) {
            if (offset == buffer_.size())
                buffer_.clear();
            else
                buffer_.erase(buffer_.begin(), buffer_.begin() + offset);
        }
    }

private:
    int fd_;
    std::vector<uint8_t> buffer_;
};

}  // namespace

int main() try {
    if (geteuid() != 0)
        throw std::runtime_error("run as root to access THP and uinput nodes");

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    const int stream_fd = open(kStreamPath, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (stream_fd < 0)
        throw std::runtime_error(std::string("open stream: ") +
                                 std::strerror(errno));
    std::optional<UInputTouch> touch;
    std::optional<UInputPen> pen;
    try {
        writeControl(kStylusPath, 1);
        writeControl(kControlPath, 1);
        touch.emplace();
        pen.emplace();
        std::cerr << "touch and pen pipelines ready\n";
        std::cerr << "waiting for a touch reference frame\n";
        LiveTouchAdapter adapter;
        nvt::StylusDecoder pen_decoder;
        nvt::FocusPenPressureQueue pen_pressure;
        FocusPenHidReader pen_transport;
        bool touch_active = false;
        bool pen_active = false;
        bool have_valid_frame = false;
        bool stream_stalled = false;
        auto last_valid_frame = std::chrono::steady_clock::now();
        StreamReader reader(stream_fd);
        while (running) {
            pen_transport.service(pen_pressure);
            pollfd descriptor{stream_fd, POLLIN, 0};
            const int status = poll(&descriptor, 1, 100);
            if (status < 0) {
                if (errno == EINTR)
                    continue;
                throw std::runtime_error(std::string("poll: ") +
                                         std::strerror(errno));
            }
            if (status > 0) {
                pen_transport.service(pen_pressure);
                reader.readAvailable([&](uint64_t timestamp,
                                         const uint8_t *frame,
                                         size_t frame_length) {
                    if (frame_length <
                        kMatrixOffset + nvt::kNodes * sizeof(int16_t))
                        throw std::runtime_error("short THP frame");
                    last_valid_frame = std::chrono::steady_clock::now();
                    have_valid_frame = true;
                    if (stream_stalled) {
                        stream_stalled = false;
                        std::cerr << "THP stream recovered\n";
                    }
                    const uint8_t data_type =
                        frame[kTransportLength + 0x38];
                    if (data_type == 0x1d) {
                        const int pressure = pen_pressure.consume();
                        nvt::RawStylusFrame raw_stylus;
                        if (!nvt::parseRawStylusFrame(
                                frame, frame_length, raw_stylus))
                            throw std::runtime_error("invalid stylus frame");
                        const auto result = pen_decoder.process(raw_stylus);
                        PenState state;
                        if (result.active) {
                            state.active = true;
                            state.contact = pressure > 0;
                            state.pressure = pressure;
                            state.x = std::clamp(
                                result.coordinates.tip_y, 0, kPenMaxX);
                            state.y = kPenMaxY - std::clamp(
                                result.coordinates.tip_x, 0, kPenMaxY);
                            state.tilt_x = result.coordinates.tilt_x;
                            state.tilt_y = -result.coordinates.tilt_y;
                        }
                        if (pen && (result.active || pen_active))
                            pen->report(state);
                        pen_active = result.active;
                        return;
                    }
                    const uint8_t frame_type =
                        frame[kTransportLength + 24];
                    const uint16_t counter =
                        readLe16(frame + kTransportLength + 2);
                    nvt::Matrix matrix{};
                    for (int node = 0; node < nvt::kNodes; ++node)
                        matrix[node] = readLeI16(
                            frame + kMatrixOffset + node * 2);
                    auto update = adapter.feed(
                        matrix, counter, frame_type, timestamp);
                    if (update.baseline_ready) {
                        if (touch)
                            touch->report({});
                        if (pen)
                            pen->report({});
                        touch_active = false;
                        pen_active = false;
                        std::cerr << "touch reference ready\n";
                        return;
                    }
                    if (!update.result)
                        return;
                    touch_active = !update.result->slots.empty();
                    if (touch)
                        touch->report(update.result->slots);
                });
            }
            if (have_valid_frame && (touch_active || pen_active) &&
                !stream_stalled &&
                std::chrono::steady_clock::now() - last_valid_frame >=
                    kStreamStallTimeout) {
                if (touch)
                    touch->report({});
                if (pen)
                    pen->report({});
                touch_active = false;
                pen_active = false;
                stream_stalled = true;
                std::cerr << "THP stream stalled; released active inputs\n";
            }
        }
    } catch (...) {
        try { writeControl(kStylusPath, 0); } catch (...) {}
        try { writeControl(kControlPath, 0); } catch (...) {}
        close(stream_fd);
        throw;
    }
    pen.reset();
    touch.reset();
    writeControl(kStylusPath, 0);
    writeControl(kControlPath, 0);
    close(stream_fd);
    return 0;
} catch (const std::exception &error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
}
