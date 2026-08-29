// SPDX-License-Identifier: Apache-2.0

#include "nvt_touch_core.hpp"
#include "nvt_finger_filter.hpp"
#include "nvt_stylus.hpp"
#include "nvt_focus_pen_pressure.hpp"
#include "nvt_p81c_control.hpp"
#include "nvt_pencil_posture.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <grp.h>
#include <iostream>
#include <glib.h>
extern "C" {
#include <libssc-sensor-accelerometer.h>
}
#include <linux/input.h>
#include <linux/uinput.h>
#include <optional>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

constexpr const char *kControlPath = "/proc/nvt_thp_raw";
constexpr const char *kStreamPath = "/proc/nvt_thp_stream";
constexpr const char *kStylusPath = "/proc/nvt_thp_stylus";
constexpr const char *kButtonMappingSocket =
    "/run/xiaomi-sheng-thp/button-mapping.sock";
constexpr uint32_t kStreamMagic = 0x3150544e;
constexpr uint16_t kStreamFlagValid = 1U << 0;
constexpr uint16_t kStreamFlagEpoch = 1U << 1;
constexpr size_t kTransportLength = 257;
constexpr size_t kMatrixOffset = kTransportLength + 0x40;
constexpr int kMaxX = 30479;
constexpr int kMaxY = 20319;
constexpr int kPenMaxX = 30479;
constexpr int kPenMaxY = 20319;
// Both pens share these pre-display-rotation axes; only P81c posture fusion
// needs their maxima explicitly.
constexpr int kPostureRawPenMaxX = 20319;
constexpr int kPostureRawPenMaxY = 30479;
constexpr size_t kStartupReferenceFrames = 72;
constexpr auto kStreamStallTimeout = std::chrono::milliseconds(100);
constexpr auto kAirPointerActivityTimeout = std::chrono::seconds(2);
constexpr std::string_view kFocusPenName = "Xiaomi Focus Pen";
constexpr std::string_view kFocusPenMouseName = "Xiaomi Focus Pen Mouse";
constexpr std::string_view kFocusPenKeyboardName =
    "Xiaomi Focus Pen Keyboard";
constexpr std::string_view kFocusPenProName = "Xiaomi Focus Pen Pro";
constexpr std::string_view kFocusPenProKeyboardName =
    "Xiaomi Focus Pen Pro Keyboard";

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

nvt::Matrix readTouchMatrix(const uint8_t *frame) {
    nvt::Matrix matrix{};
    for (int node = 0; node < nvt::kNodes; ++node)
        matrix[node] = readLeI16(frame + kMatrixOffset + node * 2);
    return matrix;
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

void writeInputEvents(int fd, const input_event *events, size_t count) {
    const uint8_t *data = reinterpret_cast<const uint8_t *>(events);
    size_t remaining = count * sizeof(*events);
    while (remaining > 0) {
        const ssize_t written = write(fd, data, remaining);
        if (written < 0 && errno == EINTR)
            continue;
        if (written <= 0 ||
            written % static_cast<ssize_t>(sizeof(*events)) != 0)
            throw std::runtime_error("uinput event write failed");
        data += written;
        remaining -= static_cast<size_t>(written);
    }
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
        checkedIoctl(UI_SET_KEYBIT, BTN_TOOL_FINGER);
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
        setupAxis(fd_, ABS_MT_PRESSURE, 0, 1000);
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
        std::array<input_event, nvt::kFingerSlots * 7 + 3> events;
        size_t event_count = 0;
        auto event = [&](uint16_t type, uint16_t code, int32_t value) {
            input_event &input = events[event_count++];
            input = {};
            input.type = type;
            input.code = code;
            input.value = value;
        };
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
                  std::min(255, contact.area));
            event(EV_ABS, ABS_MT_PRESSURE, 1);
        }
        event(EV_KEY, BTN_TOUCH, slots.empty() ? 0 : 1);
        event(EV_KEY, BTN_TOOL_FINGER, slots.empty() ? 0 : 1);
        event(EV_SYN, SYN_REPORT, 0);
        writeInputEvents(fd_, events.data(), event_count);
    }

private:
    int fd_ = -1;
    std::array<int, nvt::kFingerSlots> active_{};

    void checkedIoctl(unsigned long request, unsigned long value) {
        if (ioctl(fd_, request, value) < 0)
            throw std::runtime_error("uinput capability ioctl failed");
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

struct PenButtons {
    bool button1 = false;
    bool button2 = false;

    bool operator==(const PenButtons &) const = default;
};

enum class ButtonAction {
    Native,
    LeftClick,
    RightClick,
    MiddleClick,
    Back,
    Forward,
    Undo,
    Redo,
    Screenshot,
    Overview,
    Disabled,
};

struct ButtonContextMapping {
    ButtonAction primary = ButtonAction::Native;
    ButtonAction secondary = ButtonAction::Native;

    bool operator==(const ButtonContextMapping &) const = default;
};

struct ButtonMapping {
    ButtonContextMapping pen{};
    ButtonContextMapping air{
        ButtonAction::LeftClick,
        ButtonAction::RightClick,
    };

    bool operator==(const ButtonMapping &) const = default;
};

struct MouseButtons {
    bool left = false;
    bool right = false;
    bool middle = false;
    bool back = false;
    bool forward = false;

    bool operator==(const MouseButtons &) const = default;
};

using ShortcutKeys = std::array<bool, KEY_MAX + 1>;

std::optional<ButtonAction> parseButtonAction(std::string_view name) {
    if (name == "native")
        return ButtonAction::Native;
    if (name == "left")
        return ButtonAction::LeftClick;
    if (name == "right")
        return ButtonAction::RightClick;
    if (name == "middle")
        return ButtonAction::MiddleClick;
    if (name == "back")
        return ButtonAction::Back;
    if (name == "forward")
        return ButtonAction::Forward;
    if (name == "undo")
        return ButtonAction::Undo;
    if (name == "redo")
        return ButtonAction::Redo;
    if (name == "screenshot")
        return ButtonAction::Screenshot;
    if (name == "overview")
        return ButtonAction::Overview;
    if (name == "disabled")
        return ButtonAction::Disabled;
    return std::nullopt;
}

const char *buttonActionName(ButtonAction action) {
    switch (action) {
    case ButtonAction::Native: return "native";
    case ButtonAction::LeftClick: return "left";
    case ButtonAction::RightClick: return "right";
    case ButtonAction::MiddleClick: return "middle";
    case ButtonAction::Back: return "back";
    case ButtonAction::Forward: return "forward";
    case ButtonAction::Undo: return "undo";
    case ButtonAction::Redo: return "redo";
    case ButtonAction::Screenshot: return "screenshot";
    case ButtonAction::Overview: return "overview";
    case ButtonAction::Disabled: return "disabled";
    }
    return "disabled";
}

void addButtonAction(ButtonAction action, bool pressed, bool primary,
                     PenButtons &native, MouseButtons &mouse,
                     ShortcutKeys &keys) {
    if (!pressed)
        return;
    switch (action) {
    case ButtonAction::Native:
        (primary ? native.button1 : native.button2) = true;
        break;
    case ButtonAction::LeftClick:
        mouse.left = true;
        break;
    case ButtonAction::RightClick:
        mouse.right = true;
        break;
    case ButtonAction::MiddleClick:
        mouse.middle = true;
        break;
    case ButtonAction::Back:
        mouse.back = true;
        break;
    case ButtonAction::Forward:
        mouse.forward = true;
        break;
    case ButtonAction::Undo:
        keys[KEY_LEFTCTRL] = true;
        keys[KEY_Z] = true;
        break;
    case ButtonAction::Redo:
        keys[KEY_LEFTCTRL] = true;
        keys[KEY_LEFTSHIFT] = true;
        keys[KEY_Z] = true;
        break;
    case ButtonAction::Screenshot:
        keys[KEY_SYSRQ] = true;
        break;
    case ButtonAction::Overview:
        keys[KEY_LEFTMETA] = true;
        break;
    case ButtonAction::Disabled:
        break;
    }
}

bool updatePenButtons(PenButtons &buttons, const input_event &event) {
    if (event.type != EV_KEY)
        return false;
    bool *button = nullptr;
    if (event.code == KEY_PAGEDOWN)
        button = &buttons.button1;
    else if (event.code == KEY_PAGEUP)
        button = &buttons.button2;
    if (!button)
        return false;
    const bool pressed = event.value != 0;
    if (*button == pressed)
        return false;
    *button = pressed;
    return true;
}

class UInputPen {
public:
    explicit UInputPen(nvt::StylusModel model)
        : has_stylus_buttons_(model == nvt::StylusModel::M80p),
          has_brake_axis_(model == nvt::StylusModel::P81c) {
        const nvt::StylusProfile &profile = nvt::stylusProfile(model);
        fd_ = open("/dev/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0)
            throw std::runtime_error(std::string("open /dev/uinput: ") +
                                     std::strerror(errno));
        for (unsigned type : {EV_KEY, EV_ABS})
            checkedIoctl(UI_SET_EVBIT, type);
        for (unsigned key : {BTN_TOOL_PEN, BTN_TOUCH})
            checkedIoctl(UI_SET_KEYBIT, key);
        if (has_stylus_buttons_) {
            checkedIoctl(UI_SET_KEYBIT, BTN_STYLUS);
            checkedIoctl(UI_SET_KEYBIT, BTN_STYLUS2);
        } else {
            // The stock P81c node advertises BTN_TRIGGER, although its normal
            // down/move/up reports never synthesize an event for this code.
            checkedIoctl(UI_SET_KEYBIT, KEY_WAKEUP);
            checkedIoctl(UI_SET_KEYBIT, BTN_TRIGGER);
        }
        for (unsigned axis : {ABS_X, ABS_Y, ABS_PRESSURE, ABS_DISTANCE,
                              ABS_TILT_X, ABS_TILT_Y})
            checkedIoctl(UI_SET_ABSBIT, axis);
        if (profile.has_brake_axis)
            checkedIoctl(UI_SET_ABSBIT, ABS_BRAKE);
        checkedIoctl(UI_SET_PROPBIT, INPUT_PROP_DIRECT);

        uinput_setup setup{};
        setup.id.bustype = BUS_VIRTUAL;
        setup.id.vendor = 0x2717;
        setup.id.product = 0x3654;
        setup.id.version = 1;
        std::strncpy(setup.name, profile.input_name, sizeof(setup.name) - 1);
        if (ioctl(fd_, UI_DEV_SETUP, &setup) < 0)
            throw std::runtime_error("UI_DEV_SETUP failed");
        if (ioctl(fd_, UI_SET_PHYS, profile.input_phys) < 0)
            throw std::runtime_error("UI_SET_PHYS failed");
        setupAxis(fd_, ABS_X, 0, kPenMaxX, 113);
        setupAxis(fd_, ABS_Y, 0, kPenMaxY, 113);
        setupAxis(fd_, ABS_PRESSURE, 0, profile.maximum_pressure);
        setupAxis(fd_, ABS_DISTANCE, 0, 1);
        setupAxis(fd_, ABS_TILT_X, -60, 60);
        setupAxis(fd_, ABS_TILT_Y, -60, 60);
        if (profile.has_brake_axis)
            setupAxis(fd_, ABS_BRAKE, 0, 360);
        if (ioctl(fd_, UI_DEV_CREATE) < 0)
            throw std::runtime_error("UI_DEV_CREATE failed");
        usleep(100000);
    }

    ~UInputPen() {
        if (fd_ < 0)
            return;
        try {
            reportButtons({});
            report({});
        } catch (...) {
        }
        ioctl(fd_, UI_DEV_DESTROY);
        close(fd_);
    }

    void report(const PenState &state) {
        std::array<input_event, 11> events;
        size_t event_count = 0;
        auto event = [&](uint16_t type, uint16_t code, int32_t value) {
            input_event &input = events[event_count++];
            input = {};
            input.type = type;
            input.code = code;
            input.value = value;
        };
        const bool contact = state.active && state.contact;
        event(EV_KEY, BTN_TOOL_PEN, state.active);
        event(EV_KEY, BTN_TOUCH, contact);
        if (has_stylus_buttons_) {
            event(EV_KEY, BTN_STYLUS, buttons_.button1);
            event(EV_KEY, BTN_STYLUS2, buttons_.button2);
        }
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
        writeInputEvents(fd_, events.data(), event_count);
    }

    void reportButtons(const PenButtons &buttons) {
        if (!has_stylus_buttons_ || buttons == buttons_)
            return;
        buttons_ = buttons;
        std::array<input_event, 3> events{};
        events[0].type = EV_KEY;
        events[0].code = BTN_STYLUS;
        events[0].value = buttons_.button1;
        events[1].type = EV_KEY;
        events[1].code = BTN_STYLUS2;
        events[1].value = buttons_.button2;
        events[2].type = EV_SYN;
        events[2].code = SYN_REPORT;
        writeInputEvents(fd_, events.data(), events.size());
    }

    void reportBrake(int angle) {
        if (!has_brake_axis_)
            return;
        std::array<input_event, 2> events{};
        events[0].type = EV_ABS;
        events[0].code = ABS_BRAKE;
        events[0].value = angle;
        events[1].type = EV_SYN;
        events[1].code = SYN_REPORT;
        writeInputEvents(fd_, events.data(), events.size());
    }

private:
    int fd_ = -1;
    bool has_stylus_buttons_;
    bool has_brake_axis_;
    PenButtons buttons_{};

    void checkedIoctl(unsigned long request, unsigned long value) {
        if (ioctl(fd_, request, value) < 0)
            throw std::runtime_error("uinput capability ioctl failed");
    }

};

class UInputActionMouse {
public:
    UInputActionMouse() {
        fd_ = open("/dev/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0)
            throw std::runtime_error(std::string("open /dev/uinput: ") +
                                     std::strerror(errno));
        for (unsigned type : {EV_KEY, EV_REL})
            checkedIoctl(UI_SET_EVBIT, type);
        for (unsigned key : {BTN_LEFT, BTN_RIGHT, BTN_MIDDLE,
                             BTN_SIDE, BTN_EXTRA})
            checkedIoctl(UI_SET_KEYBIT, key);
        for (unsigned axis : {REL_X, REL_Y})
            checkedIoctl(UI_SET_RELBIT, axis);

        uinput_setup setup{};
        setup.id.bustype = BUS_VIRTUAL;
        setup.id.vendor = 0x2717;
        setup.id.product = 0x4d80;
        setup.id.version = 1;
        std::strncpy(setup.name, "Xiaomi Focus Pen Actions Mouse",
                     sizeof(setup.name) - 1);
        if (ioctl(fd_, UI_DEV_SETUP, &setup) < 0)
            throw std::runtime_error("UI_DEV_SETUP failed");
        if (ioctl(fd_, UI_DEV_CREATE) < 0)
            throw std::runtime_error("UI_DEV_CREATE failed");
        usleep(100000);
    }

    ~UInputActionMouse() {
        if (fd_ < 0)
            return;
        try {
            report({});
        } catch (...) {
        }
        ioctl(fd_, UI_DEV_DESTROY);
        close(fd_);
    }

    void report(const MouseButtons &buttons) {
        if (buttons == buttons_)
            return;
        buttons_ = buttons;
        std::array<input_event, 6> events{};
        const std::array<std::pair<unsigned, bool>, 5> values{{
            {BTN_LEFT, buttons_.left},
            {BTN_RIGHT, buttons_.right},
            {BTN_MIDDLE, buttons_.middle},
            {BTN_SIDE, buttons_.back},
            {BTN_EXTRA, buttons_.forward},
        }};
        for (size_t index = 0; index < values.size(); ++index) {
            events[index].type = EV_KEY;
            events[index].code = values[index].first;
            events[index].value = values[index].second;
        }
        events.back().type = EV_SYN;
        events.back().code = SYN_REPORT;
        writeInputEvents(fd_, events.data(), events.size());
        std::cerr << "Focus Pen action mouse: left=" << buttons_.left
                  << " right=" << buttons_.right
                  << " middle=" << buttons_.middle
                  << " back=" << buttons_.back
                  << " forward=" << buttons_.forward << '\n';
    }

private:
    int fd_ = -1;
    MouseButtons buttons_{};

    void checkedIoctl(unsigned long request, unsigned long value) {
        if (ioctl(fd_, request, value) < 0)
            throw std::runtime_error("uinput capability ioctl failed");
    }
};

class UInputActionKeyboard {
public:
    UInputActionKeyboard() {
        fd_ = open("/dev/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0)
            throw std::runtime_error(std::string("open /dev/uinput: ") +
                                     std::strerror(errno));
        checkedIoctl(UI_SET_EVBIT, EV_KEY);
        for (unsigned key : kKeys)
            checkedIoctl(UI_SET_KEYBIT, key);

        uinput_setup setup{};
        setup.id.bustype = BUS_VIRTUAL;
        setup.id.vendor = 0x2717;
        setup.id.product = 0x4d81;
        setup.id.version = 1;
        std::strncpy(setup.name, "Xiaomi Focus Pen Actions Keyboard",
                     sizeof(setup.name) - 1);
        if (ioctl(fd_, UI_DEV_SETUP, &setup) < 0)
            throw std::runtime_error("UI_DEV_SETUP failed");
        if (ioctl(fd_, UI_DEV_CREATE) < 0)
            throw std::runtime_error("UI_DEV_CREATE failed");
        usleep(100000);
    }

    ~UInputActionKeyboard() {
        if (fd_ < 0)
            return;
        try {
            report({});
        } catch (...) {
        }
        ioctl(fd_, UI_DEV_DESTROY);
        close(fd_);
    }

    void report(const ShortcutKeys &keys) {
        if (keys == keys_)
            return;
        std::array<input_event, kKeys.size() + 1> events{};
        size_t count = 0;
        auto append = [&](unsigned code, int value) {
            input_event &event = events[count++];
            event.type = EV_KEY;
            event.code = code;
            event.value = value;
        };
        for (unsigned key : kKeys) {
            if (!keys_[key] && keys[key])
                append(key, 1);
        }
        for (auto iterator = kKeys.rbegin(); iterator != kKeys.rend();
             ++iterator) {
            if (keys_[*iterator] && !keys[*iterator])
                append(*iterator, 0);
        }
        events[count].type = EV_SYN;
        events[count].code = SYN_REPORT;
        ++count;
        writeInputEvents(fd_, events.data(), count);
        keys_ = keys;
    }

private:
    static constexpr std::array<unsigned, 5> kKeys{
        KEY_LEFTCTRL,
        KEY_LEFTSHIFT,
        KEY_LEFTMETA,
        KEY_Z,
        KEY_SYSRQ,
    };
    int fd_ = -1;
    ShortcutKeys keys_{};

    void checkedIoctl(unsigned long request, unsigned long value) {
        if (ioctl(fd_, request, value) < 0)
            throw std::runtime_error("uinput capability ioctl failed");
    }
};

class ButtonMappingControl {
public:
    ButtonMappingControl() {
        fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0)
            throw std::runtime_error(std::string("button mapping socket: ") +
                                     std::strerror(errno));
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        if (std::strlen(kButtonMappingSocket) >= sizeof(address.sun_path))
            throw std::runtime_error("button mapping socket path is too long");
        std::strncpy(address.sun_path, kButtonMappingSocket,
                     sizeof(address.sun_path) - 1);
        unlink(kButtonMappingSocket);
        if (bind(fd_, reinterpret_cast<sockaddr *>(&address),
                 sizeof(address)) < 0) {
            const int saved_errno = errno;
            close(fd_);
            fd_ = -1;
            throw std::runtime_error(std::string("bind button mapping socket: ") +
                                     std::strerror(saved_errno));
        }
        if (const group *input_group = getgrnam("input"))
            chown(kButtonMappingSocket, 0, input_group->gr_gid);
        if (chmod(kButtonMappingSocket, 0660) < 0)
            std::cerr << "button mapping socket chmod failed: "
                      << std::strerror(errno) << '\n';
        std::cerr << "Focus Pen button mapping control ready ("
                  << kButtonMappingSocket << ")\n";
    }

    ~ButtonMappingControl() {
        if (fd_ >= 0)
            close(fd_);
        unlink(kButtonMappingSocket);
    }

    int fd() const {
        return fd_;
    }

    bool drain(ButtonMapping &mapping) {
        bool changed = false;
        while (true) {
            std::array<char, 256> buffer{};
            const ssize_t size = recv(fd_, buffer.data(), buffer.size(), 0);
            if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                return changed;
            if (size <= 0)
                return changed;

            std::istringstream input(
                std::string(buffer.data(), static_cast<size_t>(size)));
            std::string command;
            std::array<std::string, 4> names;
            std::string trailing;
            if (!(input >> command >> names[0] >> names[1] >> names[2] >>
                  names[3]) || command != "map" || (input >> trailing)) {
                std::cerr << "ignored invalid Focus Pen button mapping\n";
                continue;
            }
            std::array<ButtonAction, 4> actions{};
            bool valid = true;
            for (size_t index = 0; index < names.size(); ++index) {
                const auto action = parseButtonAction(names[index]);
                if (!action) {
                    valid = false;
                    break;
                }
                actions[index] = *action;
            }
            if (!valid) {
                std::cerr << "ignored unknown Focus Pen button action\n";
                continue;
            }
            const ButtonMapping next{
                {actions[0], actions[1]},
                {actions[2], actions[3]},
            };
            if (next == mapping)
                continue;
            mapping = next;
            changed = true;
            std::cerr << "Focus Pen button mapping: pen="
                      << buttonActionName(mapping.pen.primary) << ','
                      << buttonActionName(mapping.pen.secondary)
                      << " air=" << buttonActionName(mapping.air.primary)
                      << ',' << buttonActionName(mapping.air.secondary) << '\n';
        }
    }

private:
    int fd_ = -1;
};

struct ProGestureButtons {
    bool pinch = false;
    bool double_tap = false;
    bool slide_up = false;
    bool slide_down = false;

    bool operator==(const ProGestureButtons &) const = default;
};

class UInputProGestures {
public:
    UInputProGestures() {
        fd_ = open("/dev/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0)
            throw std::runtime_error(std::string("open /dev/uinput: ") +
                                     std::strerror(errno));
        checkedIoctl(UI_SET_EVBIT, EV_KEY);
        // Android's logical BUTTON_7..BUTTON_10 correspond to Linux input
        // codes 262..265, which are BTN_6..BTN_9.
        for (unsigned key : {BTN_6, BTN_7, BTN_8, BTN_9})
            checkedIoctl(UI_SET_KEYBIT, key);

        uinput_setup setup{};
        setup.id.bustype = BUS_VIRTUAL;
        setup.id.vendor = 0x0022;
        setup.id.product = 0x5081;
        setup.id.version = 1;
        std::strncpy(setup.name, "Xiaomi Focus Pen Pro Gestures",
                     sizeof(setup.name) - 1);
        if (ioctl(fd_, UI_DEV_SETUP, &setup) < 0)
            throw std::runtime_error("UI_DEV_SETUP failed");
        if (ioctl(fd_, UI_SET_PHYS, "input/pen_p81c/gestures") < 0)
            throw std::runtime_error("UI_SET_PHYS failed");
        if (ioctl(fd_, UI_DEV_CREATE) < 0)
            throw std::runtime_error("UI_DEV_CREATE failed");
        usleep(100000);
    }

    ~UInputProGestures() {
        if (fd_ < 0)
            return;
        try {
            report({});
        } catch (...) {
        }
        ioctl(fd_, UI_DEV_DESTROY);
        close(fd_);
    }

    void report(const ProGestureButtons &buttons) {
        std::array<input_event, 5> events{};
        constexpr std::array<unsigned, 4> codes = {
            BTN_6, BTN_7, BTN_8, BTN_9};
        const std::array<bool, 4> values = {
            buttons.pinch, buttons.double_tap,
            buttons.slide_up, buttons.slide_down};
        for (std::size_t index = 0; index < codes.size(); ++index) {
            events[index].type = EV_KEY;
            events[index].code = static_cast<std::uint16_t>(codes[index]);
            events[index].value = values[index];
        }
        events.back().type = EV_SYN;
        events.back().code = SYN_REPORT;
        writeInputEvents(fd_, events.data(), events.size());
    }

private:
    int fd_ = -1;

    void checkedIoctl(unsigned long request, unsigned long value) {
        if (ioctl(fd_, request, value) < 0)
            throw std::runtime_error("uinput capability ioctl failed");
    }
};

std::string readFirstLine(const std::filesystem::path &path) {
    std::ifstream input(path);
    std::string line;
    std::getline(input, line);
    return line;
}

class DisplayStateReader {
public:
    bool update() {
        const auto now = std::chrono::steady_clock::now();
        if (now < next_poll_)
            return false;
        next_poll_ = now + std::chrono::milliseconds(100);
        const std::optional<bool> state = readState();
        if (!state || *state == screen_on_)
            return false;
        screen_on_ = *state;
        return true;
    }

    bool screenOn() const {
        return screen_on_;
    }

private:
    static constexpr const char *kDpmsPath =
        "/sys/class/drm/card0-DSI-1/dpms";

    bool screen_on_ = true;
    std::chrono::steady_clock::time_point next_poll_{};

    static std::optional<bool> readState() {
        const std::string dpms = readFirstLine(kDpmsPath);
        if (dpms == "On")
            return true;
        if (dpms == "Off")
            return false;
        return std::nullopt;
    }
};

struct FocusPenIdentity {
    std::string name;
    std::string phys;
    std::string uniq;
    unsigned bus = 0;
    unsigned vendor = 0;
    unsigned product = 0;
};

unsigned parseHex(std::string_view text) {
    std::string value(text);
    char *end = nullptr;
    const unsigned long result = std::strtoul(value.c_str(), &end, 16);
    return end == value.c_str() || *end != '\0'
               ? 0U : static_cast<unsigned>(result);
}

FocusPenIdentity readHidIdentity(const std::filesystem::path &path) {
    FocusPenIdentity identity;
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
        constexpr std::string_view id_prefix = "HID_ID=";
        constexpr std::string_view name_prefix = "HID_NAME=";
        constexpr std::string_view phys_prefix = "HID_PHYS=";
        constexpr std::string_view uniq_prefix = "HID_UNIQ=";
        if (line.starts_with(id_prefix)) {
            const std::string_view id =
                std::string_view(line).substr(id_prefix.size());
            const std::size_t first = id.find(':');
            const std::size_t second = first == std::string_view::npos
                ? first : id.find(':', first + 1);
            if (second != std::string_view::npos) {
                identity.bus = parseHex(id.substr(0, first));
                identity.vendor = parseHex(
                    id.substr(first + 1, second - first - 1));
                identity.product = parseHex(id.substr(second + 1));
            }
        } else if (line.starts_with(name_prefix)) {
            identity.name = line.substr(name_prefix.size());
        } else if (line.starts_with(phys_prefix)) {
            identity.phys = line.substr(phys_prefix.size());
        } else if (line.starts_with(uniq_prefix)) {
            identity.uniq = line.substr(uniq_prefix.size());
        }
    }
    return identity;
}

FocusPenIdentity readEvdevIdentity(const std::filesystem::path &entry) {
    FocusPenIdentity identity;
    identity.name = readFirstLine(entry / "device/name");
    identity.phys = readFirstLine(entry / "device/phys");
    identity.uniq = readFirstLine(entry / "device/uniq");
    identity.bus = parseHex(readFirstLine(entry / "device/id/bustype"));
    identity.vendor = parseHex(readFirstLine(entry / "device/id/vendor"));
    identity.product = parseHex(readFirstLine(entry / "device/id/product"));
    return identity;
}

std::optional<nvt::StylusModel> modelForPenIdentity(
    const FocusPenIdentity &identity) {
    if (identity.bus != BUS_BLUETOOTH || identity.vendor != 0x22)
        return std::nullopt;
    if (identity.name == kFocusPenName && identity.product == 0x4d80)
        return nvt::StylusModel::M80p;
    if (identity.name == kFocusPenProName && identity.product == 0x5081)
        return nvt::StylusModel::P81c;
    return std::nullopt;
}

bool samePhysicalPen(const FocusPenIdentity &first,
                     const FocusPenIdentity &second) {
    if (first.bus != second.bus || first.vendor != second.vendor ||
        first.product != second.product)
        return false;
    if (!first.uniq.empty() && !second.uniq.empty() &&
        first.uniq != second.uniq)
        return false;
    return first.phys.empty() || second.phys.empty() ||
           first.phys == second.phys;
}

struct PenTransportUpdate {
    std::optional<nvt::StylusModel> model;
    std::optional<std::string> connected_address;
    std::optional<PenButtons> buttons;
    std::optional<ProGestureButtons> gestures;
    std::optional<int> brake;
    unsigned double_tap_haptics = 0;
    unsigned slide_haptics = 0;
    bool reset = false;
};

const char *modelName(nvt::StylusModel model) {
    return model == nvt::StylusModel::P81c ? "P81c" : "M80p";
}

class FocusPenHidReader {
public:
    ~FocusPenHidReader() {
        closeHidraw();
        closePressureEvent();
        closeButtonEvent();
        closePointerEvent();
    }

    PenTransportUpdate service(nvt::FocusPenPressureQueue &queue,
                               nvt::PencilPostureDecoder *posture) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_scan_) {
            next_scan_ = now + std::chrono::seconds(1);
            if (hidraw_fd_ < 0)
                openHidraw();
            if (pressure_event_fd_ < 0)
                grabReportEvent();
            if (button_event_fd_ < 0)
                grabButtonEvent();
            if (model_ == nvt::StylusModel::M80p && pointer_event_fd_ < 0)
                openPointerEvent();
        }
        PenTransportUpdate update;
        if (transport_reset_pending_) {
            update.reset = true;
            transport_reset_pending_ = false;
        }
        if (pending_model_) {
            update.model = pending_model_;
            pending_model_.reset();
        }
        if (connected_address_pending_) {
            update.connected_address = connected_address_pending_;
            connected_address_pending_.reset();
        }
        if (update.model)
            return update;
        drainHidraw(queue, posture);
        drainPressureEvent();
        drainButtonEvent();
        drainPointerEvent();
        refreshAirPointerState(std::chrono::steady_clock::now());
        if (button_update_pending_) {
            update.buttons = buttons_;
            button_update_pending_ = false;
        }
        if (gesture_update_pending_) {
            update.gestures = gestures_;
            gesture_update_pending_ = false;
        }
        if (brake_update_) {
            update.brake = brake_update_;
            brake_update_.reset();
        }
        update.double_tap_haptics = double_tap_haptics_;
        update.slide_haptics = slide_haptics_;
        double_tap_haptics_ = 0;
        slide_haptics_ = 0;
        return update;
    }

    int buttonEventFd() const {
        return button_event_fd_;
    }

    int pointerEventFd() const {
        return pointer_event_fd_;
    }

    bool airPointerActive() const {
        return air_pointer_seen_ &&
            std::chrono::steady_clock::now() - last_air_pointer_activity_ <=
                kAirPointerActivityTimeout;
    }

private:
    int hidraw_fd_ = -1;
    int pressure_event_fd_ = -1;
    int button_event_fd_ = -1;
    int pointer_event_fd_ = -1;
    nvt::StylusModel model_ = nvt::StylusModel::M80p;
    std::optional<nvt::StylusModel> pending_model_;
    PenButtons buttons_{};
    ProGestureButtons gestures_{};
    bool button_update_pending_ = false;
    bool gesture_update_pending_ = false;
    bool button_resync_pending_ = false;
    bool transport_reset_pending_ = false;
    bool air_pointer_active_ = false;
    bool air_pointer_seen_ = false;
    std::optional<int> brake_update_;
    unsigned double_tap_haptics_ = 0;
    unsigned slide_haptics_ = 0;
    std::optional<std::string> connected_address_pending_;
    FocusPenIdentity identity_{};
    std::chrono::steady_clock::time_point next_scan_{};
    std::chrono::steady_clock::time_point last_air_pointer_activity_{};

    void openHidraw() {
        std::error_code error;
        struct Candidate {
            std::filesystem::path path;
            nvt::StylusModel model;
            FocusPenIdentity identity;
        };
        std::vector<Candidate> candidates;
        for (const auto &entry : std::filesystem::directory_iterator(
                 "/sys/class/hidraw", error)) {
            FocusPenIdentity identity = readHidIdentity(
                entry.path() / "device/uevent");
            const auto model = modelForPenIdentity(identity);
            if (model)
                candidates.push_back(
                    {entry.path(), *model, std::move(identity)});
        }
        std::sort(candidates.begin(), candidates.end(),
                  [&](const Candidate &first, const Candidate &second) {
                      const bool first_current =
                          first.model == model_;
                      const bool second_current =
                          second.model == model_;
                      if (first_current != second_current)
                          return first_current;
                      return first.path < second.path;
                  });
        for (const Candidate &candidate : candidates) {
            const std::string path =
                "/dev/" + candidate.path.filename().string();
            const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
            if (fd < 0)
                continue;
            selectModel(candidate.model);
            identity_ = candidate.identity;
            if (candidate.model == nvt::StylusModel::P81c)
                connected_address_pending_ = identity_.uniq;
            else
                connected_address_pending_.reset();
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
            const FocusPenIdentity identity = readEvdevIdentity(entry);
            const auto model = modelForPenIdentity(identity);
            if (!model || *model != model_ ||
                !samePhysicalPen(identity_, identity))
                continue;
            const std::string path = "/dev/input/" + entry.filename().string();
            const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
            if (fd < 0)
                continue;
            if (ioctl(fd, EVIOCGRAB, 1) < 0) {
                close(fd);
                continue;
            }
            pressure_event_fd_ = fd;
            std::cerr << "claimed Focus Pen report-5 evdev transport ("
                      << path << "); keypad-plus events suppressed\n";
            return;
        }
    }

    void grabButtonEvent() {
        std::error_code error;
        std::vector<std::filesystem::path> entries;
        for (const auto &entry : std::filesystem::directory_iterator(
                 "/sys/class/input", error)) {
            if (entry.path().filename().string().starts_with("event"))
                entries.push_back(entry.path());
        }
        std::sort(entries.begin(), entries.end());
        const std::string_view expected_name =
            model_ == nvt::StylusModel::P81c
                ? kFocusPenProKeyboardName
                : kFocusPenKeyboardName;
        for (const auto &entry : entries) {
            const FocusPenIdentity identity = readEvdevIdentity(entry);
            if (identity.name != expected_name ||
                !samePhysicalPen(identity_, identity))
                continue;
            const std::string path = "/dev/input/" + entry.filename().string();
            const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
            if (fd < 0)
                continue;
            if (ioctl(fd, EVIOCGRAB, 1) < 0) {
                close(fd);
                continue;
            }
            button_event_fd_ = fd;
            if (model_ == nvt::StylusModel::P81c) {
                std::cerr << "claimed Focus Pen Pro gesture transport ("
                          << path << "; pinch, double-tap, and slides)\n";
            } else {
                std::cerr << "claimed Focus Pen button evdev transport ("
                          << path
                          << "; PageDown/PageUp mapped to stylus buttons)\n";
            }
            synchronizeButtonState();
            return;
        }
    }

    void openPointerEvent() {
        std::error_code error;
        std::vector<std::filesystem::path> entries;
        for (const auto &entry : std::filesystem::directory_iterator(
                 "/sys/class/input", error)) {
            if (entry.path().filename().string().starts_with("event"))
                entries.push_back(entry.path());
        }
        std::sort(entries.begin(), entries.end());
        for (const auto &entry : entries) {
            const FocusPenIdentity identity = readEvdevIdentity(entry);
            if (identity.name != kFocusPenMouseName ||
                !samePhysicalPen(identity_, identity))
                continue;
            const std::string path = "/dev/input/" + entry.filename().string();
            const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
            if (fd < 0)
                continue;
            pointer_event_fd_ = fd;
            std::cerr << "watching Focus Pen air-pointer transport ("
                      << path << "; side buttons become left/right click)\n";
            return;
        }
    }

    void drainHidraw(nvt::FocusPenPressureQueue &queue,
                     nvt::PencilPostureDecoder *posture) {
        if (hidraw_fd_ < 0)
            return;
        std::array<uint8_t, 64> report{};
        while (true) {
            const ssize_t size = read(hidraw_fd_, report.data(), report.size());
            if (size > 0) {
                const std::span<const std::uint8_t> input(
                    report.data(), static_cast<std::size_t>(size));
                queue.pushReport(input);
                if (model_ == nvt::StylusModel::P81c && posture) {
                    if (const auto angle = posture->processReport(input))
                        brake_update_ = angle;
                }
                continue;
            }
            if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                return;
            closeHidraw();
            return;
        }
    }

    void drainPressureEvent() {
        if (pressure_event_fd_ < 0)
            return;
        std::array<input_event, 32> events{};
        while (true) {
            const ssize_t size = read(
                pressure_event_fd_, events.data(), sizeof(events));
            if (size > 0)
                continue;
            if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                return;
            closePressureEvent();
            return;
        }
    }

    void drainButtonEvent() {
        if (button_event_fd_ < 0)
            return;
        std::array<input_event, 32> events{};
        while (true) {
            const ssize_t size = read(
                button_event_fd_, events.data(), sizeof(events));
            if (size > 0) {
                if (size % static_cast<ssize_t>(sizeof(input_event)) != 0) {
                    closeButtonEvent();
                    return;
                }
                const size_t count = static_cast<size_t>(size) / sizeof(input_event);
                for (size_t index = 0; index < count; ++index)
                    consumeButtonEvent(events[index]);
                continue;
            }
            if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                return;
            closeButtonEvent();
            return;
        }
    }

    void drainPointerEvent() {
        if (pointer_event_fd_ < 0)
            return;
        std::array<input_event, 32> events{};
        while (true) {
            const ssize_t size = read(
                pointer_event_fd_, events.data(), sizeof(events));
            if (size > 0) {
                if (size % static_cast<ssize_t>(sizeof(input_event)) != 0) {
                    closePointerEvent();
                    return;
                }
                const size_t count = static_cast<size_t>(size) /
                                     sizeof(input_event);
                for (size_t index = 0; index < count; ++index) {
                    const input_event &event = events[index];
                    if (event.type == EV_REL &&
                        (event.code == REL_X || event.code == REL_Y) &&
                        event.value != 0) {
                        air_pointer_seen_ = true;
                        last_air_pointer_activity_ =
                            std::chrono::steady_clock::now();
                    }
                }
                continue;
            }
            if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                return;
            closePointerEvent();
            return;
        }
    }

    void refreshAirPointerState(
        std::chrono::steady_clock::time_point now) {
        const bool active = air_pointer_seen_ &&
            now - last_air_pointer_activity_ <= kAirPointerActivityTimeout;
        if (active == air_pointer_active_)
            return;
        air_pointer_active_ = active;
        std::cerr << "Focus Pen air pointer "
                  << (active ? "active" : "inactive") << '\n';
    }

    static bool keyBitIsSet(const unsigned long *bits, unsigned code) {
        constexpr unsigned bits_per_word = sizeof(unsigned long) * 8;
        return (bits[code / bits_per_word] >> (code % bits_per_word)) & 1UL;
    }

    void consumeButtonEvent(const input_event &event) {
        if (button_resync_pending_) {
            if (event.type == EV_SYN && event.code == SYN_REPORT) {
                button_resync_pending_ = false;
                synchronizeButtonState();
            }
            return;
        }
        if (event.type == EV_SYN && event.code == SYN_DROPPED) {
            releaseButtonState();
            button_resync_pending_ = true;
            std::cerr << "Focus Pen button event queue overflow; "
                         "waiting to resynchronize\n";
            return;
        }
        if (model_ == nvt::StylusModel::M80p) {
            if (updatePenButtons(buttons_, event)) {
                button_update_pending_ = true;
                std::cerr << "Focus Pen barrel buttons: primary="
                          << buttons_.button1
                          << " secondary=" << buttons_.button2 << '\n';
            }
            return;
        }
        if (event.type != EV_KEY || (event.value != 0 && event.value != 1))
            return;

        bool *button = nullptr;
        if (event.code == KEY_F19)
            button = &gestures_.pinch;
        else if (event.code == KEY_KPENTER)
            button = &gestures_.double_tap;
        else if (event.code == KEY_KP9)
            button = &gestures_.slide_up;
        else if (event.code == KEY_KP3)
            button = &gestures_.slide_down;
        if (!button)
            return;
        const bool pressed = event.value == 1;
        if (*button == pressed)
            return;
        *button = pressed;
        gesture_update_pending_ = true;
        if (pressed && event.code == KEY_KPENTER)
            ++double_tap_haptics_;
        if (pressed && (event.code == KEY_KP9 || event.code == KEY_KP3))
            ++slide_haptics_;
    }

    void synchronizeButtonState() {
        if (button_event_fd_ < 0)
            return;
        constexpr std::size_t bits_per_word = sizeof(unsigned long) * 8;
        std::array<unsigned long, (KEY_MAX + bits_per_word) / bits_per_word>
            bits{};
        if (ioctl(button_event_fd_, EVIOCGKEY(sizeof(bits)), bits.data()) < 0) {
            std::cerr << "Focus Pen button state resync failed: "
                      << std::strerror(errno) << '\n';
            return;
        }
        if (model_ == nvt::StylusModel::M80p) {
            const PenButtons state{
                keyBitIsSet(bits.data(), KEY_PAGEDOWN),
                keyBitIsSet(bits.data(), KEY_PAGEUP)};
            if (!(state.button1 == buttons_.button1 &&
                  state.button2 == buttons_.button2)) {
                buttons_ = state;
                button_update_pending_ = true;
            }
            return;
        }
        const ProGestureButtons state{
            keyBitIsSet(bits.data(), KEY_F19),
            keyBitIsSet(bits.data(), KEY_KPENTER),
            keyBitIsSet(bits.data(), KEY_KP9),
            keyBitIsSet(bits.data(), KEY_KP3)};
        if (!(state == gestures_)) {
            gestures_ = state;
            gesture_update_pending_ = true;
        }
    }

    void releaseButtonState() {
        if (buttons_.button1 || buttons_.button2) {
            buttons_ = {};
            button_update_pending_ = true;
        }
        if (!(gestures_ == ProGestureButtons{})) {
            gestures_ = {};
            gesture_update_pending_ = true;
        }
    }

    void closeHidraw() {
        if (hidraw_fd_ >= 0) {
            close(hidraw_fd_);
            transport_reset_pending_ = true;
        }
        hidraw_fd_ = -1;
        brake_update_.reset();
    }

    void closePressureEvent() {
        if (pressure_event_fd_ >= 0) {
            ioctl(pressure_event_fd_, EVIOCGRAB, 0);
            close(pressure_event_fd_);
        }
        pressure_event_fd_ = -1;
    }

    void closeButtonEvent() {
        if (button_event_fd_ >= 0) {
            ioctl(button_event_fd_, EVIOCGRAB, 0);
            close(button_event_fd_);
        }
        button_event_fd_ = -1;
        button_resync_pending_ = false;
        releaseButtonState();
    }

    void closePointerEvent() {
        if (pointer_event_fd_ >= 0)
            close(pointer_event_fd_);
        pointer_event_fd_ = -1;
        air_pointer_seen_ = false;
        air_pointer_active_ = false;
    }

    void selectModel(nvt::StylusModel model) {
        if (model_ == model)
            return;
        closeHidraw();
        closePressureEvent();
        closeButtonEvent();
        closePointerEvent();
        model_ = model;
        pending_model_ = model;
        std::cerr << "Focus Pen model: " << modelName(model) << '\n';
    }
};

class PadAccelerometer {
public:
    explicit PadAccelerometer(nvt::PencilPostureDecoder &decoder)
        : decoder_(&decoder) {
        GError *error = nullptr;
        sensor_ = ssc_sensor_accelerometer_new_sync(nullptr, &error);
        if (!sensor_) {
            const std::string message = error && error->message
                ? error->message : "unknown error";
            g_clear_error(&error);
            throw std::runtime_error(
                "initialize SSC accelerometer: " + message);
        }
        g_signal_connect(sensor_, "measurement",
                         G_CALLBACK(measurement), this);
        if (!ssc_sensor_accelerometer_open_sync(sensor_, nullptr, &error)) {
            const std::string message = error && error->message
                ? error->message : "unknown error";
            g_clear_error(&error);
            g_object_unref(sensor_);
            sensor_ = nullptr;
            throw std::runtime_error("open SSC accelerometer: " + message);
        }
    }

    PadAccelerometer(const PadAccelerometer &) = delete;
    PadAccelerometer &operator=(const PadAccelerometer &) = delete;

    ~PadAccelerometer() {
        if (sensor_) {
            GError *error = nullptr;
            if (!ssc_sensor_accelerometer_close_sync(sensor_, nullptr, &error)) {
                std::cerr << "close SSC accelerometer: "
                          << (error && error->message
                                  ? error->message : "unknown error")
                          << '\n';
            }
            g_clear_error(&error);
        }
        if (sensor_)
            g_object_unref(sensor_);
    }

    void dispatch() {
        while (g_main_context_iteration(nullptr, false)) {
        }
    }

private:
    SSCSensorAccelerometer *sensor_ = nullptr;
    nvt::PencilPostureDecoder *decoder_;
    std::optional<std::chrono::steady_clock::time_point> next_measurement_;

    static void measurement(SSCSensorAccelerometer *, float x, float y,
                            float z, gpointer user_data) {
        auto *self = static_cast<PadAccelerometer *>(user_data);
        const auto now = std::chrono::steady_clock::now();
        if (self->next_measurement_ && now < *self->next_measurement_)
            return;

        self->decoder_->updatePadAcceleration(x, y, z);
        if (!self->next_measurement_) {
            self->next_measurement_ = now + std::chrono::milliseconds(100);
            return;
        }
        do {
            *self->next_measurement_ += std::chrono::milliseconds(100);
        } while (*self->next_measurement_ <= now);
    }
};

class LiveTouchAdapter {
public:
    struct Update {
        std::optional<nvt::FrameResult> result;
        bool baseline_ready = false;
    };

    void reset() {
        core_.reset();
        filter_.reset();
        reference_samples_.clear();
        interference_delta_.fill(0);
        interference_active_ = false;
    }

    Update feed(const nvt::Matrix &matrix, uint16_t counter,
                uint8_t frame_type, uint64_t timestamp_ns) {
        if (frame_type != 2 && frame_type != 4)
            return {};
        if (frame_type == 2 && !core_.hasReference()) {
            core_.process(matrix, counter, frame_type);
            reference_samples_.clear();
            interference_delta_.fill(0);
            interference_active_ = false;
            return {std::nullopt, true};
        }
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
            reset();
            core_.process(reference, counter, 2);
            return {std::nullopt, true};
        }
        nvt::FrameResult result = core_.process(matrix, counter, frame_type);
        interference_delta_ = result.interference_delta;
        interference_active_ = !result.search_peaks.empty();
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

    void feedNoTouch() {
        core_.processNoTouch();
    }

    void feedStylusMutual(const nvt::Matrix &matrix, uint16_t counter,
                          uint8_t frame_type) {
        if (!core_.hasReference())
            return;
        nvt::MutualState state =
            core_.processMutualState(matrix, counter, frame_type);
        interference_delta_ = state.delta;
        interference_active_ = state.interference;
    }

    bool interferenceActive() const {
        return interference_active_;
    }

    const nvt::Matrix &interferenceDelta() const {
        return interference_delta_;
    }

private:
    nvt::TouchCore core_;
    nvt::FingerFilter filter_;
    std::vector<nvt::Matrix> reference_samples_;
    nvt::Matrix interference_delta_{};
    bool interference_active_ = false;
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
            if (header.flags & kStreamFlagValid)
                function(header.timestamp_ns, header.flags,
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
    std::optional<UInputPen> pen_m80p;
    std::optional<UInputPen> pen_p81c;
    std::optional<UInputActionMouse> action_mouse;
    std::optional<UInputActionKeyboard> action_keyboard;
    std::optional<UInputProGestures> pro_gestures;
    try {
        nvt::StylusModel stylus_model = nvt::StylusModel::M80p;
        const nvt::StylusProfile *stylus_profile =
            &nvt::stylusProfile(stylus_model);
        writeControl(kStylusPath, stylus_profile->controller_switch);
        writeControl(kControlPath, 1);
        touch.emplace();
        pen_m80p.emplace(nvt::StylusModel::M80p);
        pen_p81c.emplace(nvt::StylusModel::P81c);
        action_mouse.emplace();
        action_keyboard.emplace();
        std::cerr << "touch and pen pipelines ready\n";
        std::cerr << "waiting for a touch reference frame\n";
        LiveTouchAdapter adapter;
        nvt::StylusDecoder pen_decoder(stylus_model);
        nvt::StylusMutualAssembler stylus_mutual;
        nvt::FocusPenPressureQueue pen_pressure(
            stylus_profile->maximum_pressure);
        nvt::PencilPostureDecoder pencil_posture;
        std::optional<PadAccelerometer> pad_accelerometer;
        DisplayStateReader display_state;
        nvt::P81cAgController p81c_ag_controller;
        FocusPenHidReader pen_transport;
        ButtonMappingControl button_mapping_control;
        ButtonMapping button_mapping;
        bool touch_active = false;
        bool pen_active = false;
        PenButtons m80p_buttons{};
        bool have_valid_frame = false;
        bool stream_stalled = false;
        auto last_valid_frame = std::chrono::steady_clock::now();
        StreamReader reader(stream_fd);
        auto activePen = [&]() -> UInputPen * {
            if (stylus_model == nvt::StylusModel::P81c)
                return pen_p81c ? &*pen_p81c : nullptr;
            return pen_m80p ? &*pen_m80p : nullptr;
        };
        auto releaseActivePen = [&]() {
            if (!pen_active)
                return;
            if (UInputPen *pen = activePen())
                pen->report({});
        };
        auto updateButtonActions = [&]() {
            PenButtons native_buttons{};
            MouseButtons mouse_buttons{};
            ShortcutKeys shortcut_keys{};
            if (stylus_model == nvt::StylusModel::M80p) {
                const bool air_mode = pen_transport.airPointerActive() &&
                                      !pen_active;
                const ButtonContextMapping &context = air_mode
                    ? button_mapping.air : button_mapping.pen;
                addButtonAction(context.primary, m80p_buttons.button1, true,
                                native_buttons, mouse_buttons, shortcut_keys);
                addButtonAction(context.secondary, m80p_buttons.button2, false,
                                native_buttons, mouse_buttons, shortcut_keys);
            }
            if (UInputPen *pen = activePen())
                pen->reportButtons(native_buttons);
            if (action_mouse)
                action_mouse->report(mouse_buttons);
            if (action_keyboard)
                action_keyboard->report(shortcut_keys);
        };
        auto resetPipelines = [&]() {
            if (touch)
                touch->report({});
            releaseActivePen();
            adapter.reset();
            pen_decoder.reset();
            stylus_mutual = {};
            pen_pressure.reset();
            if (stylus_model == nvt::StylusModel::P81c)
                pencil_posture.reset();
            touch_active = false;
            pen_active = false;
            updateButtonActions();
            have_valid_frame = false;
            stream_stalled = false;
        };
        auto switchStylusModel = [&](nvt::StylusModel model) {
            if (model == stylus_model)
                return;
            releaseActivePen();
            m80p_buttons = {};
            updateButtonActions();
            if (stylus_model == nvt::StylusModel::P81c) {
                p81c_ag_controller.disable();
                pro_gestures.reset();
                pad_accelerometer.reset();
            }
            stylus_profile = &nvt::stylusProfile(model);
            writeControl(kStylusPath, stylus_profile->controller_switch);
            stylus_model = model;
            pen_decoder = nvt::StylusDecoder(stylus_model);
            stylus_mutual = {};
            pen_pressure = nvt::FocusPenPressureQueue(
                stylus_profile->maximum_pressure);
            if (stylus_model == nvt::StylusModel::P81c) {
                pencil_posture.reset();
                pro_gestures.emplace();
                try {
                    pad_accelerometer.emplace(pencil_posture);
                    std::cerr << "P81c posture accelerometer ready\n";
                } catch (const std::exception &error) {
                    std::cerr << "P81c ABS_BRAKE disabled: "
                              << error.what() << '\n';
                }
            }
            pen_active = false;
            std::cerr << "stylus pipeline switched to "
                      << modelName(stylus_model) << '\n';
        };
        auto servicePenTransport = [&]() {
            if (stylus_model == nvt::StylusModel::P81c &&
                display_state.update())
                p81c_ag_controller.setScreenOn(display_state.screenOn());
            if (pad_accelerometer)
                pad_accelerometer->dispatch();
            p81c_ag_controller.service();
            PenTransportUpdate update = pen_transport.service(
                pen_pressure,
                stylus_model == nvt::StylusModel::P81c
                    ? &pencil_posture : nullptr);
            if (update.reset) {
                pen_pressure.reset();
                if (stylus_model == nvt::StylusModel::P81c) {
                    p81c_ag_controller.disable();
                    pencil_posture.reset();
                }
                if (pen_active) {
                    if (UInputPen *pen = activePen())
                        pen->report({});
                }
                pen_active = false;
                updateButtonActions();
            }
            if (update.model)
                switchStylusModel(*update.model);
            if (update.connected_address &&
                stylus_model == nvt::StylusModel::P81c) {
                p81c_ag_controller.enable(
                    *update.connected_address, display_state.screenOn());
            }
            if (update.buttons) {
                m80p_buttons = *update.buttons;
            }
            updateButtonActions();
            if (update.gestures && pro_gestures)
                pro_gestures->report(*update.gestures);
            if (update.brake && stylus_model == nvt::StylusModel::P81c) {
                if (UInputPen *pen = activePen())
                    pen->reportBrake(*update.brake);
            }
            if (stylus_model == nvt::StylusModel::P81c &&
                display_state.screenOn()) {
                for (unsigned count = 0;
                     count < update.double_tap_haptics; ++count) {
                    p81c_ag_controller.vibrate(3, 0x80);
                }
                for (unsigned count = 0;
                     count < update.slide_haptics; ++count) {
                    p81c_ag_controller.vibrate(4, 0x80);
                }
            }
        };
        while (running) {
            if (button_mapping_control.drain(button_mapping))
                updateButtonActions();
            servicePenTransport();
            std::array<pollfd, 4> descriptors{
                pollfd{stream_fd, POLLIN, 0},
                pollfd{pen_transport.buttonEventFd(), POLLIN, 0},
                pollfd{pen_transport.pointerEventFd(), POLLIN, 0},
                pollfd{button_mapping_control.fd(), POLLIN, 0},
            };
            const int status = poll(
                descriptors.data(), descriptors.size(), 100);
            if (status < 0) {
                if (errno == EINTR)
                    continue;
                throw std::runtime_error(std::string("poll: ") +
                                         std::strerror(errno));
            }
            if (status > 0) {
                servicePenTransport();
            }
            if (status > 0 && (descriptors[3].revents & POLLIN) &&
                button_mapping_control.drain(button_mapping))
                updateButtonActions();
            if (status > 0 &&
                (descriptors[0].revents & (POLLIN | POLLERR | POLLHUP))) {
                reader.readAvailable([&](uint64_t timestamp,
                                         uint16_t stream_flags,
                                         const uint8_t *frame,
                                         size_t frame_length) {
                    if (frame_length <
                        kMatrixOffset + nvt::kNodes * sizeof(int16_t))
                        throw std::runtime_error("short THP frame");
                    if (stream_flags & kStreamFlagEpoch) {
                        resetPipelines();
                        std::cerr << "THP controller epoch changed; "
                                     "waiting for a new reference\n";
                    }
                    last_valid_frame = std::chrono::steady_clock::now();
                    have_valid_frame = true;
                    if (stream_stalled) {
                        stream_stalled = false;
                        std::cerr << "THP stream recovered\n";
                    }
                    const uint8_t data_type =
                        frame[kTransportLength + 0x38];
                    if (data_type == 0x1d) {
                        adapter.feedNoTouch();
                        const int pressure = pen_pressure.consume();
                        nvt::RawStylusFrame raw_stylus;
                        if (!nvt::parseRawStylusFrame(
                                frame, frame_length, raw_stylus))
                            throw std::runtime_error("invalid stylus frame");
                        if (adapter.interferenceActive()) {
                            nvt::preprocessStylusInterference(
                                raw_stylus, adapter.interferenceDelta());
                        }
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
                        if (stylus_model == nvt::StylusModel::P81c) {
                            pencil_posture.updatePenState(
                                result.active,
                                result.coordinates.tilt_x,
                                result.coordinates.tilt_y,
                                result.coordinates.tip_x,
                                result.coordinates.tip_y,
                                kPostureRawPenMaxX, kPostureRawPenMaxY);
                        }
                        if (result.active || pen_active) {
                            if (UInputPen *pen = activePen())
                                pen->report(state);
                        }
                        pen_active = result.active;
                        updateButtonActions();
                        stylus_mutual.ingest(raw_stylus);
                        if (stylus_mutual.hasMatrix()) {
                            const uint8_t frame_type =
                                frame[kTransportLength + 24];
                            const uint16_t counter =
                                readLe16(frame + kTransportLength + 2);
                            adapter.feedStylusMutual(
                                stylus_mutual.matrix(), counter, frame_type);
                        }
                        return;
                    }
                    if (pen_active) {
                        if (UInputPen *pen = activePen())
                            pen->report({});
                        pen_active = false;
                    }
                    if (stylus_model == nvt::StylusModel::P81c) {
                        pencil_posture.updatePenState(
                            false, 0, 0, 0, 0,
                            kPostureRawPenMaxX, kPostureRawPenMaxY);
                    }
                    const uint8_t frame_type =
                        frame[kTransportLength + 24];
                    const uint16_t counter =
                        readLe16(frame + kTransportLength + 2);
                    const nvt::Matrix matrix = readTouchMatrix(frame);
                    stylus_mutual.setOrdinaryMatrix(matrix);
                    auto update = adapter.feed(
                        matrix, counter, frame_type, timestamp);
                    if (update.baseline_ready) {
                        if (touch)
                            touch->report({});
                        releaseActivePen();
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
                releaseActivePen();
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
    pro_gestures.reset();
    pen_p81c.reset();
    pen_m80p.reset();
    touch.reset();
    writeControl(kStylusPath, 0);
    writeControl(kControlPath, 0);
    close(stream_fd);
    return 0;
} catch (const std::exception &error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
}
