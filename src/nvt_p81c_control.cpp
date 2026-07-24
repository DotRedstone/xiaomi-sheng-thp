// SPDX-License-Identifier: Apache-2.0

#include "nvt_p81c_control.hpp"

#include <array>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <gio/gio.h>
#include <iostream>
#include <span>
#include <utility>

namespace nvt {
namespace {

constexpr std::string_view kBluezDestination = "org.bluez";
constexpr std::string_view kCharacteristicInterface =
    "org.bluez.GattCharacteristic1";
constexpr std::string_view kControlUuid =
    "0000fe11-aa6c-462a-964a-7f2ed5b3e512";
constexpr const char *kReadyStatePath =
    "/run/xiaomi-sheng-thp/p81c-fe11-ready";
constexpr const char *kReadyStateTemporaryPath =
    "/run/xiaomi-sheng-thp/.p81c-fe11-ready.tmp";
constexpr auto kRetryInterval = std::chrono::seconds(1);
constexpr std::size_t kMaximumQueuedCommands = 16;

void clearReadyState() {
    std::error_code error;
    std::filesystem::remove(kReadyStatePath, error);
    error.clear();
    std::filesystem::remove(kReadyStateTemporaryPath, error);
}

bool publishReadyState(std::string_view address) {
    bool written = false;
    {
        std::ofstream output(
            kReadyStateTemporaryPath, std::ios::out | std::ios::trunc);
        if (!output)
            return false;
        output << address << '\n';
        written = output.good();
    }
    if (!written) {
        clearReadyState();
        return false;
    }
    std::error_code error;
    std::filesystem::rename(
        kReadyStateTemporaryPath, kReadyStatePath, error);
    if (!error)
        return true;
    clearReadyState();
    return false;
}

std::string bluezDeviceToken(std::string_view address) {
    if (address.size() != 17)
        return {};
    std::string token = "dev_";
    token.reserve(21);
    for (std::size_t index = 0; index < address.size(); ++index) {
        const unsigned char character =
            static_cast<unsigned char>(address[index]);
        if (index % 3 == 2) {
            if (character != ':')
                return {};
            token.push_back('_');
            continue;
        }
        if (!std::isxdigit(character))
            return {};
        token.push_back(static_cast<char>(std::toupper(character)));
    }
    return token;
}

std::string findControlCharacteristic(GDBusConnection *connection,
                                      std::string_view address) {
    const std::string device_token = bluezDeviceToken(address);
    if (device_token.empty())
        return {};

    GError *error = nullptr;
    GVariant *reply = g_dbus_connection_call_sync(
        connection, kBluezDestination.data(), "/",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects", nullptr,
        G_VARIANT_TYPE("(a{oa{sa{sv}}})"), G_DBUS_CALL_FLAGS_NONE, 1000,
        nullptr, &error);
    if (!reply) {
        g_clear_error(&error);
        return {};
    }

    std::string result;
    GVariant *objects = g_variant_get_child_value(reply, 0);
    GVariantIter iterator;
    g_variant_iter_init(&iterator, objects);
    while (GVariant *entry = g_variant_iter_next_value(&iterator)) {
        const char *path = nullptr;
        GVariant *interfaces = nullptr;
        g_variant_get(entry, "{&o@a{sa{sv}}}", &path, &interfaces);
        GVariant *properties = g_variant_lookup_value(
            interfaces, kCharacteristicInterface.data(),
            G_VARIANT_TYPE("a{sv}"));
        const char *uuid = nullptr;
        if (properties)
            g_variant_lookup(properties, "UUID", "&s", &uuid);
        const std::string_view object_path = path ? path : "";
        if (uuid && kControlUuid == uuid &&
            object_path.find(device_token) != std::string_view::npos) {
            result = object_path;
        }
        if (properties)
            g_variant_unref(properties);
        g_variant_unref(interfaces);
        g_variant_unref(entry);
        if (!result.empty())
            break;
    }
    g_variant_unref(objects);
    g_variant_unref(reply);
    return result;
}

bool writeCommand(GDBusConnection *connection, const std::string &path,
                  std::span<const unsigned char> command) {
    GVariant *value = g_variant_new_fixed_array(
        G_VARIANT_TYPE_BYTE, command.data(), command.size(), sizeof(guint8));
    GVariantBuilder options;
    g_variant_builder_init(&options, G_VARIANT_TYPE("a{sv}"));
    g_variant_builder_add(&options, "{sv}", "type",
                          g_variant_new_string("command"));

    GError *error = nullptr;
    GVariant *reply = g_dbus_connection_call_sync(
        connection, kBluezDestination.data(), path.c_str(),
        kCharacteristicInterface.data(), "WriteValue",
        g_variant_new("(@ay@a{sv})", value,
                      g_variant_builder_end(&options)),
        G_VARIANT_TYPE("()"), G_DBUS_CALL_FLAGS_NONE, 1000, nullptr, &error);
    if (!reply) {
        g_clear_error(&error);
        return false;
    }
    g_variant_unref(reply);
    return true;
}

}  // namespace

P81cAgController::~P81cAgController() {
    clearReadyState();
    if (attempt_.valid()) {
        try {
            attempt_.wait();
            attempt_.get();
        } catch (...) {
        }
    }
    const std::string address = !desired_address_.empty()
        ? desired_address_ : initialized_address_;
    if (!address.empty())
        execute(Request{RequestKind::DisableAg, address, true, {}});
}

void P81cAgController::enable(std::string_view address, bool screen_on) {
    if (address.empty())
        return;
    const std::string new_address(address);
    if (!desired_enabled_ || desired_address_ != new_address)
        clearReadyState();
    if (!desired_address_.empty() && desired_address_ != new_address)
        cleanup_address_ = desired_address_;
    if (!initialized_address_.empty() &&
        initialized_address_ != new_address) {
        cleanup_address_ = initialized_address_;
        initialized_address_.clear();
        applied_screen_on_.reset();
    }
    desired_address_ = new_address;
    desired_enabled_ = true;
    desired_screen_on_ = screen_on;
    failure_reported_ = false;
    next_attempt_ = {};
    service();
}

void P81cAgController::disable() {
    clearReadyState();
    desired_enabled_ = false;
    commands_.clear();
    if (!initialized_address_.empty())
        cleanup_address_ = initialized_address_;
    else if (!desired_address_.empty())
        cleanup_address_ = desired_address_;
    failure_reported_ = false;
    next_attempt_ = {};
    service();
}

void P81cAgController::setScreenOn(bool screen_on) {
    if (desired_screen_on_ == screen_on)
        return;
    desired_screen_on_ = screen_on;
    next_attempt_ = {};
    service();
}

void P81cAgController::vibrate(std::uint8_t type,
                               std::uint8_t amplitude) {
    if (!desired_enabled_ || desired_address_.empty())
        return;
    if (commands_.size() >= kMaximumQueuedCommands)
        commands_.pop_front();
    commands_.push_back({0x5e, 0x02, type, amplitude});
    service();
}

void P81cAgController::service() {
    if (attempt_.valid()) {
        if (attempt_.wait_for(std::chrono::seconds(0)) !=
            std::future_status::ready) {
            return;
        }
        try {
            finish(attempt_.get());
        } catch (...) {
            failure_reported_ = false;
            next_attempt_ = std::chrono::steady_clock::now() + kRetryInterval;
        }
    }

    if (std::chrono::steady_clock::now() < next_attempt_)
        return;
    if (std::optional<Request> request = nextRequest())
        start(std::move(*request));
}

std::optional<P81cAgController::Request> P81cAgController::nextRequest() {
    if (cleanup_address_) {
        if (desired_enabled_ && *cleanup_address_ == desired_address_) {
            cleanup_address_.reset();
        } else {
            return Request{
                RequestKind::DisableAg, *cleanup_address_, true, {}};
        }
    }
    if (!desired_enabled_ || desired_address_.empty())
        return std::nullopt;
    if (initialized_address_ != desired_address_) {
        return Request{RequestKind::Initialize, desired_address_,
                       desired_screen_on_, {}};
    }
    if (!applied_screen_on_ || *applied_screen_on_ != desired_screen_on_) {
        return Request{RequestKind::ScreenState, desired_address_,
                       desired_screen_on_, {}};
    }
    if (!commands_.empty()) {
        Request request{
            RequestKind::Command, desired_address_, true, {}};
        request.command = std::move(commands_.front());
        commands_.pop_front();
        return request;
    }
    return std::nullopt;
}

void P81cAgController::start(Request request) {
    attempt_ = std::async(
        std::launch::async,
        [request = std::move(request)]() mutable {
            return execute(std::move(request));
        });
}

void P81cAgController::finish(Result result) {
    const bool current = result.request.address == desired_address_;
    if (result.success) {
        failure_reported_ = false;
        next_attempt_ = {};
        switch (result.request.kind) {
        case RequestKind::Initialize:
            if (current && desired_enabled_) {
                initialized_address_ = result.request.address;
                applied_screen_on_ = result.request.screen_on;
                if (!publishReadyState(initialized_address_)) {
                    std::cerr << "P81c FE11 ready state unavailable\n";
                }
                std::cerr << "P81c connection initialized; accelerometer and "
                             "gyroscope enabled\n";
            } else {
                cleanup_address_ = result.request.address;
            }
            break;
        case RequestKind::ScreenState:
            if (current && initialized_address_ == result.request.address)
                applied_screen_on_ = result.request.screen_on;
            break;
        case RequestKind::DisableAg:
            if (initialized_address_ == result.request.address) {
                initialized_address_.clear();
                applied_screen_on_.reset();
            }
            if (cleanup_address_ == result.request.address)
                cleanup_address_.reset();
            if (!desired_enabled_ && current)
                desired_address_.clear();
            break;
        case RequestKind::Command:
            break;
        }
        return;
    }

    if (!failure_reported_) {
        std::cerr << "P81c " << requestName(result.request.kind)
                  << " unavailable";
        if (result.request.kind == RequestKind::Initialize ||
            result.request.kind == RequestKind::ScreenState) {
            std::cerr << "; retrying";
        }
        std::cerr << '\n';
        failure_reported_ = true;
    }
    if (result.request.kind == RequestKind::DisableAg) {
        if (cleanup_address_ == result.request.address)
            cleanup_address_.reset();
        if (initialized_address_ == result.request.address) {
            initialized_address_.clear();
            applied_screen_on_.reset();
        }
        if (!desired_enabled_ && current)
            desired_address_.clear();
        return;
    }
    if (result.request.kind == RequestKind::Command)
        return;
    if (current)
        next_attempt_ = std::chrono::steady_clock::now() + kRetryInterval;
}

P81cAgController::Result P81cAgController::execute(Request request) {
    GError *error = nullptr;
    GDBusConnection *connection =
        g_bus_get_sync(G_BUS_TYPE_SYSTEM, nullptr, &error);
    if (!connection) {
        g_clear_error(&error);
        return {std::move(request), false};
    }
    const std::string path =
        findControlCharacteristic(connection, request.address);
    bool success = !path.empty();
    if (success) {
        switch (request.kind) {
        case RequestKind::Initialize: {
            constexpr std::array<unsigned char, 3> kDoubleTap = {
                0x5f, 0x01, 0x01};
            constexpr std::array<unsigned char, 3> kPinchMotor = {
                0x5a, 0x01, 0x03};
            constexpr std::array<unsigned char, 6> kMediumPinch = {
                0x5c, 0x04, 0x01, 0x0e, 0x00, 0x87};
            const std::array<unsigned char, 3> screen = {
                0x61, 0x01,
                static_cast<unsigned char>(request.screen_on ? 1 : 0)};
            constexpr std::array<unsigned char, 3> kAgOn = {
                0x5b, 0x01, 0x01};
            const std::array<std::span<const unsigned char>, 5> commands = {
                kDoubleTap, kPinchMotor, kMediumPinch, screen, kAgOn};
            for (const auto command : commands) {
                if (!writeCommand(connection, path, command)) {
                    success = false;
                    break;
                }
            }
            break;
        }
        case RequestKind::ScreenState: {
            const std::array<unsigned char, 3> command = {
                0x61, 0x01,
                static_cast<unsigned char>(request.screen_on ? 1 : 0)};
            success = writeCommand(connection, path, command);
            break;
        }
        case RequestKind::DisableAg: {
            constexpr std::array<unsigned char, 3> command = {
                0x5b, 0x01, 0x00};
            success = writeCommand(connection, path, command);
            break;
        }
        case RequestKind::Command:
            success = writeCommand(connection, path, request.command);
            break;
        }
    }
    g_object_unref(connection);
    return {std::move(request), success};
}

const char *P81cAgController::requestName(RequestKind kind) {
    switch (kind) {
    case RequestKind::Initialize:
        return "initialization";
    case RequestKind::ScreenState:
        return "screen-state update";
    case RequestKind::DisableAg:
        return "A+G disable";
    case RequestKind::Command:
        return "haptic command";
    }
    return "request";
}

}  // namespace nvt
