// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <future>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace nvt {

class P81cAgController {
public:
    P81cAgController() = default;
    P81cAgController(const P81cAgController &) = delete;
    P81cAgController &operator=(const P81cAgController &) = delete;
    ~P81cAgController();

    void enable(std::string_view address, bool screen_on);
    void disable();
    void setScreenOn(bool screen_on);
    void vibrate(std::uint8_t type, std::uint8_t amplitude);
    void service();

private:
    enum class RequestKind {
        Initialize,
        ScreenState,
        DisableAg,
        Command,
    };

    struct Request {
        RequestKind kind = RequestKind::Initialize;
        std::string address;
        bool screen_on = true;
        std::vector<unsigned char> command;
    };

    struct Result {
        Request request;
        bool success = false;
    };

    std::string desired_address_;
    std::string initialized_address_;
    std::optional<std::string> cleanup_address_;
    std::optional<bool> applied_screen_on_;
    bool desired_enabled_ = false;
    bool desired_screen_on_ = true;
    bool failure_reported_ = false;
    std::chrono::steady_clock::time_point next_attempt_{};
    std::deque<std::vector<unsigned char>> commands_;
    std::future<Result> attempt_;

    std::optional<Request> nextRequest();
    void start(Request request);
    void finish(Result result);
    static const char *requestName(RequestKind kind);
    static Result execute(Request request);
};

}  // namespace nvt
