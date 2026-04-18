#include "control/to_stm.hpp"
#include "common/logger.hpp"

#include <cmath>
#include <cstdio>
#include <string>

ToStm::ToStm(UART& uart)
    : uart_(uart) {}

bool ToStm::sendCommand(int cmd, int value, bool quiet) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "#C%d=%d!", cmd, value);

    const bool ok = uart_.send_string(buf, quiet);
    if (!ok) {
        if (!quiet) {
            log_warn("TO_STM", std::string("send fail: ") + buf);
        }
        return false;
    }

    if (!quiet) {
        log_info("TO_STM", std::string("send ok: ") + buf);
    }
    return true;
}

bool ToStm::sendCommandWaitOk(int cmd, int value, bool quiet, int timeout_ms) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "#C%d=%d!", cmd, value);

    const bool ok = uart_.send_string_wait_ok(buf, quiet, timeout_ms);
    if (!ok) {
        if (!quiet) {
            log_warn("TO_STM", std::string("send fail waiting ok: ") + buf);
        }
        return false;
    }

    if (!quiet) {
        log_info("TO_STM", std::string("send ok ack: ") + buf);
    }
    return true;
}

bool ToStm::sendSpeed(int value) {
    const int stm32_value = static_cast<int>(std::ceil(static_cast<double>(value) / 0.89));
    return sendCommand(1, stm32_value);
}

bool ToStm::sendAngle(int value) {
    return sendCommand(2, value);
}

bool ToStm::sendMode(int value) {
    return sendCommand(3, value);
}

bool ToStm::sendStop() {
    return sendCommand(3, 0);
}

bool ToStm::sendStart() {
    return sendCommand(3, 1);
}

bool ToStm::sendRestart() {
    return sendCommand(3, 2);
}

bool ToStm::sendTurnback() {
    return sendCommand(3, 3);
}

bool ToStm::sendEmergencyStop() {
    return sendCommand(3, 4);
}

bool ToStm::sendEmergencyRelease() {
    return sendCommand(3, 5);
}

bool ToStm::sendSpeedWaitOk(int value) {
    const int stm32_value = static_cast<int>(std::ceil(static_cast<double>(value) / 0.89));
    return sendCommandWaitOk(1, stm32_value);
}

bool ToStm::sendAngleWaitOk(int value) {
    return sendCommandWaitOk(2, value);
}

bool ToStm::sendStopWaitOk() {
    return sendCommandWaitOk(3, 0, false, 1500);
}

bool ToStm::sendStartWaitOk() {
    return sendCommandWaitOk(3, 1, false, 1500);
}
