#pragma once

#include "communication/uart.hpp"

class ToStm {
public:
    explicit ToStm(UART& uart);

    bool sendCommand(int cmd, int value, bool quiet = false);
    bool sendCommandWaitOk(int cmd, int value, bool quiet = false, int timeout_ms = 300);

    bool sendSpeed(int value);
    bool sendAngle(int value);
    bool sendMode(int value);

    bool sendStop();
    bool sendStart();
    bool sendRestart();
    bool sendTurnback();
    bool sendEmergencyStop();
    bool sendEmergencyRelease();

    bool sendSpeedWaitOk(int value);
    bool sendAngleWaitOk(int value);
    bool sendStopWaitOk();
    bool sendStartWaitOk();

private:
    UART& uart_;
};
