#pragma once

#include "communication/uart.hpp"

class ToStm {
public:
    explicit ToStm(UART& uart);

    bool sendCommand(int cmd, int value, bool quiet = false);

    bool sendSpeed(int value);
    bool sendAngle(int value);
    bool sendMode(int value);
    bool probeStop();

    bool sendStop();
    bool sendStart();
    bool sendRestart();
    bool sendTurnback();

private:
    UART& uart_;
};
