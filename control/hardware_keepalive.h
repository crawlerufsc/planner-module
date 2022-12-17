#ifndef _HARDWARE_KEEPALIVE_H
#define _HARDWARE_KEEPALIVE_H

class HardwareKeepAlive
{
private:
    bool tryInitializeHardware();
    void printInitializeStatus();

public:
    static void initResources();
    void loop_forever();
};

#endif