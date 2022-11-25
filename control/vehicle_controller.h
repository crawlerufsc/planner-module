#ifndef _VEHICLE_CONTROL_H
#define _VEHICLE_CONTROL_H

#include <crawler_hal.h>
#include <stdint.h>
#include <sstream>

#include "../model/vehicle_data.h"

class VehicleController
{
private:
    VehicleData *status;
    bool isManualControl;

    void sensorIMUData(ResponseData *p);
    void sensorGPSData(ResponseData *p);

    VehicleController(const CrawlerHAL &) = delete;
    VehicleController();

public:
    ~VehicleController();

    static VehicleController *_instance;

    static void initialize(const char *device)
    {
        CrawlerHAL::initialize(device);

        if (VehicleController::_instance != nullptr)
            delete VehicleController::_instance;

        VehicleController::_instance = new VehicleController();
    }

    static VehicleController *getInstance()
    {
        return VehicleController::_instance;
    };

    void setManualControl();
    bool forwardIncrease(int increaseValue);
    bool increaseTurnLeftAngle(uint8_t increaseValue);
    bool increaseTurnRightAngle(uint8_t increaseValue);

    VehicleData *getVehicleData();

    bool stop();
};

#endif