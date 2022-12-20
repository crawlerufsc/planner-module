#ifndef _VEHICLE_CONTROL_H
#define _VEHICLE_CONTROL_H

#include <crawler_hal.h>
#include <stdint.h>
#include <sstream>
#include <mutex>

#include "../model/vehicle_data.h"

class VehicleController
{
private:
    VehicleData *status;
    bool isManualControl;
    std::mutex *mtx;
    VehicleController();
    std::function<void(IMUData *)> fIMU;
    std::function<void(GPSData *)> fGPS;

public:
    ~VehicleController();

    static VehicleController *_instance;

    static bool initialize(const char *device)
    {
        if (!CrawlerHAL::initialize(device))
        {
            return false;
        }

        if (VehicleController::_instance != nullptr)
            delete VehicleController::_instance;

        VehicleController::_instance = new VehicleController();
        return true;
    }

    static VehicleController *getInstance()
    {
        return VehicleController::_instance;
    };

    void setManualControl();
    bool forwardIncrease(int increaseValue);
    bool increaseTurnLeftAngle(uint8_t increaseValue);
    bool increaseTurnRightAngle(uint8_t increaseValue);
    bool reset();
    bool setSpeedForward(u_char value);
    bool setSpeedBackward(u_char value);
    bool setSteeringAngle(int angle);

    VehicleData *getVehicleData();

    bool stop();
    static bool isAlive();
};

#endif