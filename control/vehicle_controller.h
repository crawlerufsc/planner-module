#ifndef _VEHICLE_CONTROL_H
#define _VEHICLE_CONTROL_H

#include <crawler_hal.h>
#include <stdint.h>
#include <sstream>

#include "../model/vehicle_data.h"

class VehicleController
{
private:
    CrawlerHAL *crawlerHAL;
    VehicleData *status;
    bool isManualControl;

public:
    VehicleController();
    ~VehicleController();

    void setManualControl();
    void forwardIncrease(uint8_t increaseValue);
    void increaseTurnLeftAngle(uint8_t increaseValue);
    void increaseTurnRightAngle(uint8_t increaseValue);
    void sensorIMUData(ResponseData *p);
    void sensorGPSData(ResponseData *p);

    VehicleData *getVehicleData();

    void stop();
};

#endif