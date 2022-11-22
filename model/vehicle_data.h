#ifndef _VEHICLE_DATA_H
#define _VEHICLE_DATA_H

#include <crawler_hal.h>

class VehicleData
{
public:
    IMUData *imu;
    GPSData *gps;
    int forwardPower;
    uint8_t frontAngle;
    uint8_t backAngle;

    VehicleData()
    {
        imu = nullptr;
        gps = nullptr;
    }

    ~VehicleData()
    {
        if (imu != nullptr)
            delete imu;

        if (gps != nullptr)
            delete gps;
    }

    VehicleData *clone() {
        VehicleData * p = new VehicleData();
        p->frontAngle = frontAngle;
        p->backAngle = backAngle;
        p->forwardPower = forwardPower;
        p->imu = imu->clone();
        p->gps = gps->clone();
    }

    const char *toJson() {
        std::stringstream ss;
        ss << "{\n";
        ss << "'forwardPower' : " << forwardPower << "\n";
        ss << "'frontAngle' : " << frontAngle << "\n";
        ss << "'backAngle' : " << backAngle << "\n";
        ss << "'imu' : " << imu.toJson() << "\n";
        ss << "'gps' : " << gps.toJson() << "\n";
        ss << "}\n";
        return ss.str().c_str();
    }
};

#endif