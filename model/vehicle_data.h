#ifndef _VEHICLE_DATA_H
#define _VEHICLE_DATA_H

#include <crawler_hal.h>
#include <string.h>

class VehicleData
{
public:
    IMUData *imu;
    GPSData *gps;
    int forwardPower;
    int sterringAngle;

    VehicleData()
    {
        forwardPower = 0;
        sterringAngle = 0;
        imu = nullptr;
        gps = nullptr;

        imu = new IMUData();
        gps = new GPSData();
    }

    ~VehicleData()
    {
        if (imu != nullptr)
            delete imu;

        if (gps != nullptr)
            delete gps;
    }

    VehicleData *clone()
    {
        VehicleData *p = new VehicleData();
        p->sterringAngle = sterringAngle;
        p->forwardPower = forwardPower;
        if (imu != nullptr)
            p->imu = imu->clone();
        if (gps != nullptr)
            p->gps = gps->clone();
        return p;
    }

    std::string toJson()
    {
        std::stringstream *ss = new std::stringstream();
        ;
        *ss << "{\n";
        *ss << "'forwardPower' : " << forwardPower << ",\n";
        *ss << "'sterringAngle' : " << sterringAngle << ",\n";

        if (imu != nullptr)
            *ss << "'imu' : " << imu->toJson() << ",\n";
        else
            *ss << "'imu' : {},\n";

        if (gps != nullptr)
            *ss << "'gps' : " << gps->toJson() << ",\n";
        else
            *ss << "'gps' : {}\n";

        *ss << "}\n";

        return ss->str();
    }
};

#endif