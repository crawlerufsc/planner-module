#include "vehicle_controller.h"

#include <functional>

using namespace std::placeholders;

VehicleController::VehicleController()
{
    crawlerHAL = new CrawlerHAL("/dev/ttyUSB0");
    isManualControl = false;

    std::function<void(ResponseData *)> imuCallback = std::bind(&VehicleController::sensorIMUData, this, std::placeholders::_1);
    crawlerHAL->addCallbackHandler(SENSOR_IMU, imuCallback);

    std::function<void(ResponseData *)> gpsCallback = std::bind(&VehicleController::sensorGPSData, this, std::placeholders::_1);
    crawlerHAL->addCallbackHandler(SENSOR_GPS, gpsCallback);

    stop();
}

void VehicleController::sensorIMUData(ResponseData *p)
{
    if (status->imu != nullptr)
        delete status->imu;

    status->imu = CrawlerHAL::parseData_IMU(p);
}

void VehicleController::sensorGPSData(ResponseData *p)
{
    if (status->gps != nullptr)
        delete status->gps;

    status->gps = CrawlerHAL::parseData_GPS(p);
}

VehicleController::~VehicleController()
{
    delete status;
    delete crawlerHAL;
}

void VehicleController::forwardIncrease(uint8_t increaseValue)
{
    status->forwardPower += increaseValue;
    if (status->forwardPower > 250)
    {
        status->forwardPower = 250;
    }
    if (status->forwardPower < -250)
    {
        status->forwardPower = -250;
    }

    if (status->forwardPower >= 0)
        crawlerHAL->setEngineForward(status->forwardPower);
    else if (status->forwardPower == 0)
        crawlerHAL->setEngineStop();
    else
        crawlerHAL->setEngineBackward(-status->forwardPower);
}

void VehicleController::increaseTurnLeftAngle(uint8_t increaseValue)
{
    status->frontAngle -= increaseValue;

    if (status->frontAngle < 0)
        status->frontAngle = 0;
    if (status->frontAngle > 90)
        status->frontAngle = 90;

    status->backAngle += increaseValue;

    if (status->backAngle < 0)
        status->backAngle = 0;
    if (status->backAngle > 90)
        status->backAngle = 90;

    crawlerHAL->setWheelFrontAngle(status->frontAngle);
    crawlerHAL->setWheelBackAngle(status->backAngle);
}

void VehicleController::increaseTurnRightAngle(uint8_t increaseValue)
{
    status->frontAngle += increaseValue;

    if (status->frontAngle < 0)
        status->frontAngle = 0;
    if (status->frontAngle > 90)
        status->frontAngle = 90;

    status->backAngle -= increaseValue;

    if (status->backAngle < 0)
        status->backAngle = 0;
    if (status->backAngle > 90)
        status->backAngle = 90;

    crawlerHAL->setWheelFrontAngle(status->frontAngle);
    crawlerHAL->setWheelBackAngle(status->backAngle);
}

void VehicleController::setManualControl()
{
    isManualControl = true;
}

VehicleData *VehicleController::getVehicleData()
{
    return status->clone();
}

void VehicleController::stop()
{
    crawlerHAL->setEngineStop();
    status->forwardPower = 0;
    status->frontAngle = 45;
    status->backAngle = 45;
}