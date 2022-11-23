#include "vehicle_controller.h"

#include <functional>

using namespace std::placeholders;

VehicleController::VehicleController(const char *arduinoDevice)
{
    crawlerHAL = new CrawlerHAL(arduinoDevice);
    status = new VehicleData();

    isManualControl = false;

    // std::function<void(ResponseData *)> imuCallback = std::bind(&VehicleController::sensorIMUData, this, std::placeholders::_1);
    // crawlerHAL->addCallbackHandler(SENSOR_IMU, imuCallback);

    // std::function<void(ResponseData *)> gpsCallback = std::bind(&VehicleController::sensorGPSData, this, std::placeholders::_1);
    // crawlerHAL->addCallbackHandler(SENSOR_GPS, gpsCallback);

    stop();
}

void VehicleController::sensorIMUData(ResponseData *p)
{
    CrawlerHAL::parseData_IMU(p, status->imu);
}

void VehicleController::sensorGPSData(ResponseData *p)
{
    CrawlerHAL::parseData_GPS(p, status->gps);
}

VehicleController::~VehicleController()
{
    delete status;
    delete crawlerHAL;
}

void VehicleController::forwardIncrease(int increaseValue)
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

    if (status->forwardPower >= 0) {
        printf ("crawlerHAL->setEngineForward(%d)\n", status->forwardPower);
        crawlerHAL->setEngineForward(status->forwardPower);
    }
    else if (status->forwardPower == 0) {
        printf ("crawlerHAL->setEngineStop()\n");
        crawlerHAL->setEngineStop();
    } else {
        printf ("crawlerHAL->setEngineBackward(%d)\n", -status->forwardPower);
        crawlerHAL->setEngineBackward(-status->forwardPower);
    }
}



void VehicleController::increaseTurnLeftAngle(uint8_t increaseValue)
{
    status->sterringAngle -= increaseValue;
    crawlerHAL->setSteeringAngle(status->sterringAngle);
}

void VehicleController::increaseTurnRightAngle(uint8_t increaseValue)
{
    status->sterringAngle += increaseValue;
    crawlerHAL->setSteeringAngle(status->sterringAngle);
}

void VehicleController::setManualControl()
{
    isManualControl = true;
}

VehicleData *VehicleController::getVehicleData()
{
    if (status == nullptr) {
        return new VehicleData();
    }
    return status->clone();
}

void VehicleController::stop()
{
    crawlerHAL->setEngineStop();
    if (status != nullptr)
    {
        status->forwardPower = 0;
        status->sterringAngle = 0;
    }
}