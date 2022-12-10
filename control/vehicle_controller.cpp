#include "vehicle_controller.h"

#include <functional>

using namespace std::placeholders;

VehicleController *VehicleController::_instance = nullptr;

VehicleController::VehicleController()
{
    status = new VehicleData();

    isManualControl = false;

    std::function<void(ResponseData *)> imuCallback = std::bind(&VehicleController::sensorIMUData, this, std::placeholders::_1);
    CrawlerHAL::getInstance()->addCallbackHandler(SENSOR_IMU, imuCallback);

    std::function<void(ResponseData *)> gpsCallback = std::bind(&VehicleController::sensorGPSData, this, std::placeholders::_1);
    CrawlerHAL::getInstance()->addCallbackHandler(SENSOR_GPS, gpsCallback);
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
}

bool VehicleController::forwardIncrease(int increaseValue)
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
    {
        printf("crawlerHAL.setEngineForward(%d)\n", status->forwardPower);
        return CrawlerHAL::getInstance()->setEngineForward(status->forwardPower);
    }
    else if (status->forwardPower == 0)
    {
        printf("crawlerHAL.setEngineStop()\n");
        return CrawlerHAL::getInstance()->setEngineStop();
    }
    else
    {
        printf("crawlerHAL.setEngineBackward(%d)\n", -status->forwardPower);
        return CrawlerHAL::getInstance()->setEngineBackward(-status->forwardPower);
    }
}

bool VehicleController::increaseTurnLeftAngle(uint8_t increaseValue)
{
    status->sterringAngle -= increaseValue;
    return CrawlerHAL::getInstance()->setSteeringAngle(status->sterringAngle);
}

bool VehicleController::increaseTurnRightAngle(uint8_t increaseValue)
{
    status->sterringAngle += increaseValue;
    return CrawlerHAL::getInstance()->setSteeringAngle(status->sterringAngle);
}

void VehicleController::setManualControl()
{
    isManualControl = true;
}

VehicleData *VehicleController::getVehicleData()
{
    if (status == nullptr)
    {
        return new VehicleData();
    }
    return status->clone();
}

bool VehicleController::stop()
{
    if (status != nullptr)
    {
        status->forwardPower = 0;
        status->sterringAngle = 0;
    }
    return CrawlerHAL::getInstance()->setEngineStop();
}

bool VehicleController::reset()
{
    if (status != nullptr)
    {
        status->forwardPower = 0;
        status->sterringAngle = 0;
    }
    return CrawlerHAL::getInstance()->reset();
}

bool VehicleController::isAlive()
{
    CrawlerHAL *hal = CrawlerHAL::getInstance();
    if (hal == nullptr)
        return false;

    return hal->deviceExists();
}

bool VehicleController::setSpeedForward(u_char value)
{
    if (value == 0)
        return CrawlerHAL::getInstance()->setEngineStop();

    return CrawlerHAL::getInstance()->setEngineForward(value);
}

bool VehicleController::setSpeedBackward(u_char value)
{
    if (value == 0)
        return CrawlerHAL::getInstance()->setEngineStop();

    return CrawlerHAL::getInstance()->setEngineBackward(value);
}

bool VehicleController::setSteeringAngle(int angle)
{
    return CrawlerHAL::getInstance()->setSteeringAngle(angle);
}
