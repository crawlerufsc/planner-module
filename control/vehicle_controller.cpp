#include "vehicle_controller.h"

#include <iostream>

VehicleController *VehicleController::_instance = nullptr;

VehicleController::VehicleController()
{
    status = new VehicleData();
    mtx = new std::mutex();

    isManualControl = false;

    fIMU = [this](IMUData *p) { //
        mtx->lock();
        if (this->status->imu != nullptr)
            delete this->status->imu;
        this->status->imu = p;
        mtx->unlock();
    };

    fGPS = [this](GPSData *p) { //
        mtx->lock();
        if (this->status->gps != nullptr)
            delete this->status->gps;
        this->status->gps = p;
        mtx->unlock();
    };

    CrawlerHAL::getInstance()->addIMUCallbackHandler(fIMU);
    CrawlerHAL::getInstance()->addGPSCallbackHandler(fGPS);
}

VehicleController::~VehicleController()
{
    CrawlerHAL::getInstance()->removeIMUCallbackHandler();
    CrawlerHAL::getInstance()->removeGPSCallbackHandler();
    delete status;
}

bool VehicleController::forwardIncrease(int increaseValue)
{
    mtx->lock();
    status->forwardPower += increaseValue;
    if (status->forwardPower > 250)
    {
        status->forwardPower = 250;
    }
    if (status->forwardPower < -250)
    {
        status->forwardPower = -250;
    }
    mtx->unlock();

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
    mtx->lock();
    status->sterringAngle -= increaseValue;
    mtx->unlock();
    return CrawlerHAL::getInstance()->setSteeringAngle(status->sterringAngle);
}

bool VehicleController::increaseTurnRightAngle(uint8_t increaseValue)
{
    mtx->lock();
    status->sterringAngle += increaseValue;
    mtx->unlock();
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
    mtx->lock();
    VehicleData *p = status->clone();
    mtx->unlock();
    return p;
}

bool VehicleController::stop()
{
    mtx->lock();
    status->forwardPower = 0;
    status->sterringAngle = 0;
    mtx->unlock();
    return CrawlerHAL::getInstance()->setEngineStop();
}

bool VehicleController::reset()
{
    mtx->lock();
    status->forwardPower = 0;
    status->sterringAngle = 0;
    mtx->unlock();
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
