#include "short_term_planner.h"

ShortTermPlannerEngine::ShortTermPlannerEngine()
{
    onProcess = new std::function<void(Frame<u_char> *)>([this](Frame<u_char> *frame) { // 0fGPS
        VehicleData *sensorData = VehicleController::getInstance()->getVehicleData();
        this->onShortTermPlan(frame, sensorData);
    });
}
ShortTermPlannerEngine::~ShortTermPlannerEngine()
{
    delete onProcess;
}

void ShortTermPlannerEngine::run()
{
    HardwareKeepAlive::initResources();

    // Set on process callback
    (ResourceManager::getSingletonResource<NetworkStreamReader>(RESOURCE_NAME_STREAM_READER_OCCUPANCYGRID))
        ->withOnProcessCallback(onProcess);
    // run hardware keep alive loop
    ResourceManager::usingResourceScope<HardwareKeepAlive>([=](HardwareKeepAlive *hw) { //
        hw->loop_forever();
    });
}
