#include "hardware_keepalive.h"
#include "model/global_definitions.h"
#include "control/master_control_api.h"
#include "communication/webrtc_api_stream.h"
#include "control/vehicle_controller.h"
#include "log/sensor_logger.h"
#include "log/video_logger.h"

#include <resource_manager.h>
#include <network_stream_reader.h>

void HardwareKeepAlive::initResources()
{
    ResourceManager::addResourceFactory<HardwareKeepAlive>([=] { //
        return new HardwareKeepAlive();
    });

    ResourceManager::addResourceFactory<NetworkStreamReader>(RESOURCE_NAME_STREAM_READER_OCCUPANCYGRID, [=] { //
        return (new NetworkStreamReader(BROKER_IP, BROKER_PORT, PLANNER_IP, LOCALPORT_STREAM_READER_OCCUPANCYGRID))
            ->withStreamRequestUri(PUBSUB_STREAM_REQUEST_URI_OCCUPANCYGRID)
            ->withBufferSize(1)
            ->async();
    });

    ResourceManager::addResourceFactory<VehicleController>([=] { //
        VehicleController::initialize(HW_CONTROLLER_USB_DEVICE);
        return VehicleController::getInstance();
    });

    ResourceManager::addResourceFactory<MasterControlAPI>([=] { //
        MasterControlAPI::initialize(BROKER_IP, BROKER_PORT, PLANNER_IP);
        return MasterControlAPI::getInstance();
    });

    ResourceManager::addResourceFactory<SensorLogger>([=] { //
        return new SensorLogger(FILE_LOG_SENSORS);
    });

    ResourceManager::getSingletonResource<SensorLogger>();
    // const char *file = getLogFileName();


    ResourceManager::addResourceFactory<VideoLogger>(RESOURCE_NAME_STREAM_LOGGER_ORIGINAL, [=] { //
        return new VideoLogger(RESOURCE_NAME_STREAM_LOGGER_ORIGINAL, PUBSUB_API_ORIGINAL_STREAM_LOGGING_REQUEST_URI);
    });

    ResourceManager::addResourceFactory<VideoLogger>(RESOURCE_NAME_STREAM_LOGGER_SEGMENTED, [=] { //
        return new VideoLogger(RESOURCE_NAME_STREAM_LOGGER_SEGMENTED, PUBSUB_API_SEGMENTED_STREAM_LOGGING_REQUEST_URI);
    });

    ResourceManager::addResourceFactory<VideoLogger>(RESOURCE_NAME_STREAM_LOGGER_OCCUPANCYGRID, [=] { //
        return new VideoLogger(RESOURCE_NAME_STREAM_LOGGER_OCCUPANCYGRID, PUBSUB_API_OCCUPANCYGRID_STREAM_LOGGING_REQUEST_URI);
    });

    ResourceManager::getSingletonResource<VideoLogger>(RESOURCE_NAME_STREAM_LOGGER_ORIGINAL);
    ResourceManager::getSingletonResource<VideoLogger>(RESOURCE_NAME_STREAM_LOGGER_SEGMENTED);
    ResourceManager::getSingletonResource<VideoLogger>(RESOURCE_NAME_STREAM_LOGGER_OCCUPANCYGRID);

}

bool HardwareKeepAlive::tryInitializeHardware()
{
    bool result = true;

    if (VehicleController::isAlive())
    {
        if (ResourceManager::getSingletonResource<MasterControlAPI>() == nullptr)
            result = false;
    }
    else
    {
        VehicleController::initialize(HW_CONTROLLER_USB_DEVICE);
        result = false;
    }

    NetworkStreamReader *reader = ResourceManager::getSingletonResource<NetworkStreamReader>(RESOURCE_NAME_STREAM_READER_OCCUPANCYGRID);

    if (!reader->isConnected())
    {
        reader->connect();
        result = false;
    }

    return result;
}
void HardwareKeepAlive::printInitializeStatus()
{
    printf("Hardware Status:\n");
    printf("----------------\n");
    printf("[Vision Module]:\t%s\n", (ResourceManager::getSingletonResource<NetworkStreamReader>(RESOURCE_NAME_STREAM_READER_OCCUPANCYGRID))->isConnected() ? "ok" : "?");
    printf("[Crawler HAL]: \t%s\n", VehicleController::isAlive() ? "ok" : "?");
    printf("[Master control API]:\t%s\n\n", MasterControlAPI::isAlive() ? "ok" : "?");
}

void HardwareKeepAlive::loop_forever()
{
    initResources();
    bool lastCheckModulesUp = false;

    while (true)
    {
        if (tryInitializeHardware())
        {
            if (!lastCheckModulesUp) {
                printInitializeStatus();
                printf("All modules are up and running now\n");
            }

            lastCheckModulesUp = true;
        }
        else
        {
            lastCheckModulesUp = false;
            printInitializeStatus();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
}
