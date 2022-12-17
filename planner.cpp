// Include OpenCV library
#include <network_stream_reader.h>
#include <resource_manager.h>
#include "control/hardware_keepalive.h"
#include "model/global_definitions.h"
#include "model/vehicle_data.h"
#include "control/vehicle_controller.h"

void onProcess(StreamData *frame)
{
    printf("new frame: size: %d, format: %d x %d\n", frame->len, frame->width, frame->height);

    // VehicleData *sensorData = (ResourceManager::getSingletonResource<VehicleController>())
    //                               ->getVehicleData();
    // if (sensorData == nullptr) {
    //     printf("sensor data is null");
    // } else 
    //     printf("Sensor data: \n%s\n", sensorData->toJson());

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

int main(int argc, char *argv[])
{
    HardwareKeepAlive::initResources();

    // Set on process callback
    NetworkStreamReader *ogStreamReader = ResourceManager::getSingletonResource<NetworkStreamReader>(RESOURCE_NAME_STREAM_READER_OCCUPANCYGRID);
    ogStreamReader->withOnProcessCallback(onProcess);

    // run hardware keep alive loop
    ResourceManager::usingResourceScope<HardwareKeepAlive>([=](HardwareKeepAlive *hw) { //
        hw->loop_forever();
    });
    return 0;
}
