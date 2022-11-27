/**
 * Based on:
 * https://stackoverflow.com/questions/10403588/adding-opencv-processing-to-gstreamer-application
 */

// Include atomic std library
#include <atomic>

// Include gstreamer library
#include <gst/gst.h>
#include <gst/app/app.h>

// Include OpenCV library
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <stdio.h>
#include <queue>
#include <mutex>
#include "communication/network_stream_reader.h"
#include "control/vehicle_controller.h"
#include "control/manual_control_api.h"
#include "utils/file_utils.h"

#define CONTROL_DEVICE "/dev/ttyUSB0"
#define VISION_MODULE_IP "10.0.0.60"
#define VISION_MODULE_OG_REMOTE_PORT 20000
#define VISION_MODULE_OG_LOCAL_PORT 20000


void onProcess(StreamData *frame)
{
}


class CrawlerDevices
{
public:
    bool apiInitialized;
    NetworkStreamReader * reader; 

    CrawlerDevices(NetworkStreamReader * reader)
    {
        this->reader = reader;
    }

    bool tryInitializeDevices()
    {
        int initDevices = 3;
        if (!VehicleController::isAlive())
        {
            if (VehicleController::initialize(CONTROL_DEVICE))
                printf("Vehicle controller initialized\n");
            else
                initDevices--;
        }
        if (VehicleController::isAlive() && !ManualControlAPI::isAlive())
        {
            if (ManualControlAPI::initialize());
                printf("WebAPI initialized\n");
        }
        if (!reader->isConnected())
        {
            reader->requestConnectionToStreamServer();

            if (reader->isConnected())
            {
                printf("Connected to the vision module on %s:%d.\n", VISION_MODULE_IP, VISION_MODULE_OG_REMOTE_PORT);
                reader->run(false);
            }
            else
                initDevices--;
        }

        return initDevices == 3;
    }

    void printInitializeStatus() {
        printf ("Hardware Status:\n");
        printf ("[Vision Module]:\t%s\n", reader->isConnected() ?  "ok" : "?");
        printf ("[Crawler HAL]:\t%s\n", VehicleController::isAlive() ?  "ok" : "?");
        printf ("[MQTT manual control]:\t%s\n", ManualControlAPI::isAlive() ?  "ok" : "?");
    }
};

NetworkStreamReader * reader;

int main(int argc, char *argv[])
{
    bool lastCheckModulesUp = false;

    int serverPort = atoi(argv[2]);
    int localPort = atoi(argv[3]);

    reader = (new NetworkStreamReader(VISION_MODULE_IP, VISION_MODULE_OG_REMOTE_PORT, VISION_MODULE_OG_LOCAL_PORT))
                 ->withBufferSize(1)
                 ->withOnProcessCallback(onProcess);

    CrawlerDevices devices(reader);

    while (true)
    {
        if (!devices.tryInitializeDevices()) {
            devices.printInitializeStatus();
            lastCheckModulesUp = false;
        } else {
            if (!lastCheckModulesUp) {
                printf ("All modules are up and running now\n");
            }
            lastCheckModulesUp = true;
        }     

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    return 0;
}