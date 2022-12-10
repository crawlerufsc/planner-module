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
#include <network_stream_reader.h>
#include "communication/webrtc_api_stream.h"
#include "control/vehicle_controller.h"
#include "control/master_control_api.h"
#include "utils/file_utils.h"

#define CONTROL_DEVICE "/dev/ttyUSB0"
#define MqttHost "10.42.0.1"
#define MqttPort 1883
#define OG_STREAM_LOCAL_IP "10.42.1.1"
#define OG_STREAM_LOCAL_Port 20003

void onProcess(StreamData *frame)
{
    printf("new frame: size: %d, format: %d x %d\n", frame->len, frame->width, frame->height);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

class CrawlerDevices
{
public:
    bool apiInitialized;
    NetworkStreamReader *occupancyGridReader;

    CrawlerDevices(NetworkStreamReader *occupancyGridReader)
    {
        this->occupancyGridReader = occupancyGridReader;
    }

    bool tryInitializeDevices()
    {
        int initDevices = 4;
        if (!VehicleController::isAlive())
        {
            if (VehicleController::initialize(CONTROL_DEVICE))
                printf("Vehicle controller initialized\n");
            else
                initDevices--;
        }
        if (VehicleController::isAlive() && !ManualControlAPI::isAlive())
        {
            if (ManualControlAPI::initialize(MqttHost, MqttPort))
                printf("WebAPI initialized\n");
        }
        if (!occupancyGridReader->isConnected())
        {
            occupancyGridReader->connect();
            initDevices--;
        }
        if (!WebRTCApiStream::isAlive())
        {
            if (WebRTCApiStream::initialize(MqttHost, MqttPort, OG_STREAM_LOCAL_IP))
                printf("WebRTC initialized\n");
            else
                initDevices--;
        }
        return initDevices == 4;
    }

    void printInitializeStatus()
    {
        printf("Hardware Status:\n");
        printf("[Vision Module]:\t%s\n", occupancyGridReader->isConnected() ? "ok" : "?");
        printf("[Crawler HAL]:\t%s\n", VehicleController::isAlive() ? "ok" : "?");
        printf("[MQTT master control]:\t%s\n", ManualControlAPI::isAlive() ? "ok" : "?");
        printf("[WebRTC streaming]:\t%s\n", WebRTCApiStream::isAlive() ? "ok" : "?");
    }
};

NetworkStreamReader *occupancyGridReader;

int main(int argc, char *argv[])
{
    bool lastCheckModulesUp = false;

    int serverPort = atoi(argv[2]);
    int localPort = atoi(argv[3]);

    occupancyGridReader = (new NetworkStreamReader(MqttHost, MqttPort, OG_STREAM_LOCAL_IP, OG_STREAM_LOCAL_Port))
                ->withStreamRequestUri("/vision-module/cmd/og")
                ->withBufferSize(1)
                ->withOnProcessCallback(onProcess)
                ->async();

    CrawlerDevices devices(occupancyGridReader);

    while (true)
    {
        if (!devices.tryInitializeDevices())
        {
            devices.printInitializeStatus();
            lastCheckModulesUp = false;
        }
        else
        {
            if (!lastCheckModulesUp)
            {
                printf("All modules are up and running now\n");
            }
            lastCheckModulesUp = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    return 0;
}