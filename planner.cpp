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

#define CONTROL_DEVICE "/dev/ttyUSB0"

NetworkStreamReader *reader;


void onProcess(StreamData * frame) {

}

void controlInitialize() {
    VehicleController::initialize(CONTROL_DEVICE);
    ManualControlAPI::initialize();
}

int main(int argc, char *argv[])
{
    if (argc < 4) {
        fprintf(stderr, "please use %s [server IP] [server Port] [local Port]\n", argv[0]);
        return 1;
    }

    bool firstConnect = true;

    int serverPort = atoi(argv[2]);
    int localPort = atoi(argv[3]);

    reader = (new NetworkStreamReader(argv[1], serverPort, localPort))
        ->withBufferSize(1)
        ->withOnProcessCallback(onProcess);

    controlInitialize();

     while (!reader->isConnected()) {
        if (firstConnect)
            printf ("Connecting to %s:%d -> %d...\n", argv[1], serverPort, localPort);
        else
            printf ("Reconnecting...\n");

        firstConnect = false;

        reader->requestConnectionToStreamServer();
        
        if (reader->isConnected())
            reader->run();       
    }

    return 0;
}