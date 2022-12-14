#include <pubsub_client.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <chrono>
#include "vehicle_controller.h"
#include "../communication/webrtc.h"
#include <network_stream_logger.h>

#define CAR_STATUS_TOPIC "/crawler/status"
#define CAR_CMD_TOPIC "/crawler/cmd"
#define ORIGINAL_STREAM_CMD_TOPIC "/vision-module/cmd"
#define ORIGINAL_STREAM_LOGGING_CMD_TOPIC "/stream/original/log"

#define CMD_INCREASE_SPEED '1'
#define CMD_DECREASE_SPEED '2'
#define CMD_INC_TURN_RIGHT '3'
#define CMD_INC_TURN_LEFT '4'
#define CMD_STOP '5'
#define CMD_RST '6'
#define CMD_SET_SPEED_FORWARD '7'
#define CMD_SET_SPEED_BACKWARD '8'
#define CMD_SET_STEERING_RIGHT '9'
#define CMD_SET_STEERING_LEFT 'A'

#define DEBUG 1

// https://levelup.gitconnected.com/building-an-api-in-c-with-pistache-413247535fd3
// https://github.com/pistacheio/pistache

class MasterControlAPI : protected PubSubClient
{
private:
    std::thread *statusPublishThr;
    WebRTCService<u_char> *original;
    WebRTCService<u_char> *segmented;
    WebRTCService<u_char> *occupancyGrid;
    NetworkStreamLogger *originalStreamLogger;

    const char *pubSubHost;
    int pubSubPort;
    const char *localIP;

    void statusPublish();

    void manualCommandProcess(std::string payload);
    void originalStreamLogCommandProcess(std::string payload);

protected:
    void onReceived(std::string topic, std::string payload) override;
    void onStop() override;

public:
    MasterControlAPI(const char *pubSubHost, int pubSubPort, const char *localIP);

    static MasterControlAPI *_instance;

    static bool initialize(const char *pubSubHost, int pubSubPort, const char *localIP)
    {
        if (MasterControlAPI::_instance != nullptr)
            delete MasterControlAPI::_instance;

        MasterControlAPI::_instance = new MasterControlAPI(pubSubHost, pubSubPort, localIP);
        
        if (!MasterControlAPI::_instance->instanceInit()) {
            delete MasterControlAPI::_instance;
            MasterControlAPI::_instance = nullptr;
            return false;
        }

        return true;
    }

    static MasterControlAPI *getInstance()
    {
        return MasterControlAPI::_instance;
    };

    VehicleData *getVehicleData();

    static bool isAlive();

    bool instanceInit();
};
