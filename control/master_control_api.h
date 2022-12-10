#include <pubsub_client.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <chrono>
#include "vehicle_controller.h"
#include "../communication/webrtc.h"

#define CAR_STATUS_TOPIC "/crawler/status"
#define CAR_CMD_TOPIC "/crawler/cmd"

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

class ManualControlAPI : protected PubSubClient
{
private:
    std::thread *statusPublishThr;
    WebRTCService<u_char> *original;
    WebRTCService<u_char> *segmented;
    WebRTCService<u_char> *occupancyGrid;

    void statusPublish();
protected:

    void onReceived(std::string topic, std::string payload) override;
    void onStop() override;

public:
    ManualControlAPI(const char *pubSubHost, int pubSubPort);

    static ManualControlAPI *_instance;

    static bool initialize(const char *pubSubHost, int pubSubPort)
    {
        if (ManualControlAPI::_instance != nullptr)
            delete ManualControlAPI::_instance;

        ManualControlAPI::_instance = new ManualControlAPI(pubSubHost, pubSubPort);

        return true;
    }

    static ManualControlAPI *getInstance()
    {
        return ManualControlAPI::_instance;
    };

    VehicleData *getVehicleData();

    static bool isAlive();

};
