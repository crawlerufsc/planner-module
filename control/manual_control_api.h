#include <mosquittopp.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <chrono>
#include "vehicle_controller.h"

#define BROKER_HOST "127.0.0.1"
#define BROKER_PORT 1883
#define BROKER_USER "crawler"
#define BROKER_PWD "435FKDVpp48ddf"

#define CAR_STATUS_TOPIC "/crawler/status"
#define CAR_CMD_TOPIC "/crawler/cmd"

#define CMD_INCREASE_SPEED '1'
#define CMD_DECREASE_SPEED '2'
#define CMD_INC_TURN_RIGHT '3'
#define CMD_INC_TURN_LEFT '4'
#define CMD_STOP '5'
#define CMD_RST '6'

// https://levelup.gitconnected.com/building-an-api-in-c-with-pistache-413247535fd3
// https://github.com/pistacheio/pistache

class ManualControlAPI : public mosqpp::mosquittopp
{
private:
    char *host;
    int port;
    std::thread *statusPublishThr;
    std::thread *runMqttLoopThr;

    unsigned int messageId;
    bool isConnected;

    int getNextMessageId();
    void on_message(const struct mosquitto_message *message);
    void on_disconnect(int rc);
    void statusPublish();
    void on_connect(int rc);
    void runMqttLoop();

public:
    ManualControlAPI();

    static ManualControlAPI *_instance;

    static bool initialize()
    {
        if (ManualControlAPI::_instance != nullptr)
            delete ManualControlAPI::_instance;

        ManualControlAPI::_instance = new ManualControlAPI();

        return true;
    }

    static ManualControlAPI *getInstance()
    {
        return ManualControlAPI::_instance;
    };

    VehicleData *getVehicleData();

    bool isConnectedToMQTT() {
        return isConnected;
    }

    static bool isAlive();

};
