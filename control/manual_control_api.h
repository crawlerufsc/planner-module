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

#define CMD_INCREASE_SPEED 1
#define CMD_DECREASE_SPEED 2
#define CMD_INC_TURN_RIGHT 3
#define CMD_INC_TURN_LEFT 4
#define CMD_RST 5

class ManualControlAPI : public mosqpp::mosquittopp
{
private:
    char *host;
    int port;
    VehicleController *vehicleController;
    std::thread *statusPublishThr;
    std::thread *runMqttLoopThr;
    
    unsigned int messageId;
    bool isConnected;
    bool isRunning;

    int getNextMessageId()
    {
        if (messageId > 1000000)
            messageId = -1;
        return ++messageId;
    }

    void on_message(const struct mosquitto_message *message)
    {
        if (!strcmp(message->topic, CAR_CMD_TOPIC))
        {
            uint8_t *msg = (uint8_t *)malloc(message->payloadlen);
            memcpy(msg, message->payload, message->payloadlen);

            if (message->payloadlen < 1)
                return;

            switch (msg[0])
            {
            case CMD_RST:
                vehicleController->stop();
                break;
            case CMD_INCREASE_SPEED:
                vehicleController->forwardIncrease(25);
                break;
            case CMD_DECREASE_SPEED:
                vehicleController->forwardIncrease(-25);
                break;
            case CMD_INC_TURN_LEFT:
                vehicleController->increaseTurnLeftAngle(5);
                break;
            case CMD_INC_TURN_RIGHT:
                vehicleController->increaseTurnRightAngle(5);
                break;
            default:
                break;
            }
        }
    }

    void on_disconnect(int rc)
    {
        isConnected = false;
        statusPublishThr->join();
        delete statusPublishThr;
        connect(this->host, this->port, 60);
    }

    void statusPublish() {
        while (isConnected) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            VehicleData * data = vehicleController->getVehicleData();
            const char * payload = data->toJson();
            int messageId = getNextMessageId();
            publish(&messageId, CAR_STATUS_TOPIC, strlen(payload), payload, 1);
        }
    }

    void on_connect(int rc)
    {
        if (!rc)
        {
            isConnected = true;
#ifdef DEBUG
            std::cout << "Connected - code " << rc << std::endl;
#endif
        }
        int id = getNextMessageId();
        this->subscribe(&id, CAR_CMD_TOPIC, 1);
        statusPublishThr = new std::thread(&ManualControlAPI::statusPublish, this);
    }

    void runMqttLoop() {
        loop();
    }

public:
    ManualControlAPI(VehicleController *vehicleController)
    {
        this->vehicleController = vehicleController;
        this->host = strdup(BROKER_HOST);
        this->port = BROKER_PORT;
        this->messageId = -1;
        this->isConnected = false;
        this->isRunning = false;
        //this->username_pw_set(BROKER_USER, BROKER_PWD);
    }

    void initialize()
    {
        connect(this->host, this->port, 60);
        runMqttLoopThr = new std::thread(&ManualControlAPI::runMqttLoop, this);
    }

};
