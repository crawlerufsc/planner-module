#include "manual_control_api.h"

ManualControlAPI *ManualControlAPI::_instance = nullptr;

int ManualControlAPI::getNextMessageId()
{
    if (messageId > 1000000)
        messageId = -1;
    return ++messageId;
}

void ManualControlAPI::on_message(const struct mosquitto_message *message)
{
#ifdef DEBUG
    printf("received message on topic %s\n", message->topic);
#endif

    if (!strcmp(message->topic, CAR_CMD_TOPIC))
    {
        char *msg = (char *)malloc(message->payloadlen);
        memcpy(msg, message->payload, message->payloadlen);

        if (message->payloadlen < 1)
            return;

        switch (msg[0])
        {
        case CMD_RST:
            VehicleController::getInstance()->stop();
            break;
        case CMD_INCREASE_SPEED:
            VehicleController::getInstance()->forwardIncrease(25);
            break;
        case CMD_DECREASE_SPEED:
            VehicleController::getInstance()->forwardIncrease(-25);
            break;
        case CMD_INC_TURN_LEFT:
            VehicleController::getInstance()->increaseTurnLeftAngle(5);
            break;
        case CMD_INC_TURN_RIGHT:
            VehicleController::getInstance()->increaseTurnRightAngle(5);
            break;
        default:
            break;
        }
    }
}

void ManualControlAPI::on_disconnect(int rc)
{
    isConnected = false;
    statusPublishThr->join();
    delete statusPublishThr;
    connect(this->host, this->port, 60);
}

void ManualControlAPI::statusPublish()
{
    while (isConnected)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        VehicleData *data = VehicleController::getInstance()->getVehicleData();
        const char *payload = data->toJson();
        int messageId = getNextMessageId();
        publish(&messageId, CAR_STATUS_TOPIC, strlen(payload), payload, 1);
    }
}

void ManualControlAPI::on_connect(int rc)
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

void ManualControlAPI::runMqttLoop()
{
    loop_forever();
}

ManualControlAPI::ManualControlAPI()
{
    this->host = strdup(BROKER_HOST);
    this->port = BROKER_PORT;
    this->messageId = -1;
    this->isConnected = false;
    this->isRunning = false;
    // this->username_pw_set(BROKER_USER, BROKER_PWD);

    connect_async(this->host, this->port, 60);
    runMqttLoopThr = new std::thread(&ManualControlAPI::runMqttLoop, this);
}

VehicleData *ManualControlAPI::getVehicleData()
{
    return VehicleController::getInstance()->getVehicleData();
}