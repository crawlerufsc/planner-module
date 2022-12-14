#include "master_control_api.h"
#include <file_utils.h>

#include <iostream>
#include <fstream>
#include <sstream>

MasterControlAPI *MasterControlAPI::_instance = nullptr;

void MasterControlAPI::onReceived(std::string topic, std::string payload)
{
    if (topic == CAR_CMD_TOPIC)
        manualCommandProcess(payload);
    else if (topic == ORIGINAL_STREAM_LOGGING_CMD_TOPIC)
        originalStreamLogCommandProcess(payload);
}

// const char *getLogFileName(int i = 0)
// {
//     std::stringstream *ss = new std::stringstream();
//     (*ss) << "vision_log_" << i << ".mkv";
//     if (FileUtils::fileExists(ss->str()))
//         return getLogFileName(i + 1);
//     return ss->str().c_str();
// }

void MasterControlAPI::originalStreamLogCommandProcess(std::string payload)
{
    if (payload == "start")
    {
        if (this->originalStreamLogger != nullptr)
            delete this->originalStreamLogger;

        //const char *file = getLogFileName();
        this->originalStreamLogger = (new NetworkStreamLogger("/home/cristiano/original_vision_output.mkv", pubSubHost, pubSubPort, localIP, 21001))
            ->withStreamRequestUri("/vision-module/cmd/original");

        this->originalStreamLogger->requestStreamStart();

        printf ("Start logging original image to file: ~/original_vision_output.mkv\n");

    } else {
        if (this->originalStreamLogger != nullptr) {
            printf ("Stop logging original image\n");
            this->originalStreamLogger->requestStreamStop();
            delete this->originalStreamLogger;
            this->originalStreamLogger = nullptr;
        } else {
            printf ("Requested to stop logging but we're not logging anything\n");
        }       
    }
}

void MasterControlAPI::manualCommandProcess(std::string payload)
{
    switch (payload[0])
    {
    case CMD_STOP:
        VehicleController::getInstance()->stop();
        break;
    case CMD_RST:
        VehicleController::getInstance()->reset();
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
    case CMD_SET_SPEED_FORWARD:
        VehicleController::getInstance()->setSpeedForward((u_char)payload[1]);
        break;
    case CMD_SET_SPEED_BACKWARD:
        VehicleController::getInstance()->setSpeedBackward((u_char)payload[1]);
        break;
    case CMD_SET_STEERING_RIGHT:
    {
        int angle = (int)payload[1];
        VehicleController::getInstance()->setSteeringAngle(angle);
    }
    break;
    case CMD_SET_STEERING_LEFT:
    {
        int angle = -1 * (int)payload[1];
        VehicleController::getInstance()->setSteeringAngle(angle);
    }
    break;
    }
}

void MasterControlAPI::statusPublish()
{
    while (isConnected())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        VehicleData *data = VehicleController::getInstance()->getVehicleData();
        const char *payload = data->toJson();
        publishTo(CAR_STATUS_TOPIC, std::string(payload));
    }
}

MasterControlAPI::MasterControlAPI(const char *pubSubHost, int pubSubPort, const char *localIP) : PubSubClient(pubSubHost, pubSubPort, CAR_CMD_TOPIC)
{
    // this->username_pw_set(BROKER_USER, BROKER_PWD);
    this->originalStreamLogger = nullptr;
    this->pubSubHost = pubSubHost;
    this->pubSubPort = pubSubPort;
    this->localIP = localIP;

    original = new WebRTCService<u_char>();
    std::string sdp = original->getSdpService();

    std::ofstream outp;
    outp.open("/tmp/crawler_sdp_original.dat", std::ios_base::out | std::ios_base::trunc);
    outp.write(sdp.c_str(), sdp.size());
    outp.close();
}

bool MasterControlAPI::instanceInit()
{
    if (!blockUntilConnected(2000))
        return false;

    subscribeTo(ORIGINAL_STREAM_CMD_TOPIC);
    subscribeTo(ORIGINAL_STREAM_LOGGING_CMD_TOPIC);
    return true;
}

VehicleData *MasterControlAPI::getVehicleData()
{
    return VehicleController::getInstance()->getVehicleData();
}

bool MasterControlAPI::isAlive()
{
    return MasterControlAPI::getInstance() != nullptr;
}

void MasterControlAPI::onStop()
{
}
