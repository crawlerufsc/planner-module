#include "master_control_api.h"
#include <file_utils.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <resource_manager.h>
#include "../model/global_definitions.h"
#include "../communication/webrtc_api_stream.h"

MasterControlAPI *MasterControlAPI::_instance = nullptr;

void MasterControlAPI::onReceived(std::string topic, std::string payload)
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
        std::string payload = data->toJson();
        publishTo(PUBSUB_API_SENSOR_STATUS_URI, payload);
    }
}

MasterControlAPI::MasterControlAPI(const char *pubSubHost, int pubSubPort, const char *localIP) : PubSubClient(pubSubHost, pubSubPort, PUBSUB_API_MANUAL_COMMAND_URI)
{
    // this->username_pw_set(BROKER_USER, BROKER_PWD);
    this->pubSubHost = pubSubHost;
    this->pubSubPort = pubSubPort;
    this->localIP = localIP;

    std::string sdp = ResourceManager::getSingletonResource<WebRTCApiStream>()->getServiceSpd();

    std::ofstream outp;
    outp.open("/tmp/crawler_sdp_original.dat", std::ios_base::out | std::ios_base::trunc);
    outp.write(sdp.c_str(), sdp.size());
    outp.close();
}

bool MasterControlAPI::instanceInit()
{
    if (!blockUntilConnected(2000))
        return false;

    subscribeTo(PUBSUB_API_ORIGINAL_STREAM_LOGGING_REQUEST_URI);
    subscribeTo(PUBSUB_API_SEGMENTED_STREAM_LOGGING_REQUEST_URI);
    subscribeTo(PUBSUB_API_OCCUPANCYGRID_STREAM_LOGGING_REQUEST_URI);
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
