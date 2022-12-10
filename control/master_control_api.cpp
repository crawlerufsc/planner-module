#include "master_control_api.h"

#include <iostream>
#include <fstream>

ManualControlAPI *ManualControlAPI::_instance = nullptr;

void ManualControlAPI::onReceived(std::string topic, std::string payload)
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


void ManualControlAPI::statusPublish()
{
    while (isConnected())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        VehicleData *data = VehicleController::getInstance()->getVehicleData();
        const char *payload = data->toJson();
        publishTo(CAR_STATUS_TOPIC, std::string(payload));
    }
}

ManualControlAPI::ManualControlAPI(const char *pubSubHost, int pubSubPort) : PubSubClient(pubSubHost,pubSubPort, CAR_CMD_TOPIC)
{
    // this->username_pw_set(BROKER_USER, BROKER_PWD);

    original = new WebRTCService<u_char>();
    std::string sdp = original->getSdpService();

    std::ofstream outp;
    outp.open("/tmp/crawler_sdp_original.dat", std::ios_base::out | std::ios_base::trunc);
    outp.write(sdp.c_str(), sdp.size());
    outp.close();
}

VehicleData *ManualControlAPI::getVehicleData()
{
    return VehicleController::getInstance()->getVehicleData();
}

bool ManualControlAPI::isAlive()
{
    return ManualControlAPI::getInstance() != nullptr;
}

void ManualControlAPI::onStop() {

}
