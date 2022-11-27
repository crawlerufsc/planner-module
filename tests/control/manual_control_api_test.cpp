#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <mosquittopp.h>
#include "../../control/vehicle_controller.h"
#include "../../control/manual_control_api.h"
#include "../../model/vehicle_data.h"
#include "../../utils/file_utils.h"

char menu(VehicleData *data)
{
    std::string hs;
    std::string ms;
    std::string ack;

    if (data != nullptr && data->sterringAngle < 0)
        hs = "[-]";
    else
        hs = "[+]";

    if (data != nullptr && data->forwardPower < 0)
        ms = "[-]";
    else
        ms = "[+]";

    // system("clear");
    printf("Hardware testing menu\n\n");
    printf("                  (w)       forward pwr++\n");
    printf("left pwr++   (a)       (d)      right pwr++\n");
    printf("                  (s)       backward pwr++\n");
    printf("\n\n");

    if (data != nullptr)
    {
        printf(">>>> moving wheeldrive: H: %s, power: %d\n", ms.c_str(), data->forwardPower);
        printf("<--> heading wheeldrive: H: %s, angle: %d\n", hs.c_str(), data->sterringAngle);
    }
    else
    {
        printf(">>>> moving wheeldrive: H: ?, power: ?\n");
        printf(">>>> heading wheeldrive: H: ?, power: ?\n");
    }

    printf("\n\n");
    printf("(r) reset\n");
    printf("(q) stop\n");
    printf("(Esc) to quit\n\n");
    return getchar();
}

unsigned int setupTerminal()
{
    static struct termios term_flags;
    tcgetattr(STDIN_FILENO, &term_flags);
    unsigned int oldFlags = term_flags.c_lflag;
    // newt = oldt;
    term_flags.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term_flags);
    return oldFlags;
}

void restoreTerminal(int oldFlags)
{
    static struct termios term_flags;
    tcgetattr(STDIN_FILENO, &term_flags);
    term_flags.c_lflag = oldFlags;
    tcsetattr(STDIN_FILENO, TCSANOW, &term_flags);
}

#define DEVICE "/dev/ttyUSB0"
//#define DEVICE "/dev/ttyACM0"


class RemoteController : public mosqpp::mosquittopp
{
private:
    char *host_addr;
    int port;
    std::thread *mqttLoopThr;

    void runMqttLoop() {
        loop_forever();
    }

public:
    RemoteController()
    {
        this->host_addr = strdup("127.0.0.1");
        this->port = 1883;
        //this->username_pw_set("crawler", "435FKDVpp48ddf");
        this->mqttLoopThr = new std::thread(&RemoteController::runMqttLoop, this);
        connect(host_addr, port, 60);
    }

    ~RemoteController()
    {
        disconnect();
        free(host_addr);
    }

    void requestForwardIncrement()
    {
        int message_id = 1;
        char *p = (char *)malloc(sizeof(char) * 2);
        p[0] = CMD_INCREASE_SPEED;
        p[1] = 0;
        publish(&message_id, "/crawler/cmd", 2, p);
    }

    void requestForwardDecrement()
    {
        int message_id = 1;
        char *p = (char *)malloc(sizeof(char) * 2);
        p[0] = CMD_DECREASE_SPEED;
        p[1] = 0;
        publish(&message_id, "/crawler/cmd", 2, p);
    }

    void requestLeftIncrement()
    {
        int message_id = 1;
        char *p = (char *)malloc(sizeof(char) * 2);
        p[0] = CMD_INC_TURN_LEFT;
        p[1] = 0;
        publish(&message_id, "/crawler/cmd", 2, p);
    }

    void requestRightIncrement()
    {
        int message_id = 1;
        char *p = (char *)malloc(sizeof(char) * 2);
        p[0] = CMD_INC_TURN_RIGHT;
        p[1] = 0;
        publish(&message_id, "/crawler/cmd", 2, p);
    }

    void requestStop()
    {
        int message_id = 1;
        char *p = (char *)malloc(sizeof(char) * 2);
        p[0] = CMD_STOP;
        p[1] = 0;
        publish(&message_id, "/crawler/cmd", 2, p);
    }

    void requestReset()
    {
        int message_id = 1;
        char *p = (char *)malloc(sizeof(char) * 2);
        p[0] = CMD_RST;
        p[1] = 0;
        publish(&message_id, "/crawler/cmd", 2, p);
    }

    void on_message(const struct mosquitto_message *message)
    {
        
    }
};

int main(int argc, char *argv[])
{
    if (!FileUtils::fileExists(std::string(DEVICE)))
    {
        printf("%s not found", DEVICE);
        return 1;
    }

    VehicleController::initialize(DEVICE);

    ManualControlAPI::initialize();

    // client for the MQTT messaging bus

    RemoteController controller;
    bool run = true;

    auto flags = setupTerminal();

    printf ("initialized\n");

    while (run)
    {
        char ch = menu(VehicleController::getInstance()->getVehicleData());

        printf("a\n");

        switch (ch)
        {
        case 'w':
            controller.requestForwardIncrement();
            break;
        case 's':
            controller.requestForwardDecrement();
            break;
        case 'a':
            controller.requestLeftIncrement();
            break;
        case 'd':
            controller.requestRightIncrement();
            break;
        case 'q':
            controller.requestStop();
            break;
        case 'r':
            controller.requestReset();
        case 27:
            run = false;
            break;
        default:
            break;
        }

        if (!run)
            break;
    }

    restoreTerminal(flags);
    return 0;
}