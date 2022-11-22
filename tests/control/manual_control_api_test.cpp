#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <mosquittopp.h>
#include "../../control/vehicle_controller.h"
#include "../../control/manual_control_api.h"
#include "../../model/vehicle_data.h"
#include "../../utils/filesystem.h"

char menu(VehicleData *data)
{
    std::string hs;
    std::string ms;
    std::string ack;

    if (data != nullptr && data->frontAngle < 45)
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
        printf("<--> heading wheeldrive: H: %s, angle: %d\n", hs.c_str(), data->frontAngle);
    }
    else
    {
        printf(">>>> moving wheeldrive: H: ?, power: ?\n");
        printf(">>>> heading wheeldrive: H: ?, power: ?\n");
    }

    printf("\n\n");
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

ManualControlAPI *init_server()
{
    VehicleController *vehicleController = new VehicleController();

    ManualControlAPI *manualControlAPI = new ManualControlAPI(vehicleController);
    manualControlAPI->initialize();
    return manualControlAPI;
}

class RemoteController : public mosqpp::mosquittopp
{
private:
    char *host_addr;
    int port;

public:
    RemoteController()
    {
        this->host_addr = strdup("127.0.0.1");
        this->port = 1883;
        this->username_pw_set("crawler", "435FKDVpp48ddf");
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
        publish(&message_id, "/crawler/cmd", 2, p, 1);
    }

    void requestForwardDecrement()
    {
        int message_id = 1;
        char *p = (char *)malloc(sizeof(char) * 2);
        p[0] = CMD_DECREASE_SPEED;
        publish(&message_id, "/crawler/cmd", 2, p, 1);
    }

    void requestLeftIncrement()
    {
        int message_id = 1;
        char *p = (char *)malloc(sizeof(char) * 2);
        p[0] = CMD_INC_TURN_LEFT;
        publish(&message_id, "/crawler/cmd", 2, p, 1);
    }

    void requestRightIncrement()
    {
        int message_id = 1;
        char *p = (char *)malloc(sizeof(char) * 2);
        p[0] = CMD_INC_TURN_RIGHT;
        publish(&message_id, "/crawler/cmd", 2, p, 1);
    }

    void requestReset()
    {
        int message_id = 1;
        char *p = (char *)malloc(sizeof(char) * 2);
        p[0] = CMD_RST;
        publish(&message_id, "/crawler/cmd", 2, p, 1);
    }

    void on_message(const struct mosquitto_message *message)
    {
        
    }
};

ManualControlAPI *manualControlAPI;
VehicleData *lastData;

int main(int argc, char *argv[])
{
    if (!fileExists(DEVICE))
    {
        printf("%s not found", DEVICE);
        return 1;
    }

    ManualControlAPI *server = init_server();

    // client for the MQTT messaging bus

    RemoteController controller;
    bool run = true;

    auto flags = setupTerminal();

    while (run)
    {
        char ch = menu(lastData);

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
            controller.requestReset();
            break;
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

    return 0;
}