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

    if (data != nullptr && data->sterringAngle < 45)
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

int main(int argc, char *argv[])
{
    if (!FileUtils::fileExists(std::string(DEVICE)))
    {
        printf("%s not found", DEVICE);
        return 1;
    }

   VehicleController *vehicleController = new VehicleController(DEVICE);  

    // client for the MQTT messaging bus

    bool run = true;

    auto flags = setupTerminal();

    while (run)
    {
        char ch = menu(vehicleController->getVehicleData());

        switch (ch)
        {
        case 'w':
            vehicleController->forwardIncrease(50);
            break;
        case 's':
            vehicleController->forwardIncrease(-50);
            break;
        case 'a':
            vehicleController->increaseTurnLeftAngle(5);
            break;
        case 'd':
            vehicleController->increaseTurnRightAngle(5);
            break;
        case 'q':
            vehicleController->stop();
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