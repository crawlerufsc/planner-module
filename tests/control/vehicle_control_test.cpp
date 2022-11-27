#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include "../../utils/file_utils.h"
#include <crawler_hal.h>
#include "../../control/vehicle_controller.h"

class DriveSetting
{
public:
    int headingAngle;
    int movingPower;

    DriveSetting()
    {
        headingAngle = 0;
        movingPower = 0;
    }
};

char menu(DriveSetting &settings, bool lastAck)
{
    std::string hs;
    std::string ms;
    std::string ack;

    if (settings.headingAngle >= 0)
        hs = "[+]";
    else
        hs = "[-]";

    if (settings.headingAngle >= 0)
        ms = "[+]";
    else
        ms = "[-]";

    if (lastAck)
        ack = "ACK";
    else
        ack = "-";

    // system("clear");
    printf("Hardware testing menu\n\n");
    printf("                  (w)       forward pwr++\n");
    printf("left pwr++   (a)       (d)      right pwr++\n");
    printf("                  (s)       backward pwr++\n");
    printf("\n\n");
    printf(">>>> moving wheeldrive: H: %s, power: %d\n", ms.c_str(), settings.movingPower);
    printf("<--> heading wheeldrive: H: %s, angle: %d\n", hs.c_str(), settings.headingAngle);
    printf("\n\n");
    printf("last command status: %s\n", ack.c_str());
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

static struct termios oldt;

#define DEVICE "/dev/ttyUSB0"
//#define DEVICE "/dev/ttyACM0"

int main(int argc, char **argv)
{
    if (!FileUtils::fileExists(DEVICE))
    {
        printf("%s not found", DEVICE);
        return 1;
    }

    auto flags = setupTerminal();
    VehicleController::initialize(DEVICE);

    DriveSetting settings;

    bool run = true;
    bool stop = false;
    bool lastAck = false;

    while (run)
    {
        char ch = menu(settings, lastAck);
        stop = false;

        switch (ch)
        {
        case 'w':
            lastAck = VehicleController::getInstance()->forwardIncrease(25);
            break;
        case 's':
            lastAck = VehicleController::getInstance()->forwardIncrease(-25);
            break;
        case 'a':
            lastAck = VehicleController::getInstance()->increaseTurnLeftAngle(5);
            break;
        case 'd':
            lastAck = VehicleController::getInstance()->increaseTurnRightAngle(5);
            break;
        case 'q':
            lastAck = VehicleController::getInstance()->stop();
            break;
        case 'r':
            lastAck = VehicleController::getInstance()->reset();
            continue;
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