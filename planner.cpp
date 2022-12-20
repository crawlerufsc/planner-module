// Include OpenCV library

#include "planning/short_term_planner.h"

class MyPlanner : public ShortTermPlannerEngine
{
    void onShortTermPlan(StreamData *occupancyGrid, VehicleData *sensorData) override
    {
        // printf("new frame: size: %d, format: %d x %d\n", occupancyGrid->len, occupancyGrid->width, occupancyGrid->height);
        // if (sensorData == nullptr)
        // {
        //     printf("sensor data is null");
        // }
        // else
        //     std::cout << "Sensor data: \n " << sensorData->toJson() << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
};

int main(int argc, char *argv[])
{
    (new MyPlanner())->run();
    return 0;
}
