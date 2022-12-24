// Include OpenCV library

#include "planning/short_term_planner.h"

class MyPlanner : public ShortTermPlannerEngine
{
    void onShortTermPlan(StreamData *occupancyGrid, VehicleData *sensorData) override
    {
        // planner
    }
};

int main(int argc, char *argv[])
{
    (new MyPlanner())->run();
    return 0;
}
