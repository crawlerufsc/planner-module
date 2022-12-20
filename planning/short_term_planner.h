#ifndef _SHORT_TERM_PLANNER_H
#define _SHORT_TERM_PLANNER_H

#include <network_stream_reader.h>
#include <resource_manager.h>
#include "control/hardware_keepalive.h"
#include "model/global_definitions.h"
#include "model/vehicle_data.h"
#include "control/vehicle_controller.h"



class ShortTermPlannerEngine
{
private:
    std::function<void(Frame<u_char> *)> *onProcess;
    
protected:
    virtual void onShortTermPlan(StreamData *occupancyGrid, VehicleData *sensorData) = 0;

public:
    ShortTermPlannerEngine();
    ~ShortTermPlannerEngine();
    void run();
};



#endif