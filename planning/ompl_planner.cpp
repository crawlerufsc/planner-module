#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "ompl_planner.h"

OMPLPlanner::OMPLPlanner(int width, int height, int camera_distance)
{
    state_space = std::make_shared<ob::SE2StateSpace>();
    state_space_bounds = new ob::RealVectorBounds(2);

    control_space = std::make_shared<oc::RealVectorControlSpace>(state_space, 2);
    control_space_bounds = new ob::RealVectorBounds(2);

    space_info = std::make_shared<oc::SpaceInformation>(state_space, control_space);
    ompl_simple_setup = new oc::SimpleSetup(space_info);

    this->width = width;
    this->height = height;
    this->camera_distance = camera_distance;

    setup_planning();
}

OMPLPlanner::~OMPLPlanner()
{
    delete state_space_bounds;
    delete control_space_bounds;
    delete ompl_simple_setup;
}

void OMPLPlanner::setup_planning()
{
    state_space_bounds->setLow(0, -width / 2);
    state_space_bounds->setHigh(0, width / 2 - 0.00001); // make sure that 0CC_WIDTH/2 is out of the boundary
    state_space_bounds->setLow(1, -camera_distance);
    state_space_bounds->setHigh(1, height - 0.00001);
    state_space->setBounds(*state_space_bounds);

    double new_yaw_weight = 5.0 / degree2rad(2.5);
    state_space->setSubspaceWeight(1, new_yaw_weight);

    double speed_in_cm_s = 100 * this->speed_meters_per_second;
    control_space_bounds->setLow(0, speed_in_cm_s * this->steptime_s);
}

void OMPLPlanner::plan(StreamData *data)
{
    debugUtils.showImage("OG-orig", data->data, data->raw_length);
    printf("plan on OG\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
