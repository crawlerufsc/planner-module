#ifndef _OMPL_PLANNER_H
#define _OMPL_PLANNER_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/util/String.h>
#include "planner.h"
#include "../tests/utils/debug_utils.h"


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

/**
 * OMPL implementation of path planning
 */
class OMPLPlanner : public Planner
{
private:
    std::shared_ptr<ob::SE2StateSpace> state_space;
    ob::RealVectorBounds *state_space_bounds;
    std::shared_ptr<oc::RealVectorControlSpace> control_space;
    ob::RealVectorBounds *control_space_bounds;
    std::shared_ptr<oc::SpaceInformation> space_info;
    oc::SimpleSetup *ompl_simple_setup;

    int width;
    int height;
    int camera_distance;
    double speed_meters_per_second;
    double steptime_s;
    ImageDebugUtils debugUtils;

    inline double degree2rad(double value)
    {
        return value * (M_PI / 180.0);
    }

    void setMaxHorizontalSpeed(double speed_meters_per_second)
    {
        this->speed_meters_per_second = speed_meters_per_second;
    }
    void setStepTime(double steptime_s)
    {
        this->steptime_s = steptime_s;
    }

    void setup_planning();

public:
    OMPLPlanner(int width, int height, int camera_distance);

    ~OMPLPlanner();

    void plan(StreamData *data) override;
};

#endif