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
class OMPLPlanner : public PathPlanner
{
private:
    inline static OMPLPlanner *pinstance_ = nullptr;
    inline static std::mutex singletonMtx;

    std::shared_ptr<ob::SE2StateSpace> state_space;
    ob::RealVectorBounds *state_space_bounds;

    std::shared_ptr<oc::RealVectorControlSpace> control_space;
    ob::RealVectorBounds *control_space_bounds;

    std::shared_ptr<oc::SpaceInformation> space_info;
    oc::SimpleSetup *ompl_simple_setup;

    inline static int ogWidth = 0;
    inline static int ogHeight = 0;
    inline static int cameraDistance = 0;

    inline static int ogCellWidth = 0;
    inline static int ogCellHeight = 0;
    inline static int ogCellSize = 0;

    inline static int vehicleSizeLength = 0;

    double speed_meters_per_second;
    double steptime_s;

    inline static char *occupancyGrid = nullptr;
    inline static float *pixelDistanceFromBackground = nullptr;

    double goalStateX, goalStateY;

    // temp
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

    static inline uint ComputeGridPosWidth(double x)
    {
        return (uint)((x + (OMPLPlanner::ogCellWidth / 2)) / OMPLPlanner::ogCellSize);
    }

    static inline uint ComputeGridPosHeight(double y)
    {
        return (uint)(OMPLPlanner::ogCellHeight - 1 - (y / OMPLPlanner::ogCellSize));
    }

    void setupPlanning();
    void computeMatrixOfDistancesToBackground(cv::Mat *og_mat);

    ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

    static bool checkStateValid(const ob::State *state);
    static bool checkStateWithinBorders(const ob::SE2StateSpace::StateType *SE2state, int x, int z);
    static bool checkStatePositionIsNavegable(int x, int z, int minimalSafeDistanceFromBackground);
    static void statePropagator(const ob::State *start, const oc::Control *control, const double duration, ob::State *result);

protected:
    OMPLPlanner(int width, int height, int occupancyGridCellSizeInCm, int cameraDistance, int vehicleSizeLength);

    ~OMPLPlanner();

public:
    OMPLPlanner(OMPLPlanner &other) = delete;
    void operator=(const OMPLPlanner &) = delete;

    //static void ModuleInitialize() {
    //    OMPLPlanner::singletonMtx = new std::mutex();
    //}
    static OMPLPlanner *GetInstance(int occupancyGridWidth, int occupancyGridHeight, int occupancyGridCellSizeInCm, int cameraDistance, int vehicleSizeLength);

    static void ClearInstance();

    void plan(StreamData *data) override;
};



#endif