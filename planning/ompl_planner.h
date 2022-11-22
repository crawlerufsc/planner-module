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
#include "../model/planning_waypoint.h"
#include "planning_data.h"

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

    inline static Frame<unsigned char> *occupancyGrid = nullptr;
    inline static Frame<float> *pixelDistanceFromBackground = nullptr;
    inline static SkeletonizeData *skeletonizedData = nullptr;
    inline static Frame<unsigned char> *transformedOccupancyGridProcBuffer = nullptr;
    inline static Frame<unsigned char> *skeletonizedOccupancyGridProcBuffer = nullptr;

    std::pair<int, int> *min_dist_crawler_pixel;

    double goalStateX, goalStateY;
    PathPlanningData *planningData;

    

    // temp
    ImageDebugUtils debugUtils;

    static int convertMatrixToVectorPos(int width, int heightPos, int widthPos)
    {
        return (width * heightPos) + widthPos;
    }

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

    inline double calculate_dist2crawler(Point &p)
    {
        return ((this->occupancyGrid->width / 2.0) - p.X) * ((this->occupancyGrid->width / 2.0) - pixel.X) +
               (this->occupancyGrid->height - p.Y) * (this->occupancyGrid->height - p.Y);
    }


    void setupPlanning();
    void computeMatrixOfDistancesToBackground(cv::Mat *og_mat);

    ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr &si);

    static bool checkStateValid(const ob::State *state);
    static bool checkStateWithinBorders(const ob::SE2StateSpace::StateType *SE2state, int x, int z);
    static bool checkStatePositionIsNavegable(int x, int z, int minimalSafeDistanceFromBackground);
    static void statePropagator(const ob::State *start, const oc::Control *control, const double duration, ob::State *result);

    void preProcessOccupancyGridFrame();
    void initializeOccupancyGridBuffers(int width, int height);
    void skeletonizeOccupancyGrid();
    void get_start_point_and_connections();
    void copyOccupancyGrid(StreamData *frame);
    void bfs();
    void get_max_dist_crawler_pixel();
    void get_min_dist_checkpoint_pixel();
    void get_min_or_max_pixel();
    double get_median_contour_distance();
    void get_path();

protected:
    OMPLPlanner(int width, int height, int occupancyGridCellSizeInCm, int cameraDistance, int vehicleSizeLength);

    ~OMPLPlanner();

public:
    OMPLPlanner(OMPLPlanner &other) = delete;
    void operator=(const OMPLPlanner &) = delete;

    static OMPLPlanner *GetInstance(int occupancyGridWidth, int occupancyGridHeight, int occupancyGridCellSizeInCm, int cameraDistance, int vehicleSizeLength);

    static void ClearInstance();

    void plan(Frame<unsigned char> *frame) override;
};

#endif