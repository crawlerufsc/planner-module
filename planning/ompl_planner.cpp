#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "ompl_planner.h"

OMPLPlanner::OMPLPlanner(int width, int height, int occupancyGridCellSizeInCm, int cameraDistance, int vehicleSizeLength)
{
    OMPLPlanner::occupancyGrid = nullptr;
    OMPLPlanner::pixelDistanceFromBackground = nullptr;

    state_space = std::make_shared<ob::SE2StateSpace>();
    state_space_bounds = new ob::RealVectorBounds(2);

    control_space = std::make_shared<oc::RealVectorControlSpace>(state_space, 2);
    control_space_bounds = new ob::RealVectorBounds(2);

    space_info = std::make_shared<oc::SpaceInformation>(state_space, control_space);
    ompl_simple_setup = new oc::SimpleSetup(space_info);

    OMPLPlanner::ogWidth = width;
    OMPLPlanner::ogHeight = height;
    OMPLPlanner::cameraDistance = cameraDistance;
    OMPLPlanner::vehicleSizeLength = vehicleSizeLength;

    OMPLPlanner::ogCellSize = occupancyGridCellSizeInCm;
    OMPLPlanner::ogCellWidth = OMPLPlanner::ogWidth / OMPLPlanner::ogCellSize;
    OMPLPlanner::ogCellHeight = OMPLPlanner::ogHeight / OMPLPlanner::ogCellSize;

    goalStateX = 0;
    goalStateY = 0;

    setupPlanning();
}

OMPLPlanner::~OMPLPlanner()
{
    delete state_space_bounds;
    delete control_space_bounds;
    delete ompl_simple_setup;
}

OMPLPlanner *OMPLPlanner::GetInstance(int occupancyGridWidth, int occupancyGridHeight, int occupancyGridCellSizeInCm, int cameraDistance, int vehicleSizeLength)
{
    std::lock_guard<std::mutex> lock(singletonMtx);
    if (OMPLPlanner::pinstance_ == nullptr)
    {
        OMPLPlanner::pinstance_ = new OMPLPlanner(occupancyGridWidth, occupancyGridHeight, occupancyGridCellSizeInCm, cameraDistance, vehicleSizeLength);
    }
    return OMPLPlanner::pinstance_;
}

void OMPLPlanner::ClearInstance()
{
    std::lock_guard<std::mutex> lock(singletonMtx);
    if (OMPLPlanner::pinstance_ != nullptr)
        delete OMPLPlanner::pinstance_;
    OMPLPlanner::pinstance_ = nullptr;
}

bool OMPLPlanner::checkStateWithinBorders(const ob::SE2StateSpace::StateType *SE2state, int x, int z)
{
    if (SE2state->getY() < -OMPLPlanner::cameraDistance || SE2state->getY() >= 1000)
    {
        // std::cout << "**not valid - z out of bounds**\n";
        return false;
    }

    if (SE2state->getY() <= 0 && SE2state->getY() >= -OMPLPlanner::cameraDistance)
    {
        // std::cout << "**valid**\n";
        return true;
    }

    if (x >= OMPLPlanner::ogCellWidth || z >= OMPLPlanner::ogCellHeight || x < 0)
    {
        // std::cout << "**not valid**\n";
        return false;
    }

    return true;
}

bool OMPLPlanner::checkStatePositionIsNavegable(int x, int z, int minimalSafeDistanceFromBackground)
{

    // Check whether x,z is valid against the resulting valid navigable path detected by the vision module,
    // translated from the segmented image to the O.G.
    int pos = z * OMPLPlanner::ogWidth + x;
    return OMPLPlanner::occupancyGrid[pos] && OMPLPlanner::pixelDistanceFromBackground[pos] >= minimalSafeDistanceFromBackground;
}

bool OMPLPlanner::checkStateValid(const ob::State *state)
{

    const auto *SE2state = state->as<ob::SE2StateSpace::StateType>();

    int x = ComputeGridPosWidth(SE2state->getX());
    int z = ComputeGridPosHeight(SE2state->getY());

    if (!checkStateWithinBorders(SE2state, x, z))
        return false;

    return checkStatePositionIsNavegable(x, z, 10.0);
}

void OMPLPlanner::computeMatrixOfDistancesToBackground(cv::Mat *og_mat)
{
    // cv::Mat og_mat = cv::Mat(this->ogHeight, this->ogWidth, CV_8U, this->occupancyGrid);
    cv::Mat distance_mat = cv::Mat(this->ogHeight, this->ogWidth, CV_32F, this->pixelDistanceFromBackground);
    cv::distanceTransform(*og_mat, distance_mat, cv::DIST_L2, cv::DIST_MASK_PRECISE);
}

void OMPLPlanner::statePropagator(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    const double *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double yaw = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
    const double *ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    result->as<ob::SE2StateSpace::StateType>()->setXY(
        pos[0] + ctrl[0] * cos(yaw),
        pos[1] + ctrl[0] * sin(yaw));

    // std::cout << "old: " << pos[0] << "; new: " << pos[0] + ctrl[0] * cos(yaw) << std::endl;
    // std::cout << "old: " << pos[1] << "; new: " << pos[1] + ctrl[0] * sin(yaw) << std::endl;

    double new_yaw = yaw;
    // std::cout << "yaw: " << yaw <<std::endl;
    new_yaw += (ctrl[0] / OMPLPlanner::vehicleSizeLength) * tan(ctrl[1]);
    // std::cout << "new_yaw: " << new_yaw <<std::endl;

    // intervalo eh de [0, pi][-pi, 0]
    if (new_yaw > M_PI && new_yaw < 2 * M_PI)
    {
        new_yaw -= 2 * M_PI;
    }
    else if (new_yaw < -M_PI && new_yaw > -2 * M_PI)
    {
        new_yaw += 2 * M_PI;
    }
    else if (new_yaw >= 2 * M_PI)
    {
        new_yaw -= 2 * M_PI;
    }
    else if (new_yaw < -2 * M_PI)
    {
        new_yaw += 2 * M_PI;
    }
    // std::cout << "final_new_yaw: " << new_yaw <<std::endl;
    result->as<ob::SE2StateSpace::StateType>()->setYaw(new_yaw);
}

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.
ob::OptimizationObjectivePtr OMPLPlanner::getThresholdPathLengthObj(const ob::SpaceInformationPtr &si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(space_info));
    double manhattan_dist = abs(goalStateX) + OMPLPlanner::cameraDistance + goalStateY;
    std::cout << "Threshold: " << manhattan_dist << std::endl;
    // obj->setCostThreshold(ob::Cost(manhattan_dist));
    return obj;
}

void OMPLPlanner::setupPlanning()
{
    state_space_bounds->setLow(0, -ogWidth / 2);
    state_space_bounds->setHigh(0, ogWidth / 2 - 0.00001); // make sure that 0CC_WIDTH/2 is out of the boundary
    state_space_bounds->setLow(1, -cameraDistance);
    state_space_bounds->setHigh(1, ogHeight - 0.00001);
    state_space->setBounds(*this->state_space_bounds);

    double new_yaw_weight = 5.0 / degree2rad(2.5);
    state_space->setSubspaceWeight(1, new_yaw_weight);

    double speed_in_cm_s = 100 * this->speed_meters_per_second;
    control_space_bounds->setLow(0, speed_in_cm_s * this->steptime_s);
    control_space_bounds->setHigh(0, speed_in_cm_s * this->steptime_s);
    control_space_bounds->setLow(1, degree2rad(-12));
    control_space_bounds->setHigh(1, degree2rad(12));
    control_space->setBounds(*this->control_space_bounds);

    ompl_simple_setup->getSpaceInformation()->setStateValidityChecker(checkStateValid);
    ompl_simple_setup->getSpaceInformation()->setStatePropagator(statePropagator);
    ompl_simple_setup->getSpaceInformation()->setMinMaxControlDuration(2, 5);
    ompl_simple_setup->getSpaceInformation()->setPropagationStepSize(5); // deixar sempre igual ao maxControlDuration para não influenciar em nada

    ob::ScopedState<ob::SE2StateSpace> start(state_space);
    start->setXY(0, -OMPLPlanner::cameraDistance);
    start->setYaw(degree2rad(90));

    ompl_simple_setup->setOptimizationObjective(getThresholdPathLengthObj(ompl_simple_setup->getSpaceInformation()));
    // std::cout << "Threshold: "<<ss.getOptimizationObjective()->getCostThreshold() <<std::endl;

    // ob::ScopedState<ob::SE2StateSpace> start(space);

    // sst params
    auto plannerEngine(std::make_shared<oc::SST>(space_info));

    ompl::base::ParamSet &params = plannerEngine->params();
    double sst_selection_radius = 25;
    double sst_pruning_radius = 5;
    params.setParam("selection_radius", ompl::toString(sst_selection_radius));
    params.setParam("pruning_radius", ompl::toString(sst_pruning_radius));
    ompl_simple_setup->setPlanner(plannerEngine);
}

void OMPLPlanner::preProcessOccupancyGridFrame(StreamData *frame)
{
    debugUtils.showImageRaw("OG-orig", data->data, data->width, data->height);
    printf("plan on OG %dx%d\n", data->width, data->height);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
