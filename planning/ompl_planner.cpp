#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "ompl_planner.h"

template <typename PType>
void printOccupancyGrid(int width, int height, PType *data)
{
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
            printf("%d", data[i * width + j]);
        printf("\n");
    }
}

template <typename DataType>
DataType *allocOG(int width, int height)
{
    int size = height * width;
    DataType *data = new DataType[size];
    memset(data, 0, sizeof(DataType) * size);
    return data;
}

OMPLPlanner::OMPLPlanner(int width, int height, int occupancyGridCellSizeInCm, int cameraDistance, int vehicleSizeLength)
{
    OMPLPlanner::occupancyGrid = nullptr;
    OMPLPlanner::pixelDistanceFromBackground = nullptr;

    checkpoints = new PlanningCheckpoints();

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

    planningData = new PathPlanningData();

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
    return OMPLPlanner::occupancyGrid->data[pos] && OMPLPlanner::pixelDistanceFromBackground->data[pos] >= minimalSafeDistanceFromBackground;
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

void OMPLPlanner::initializeOccupancyGridBuffers(int width, int height)
{
    // if (OMPLPlanner::occupancyGrid == nullptr)
    //     OMPLPlanner::occupancyGrid = Frame<unsigned char>::newEmptyFrame(width, height);

    if (OMPLPlanner::transformedOccupancyGridProcBuffer == nullptr)
        OMPLPlanner::transformedOccupancyGridProcBuffer = Frame<unsigned char>::newEmptyFrame(width, height);
    else
        OMPLPlanner::transformedOccupancyGridProcBuffer->clear();

    if (OMPLPlanner::skeletonizedData == nullptr)
        OMPLPlanner::skeletonizedData = new SkeletonizeData(width, height);
    else
        OMPLPlanner::skeletonizedData->clear();

    if (OMPLPlanner::skeletonizedOccupancyGridProcBuffer == nullptr)
        OMPLPlanner::skeletonizedOccupancyGridProcBuffer = Frame<unsigned char>::newEmptyFrame(width + 2, height + 2);
    else
        OMPLPlanner::skeletonizedOccupancyGridProcBuffer->clear();

    if (OMPLPlanner::pixelDistanceFromBackground == nullptr)
        OMPLPlanner::pixelDistanceFromBackground = Frame<float>::newEmptyFrame(width, height);
    else
        OMPLPlanner::pixelDistanceFromBackground->clear();
}

void OMPLPlanner::preProcessOccupancyGridFrame()
{
    cv::Mat og = *OMPLPlanner::occupancyGrid->getMatrix(CV_8U);
    cv::Mat ogt = *OMPLPlanner::transformedOccupancyGridProcBuffer->getMatrix(CV_8U);
    cv::Mat dist = *OMPLPlanner::pixelDistanceFromBackground->getMatrix(CV_32F);

    cv::morphologyEx(og, ogt, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
    cv::morphologyEx(ogt, ogt, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(30, 30)));
    cv::distanceTransform(ogt, dist, cv::DIST_L2, cv::DIST_MASK_PRECISE);
}

void OMPLPlanner::skeletonizeOccupancyGrid()
{
    if (OMPLPlanner::transformedOccupancyGridProcBuffer == nullptr)
    {
        std::cout << "erro\n";
        return;
    }

    int lut[256] =
        {0, 0, 0, 1, 0, 0, 1, 3, 0, 0, 3, 1, 1, 0, 1, 3, 0, 0, 0, 0, 0, 0,
         0, 0, 2, 0, 2, 0, 3, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 3, 0, 2, 2, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 0,
         0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 3, 0, 2, 0, 0, 0, 3, 1,
         0, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 1, 3, 0, 0,
         1, 3, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 2, 3, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3,
         0, 1, 0, 0, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0};

    int neighbors_case = 0;
    bool firts_pass;
    bool pixel_removed = true;
    int zs_it_counter = 0;
    std::list<Point> skeleton_active;
    int skeleton_counter;
    double dist_close_min = 10.0;

    int skeleton_to_clean[OMPLPlanner::ogHeight * OMPLPlanner::ogWidth][2];

    auto skelOG = OMPLPlanner::skeletonizedOccupancyGridProcBuffer;
    auto OG = OMPLPlanner::transformedOccupancyGridProcBuffer;
    auto distance = OMPLPlanner::pixelDistanceFromBackground;

    for (int y = 0; y < OG->height; i++)
        for (int x = 0; x < OG->width; j++)
        {
            auto val = OG->get(i, j);
            OMPLPlanner::skeletonizedOccupancyGridProcBuffer->set(i + 1, j + 1, val);
            if (val)
            {
                skeleton_active.push_back(Point(x, y));
            }
        }

    for (int i = 0; i < skelOG->height; i++)
    {
        OMPLPlanner::skeletonizedOccupancyGridProcBuffer->set(i, 0, 0);
        OMPLPlanner::skeletonizedOccupancyGridProcBuffer->set(i, OG->width + 1, 0);
    }

    for (int j = 0, pos = 0; j < OG->width + 2; j++)
    {
        OMPLPlanner::skeletonizedOccupancyGridProcBuffer->set(0, j, 0);
        OMPLPlanner::skeletonizedOccupancyGridProcBuffer->set(OG->height + 1, j, 0);
    }

    while (pixel_removed)
    {
        pixel_removed = false;
        for (int sub_it = 0; sub_it < 2; sub_it++)
        {
            skeleton_counter = 0;
            firts_pass = (sub_it == 0);
            int i, j;
            for (Point it = skeleton_active.begin(); it != skeleton_active.end();)
            {
                i = it->Y + 1;
                j = it->X + 1;
                if (skelOG->get(i, j))
                {
                    neighbors_case = lut[1 * skelOG->get(i - 1, j - 1) + 2 * skelOG->get(i - 1, j) +
                                         4 * skelOG->get(i - 1, j + 1) + 8 * skelOG->get(i, j + 1) +
                                         16 * skelOG->get(i + 1, j + 1) + 32 * skelOG->get(i + 1, j) +
                                         64 * skelOG->get(i + 1, j - 1) + 128 * skelOG->get(i, j - 1)];
                    if ((neighbors_case == 1 && firts_pass) ||
                        (neighbors_case == 2 && !firts_pass) ||
                        (neighbors_case == 3))
                    {
                        skeleton_to_clean[skeleton_counter][0] = i;
                        skeleton_to_clean[skeleton_counter][1] = j;
                        skeleton_counter++;
                        pixel_removed = true;
                        it = skeleton_active.erase(it);
                        continue;
                    }
                }
                ++it;
            }

            for (int i = 0; i < skeleton_counter; i++)
            {
                OMPLPlanner::skeletonizedOccupancyGridProcBuffer->set(skeleton_to_clean[i][0], skeleton_to_clean[i][1], 0);
            }
        }
        zs_it_counter++;
    }

    int y, x;
    for (auto it = skeleton_active.begin(); it != skeleton_active.end(); ++it)
    {
        y = it->Y;
        x = it->X;
        OMPLPlanner::skeletonizedData->skeletonizedOccupancyGrid->set(i, j, 1);

        OMPLPlanner::skeletonizedData->skeleton_all.insert(Point(x, y));
        if (distance->get(i, j) >= dist_close_min)
        {
            OMPLPlanner::skeletonizedData->skeleton_rem.insert(Point(x, y));
        }
    }
}

void OMPLPlanner::get_start_point_and_connections()
{
    Point cur_pixel;
    Point neighbor;
    double cur_dist;

    for (auto it = skeletonizedData->skeleton_rem->begin(); it != skeletonizedData->skeleton_rem->end(); ++it)
    {
        // get start point
        cur_pixel = *it;
        cur_dist = calculate_dist2crawler(cur_pixel);
        if (cur_dist < this->planningData->planningCheckPoints->getMinDistanceValue())
        {
            this->planningData->planningCheckPoints->setMinDistanceValue(cur_dist);
            this->planningData->planningCheckPoints->setMinDistancePoint(cur_pixel);
        }

        // get connections
        if (cur_pixel.Y != this->occupancyGrid->height)
        {
            neighbor.X = cur_pixel.X
                             neighbor.Y = cur_pixel.Y + 1;
            if (skeletonizedData->skeleton_rem->find(neighbor) != skeletonizedData->skeleton_rem->end())
            {
                (*skeletonizedData->connections)[cur_pixel].push_back(neighbor);
                (*skeletonizedData->connections)[neighbor].push_back(cur_pixel);
            }
        }
        if (cur_pixel.X != his->occupancyGrid->width)
        {
            neighbor.X = cur_pixel.X + 1;
            neighbor.Y = cur_pixel.Y;
            if (skeletonizedData->skeleton_rem->find(neighbor) != skeletonizedData->skeleton_rem->end())
            {
                (*skeletonizedData->connections)[cur_pixel].push_back(neighbor);
                (*skeletonizedData->connections)[neighbor].push_back(cur_pixel);
            }
        }
        if (cur_pixel.Y != this->occupancyGrid->height && cur_pixel.X != this->occupancyGrid->width)
        {
            neighbor.X = cur_pixel.X + 1;
            neighbor.Y = cur_pixel.Y;
            if (skeletonizedData->skeleton_rem->find(neighbor) != skeletonizedData->skeleton_rem->end())
            {
                (*skeletonizedData->connections)[cur_pixel].push_back(neighbor);
                (*skeletonizedData->connections)[neighbor].push_back(cur_pixel);
            }
        }
        if (cur_pixel.Y != this->occupancyGrid->height && cur_pixel.X != 0)
        {
            neighbor.X = cur_pixel.X - 1;
            neighbor.Y = cur_pixel.Y + 1;

            if (skeletonizedData->skeleton_rem->find(neighbor) != skeletonizedData->skeleton_rem->end())
            {
                (*skeletonizedData->connections)[cur_pixel].push_back(neighbor);
                (*skeletonizedData->connections)[neighbor].push_back(cur_pixel);
            }
        }
    }
}

void OMPLPlanner::bfs()
{
    std::pair<int, int> u, v;
    std::vector<std::pair<int, int>> cur_connections;
    OMPLPlanner::skeletonizedData->skeleton_visited->insert(*min_dist_crawler_pixel);
    (*OMPLPlanner::skeletonizedData->parent)[*min_dist_crawler_pixel] = std::pair<int, int>(-1, -1);
    OMPLPlanner::skeletonizedData->queue->push(*min_dist_crawler_pixel);
    while (!OMPLPlanner::skeletonizedData->queue->empty())
    {
        u = OMPLPlanner::skeletonizedData->queue->front();
        OMPLPlanner::skeletonizedData->queue->pop();
        cur_connections = (*OMPLPlanner::skeletonizedData->connections)[u];
        for (auto it = cur_connections.begin(); it != cur_connections.end(); ++it)
        {
            v = *it;
            if (OMPLPlanner::skeletonizedData->skeleton_visited->find(v) == OMPLPlanner::skeletonizedData->skeleton_visited->end())
            {
                OMPLPlanner::skeletonizedData->skeleton_visited->insert(v);
                OMPLPlanner::skeletonizedData->queue->push(v);
                (*OMPLPlanner::skeletonizedData->parent)[v] = u;
            }
        }
    }
}

void OMPLPlanner::get_min_or_max_pixel()
{
    if (checkpoints->hasCheckpoints())
    {
        get_min_dist_checkpoint_pixel();
    }
    else
    {
        get_max_dist_crawler_pixel();
    }
}
void OMPLPlanner::get_min_dist_checkpoint_pixel()
{
    double cur_dist = 0;
    std::pair<int, int> cur_pixel(0, 0);
    for (auto it = OMPLPlanner::skeletonizedData->skeleton_visited->begin(); it != OMPLPlanner::skeletonizedData->skeleton_visited->end(); ++it)
    {
        cur_pixel = *it;
        cur_dist = checkpoints->distanceToNextPoint(cur_pixel);
        if (cur_dist < this->checkpoints->getMinDistanceCheckpoint())
        {
            this->checkpoints->setMinDistanceCheckpoint(cur_dist);
            this->checkpoints->setNextCheckpoint(cur_pixel);
        }
    }
}
double OMPLPlanner::get_median_contour_distance()
{
    std::vector<int> contour_distances;
    std::pair<int, int> cur_pixel;
    for (auto it = OMPLPlanner::skeletonizedData->skeleton_visited->begin(); it != OMPLPlanner::skeletonizedData->skeleton_visited->end(); ++it)
    {
        cur_pixel = *it;
        contour_distances.push_back(OMPLPlanner::pixelDistanceFromBackground->get(cur_pixel.first, cur_pixel.second));
    }
    std::sort(contour_distances.begin(), contour_distances.end());
    return contour_distances[(int)(contour_distances.size() / 2.0)];
}

void OMPLPlanner::get_max_dist_crawler_pixel()
{
    double cur_dist = 0;
    std::pair<int, int> cur_pixel(0, 0);
    double median = get_median_contour_distance();
    for (auto it = OMPLPlanner::skeletonizedData->skeleton_visited->begin(); it != OMPLPlanner::skeletonizedData->skeleton_visited->end(); ++it)
    {
        cur_pixel = *it;
        if (OMPLPlanner::pixelDistanceFromBackground->get(cur_pixel.first, cur_pixel.second) > median)
        {
            cur_dist = calculate_dist2crawler(cur_pixel);
            if (cur_dist > checkpoints->getMaxDistanceCrawler())
            {
                checkpoints->setMaxDistanceCrawler(cur_dist);
                checkpoints->setMaxDistanceCrawlerPixel(cur_pixel);
            }
        }
    }
}

void OMPLPlanner::get_path()
{
    std::pair<int, int> cur_parent;
    if (checkpoints->hasCheckpoints())
        planningPathPoints.push_back(checkpoints->getMinDistancePoint());
    else
        planningPathPoints.push_back(checkpoints->getMaxDistancePoint());

    cur_parent = parent[path[0]];
    while (cur_parent.first != -1 && cur_parent.second != -1)
    {
        path.push_back(cur_parent);
        cur_parent = parent[cur_parent];
    }

    // put reverse path in extended path
    int len_path = path.size();
    extended_path.reserve(path.size());
    for (int i = 0; i < len_path; i++)
    {
        extended_path.push_back(path[0]); // push anything to increase size
    }
    for (int i = 0; i < len_path / 2; i++)
    {
        extended_path[i] = path[len_path - 1 - i];
        extended_path[len_path - 1 - i] = path[i];
    }
}

void OMPLPlanner::plan(StreamData *frame)
{
    OMPLPlanner::occupancyGrid = frame;
    initializeOccupancyGridBuffers(frame->width, frame->height);

    // std::cout << "OG address: " << reinterpret_cast<void *>(OMPLPlanner::occupancyGrid) << std::endl;

    printf("plan on OG %dx%d\n", OMPLPlanner::occupancyGrid->width, OMPLPlanner::occupancyGrid->height);
    preProcessOccupancyGridFrame();
    skeletonizeOccupancyGrid();
    get_start_point_and_connections();
    bfs();
    get_min_or_max_pixel();
    // debugUtils.showImageRaw("OG-orig", OMPLPlanner::occupancyGrid);
    // debugUtils.showImageRaw("OG-proc", *transformedOGMatrix);

    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::cout << "end planning\n";
}
