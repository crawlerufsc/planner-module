#ifndef _OMPL_SKELETONIZE_H
#define _OMPL_SKELETONIZE_H

#include <memory>
#include "../model/frame.h"
#include "../model/planning_waypoint.h"

class SkeletonizeData
{
public:
    Frame<unsigned char> *skeletonizedOccupancyGrid = nullptr;
    std::set<Point> *skeleton_all;
    std::set<Point> *skeleton_rem;
    std::set<Point> *skeleton_visited;
    std::queue<Point> *queue;

    SkeletonizeData(int baseWidth, int baseHeight)
    {
        this->skeletonizedOccupancyGrid = Frame<unsigned char>::newEmptyFrame(baseWidth, baseHeight);
        this->skeleton_rem = new std::set<Point>();
        this->skeleton_visited = new std::set<Point>();
        this->skeleton_all = new std::set<Point>();
        this->queue = new std::queue<Point>();
    }

    ~SkeletonizeData()
    {
        delete this->skeletonizedOccupancyGrid;
        delete this->skeleton_rem;
        delete this->skeleton_visited;
        delete this->queue;
        delete this->skeleton_all;
    }

    void clear()
    {
        this->skeletonizedOccupancyGrid->clear();
        this->skeleton_rem->clear();
        this->skeleton_visited->clear();
        this->skeleton_all->clear();
        while(!this->queue->empty()) this->queue->pop();
    }
};

#endif