#ifndef _PLANNING_DATA_H
#define _PLANNING_DATA_H

#include <map>
#include <vector>
#include "../model/planning_waypoint.h"

class PlanningCheckpoints
{
private:
    std::vector<Point> checkpoints;
    Point *next;

    ReferencedPoint *max;
    ReferencedPoint *min;

public:
    PlanningCheckpoints()
    {
        max = new ReferencedPoint();
        min = new ReferencedPoint(0, 0, 999999);
    }

    ~PlanningCheckpoints()
    {
        delete max;
        delete min;
    }

    Point *getMaxDistancePoint()
    {
        return new Point(max->X, max->Y);
    }

    void setMaxDistancePoint(Point &point)
    {
        max->X = point.X;
        max->Y = point.Y;
    }
    void setMaxDistanceValue(double distance)
    {
        max->distance = distance;
    }

    double getMaxDistanceValue()
    {
        return max->distance;
    }

    void setMinDistancePoint(Point &point)
    {
        min->X = point.X;
        min->Y = point.Y;
    }
    Point *getMinDistancePoint()
    {
        return new Point(min->X, min->Y);
    }
    void setMinDistanceValue(double distance)
    {
        min->distance = distance;
    }

    double getMinDistanceValue()
    {
        return min->distance;
    }

    bool hasCheckpoints()
    {
        return !checkpoints.empty();
    }

    double distanceToNextPoint(Point *p)
    {
        return (next->Y - p->Y) * (next->Y - p->Y) + (next->X - p->X) * (next->X - p->X);
    }

    void setNextCheckpoint(Point *p)
    {
        next = p;
    }
};

class PathPlanningData
{
public:    
    std::map<Point, std::vector<Point>> *connections;
    std::map<Point, Point> *parent;
    std::vector<Point> *planningPathPoints;
    PlanningCheckpoints *planningCheckPoints;

    PathPlanningData()
    {
        this->connections = new std::map<Point, std::vector<Point>>();
        this->parent = new std::map<Point, Point>();
        this->planningPathPoints = new std::vector<Point>();
        this->planningCheckPoints = new PlanningCheckpoints();
    }
    ~PathPlanningData()
    {
        delete this->connections;
        delete this->parent;
        delete this->planningPathPoints;
        delete this->planningCheckPoints;
    }
};

#endif