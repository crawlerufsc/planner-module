#ifndef _PLANNER_H
#define _PLANNER_H

#include "../model/stream_data.h"

/**
 * Interface for path planning handlers
 */
class Planner
{
public:
    virtual void plan(StreamData *data) = 0;
};

#endif