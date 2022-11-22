#ifndef _EXECUTION_CONTROL_H
#define _EXECUTION_CONTROL_H

#include <stdio.h>
#include <crawler_hal.h>

enum PlannerState {
    INIT = 1,
    MANUAL_CONTROL
};

class PlannerExecutionController {
private:
    CrawlerHAL *crawlerHAL;
    PlannerState state;
    void mainThr();
    void manualControlThr();


public:
    PlannerExecutionController(CrawlerHAL *hal);
    void run();

};


#endif