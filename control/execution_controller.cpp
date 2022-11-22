#include "execution_controller.h"

PlannerExecutionController::PlannerExecutionController(CrawlerHAL *hal) {
    this->crawlerHAL = hal;
    this->state = INIT;
}

void PlannerExecutionController::manualControlThr() {
    
}

void PlannerExecutionController::mainThr() {
    switch (state)
    {
    case INIT:
        break;
    
    default:
        break;
    }
}

void PlannerExecutionController::run() {

}