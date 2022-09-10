#ifndef __CALLBACK_PROCESS_PIPELINE_H
#define __CALLBACK_PROCESS_PIPELINE_H

#include <queue>
#include <mutex>
#include <thread>

#include "../model/stream_data.h"

template <typename T>
class CallbackProcessPipeline
{
private:
    std::mutex *requestNewFrameMutex;
    std::mutex *queueFrameProcess;
    bool loop_run;
    bool frameRequestWaitForAnswer;
    T *frame;
    void (*onProcess)(CallbackProcessPipeline<T> *, T *);
    std::thread *requestFrameThread;
    std::thread *processThread;

    void requestFrameThr()
    {
        while (this->loop_run)
        {
            this->requestNewFrameMutex->lock();
            if (!frameRequestWaitForAnswer)
            {
                frameRequestWaitForAnswer = true;
                this->onRequestNextFrame();
            }
            this->requestNewFrameMutex->unlock();
            // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void processThr()
    {
        if (this->onProcess == nullptr)
            return;


        T *procFrame = nullptr;

        while (this->loop_run)
        {
            this->queueFrameProcess->lock();

            if (this->frame != nullptr)
            {
                procFrame = this->frame;
                this->frame = nullptr;
            }

            this->queueFrameProcess->unlock();

            if (procFrame != nullptr)
            {
                this->onProcess(this, procFrame);
                delete procFrame;
                procFrame = nullptr;
            }
        }
    }

    void replaceFrame(T *newFrame)
    {
        if (this->frame != nullptr)
            delete this->frame;

        this->frame = newFrame;
    }

protected:
    virtual bool initialize() = 0;
    virtual void onRequestNextFrame() = 0;
    virtual void onTerminate() = 0;

    virtual bool isConnected()
    {
        return loop_run;
    }

public:
    CallbackProcessPipeline()
    {
        this->requestNewFrameMutex = new std::mutex();
        this->queueFrameProcess = new std::mutex();

        loop_run = false;
        frameRequestWaitForAnswer = false;
        frame = NULL;
    }
    ~CallbackProcessPipeline()
    {
        delete this->requestNewFrameMutex;
        delete this->queueFrameProcess;

        if (this->requestFrameThread != nullptr)
            delete this->requestFrameThread;

        if (this->processThread != nullptr)
            delete this->processThread;
    }

    void reportNewFrameReceived(T *frame)
    {
        printf("reportNewFrameReceived()\n");
        this->requestNewFrameMutex->lock();
        frameRequestWaitForAnswer = false;
        replaceFrame(frame);
        this->requestNewFrameMutex->unlock();
    }

    CallbackProcessPipeline<T> *withOnProcessCallback(void (*onProcess)(CallbackProcessPipeline<T> *, T *))
    {
        this->onProcess = onProcess;
        return this;
    }

    void run(bool block = true)
    {
        loop_run = initialize();
        requestFrameThread = new std::thread(&CallbackProcessPipeline::requestFrameThr, this);
        processThread = new std::thread(&CallbackProcessPipeline::processThr, this);

        if (block)
            wait();
    }

    void wait()
    {
        if (requestFrameThread != nullptr)
            requestFrameThread->join();

        if (processThread != nullptr)
            processThread->join();

        requestFrameThread = nullptr;
        processThread = nullptr;
    }

    void terminate()
    {
        loop_run = false;
    }
};

#endif