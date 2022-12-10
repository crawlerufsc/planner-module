#ifndef __CALLBACK_PROCESS_PIPELINE_H
#define __CALLBACK_PROCESS_PIPELINE_H

#include <queue>
#include <mutex>
#include <thread>
#include <frame.h>

template <typename FrameType>
class CallbackProcessPipeline
{
private:
    std::mutex *frameMutex;
    bool loop_run;
    bool frameRequestWaitForAnswer;
    Frame<FrameType> *frame;
    void (*onProcess)(CallbackProcessPipeline<T> *, T *);
    std::thread *requestFrameThread;
    std::thread *processThread;

    void requestFrameThr()
    {
        while (this->loop_run)
        {
            if (frameRequestWaitForAnswer)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            this->frameMutex->lock();
            frameRequestWaitForAnswer = true;
            this->onRequestNextFrame();
            this->frameMutex->unlock();
        }
    }

    void processThr()
    {
        if (this->onProcess == nullptr)
            return;

        while (this->loop_run)
        {
            T *procFrame = nullptr;

            this->frameMutex->lock();
            if (this->frame != nullptr)
            {
                procFrame = this->frame;
                this->frame = nullptr;
            }
            this->frameMutex->unlock();

            if (procFrame != nullptr)
            {
                this->onProcess(this, procFrame);
                delete procFrame;
            }
        }
    }

    void replaceFrame(T *newFrame)
    {
        if (this->frame != nullptr)
        {
            printf("deleting addr: %p\n", this->frame);
            delete this->frame;
        }
        else
        {
            printf("nullptr frame\n");
        }

        this->frame = newFrame;
    }

protected:
    virtual bool initialize() = 0;
    virtual void onRequestNextFrame() = 0;
    virtual void onTerminate() = 0;

public:
    CallbackProcessPipeline()
    {
        this->frameMutex = new std::mutex();

        loop_run = false;
        frameRequestWaitForAnswer = false;
        frame = new Frame<FrameType>();
    }
    ~CallbackProcessPipeline()
    {
        delete this->frameMutex;
        delete frame;

        if (this->requestFrameThread != nullptr)
            delete this->requestFrameThread;

        if (this->processThread != nullptr)
            delete this->processThread;
    }
    virtual bool isRunning()
    {
        return loop_run;
    }

    void reportNewFrameReceived()
    {
        this->frameMutex->lock();       
        frameRequestWaitForAnswer = false;
        this->frameMutex->unlock();
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