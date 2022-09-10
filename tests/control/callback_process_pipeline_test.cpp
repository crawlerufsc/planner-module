#include <gtest/gtest.h>
#include "../../control/callback_process_pipeline.h"
#include <stdlib.h>

class TestProcessPipeline : public CallbackProcessPipeline<int>
{
public:
    bool wasInitialized;
    bool requestNextFrameSuccess;
    bool terminateSuccess;
    bool onProcessCallbackCalled;

    TestProcessPipeline()
    {
        wasInitialized = false;
        requestNextFrameSuccess = false;
        terminateSuccess = false;
        onProcessCallbackCalled = false;
    }

    bool initialize() override
    {
        wasInitialized = true;
        return true;
    }
    void onRequestNextFrame() override
    {
        requestNextFrameSuccess = true;
    }
    void onTerminate() override
    {
        terminateSuccess = true;
    }

    static void processCallback(CallbackProcessPipeline<int> *proc, int *data)
    {
        ((TestProcessPipeline *)proc)->onProcessCallbackCalled = true;
        EXPECT_EQ(20, *data);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
};

TEST(PipelineTest, TestPipelineRunSuccess)
{
    TestProcessPipeline proc;
    proc.withOnProcessCallback(TestProcessPipeline::processCallback);
    proc.run(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    EXPECT_TRUE(proc.wasInitialized);
    EXPECT_TRUE(proc.requestNextFrameSuccess);
    EXPECT_FALSE(proc.onProcessCallbackCalled);
    int *frame = new int(20);
    proc.reportNewFrameReceived(frame);
    proc.terminate();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    proc.wait();

    EXPECT_TRUE(proc.wasInitialized);
    EXPECT_TRUE(proc.requestNextFrameSuccess);
    EXPECT_TRUE(proc.onProcessCallbackCalled);
}
TEST(PipelineTest, TestPipelineRunNullFrame)
{
    TestProcessPipeline proc;
    proc.withOnProcessCallback(TestProcessPipeline::processCallback);
    proc.run(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    EXPECT_TRUE(proc.wasInitialized);
    EXPECT_TRUE(proc.requestNextFrameSuccess);
    EXPECT_FALSE(proc.onProcessCallbackCalled);

    int *frame = new int(20);
    proc.reportNewFrameReceived(nullptr);
    proc.terminate();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    proc.wait();

    EXPECT_TRUE(proc.wasInitialized);
    EXPECT_TRUE(proc.requestNextFrameSuccess);
    EXPECT_FALSE(proc.onProcessCallbackCalled);
}

