#include <gtest/gtest.h>
#include "../../model/frame.h"
#include <stdlib.h>
#include <opencv2/opencv.hpp>

TEST(FrameTest, TestFrameBasic)
{
    Frame<int> *frame = Frame<int>::newEmptyFrame(400, 300);

    EXPECT_EQ(400, frame->width);
    EXPECT_EQ(300, frame->height);

    frame->set(0, 0, 10);
    frame->set(0, 1, 20);
    frame->set(1, 0, 30);

    EXPECT_EQ(0, frame->get(1, 1));
    EXPECT_EQ(10, frame->get(0, 0));
    EXPECT_EQ(20, frame->get(0, 1));
    EXPECT_EQ(30, frame->get(1, 0));

    delete frame;
}

TEST(FrameTest, TestFrameBuildFromData)
{
    int *data = new int[10]();

    for (int i = 0; i < 10; i++)
        data[i] = i;

    Frame<int> *frame = new Frame<int>(5, 2, data, 0);

    EXPECT_EQ(5, frame->width);
    EXPECT_EQ(2, frame->height);

    for (int h = 0; h < 2; h++)
        for (int w = 0; w < 5; w++)
        {
            EXPECT_EQ(h * 5 + w, frame->get(h, w));
        }

    delete frame;
}

TEST(FrameTest, TestFrameCopyFrom)
{
    int *data = new int[10]();

    for (int i = 0; i < 10; i++)
        data[i] = i;

    Frame<int> *frame1 = new Frame<int>(5, 2, data, 0);
    Frame<int> *frame2 = Frame<int>::newEmptyFrame(5, 2);

    frame2->copyFrom(frame1);

    for (int h = 0; h < 2; h++)
        for (int w = 0; w < 5; w++)
            EXPECT_EQ(frame1->get(h, w), frame2->get(h, w));
}

TEST(FrameTest, TestFrameFuncOperator)
{
    int *data = new int[10]();

    for (int i = 0; i < 10; i++)
        data[i] = i;

    Frame<int> frame(5, 2, data, 0);

    for (int h = 0; h < 2; h++)
        for (int w = 0; w < 5; w++)
            EXPECT_EQ(frame(h, w), frame.get(h, w));
}

TEST(FrameTest, TestFrameCVMatrix)
{
    int *data = (int *)malloc(sizeof(int) * 10);

    for (int i = 0; i < 10; i++)
        data[i] = i;

    Frame<int> frame(5, 2, data, 0);

    cv::Mat *m = frame.getMatrix(CV_32S);

    EXPECT_EQ(1, m->channels());

    for (int h = 0; h < 2; h++)
        for (int w = 0; w < 5; w++)
            EXPECT_EQ(m->at<int>(h, w), frame.get(h, w));
}
