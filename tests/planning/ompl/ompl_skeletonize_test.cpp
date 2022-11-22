#include <gtest/gtest.h>
#include "../../../planning/ompl_skeletonize.h"
#include <stdlib.h>
#include <set>

TEST(OMPLPlanning, SkeletonizeDataTest)
{
    SkeletonizeData data(200, 300);

    EXPECT_EQ(200, data.skeletonizedOccupancyGrid->width);
    EXPECT_EQ(300, data.skeletonizedOccupancyGrid->height);

    EXPECT_TRUE(data.skeleton_all != nullptr);
    EXPECT_TRUE(data.skeleton_rem != nullptr);
    EXPECT_TRUE(data.skeleton_visited != nullptr);
    EXPECT_TRUE(data.queue != nullptr);

    EXPECT_EQ(0, data.skeleton_all->size());
    EXPECT_EQ(0, data.skeleton_rem->size());
    EXPECT_EQ(0, data.skeleton_visited->size());
    EXPECT_EQ(0, data.queue->size());

    data.skeleton_all->insert(Point(1, 1));
    data.skeleton_rem->insert(Point(1, 1));
    data.skeleton_visited->insert(Point(1, 1));
    data.queue->push(Point(1, 1));

    EXPECT_EQ(1, data.skeleton_all->size());
    EXPECT_EQ(1, data.skeleton_rem->size());
    EXPECT_EQ(1, data.skeleton_visited->size());
    EXPECT_EQ(1, data.queue->size());

    data.clear();

    EXPECT_EQ(0, data.skeleton_all->size());
    EXPECT_EQ(0, data.skeleton_rem->size());
    EXPECT_EQ(0, data.skeleton_visited->size());
    EXPECT_EQ(0, data.queue->size());
}