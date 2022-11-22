#include <gtest/gtest.h>
#include "../../model/planning_waypoint.h"
#include <stdlib.h>
#include <set>

TEST(PlanningWaypoint, TestPointInit)
{
    Point p1;
    Point p2(1, 2);

    EXPECT_EQ(0, p1.X);
    EXPECT_EQ(0, p1.Y);
    EXPECT_EQ(1, p2.X);
    EXPECT_EQ(2, p2.Y);
}

TEST(PlanningWaypoint, TestReferencedPointInit)
{
    ReferencedPoint p1;
    ReferencedPoint p2(1, 2, 10);

    EXPECT_EQ(0, p1.X);
    EXPECT_EQ(0, p1.Y);
    EXPECT_EQ(-1, p1.distance);
    EXPECT_EQ(1, p2.X);
    EXPECT_EQ(2, p2.Y);
    EXPECT_EQ(10, p2.distance);
}

TEST(PlanningWaypoint, TestPoint)
{
    Point p1(1, 1), p2(2, 3);
    Point *p3 = new Point(1, 1);

    EXPECT_LT(p1, p2);
    EXPECT_GT(p2, p1);
    EXPECT_EQ(p1, *p3);
    EXPECT_NE(p1, p2);
    EXPECT_GE(p1, *p3);
    EXPECT_GE(p2, p1);
    EXPECT_LE(p1, *p3);
    EXPECT_LE(p1, p2);
}

TEST(PlanningWaypoint, TestPointIsCompatibleWithSet)
{
    Point p1(1, 1), p2(2, 3), p3(1, 1), p4(2, 3);

    auto p = std::set<Point>();
    p.insert(p1);
    p.insert(p2);
    p.insert(p3);
    p.insert(p4);

    EXPECT_EQ(2, p.size());
}