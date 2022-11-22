#ifndef _PLANNING_WAYPOINT_H
#define _PLANNING_WAYPOINT_H

class Point
{
public:
    int X;
    int Y;

    Point(int x, int y) : X(x), Y(y) {}
    Point() : X(0), Y(0) {}

    // operator=(const Point &__p)
    // {
    //     X = __p.X;
    //     Y = __p.Y;
    //     return *this;
    // }
};

    inline constexpr bool operator==(const Point &__x, const Point &__y)
    {
        return __x.X == __y.X && __x.Y == __y.Y;
    }

    inline constexpr bool operator<(const Point &__x, const Point &__y)
    {
        return __x.Y < __y.Y || (!(__y.Y < __x.Y) && __x.X < __y.X);
    }

    inline constexpr bool operator!=(const Point &__x, const Point &__y)
    {
        return !(__x == __y);
    }

    inline constexpr bool operator>(const Point &__x, const Point &__y)
    {
        return __y < __x;
    }

    inline constexpr bool operator<=(const Point &__x, const Point &__y)
    {
        return !(__y < __x);
    }

    inline constexpr bool operator>=(const Point &__x, const Point &__y)
    {
        return !(__x < __y);
    }


class ReferencedPoint : public Point
{
public:
    double distance;

    ReferencedPoint(int x, int y, double _distance) : Point(x, y), distance(_distance) {}
    ReferencedPoint() : Point(), distance(-1) {}
};

#endif