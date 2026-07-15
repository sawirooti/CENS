#pragma once

#include "Defenition.hpp"

struct Point
{
    float dist_front;
    float dist_right;
    float x;
    float y;
};

class Map
{
public:
    static const int X_POINT_COUNT = WIDTH / DELTA + 1;
    static const int Y_POINT_COUNT = LENGTH / DELTA + 1;
    static const int POINT_COUNT = X_POINT_COUNT * Y_POINT_COUNT;

    // Карта уже сформирована во flash, поэтому дополнительный расчёт не нужен.
    void calculateMap() {}

    int getCount() const;
    Point getPoint(int index) const;

private:
    static const Point points[POINT_COUNT];
};
