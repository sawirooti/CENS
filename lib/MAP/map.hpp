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
    float _length = LENGTH,
          _width = WIDTH, 
          _delta = DELTA;
    Point points[250];
    int _count = 0;

    public:

    Map() = default;
    ~Map() = default;

    void calculateMap();

    int getCount() const;
    const Point& getPoint(int index) const;
};

inline void Map::calculateMap()
{
    _count = 0;

    int step_x = _width / _delta;
    int step_y = _length / _delta;

    for (int i = 0; i <= step_x; i++)
    {
        for (int j = 0; j <= step_y; j++)
        {
            points[_count].dist_front = _length - j * _delta;
            points[_count].dist_right = _width - i * _delta;
            points[_count].x = i * _delta;
            points[_count].y = j * _delta;
            _count++;  
        }
    }
};

inline int Map::getCount() const
{
    return _count;
};

inline const Point& Map::getPoint(int index) const
{
    return points[index];
};
