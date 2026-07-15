#include "Map.h"

#if defined(ARDUINO_ARCH_AVR)
#include <avr/pgmspace.h>
#define MAP_FLASH PROGMEM
#else
#define MAP_FLASH
#endif

static_assert(WIDTH == 1000 && LENGTH == 1500 && DELTA == 100,
              "Обновите таблицу Map::points после изменения размеров карты");

#define MAP_POINT(xIndex, yIndex)                                           \
    {                                                                       \
        LENGTH - (yIndex) * DELTA, WIDTH - (xIndex) * DELTA,                \
        (xIndex) * DELTA, (yIndex) * DELTA                                  \
    }

#define MAP_COLUMN(xIndex)                                                  \
    MAP_POINT(xIndex, 0), MAP_POINT(xIndex, 1), MAP_POINT(xIndex, 2),       \
    MAP_POINT(xIndex, 3), MAP_POINT(xIndex, 4), MAP_POINT(xIndex, 5),       \
    MAP_POINT(xIndex, 6), MAP_POINT(xIndex, 7), MAP_POINT(xIndex, 8),       \
    MAP_POINT(xIndex, 9), MAP_POINT(xIndex, 10), MAP_POINT(xIndex, 11),     \
    MAP_POINT(xIndex, 12), MAP_POINT(xIndex, 13), MAP_POINT(xIndex, 14),    \
    MAP_POINT(xIndex, 15)

const Point Map::points[Map::POINT_COUNT] MAP_FLASH = {
    MAP_COLUMN(0),
    MAP_COLUMN(1),
    MAP_COLUMN(2),
    MAP_COLUMN(3),
    MAP_COLUMN(4),
    MAP_COLUMN(5),
    MAP_COLUMN(6),
    MAP_COLUMN(7),
    MAP_COLUMN(8),
    MAP_COLUMN(9),
    MAP_COLUMN(10)
};

#undef MAP_COLUMN
#undef MAP_POINT
#undef MAP_FLASH

int Map::getCount() const
{
    return POINT_COUNT;
}

Point Map::getPoint(int index) const
{
    Point point = {0.0f, 0.0f, 0.0f, 0.0f};
    if (index < 0 || index >= POINT_COUNT)
    {
        return point;
    }

#if defined(ARDUINO_ARCH_AVR)
    memcpy_P(&point, &points[index], sizeof(Point));
#else
    point = points[index];
#endif
    return point;
}
