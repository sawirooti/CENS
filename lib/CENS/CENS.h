#pragma once

#include "Map.h"

struct Position
{
    float x;
    float y;
    float theta;
    bool valid = true;

    Position()
        : x(0.0f),
          y(0.0f),
          theta(0.0f){
    }

    Position(float xValue, float yValue, float thetaValue)
        : x(xValue),
          y(yValue),
          theta(thetaValue) {
    }
};

class CENS
{
public:
    CENS(const Map& referenceMap) : _referenceMap(referenceMap) {}

    Position calculatePosition(float distanceFront, float distanceRight) const;

private:
    const Map& _referenceMap;

    static float calculateCorrelation(float measuredFront,
                                    float measuredRight,
                                    float referenceFront,
                                    float referenceRight);

    int findExtremum(float distanceFront,
                     float distanceRight,
                     float *maximumCorrelation = 0) const;
};
