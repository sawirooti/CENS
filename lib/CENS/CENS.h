#pragma once

#include "Map.hpp"

struct CENSResult
{
    float x;
    float y;
    bool valid;
};

class CENS
{
public:
    CENS(const Map& referenceMap) : _referenceMap(referenceMap) {}

    CENSResult calculatePosition(float distanceFront, float distanceRight) const;

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
