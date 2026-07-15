#include "CENS.h"

float CENS::calculateCorrelation(float measuredFront,
                                 float measuredRight,
                                 float referenceFront,
                                 float referenceRight)
{
    if (measuredFront < 0.0f || measuredRight < 0.0f ||
        referenceFront < 0.0f || referenceRight < 0.0f)
    {
        return -1.0f;
    }

    const float scalarProduct =
        measuredFront * referenceFront + measuredRight * referenceRight;

    const float totalEnergy =
        measuredFront * measuredFront + measuredRight * measuredRight +
        referenceFront * referenceFront + referenceRight * referenceRight;

    // глупое деление на ноль
    if (totalEnergy == 0.0f)
    {
        return 1.0f;
    }

    float correlation = 2.0f * scalarProduct / totalEnergy;

    if (correlation > 1.0f)
    {
        return 1.0f;
    }
    return correlation;
}

int CENS::findExtremum(float distanceFront,
                       float distanceRight,
                       float *maximumCorrelation) const
{
    if (distanceFront < 0.0f || distanceRight < 0.0f ||
        _referenceMap.getCount() <= 0)
    {
        return -1;
    }

    int maximumIndex = -1;
    float maximum = -1.0f;

    for (int i = 0; i < _referenceMap.getCount(); ++i)
    {
        const Point point = _referenceMap.getPoint(i);
        const float correlation =
            calculateCorrelation(distanceFront,
                                 distanceRight,
                                 point.dist_front,
                                 point.dist_right);

        if (correlation > maximum)
        {
            maximum = correlation;
            maximumIndex = i;
        }
    }

    if (maximumCorrelation != 0)
    {
        *maximumCorrelation = maximum;
    }

    return maximumIndex;
}

CENSResult CENS::calculatePosition(float distanceFront,
                                   float distanceRight) const
{
    CENSResult result;
    float maximumCorrelation = -1.0f;
    const int maximumIndex =
        findExtremum(distanceFront, distanceRight, &maximumCorrelation);

    if (maximumIndex < 0)
    {
        result.valid = false;
        return result;
    }

    const Point point = _referenceMap.getPoint(maximumIndex);
    result.x = point.x;
    result.y = point.y;
    result.valid = true;
    return result;
}
