#ifndef SUPPORTSPOTCALCULATOR_H
#define SUPPORTSPOTCALCULATOR_H

#include <vector>
#include <iostream>

#include "point.hpp"

class SupportSpotCalculator
{
private:
    struct SupportSpot
    {
        Point m_vPos;
        double m_dScore;

        SupportSpot(Point pos, double value):   m_vPos(pos),
                                                m_dScore(value)

        {}
    };

private:
    std::vector<SupportSpot>    m_Spots;

    SupportSpot*                m_pBestSupportingSpot;

public:
    SupportSpotCalculator(int numX,
                          int numY);
    ~SupportSpotCalculator();

    Point DetermineBestSupportingPosition();
    Point GetBestSupportingSpot();
};

#endif // SUPPORTSPOTCALCULATOR_H
