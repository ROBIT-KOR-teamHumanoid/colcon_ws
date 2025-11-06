#include "../include/robocup_master25/supportspotcalculator.hpp"

using namespace std;

SupportSpotCalculator::~SupportSpotCalculator()
{
}

SupportSpotCalculator::SupportSpotCalculator(int numX, int numY)
{
    Point PlayingField = Point(900, 600);

    double HeightOfSSRegion = PlayingField.y() * 0.8;
    double WidthOfSSRegion = PlayingField.x() * 0.9;
    double SliceX = WidthOfSSRegion / numX;
    double SliceY = HeightOfSSRegion / numY;

    double left = (PlayingField.x() - WidthOfSSRegion) / 2.0 + SliceX / 2.0;
    double top = (PlayingField.y() - HeightOfSSRegion) / 2.0 + SliceY / 2.0;

    for (int x = 0; x < numX; x++)
    {
        for (int y = 0; y < numY; y++)
        {
            m_Spots.push_back(SupportSpot(Point(static_cast<int>(left + x * SliceX),
                                                static_cast<int>(top + y * SliceY)),
                                          0.0));
        }
    }
}

Point SupportSpotCalculator::DetermineBestSupportingPosition()
{
    if (m_pBestSupportingSpot)
    {
        return m_pBestSupportingSpot->m_vPos;
    }

    m_pBestSupportingSpot = nullptr;

    double BestScoreSoFar = 0.0;

    std::vector<SupportSpot>::iterator curSpot;

    for (curSpot = m_Spots.begin(); curSpot != m_Spots.end(); ++curSpot)
    {
        curSpot->m_dScore = 1.0;

        if (curSpot->m_dScore > BestScoreSoFar)
        {
            BestScoreSoFar = curSpot->m_dScore;

            m_pBestSupportingSpot = &(*curSpot);
        }
    }

    return m_pBestSupportingSpot->m_vPos;
}

Point SupportSpotCalculator::GetBestSupportingSpot()
{
    if (m_pBestSupportingSpot)
    {
        return m_pBestSupportingSpot->m_vPos;
    }

    else
    {
        return DetermineBestSupportingPosition();
    }
}
