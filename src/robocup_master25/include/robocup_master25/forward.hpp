#ifndef FORWARD_HPP
#define FORWARD_HPP

#include "player.hpp"

#define SECTOR_TOP 0
#define SECTOR_MID 1
#define SECTOR_BOT 2

#define STATE_PLAYING_NORMAL 1

#define NORMAL_STATE_CONTROLL 0
#define NORMAL_STATE_SUPPORT  1
#define NORMAL_STATE_RECIEVE  2
#define NORMAL_STATE_APPROX   3

namespace robocup_master25 {

class Forward : public Player
{
public:
    Forward(std::shared_ptr<robocup_master25::MasterRcko> master);

    void stateInitial();
    void stateReady();
    void stateSet();
    void statePlay();
    void stateFinished();
// ==================== SupportSpot ====================
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
    void initSSC(int numX, //initSupportSpotCalculator
                 int numY);

    Point DetermineBestSupportingPosition();
    Point GetBestSupportingSpot();
    void Render();

// ==================== Base ====================

private:
    int64_t isKickoff;
    int64_t mySide;
    int sector;

    double error_deg;
    bool m_bStatePlayEnter = true;

    Point p_kickoff_top_L = Point(440, 220);
    Point p_kickoff_top_R = Point(660, 220);
    Point p_kickoff_mid_L = Point(500, 350);
    Point p_kickoff_mid_R = Point(600, 450);
    Point p_kickoff_bot_L = Point(460, 580);
    Point p_kickoff_bot_R = Point(640, 580);

    Point p_top_L = Point(460, 220);
    Point p_top_R = Point(640, 220);
    Point p_mid_L = Point(350, 400);
    Point p_mid_R = Point(750, 400);
    Point p_bot_L = Point(460, 580);
    Point p_bot_R = Point(640, 580);

    Point p_top_1_L = Point(460, 220);
    Point p_top_2_L = Point(350, 220);
    Point p_mid_1_L = Point(500, 350);
    Point p_mid_2_L = Point(450, 450);
    Point p_mid_3_L = Point(400, 400);
    Point p_mid_4_L = Point(350, 400);
    Point p_bot_1_L = Point(460, 580);
    Point p_bot_2_L = Point(350, 580);

    Point p_top_1_R = Point(640, 220);
    Point p_top_2_R = Point(750, 220);
    Point p_mid_1_R = Point(600, 350);
    Point p_mid_2_R = Point(650, 350);
    Point p_mid_3_R = Point(700, 400);
    Point p_mid_4_R = Point(750, 400);
    Point p_bot_1_R = Point(640, 580);
    Point p_bot_2_R = Point(750, 580);
};

} // namespace robocup_master24


#endif //FORWARD_HPP
