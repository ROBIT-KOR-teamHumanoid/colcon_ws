// GK Version 1.5

#ifndef GOALKEEPER_HPP
#define GOALKEEPER_HPP

#include "player.hpp"
#include <cmath>

// Playing Mode Define
#define MODE_Penalty 6
#define MODE_TEST 7

#define Just_My_Ball 0
#define Judging_Ball_D 1
#define three_back 2
// Team define
#define RED true
#define BLUE false

#define MOTION_CROUCH_1 0x07
#define MOTION_CROUCH_2 0x08
#define KEEPER_STATE_SIDEBALL 4
#define KEEPER_STATE_OUT_OF_BOX 6
#define KEEPER_STATE_JUST_KICK 8
#define KEEPER_STATE_CROUCH 9
#define KEEPER_STATE_NOBALL 13


namespace robocup_master25
{

    class Goalkeeper : public Player
    {
    public:
        Goalkeeper(std::shared_ptr<robocup_master25::MasterRcko> master);
        void selectRobotState(bool isPenalty);

    private:
        void stateInitial() override;
        void stateReady() override;
        void stateSet() override;
        void statePlay() override;
        void stateFinished() override;

        // keeper value;
        const int goalOffset = 65; // cm

        // 35
        const int frontOffset = 120; // cm
        bool dribble = false;
        int targetyaw;
        double gkbs;
        bool qb = false;
        Point homePoint;
        // Normal TypeFunction
        bool moveSideOnly(Point targetPoint);
        int PenaltySave();
        int playSaveMotion();
        void directionprt(int errorx);
        void gksave();
        bool TargetYaw(float robot_yaw, float target_yaw, int z);
        int stop_cnt = 0;
        int noball_cnt = 0;
        double Ball2Robot;
        double Robot2Box;
        double previous_ball_x = 0;
        double previous_ball_y = 0;
        double Ball2Opponent();
        double Ball2Opponent_Dist = 0;
        bool playCrouchMotion(int motion_num_1);
        bool crouch_flag = false;
        int motion_cnt = 0;
        float previous_yaw = 0;
        int test_cnt = 0;
        bool up_flag = false;
        bool motion_end = false;
    };

} // namespace robocup_master23

#endif // GOALKEEPER_HPP
