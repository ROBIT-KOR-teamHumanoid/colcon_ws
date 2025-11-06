#include "../include/robocup_master25/forward.hpp"

namespace robocup_master25 {

using namespace std;

Forward::Forward(std::shared_ptr<robocup_master25::MasterRcko> master)
    : Player(master) {
  cout << endl << "HELLO!!" << endl << endl;
}

void Forward::stateInitial() {
  cout << "FORWARD STATE: INIT" << endl;
  visionPublish(VISION_SCAN_MODE_INIT);

  move(false);
}

void Forward::stateReady() {
  cout << "FORWARD STATE: READY" << endl;

  Point init_point;

#define OPPONENT_KICKOFF_1  // 수비형
// #define OPPONENT_KICKOFF_2  // 상단 공격형
// #define OPPONENT_KICKOFF_3  // 하단 공격형
#define OUR_KICKOFF_1  // 상단 공격형
                       // #define OUR_KICKOFF_2       // 하단 공격형

#ifdef OPPONENT_KICKOFF_1
  p_top_L = p_top_2_L;  // robot 3
  p_mid_L = p_bot_2_L;  // robot 2
  p_bot_L = p_mid_3_L;  // robot 1

  p_top_R = p_mid_3_R;  // robot 1
  p_mid_R = p_top_2_R;  // robot 2
  p_bot_R = p_bot_2_R;  // robot 3
#endif
#ifdef OPPONENT_KICKOFF_2
  p_top_L = p_top_1_L;
  p_mid_L = p_mid_3_L;
  p_bot_L = p_bot_2_L;

  p_top_R = p_top_1_R;
  p_mid_R = p_mid_3_R;
  p_bot_R = p_bot_2_R;
#endif
#ifdef OPPONENT_KICKOFF_3
  p_top_L = p_top_2_L;
  p_mid_L = p_mid_3_L;
  p_bot_L = p_bot_1_L;

  p_top_R = p_top_2_R;
  p_mid_R = p_mid_3_R;
  p_bot_R = p_bot_1_R;
#endif

#ifdef OUR_KICKOFF_1
  p_kickoff_top_L = p_top_1_L;  // 3
  p_kickoff_mid_L = p_bot_2_L;  // 2
  p_kickoff_bot_L = p_mid_2_L;  // 1

  p_kickoff_top_R = p_mid_2_R;  // rb1
  p_kickoff_mid_R = p_top_2_R;  // rb2
  p_kickoff_bot_R = p_bot_1_R;  // rb3
#endif
#ifdef OUR_KICKOFF_2
  p_kickoff_top_L = p_top_2_L;
  p_kickoff_mid_L = p_mid_1_L;
  p_kickoff_bot_L = p_bot_1_L;

  p_kickoff_top_R = p_top_2_R;
  p_kickoff_mid_R = p_mid_1_R;
  p_kickoff_bot_R = p_bot_1_R;
#endif

  switch (master->gameControlData.robotnum) {
    case 1:
      if (master->gameControlData.iskickoff) {
        init_point = (master->gameControlData.myside == LEFT) ? p_kickoff_bot_L
                                                              : p_kickoff_top_R;
      } else {
        init_point =
            (master->gameControlData.myside == LEFT) ? p_bot_L : p_top_R;
      }
      break;
    case 2:
      if (master->gameControlData.iskickoff) {
        init_point = (master->gameControlData.myside == LEFT) ? p_kickoff_mid_L
                                                              : p_kickoff_mid_R;
      } else {
        init_point =
            (master->gameControlData.myside == LEFT) ? p_mid_L : p_mid_R;
      }
      break;
    case 3:
      if (master->gameControlData.iskickoff) {
        init_point = (master->gameControlData.myside == LEFT) ? p_kickoff_top_L
                                                              : p_kickoff_bot_R;
      } else {
        init_point =
            (master->gameControlData.myside == LEFT) ? p_top_L : p_bot_R;
      }
      break;
    default:
      break;
  }

  // 10초 남았을 때 앞에 봄
  if (master->gameControlData.readytime < 10)
    init_point = Point(master->local.robot_x, master->local.robot_y);

  move(init_point, OPPONENT);
  visionPublish(VISION_SCAN_MODE_OPER);
}

void Forward::stateSet() {
  cout << "FORWARD STATE: SET" << endl;
  move(false);
}

void Forward::statePlay() {
  visionPublish(VISION_SCAN_MODE_BALL);
  cout << "FORWARD STATE: PLAY" << endl;

  const int test = 100;
  // robot_state == test;
  switch (robot_state) {
    case test: {
      kick3();
      break;
    }
    case ROBOT_STATE_CONTROLL: {
      if (master->gameControlData.readytime > 0 &&
          !master->gameControlData.iskickoff) {
        walkStop();
        return;
      }  // not our kickoff

      Point ballPosition(master->local.ball_x, master->local.ball_y);

      if (ballPosition.x() == 0 &&
          ballPosition.y() == 0)  // 공이 없는 경우 예외처리
      {
        cout << "DETECT BALL" << endl;
        walkStart(0, 0, 5);
        return;
      } else {
        kick3();
      }

      // dribble();
      break;
    }
    case ROBOT_STATE_SUPPORT: {
      Point p;
#define /*SUPPORT_SPOT*/ MANNUAL
#ifdef SUPPORT_SPOT
      // 서포트 스팟 정하는 부분
      initSSC(20, 10);
      p = GetBestSupportingSpot();
      //        Render();
      m_Spots.clear();

      cout << "BestSupportingSpot: (" << p.x() << ", " << p.y() << ")" << endl;
#endif
#ifdef MANNUAL
      // 수동으로 위치 선정
      int myPoint;
      switch (master->gameControlData.robotnum) {
        case 1:
          myPoint = 14;
          break;
        case 2:
          myPoint = 6;
          break;
        case 3:
          myPoint = 11;
      }
      p = setPoint(myPoint);
      cout << "@@@@@@ move P x , y : " << p.x() << " " << p.y() << endl;
#endif
      // 도착 지점에서 볼 각도 정하는 부분
#define UPDOWN /*ANGLEOPPONENT*/
      int angle;
#ifdef UPDOWN  // 맵 위쪽에서는 아래쪽을 보고 아래쪽에서는 위쪽을 본다. (로봇이
               // 일부만 있을 때 효과있음)
      if (master->local.robot_y < 400)
        angle = 180;
      else
        angle = 1;
#endif
#ifdef ANGLEOPPONENT  // 항상 상대 위치를 본다 (로봇이 모두 다 들어와 있을 때
                      // 효과있음)
      angle = master->local.robot_x > 550 ? 90 : -90;
#endif

      //        if(master->gameControlData.myside == LEFT) {
      //            if(master->local.robot_x > 550) {
      //                angle = master->local.robot_x > 550 ? 90 : -90;
      //            } else {
      //                if(master->local.robot_y < 400) angle = 180;
      //                else angle = 1;
      //            }
      //        } else {
      //            if(master->local.robot_x > 550) {
      //                if(master->local.robot_y < 400) angle = 180;
      //                else angle = 1;
      //            } else {
      //                angle = master->local.robot_x > 550 ? 90 : -90;
      //            }
      //        }

      // 원하는 위치로 이동
      move(p, ABSOLUTE, COORDINATE, angle);

      break;
    }
  }
}

void Forward::stateFinished() {
  cout << "FORWARD STATE: FINISH" << endl;
  visionPublish(VISION_SCAN_MODE_INIT);
  move(false);
}

// ================== SUPPORT SPOT SYSTEM ===================

void Forward::initSSC(int numX, int numY) {
  m_pBestSupportingSpot = nullptr;

  Point PlayingField = Point(500, 500);

  double HeightOfSSRegion = PlayingField.y() * 0.8;
  double WidthOfSSRegion = PlayingField.x() * 0.9;
  double SliceX = WidthOfSSRegion / numX;
  double SliceY = HeightOfSSRegion / numY;

  //    double left = (PlayingField.x()-WidthOfSSRegion)/2.0 + SliceX/2.0;
  //    double top  = (PlayingField.y()-HeightOfSSRegion)/2.0 + SliceY/2.0;
  double left = 300;
  double top = 150;

  for (int y = 0; y < numY; y++) {
    for (int x = 0; x < numX; x++) {
      m_Spots.push_back(
          SupportSpot(Point(static_cast<int>(left + x * SliceX) + 100,
                            static_cast<int>(top + y * SliceY) + 100),
                      0.0));
    }
  }
}

Point Forward::DetermineBestSupportingPosition() {
  //    const int Prm_DistFromControllingPlayerScore = 10;

  if (m_pBestSupportingSpot) {
    return m_pBestSupportingSpot->m_vPos;
  }

  m_pBestSupportingSpot = nullptr;

  std::vector<SupportSpot>::iterator curSpot;

  // Test spots
  for (curSpot = m_Spots.begin(); curSpot != m_Spots.end(); ++curSpot) {
    curSpot->m_dScore = 1.0;

    // Test 1. 공 위치에 따른 가산점
    //          - 공 위치에 따라서 아군 로봇의 위치 가중치를 공격 위치 혹은 방어
    //          위치에 둘지를 결정함
    //          - 점수 배점 실수로 너무 공격적인 위치에만 있음: 지금은 사용하지
    //          않음
    if (false) {
      Point opponentGoalPosition =
          opponent_goal;  // Position of the opponent's goal
      Point ballPosition = Point(master->local.ball_x,
                                 master->local.ball_y);  // Position of the ball

      Point spotPosition = curSpot->m_vPos;  // Current spot position

      double distToOpponentGoal;
      calcDistance(opponentGoalPosition, spotPosition,
                   distToOpponentGoal);  // Calculate the distance between the
                                         // opponent's goal and the spot

      const double minDist = 100.0;
      const double maxDist = 1044.03;

      double adjustedDistance = (maxDist - distToOpponentGoal) /
                                (maxDist - minDist);  // Adjust the distance

      double distToBall;
      calcDistance(ballPosition, opponentGoalPosition,
                   distToBall);  // Calculate the distance between the ball and
                                 // the opponent's goal

      double score = 10 * adjustedDistance;  // Calculate the base score

      double weight =
          1.0 - (distToBall / maxDist);  // Weight for the distance between the
                                         // ball and the opponent's goal
      double adjustedScore = score * weight;  // Adjust the score meaning

      curSpot->m_dScore += adjustedScore;  // Update the spot's score
    }

    // Test 2. 로봇간 거리 유지 조건
    //           - 아군 로봇과 일정 거리를 유지하도록 배점
    if (true) {
      int NumberOfTests = 0;
      vector<int> RobotIndex;
      for (int i = 0; i < MAX_ROBOT_NUM; i++) {
        if (master->udp[i].robotcase != ROBOT_STATE_NONE) {
          NumberOfTests++;
          RobotIndex.push_back(i);
        }
      }

      for (int i = 0; i < NumberOfTests; i++) {
        const double OptimalDistance = 400;

        Point positionOfControlPlayer;
        positionOfControlPlayer = Point(master->udp[RobotIndex.at(i)].localx,
                                        master->udp[RobotIndex.at(i)].localy);

        double distToControlPlayer;
        calcDistance(positionOfControlPlayer, curSpot->m_vPos,
                     distToControlPlayer);

        double distToMyLocation;
        Point positionOfMyLocation =
            Point(master->local.robot_x, master->local.robot_y);
        calcDistance(positionOfMyLocation, curSpot->m_vPos, distToMyLocation);

        double temp = fabs(OptimalDistance - distToControlPlayer);

        if (temp < OptimalDistance) {
          double weight = 10;
          double minScore = 0;
          double maxScore = 10;
          double score = (OptimalDistance - temp) / OptimalDistance;
          if (score >= maxScore)
            score = maxScore;
          else if (score <= minScore)
            score = minScore;
          curSpot->m_dScore +=
              weight * (score - minScore) / (maxScore - minScore);
        }
      }
    }

    // Test 3. 상대 로봇 고려 조건
    //           - 상대 로봇의 위치와 가까운 지점의 점수를 마이너스
    //           - 로컬에서 상대 위치를 보내주기 전까지 프로토타입 개발
    //           - 비전에서 보는 로봇이 아군인지 상대인지 알 수 없음 그러나 차피
    //           아군 로봇에도 붙으면 안되니까 같이 감점해도 괜찮음
    //           - 아군 필드에 있는 상대 로봇은 제외한다.

    bool test3 = true;

    if (master->gameControlData.myside == LEFT) {
      if (curSpot->m_vPos.x() < 550) test3 = false;
    } else {
      if (curSpot->m_vPos.x() > 550) test3 = false;
    }

    if (test3) {
      Point Obstacle[4];
      Obstacle[0] = Point(master->local.obstacle0_x, master->local.obstacle0_y);
      Obstacle[1] = Point(master->local.obstacle1_x, master->local.obstacle1_y);
      Obstacle[2] = Point(master->local.obstacle2_x, master->local.obstacle2_y);
      Obstacle[3] = Point(master->local.obstacle3_x, master->local.obstacle3_y);

      for (int i = 0; i < 4; i++) {
        const double thresholdDistance = 300;

        Point positionOfOtherPlayer;
        positionOfOtherPlayer = Obstacle[i];
        if (positionOfOtherPlayer.x() == 0.0 &&
            positionOfOtherPlayer.y() == 0.0) {
          break;
        }

        double distToOtherPlayer;
        calcDistance(positionOfOtherPlayer, curSpot->m_vPos, distToOtherPlayer);

        if (distToOtherPlayer < thresholdDistance) {
          double weight = 100;
          if (distToOtherPlayer <= 1) distToOtherPlayer = 1;
          double score = 1 / distToOtherPlayer;
          curSpot->m_dScore -= weight * score;
        }
      }
    }

    // Test 4. 공수 결정 조건
    //           - 상대 골대와의 거리에 따라서 점수 부여
    //           - 점수를 더하거나 빼서 공수를 결정 할 수 있다.
    if (true) {
      Point opponentGoalPosition =
          opponent_goal;                     // Position of the opponent's goal
      Point spotPosition = curSpot->m_vPos;  // Current spot position

      double distToOpponentGoal;
      calcDistance(opponentGoalPosition, spotPosition,
                   distToOpponentGoal);  // Calculate the distance between the
                                         // opponent's goal and the spot

      const double minDist = 100.0;
      const double maxDist = 1044.03;

      double adjustedDistance = (maxDist - distToOpponentGoal) /
                                (maxDist - minDist);  // Adjust the distance

      double score = 0.5 * adjustedDistance;  // Calculate the base score

      curSpot->m_dScore -= score;  // 수비 조건으로 활성화
    }
  }

  SupportSpot *closestSpot = nullptr;
  double highestScore = std::numeric_limits<double>::lowest();

  for (SupportSpot &spot : m_Spots) {
    // Check if the spot's score is higher than the current highest score
    if (spot.m_dScore > highestScore) {
      highestScore = spot.m_dScore;
      closestSpot = &spot;
    }
  }

  if (closestSpot != nullptr) {
    // Perform additional filtering to select the closest spot within the top
    // 90% of scores
    double scoreThreshold = highestScore * 0.95;
    double closestDistance = std::numeric_limits<double>::max();

    for (SupportSpot &spot : m_Spots) {
      // Only consider spots with scores above the threshold
      if (spot.m_dScore > scoreThreshold) {
        Point myPosition = Point(master->local.robot_x, master->local.robot_y);

        double distance;
        calcDistance(myPosition, spot.m_vPos, distance);
        if (distance < closestDistance) {
          closestDistance = distance;
          closestSpot = &spot;
        }
      }
    }
    // At this point, 'closestSpot' will hold the pointer to the closest spot
    // within the top 90% of scores You can then use it as needed in your code
    m_pBestSupportingSpot = closestSpot;
  }

  return m_pBestSupportingSpot->m_vPos;
}

Point Forward::GetBestSupportingSpot() {
  if (m_pBestSupportingSpot) {
    return m_pBestSupportingSpot->m_vPos;
  }

  else {
    return DetermineBestSupportingPosition();
  }
}

void Forward::Render() {
  int nowy = m_Spots[0].m_vPos.y();
  cout << endl << "RENDER" << endl << endl;
  for (unsigned int spt = 0; spt < m_Spots.size(); ++spt) {
    if (nowy != m_Spots[spt].m_vPos.y()) {
      nowy = m_Spots[spt].m_vPos.y();
      cout << endl << endl << endl << endl;
    }
    //        printf("| %06.3lf |", m_Spots[spt].m_dScore);
    printf("| %6.2lf |", m_Spots[spt].m_dScore);
  }
  cout << endl << endl;
  //    ros::shutdown();
}

// =============== END OF SUPPORT SPOT SYSTEM ===============

}  // namespace robocup_master25
