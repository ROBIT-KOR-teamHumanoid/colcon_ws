#include "../include/robocup_master25/goalkeeper.hpp"

#define REG2(X, MIN, MAX) (X - MIN) / (MAX - MIN)
#define come true;
#define far false;
namespace robocup_master25 {

using namespace std;

Goalkeeper::Goalkeeper(std::shared_ptr<robocup_master25::MasterRcko> master)
    : Player(master) {
  // Robocup Korea Yongpyeong 2023 version 1.0
  // Robocup Bordeaux 2023 Goalkeeper version 1.3
}

void Goalkeeper::selectRobotState(bool isPenalty) {
  Point ballPoint(static_cast<int>(master->local.ball_x),
                  static_cast<int>(master->local.ball_y));
  bool keeperZoneX = false;
  double res = sqrt(pow(120 - master->local.robot_x, 2) +
                    pow(400 - master->local.robot_y, 2));

  cout << res << endl;
  if (master->gameControlData.myside == RIGHT) {
    targetyaw = 90;
    if (ballPoint.x() > 1000 - 200)  // origin = 10
    {
      keeperZoneX = true;
    }
  } else {
    targetyaw = -90;
    if (ballPoint.x() < 100 + 200) {
      keeperZoneX = true;
    }
  }

  if (master->gameControlData.myside == RIGHT) {
    Robot2Box = 800 - master->local.robot_x;
  } else {
    Robot2Box = master->local.robot_x - 300;
  }
  Ball2Robot = abs(sqrt(pow(master->local.robot_x - master->local.ball_x, 2) +
                        pow(master->local.robot_y - master->local.ball_y, 2)));

  if (ballPoint.x() != 0 && ballPoint.y() != 0) {
    previous_ball_x = ballPoint.x();
    previous_ball_y = ballPoint.y();
  }

  cout << "BALL.X : " << ballPoint.x() << endl;
  cout << "BALL.Y : " << ballPoint.y() << endl;

  if (master->gameControlData.secondstate == STATE2_PENALTYKICK) {
    robot_state = MODE_Penalty;
  } else if (ballPoint.x() == 0 && ballPoint.y() == 0) {
    robot_state = KEEPER_STATE_NOBALL;
  }
  // origin 250 550
  else if (200 < ballPoint.y() && ballPoint.y() < 600 && keeperZoneX) {
    robot_state = KEEPER_STATE_KEEPERZONE;
  } else if (400 - goalOffset < ballPoint.y() &&
             ballPoint.y() < 400 + goalOffset) {
    robot_state = KEEPER_STATE_CENTER;
  }
  // normal mode
  else {
    robot_state = KEEPER_STATE_SIDEBALL;
  }

  if (Ball2Robot < 20 ||
      (master->vision.ball_d < 300 && master->vision.ball_d > 0)) {
    robot_state = KEEPER_STATE_JUST_KICK;
  }

  if (crouch_flag) {
    robot_state = KEEPER_STATE_CROUCH;
  }

  if (master->local.robot_x >= 310 && master->local.robot_x <= 790) {
    robot_state = KEEPER_STATE_QUICK_BACK;
  }

  if (master->local.robot_y < 120 || master->local.robot_y > 680) {
    robot_state = KEEPER_STATE_QUICK_BACK;
  }
}

void Goalkeeper::stateInitial() {
  cout << "keeper state init" << endl;
  visionPublish(VISION_SCAN_MODE_INIT);
  walkStop();

  // 1000  100
  if (master->gameControlData.myside == RIGHT) {
    homePoint = Point(1100 - frontOffset, 400);
  } else {
    homePoint = Point(0 + frontOffset, 400);
  }
}

void Goalkeeper::stateReady() {
  if (master->gameControlData.myside == RIGHT) {
    homePoint = Point(1100 - frontOffset, 400);
  } else {
    homePoint = Point(0 + frontOffset, 400);
  }

  visionPublish(VISION_SCAN_MODE_BALL);
  cout << "keeper state ready" << endl;

  Player::stateReady(homePoint);
}

void Goalkeeper::stateSet() {
  visionPublish(VISION_SCAN_MODE_BALL);
  cout << "keeper state set" << endl;
  walkStop();
}

// dribble --> <= 200
// save >= 250 && <= 600
void Goalkeeper::statePlay() {
  // Penalty_Kick을 test할때는 밑에 줄을 활성화
  //     robot_state = MODE_Penalty;
  // 아무 함수나 변수 테스트 할 때 사용
  // robot_state = MODE_TEST;

  visionPublish(VISION_SCAN_MODE_BALL);
  cout << "keeper state play" << endl;

  Point ballPoint(static_cast<int>(master->local.ball_x),
                  static_cast<int>(master->local.ball_y));
  Point myPoint(static_cast<int>(master->local.robot_x),
                static_cast<int>(master->local.robot_y));

  cout << "BALL.X : " << ballPoint.x() << endl;
  cout << "BALL.Y : " << ballPoint.y() << endl;

  // for 2024.05.07
  if (master->gameControlData.myside == RIGHT) {
    homePoint = Point(1100 - frontOffset, 400);
  } else {
    homePoint = Point(0 + frontOffset, 400);
  }

  // playSaveMotion();
  // gksave();
  switch (robot_state) {
    case MODE_TEST: {
      // kick3();
      // gksave();

      break;
    }
    case KEEPER_STATE_OUT_OF_BOX: {
      cout << "---------- KEEPER_STATE_OUT_OF_BOX ----------" << endl;
      stop_cnt++;
      if (stop_cnt <= 1000) {
        walkStop();
        cout << "THINKING,,,,,,,,,,,, STOP_CNT : " << stop_cnt << endl;
      } else {
        cout << "Ball2Robot : " << Ball2Robot << ", Robot2Box : " << Robot2Box
             << endl;

        if (Ball2Robot < Robot2Box) {
          kick3();
          cout << "!!!!!!!! JUST KICK !!!!!!!!" << endl;
        } else {
          // walkStart(-30, 0, 2);

          cout << "!!!!!!!! BACK !!!!!!!!" << endl;
        }
      }

      //      if(robot_state != KEEPER_STATE_OUT_OF_BOX)
      //      {
      //        stop_cnt = 0;
      //      }

      break;
    }
    case KEEPER_STATE_NOBALL: {
      cout << "---------- ROBOT_STATE_NOBALL ----------" << endl;
      cout << "HOMEPOINT.X = " << homePoint.x()
           << ", HOMEPOINT.Y = " << homePoint.y() << endl;

      cout << "!!!!!!!! YEAH ROBOT IS GOING HOMEPOINT !!!!!!!!" << endl;

     

      if (moveSideOnly(homePoint)) 
      {
          walkStop();
          cout << "!!!!!!!! YEAH ROBOT IS ON HOMEPOINT !!!!!!!!" << endl;
      }

      break;
    }
    case KEEPER_STATE_KEEPERZONE: {
      cout << "---------- KEEPER_STATE_KEEPERZONE ----------" << endl;
      // 당장 공을 걷어내기
      kick3();
      cout << "!!!!!!!! KICK3 !!!!!!!!" << endl;

      break;
    }
    case KEEPER_STATE_CENTER: {
      cout << "---------- KEEPER_STATE_CENTER ----------" << endl;

      // 공과 직선이 되도록 위치 선정
      int target_x = homePoint.x();
      int target_y = ballPoint.y();

      Point targetPoint(target_x, target_y);
      if ((ballPoint.x() > 310 && master->gameControlData.myside == LEFT) ||
          (ballPoint.x() < 790 && master->gameControlData.myside ==
                                      RIGHT))  // if ball is out of KEEPER_BOX
      {
        if (moveSideOnly(targetPoint)) {
          walkStop();
          cout << "!!!!!!!! YEAH ROBOT IS STOP AND CENTER TO BALL !!!!!!!!"
               << endl;
        }
      } else {
        kick3();
        cout << "!!!!!!!! KICK BALL !!!!!!!!" << endl;
        cout << "master->vision.ball_cam_x : " << master->vision.ball_cam_x
             << endl;
      }
      break;
    }
    case KEEPER_STATE_QUICK_BACK: {
      cout << "---------- KEEPER_STATE_QUICK_BACK ----------" << endl;
      Player::stateReady(homePoint);
      break;
    }
    case KEEPER_STATE_SIDEBALL: {
      cout << "---------- KEEPER_STATE_SIDEBALL ----------" << endl;

      // offset point에기서 공 바라보기
      int target_x = homePoint.x();
      int target_y = ballPoint.y();

      // 400 + 65 = 465, NORMAL mode move side to
      int xLimitH = 400 + goalOffset;
      // 400 - 65 = 335, NORMAL mode move side to
      int xLimitL = 400 - goalOffset;

      if (ballPoint.y() > xLimitH) {
        target_y = xLimitH;
      } else if (ballPoint.y() < xLimitL) {
        target_y = xLimitL;
      }

      Point targetPoint(target_x, target_y);
      if (moveSideOnly(targetPoint)) {
        double robot_yaw = static_cast<int>(master->imu.yaw);
        double targetAngle = calcTargetAngle(ballPoint, myPoint, robot_yaw);
        if (alignRobot(robot_yaw, targetAngle)) 
        {
          walkStop();
        }
      }
      
      break;
    }
    case KEEPER_STATE_JUST_KICK: {
      cout << "---------- KEEPER_STATE_JUST_KICK ----------" << endl;
      kick3();
      cout << "!!!!!!!! JUST KICK !!!!!!!!" << endl;
      break;
    }
      // 나중에 페널티킥 할 때 활성화시키기
      //    case MODE_Penalty:
      //    {
      //        PenaltySave();
      //        break;
      //    }
  }
}

void Goalkeeper::stateFinished() {
  cout << "keeper state finished" << endl;
  walkStop();
}

// int Goalkeeper::playSaveMotion() 
// {
//   //    int doSaveMotion = 0; // 0: 모션 안 함, 1: 오른쪽 방어 모션, 2: 왼쪽
//   //    방어 모션
//   cout << "keeper state Motion" << endl;

//   // 사용 변수 설정
//   double ballSpeedX = master->local.ball_speed_x;
//   double ballSpeedY = master->local.ball_speed_y;
//   double ballSpeedLevel = master->vision.ball_speed_level;
//   int myY = static_cast<int>(master->local.robot_y);
//   int condition = 100;

//   if (master->gameControlData.myside == RIGHT) {
//     ballSpeedX = 1100 - ballSpeedX;
//     ballSpeedY = 800 - ballSpeedY;
//     myY = 800 - myY;
//     condition = 1000;
//   }

//   bool dir = (ballSpeedX > master->local.ball_x) ? true : false;

//   ballSpeedY = ballSpeedY - myY;

//   cout << "ballSpeedX    : " << ballSpeedX << endl;
//   cout << "ballSpeedY    : " << ballSpeedY << endl;
//   cout << "ballSpeedLevel: " << ballSpeedLevel << endl;

//   if (ballSpeedX < condition && ballSpeedLevel > 250 && dir) {
//     directionprt(static_cast<int>(ballSpeedY));
//     int motion;

//     // if (ballSpeedY >= 0) {
//     //   motion = MOTION_ARM_RIGHT;
//     // } else {
//     //   motion = MOTION_ARM_LEFT;
//     // }
//     playMotion(motion);
//   }

//   return 0;
// }

bool Goalkeeper::moveSideOnly(Point targetPoint) {
  cout << "  moveSideOnly  " << endl;

  bool isArrive = false;

  Point myPoint(static_cast<int>(master->local.robot_x),
                static_cast<int>(master->local.robot_y));

  const double arriveThreshold = 10;

  if (abs(targetPoint.y() - myPoint.y()) < arriveThreshold) {
    isArrive = true;
    return isArrive;
  } else {
    // 최종 출력
    int x = 0;
    int y = 0;
    int z = 0;

    // 오차
    double err_x = targetPoint.x() - myPoint.x();
    double err_y = targetPoint.y() - myPoint.y();

    // z값의 목표(reference)는 myside에 따라서 다르다
    double ref_z = master->gameControlData.myside == RIGHT ? 90 : -90;
    double err_z = ref_z - static_cast<double>(master->imu.yaw);

    // 예외처리 하지 않은 순수 오차 출력
    cout << "err_x: " << err_x << endl;
    cout << "err_y: " << err_y << endl;
    cout << "err_z: " << err_z << endl;

    // 오차의 최대값(=최소값)
    const int max_x = -10;
    const int max_y = 30;
    const int max_z = 90;

    // 최대, 최소 값을 넘는 오차 예외처리
    if (err_x > max_x)
      err_x = max_x;
    else if (err_x < -max_x)
      err_x = -max_x;
    if (err_y > max_y)
      err_y = max_y;
    else if (err_y < -max_y)
      err_y = -max_y;
    if (err_z > max_z)
      err_z = max_z;
    else if (err_z < -max_z)
      err_z = -max_z;

    // 정규화
    double reg_err_x = REG2(err_x, 0, max_x);
    double reg_err_y = REG2(err_y, 0, max_y);
    double reg_err_z = REG2(err_z, 0, max_z);

    // 정규화된 오차 출력
    cout << "reg_err_x: " << reg_err_x << endl;
    cout << "reg_err_y: " << reg_err_y << endl;
    cout << "reg_err_z: " << reg_err_z << endl;

    // 각 제어량의 비례 상수 설정
    const double Kp_x = -10;
    const double Kp_y = 30;
    //        const double Kp_z = R_YAW_MAX;
    const double Kp_z = 12;  // origin 10
    // 출력값 결정
    x = Kp_x * reg_err_x;
    y = Kp_y * reg_err_y;
    z = Kp_z * reg_err_z;

    // Side가 바뀌면 YAW방향이 바뀌므로 X, Y는 방향을 바꿔줘야 한다.
    int dir = master->gameControlData.myside == RIGHT ? 1 : -1;

    // 최종 출력
    walkStart(dir * x, dir * y, z);
  }

  return isArrive;
}
void Goalkeeper::directionprt(int errorx) {
  for (int i = -0; i < 5; i++) {
    cout << (errorx >= 0 ? "R                -###############-"
                         : " -###############-                L")
         << endl;
  }
}

int Goalkeeper::PenaltySave() {
  cout << "KEEPER_STATE_PENALTY" << endl << endl;
  double ballcamy = master->vision.ball_cam_y;
  double ballcamx = master->vision.ball_cam_x;
  double bs = master->vision.ball_speed_level;
  double bd = master->vision.ball_d;
  int errorx = static_cast<int>(ballcamx) - 330;  // 230
  int errory = static_cast<int>(ballcamy) - 120;  // 110
  dribble = (bs != 0.0 && bd < 1300 && bd != 0.0)
                ? (bs > 1300 ? false : (bs < 1000 ? true : dribble))
                : dribble;
  cout << "errorx --> " << errorx << endl;
  cout << "errory --> " << errory << endl;
  // Oringinal ball_cam_x, ball_cam_y --> 230 110

  // 로봇이 드리블을 하고있을 때와 하고있지 않을 때의 경우를 나뉘었다.
  if (!dribble) {
    cout << "judging ball clac" << endl;
    if (errory > 20 && ballcamy != 0.0) {
      directionprt(errorx);
      // playMotion(errorx >= 0 ? MOTION_ARM_RIGHT : MOTION_ARM_LEFT);
    }
  } else {
    robot_state = KEEPER_STATE_KEEPERZONE;
  }

  return 0;
}

// void Goalkeeper::gksave() {
//   cout << "keeper state gksave" << endl;
//   double lastballx, lastbally;
//   double ballcamy = master->vision.ball_cam_y;
//   double ballcamx = master->vision.ball_cam_x;
//   double bs = master->vision.ball_speed_level;
//   double bd = master->vision.ball_d;

//   if (bs == 0.0 && bd != 0.0) {
//     lastballx = master->vision.ball_cam_x;
//     lastbally = master->vision.ball_cam_y;

//     cout << "lastballx  -->" << lastballx << endl;
//     cout << "lastbally  -->" << lastbally << endl;
//     cout << "ballcamx   -->" << ballcamx << endl;
//     cout << "ballcamy   -->" << ballcamy << endl;
//     cout << "Ballspeed  -->" << bs << endl;
//     cout << "Balld      -->" << bd << endl;

//     if (master->vision.ball_speed_level >= 1000 &&
//         master->vision.ball_speed_level <= 1500) {
//       double errory = ballcamy - lastbally;
//       double errorx = ballcamx - lastballx;
//       if (errory > 20) {
//         // if (errorx > 20 && ballcamy != 0.0)

//           // playMotion(errorx >= 0 ? MOTION_ARM_RIGHT : MOTION_ARM_LEFT);
//       }
//     }
//   }
// }

double Goalkeeper::Ball2Opponent() {
  Point ballPoint(static_cast<int>(master->local.ball_x),
                  static_cast<int>(master->local.ball_y));
  Point obstacle_0(static_cast<int>(master->local.obstacle0_x),
                   static_cast<int>(master->local.obstacle0_y));
  Point obstacle_1(static_cast<int>(master->local.obstacle1_x),
                   static_cast<int>(master->local.obstacle1_y));
  Point obstacle_2(static_cast<int>(master->local.obstacle2_x),
                   static_cast<int>(master->local.obstacle2_y));
  Point obstacle_3(static_cast<int>(master->local.obstacle3_x),
                   static_cast<int>(master->local.obstacle3_y));

  double Ball2Obs[4] = {
      calcDistance(ballPoint, obstacle_0), calcDistance(ballPoint, obstacle_1),
      calcDistance(ballPoint, obstacle_2), calcDistance(ballPoint, obstacle_3)};
  Ball2Opponent_Dist = Ball2Obs[0];

  for (int i = 0; i < 4; i++) {
    if (Ball2Opponent_Dist > Ball2Obs[i]) {
      Ball2Opponent_Dist = Ball2Obs[i];
    }
  }

  return Ball2Opponent_Dist;
}

bool Goalkeeper::playCrouchMotion(int motion_num_1) {
  cout << "!!!!!!!! CROUCH MOTION !!!!!!!!" << endl;
  cout << master->motionEnd.motion_end << endl;

  master->motion.motion_num = motion_num_1;
  cout << "MOTION_CNT : " << motion_cnt << endl;
  cout << "MOTION_END : " << motion_end << endl;

  // master->motionPub.publish(master->motion);

  if (master->motionEnd.motion_end) {
    if (motion_end == false) {
      motion_end = true;
    } else {
      motion_end = false;
    }
  }

  return motion_end;
}

}  // namespace robocup_master25