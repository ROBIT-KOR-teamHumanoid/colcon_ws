#include "../include/robocup_master25/player.hpp"
#define REG2(X, MIN, MAX) (X - MIN) / (MAX - MIN)

namespace robocup_master25 {

using namespace std;

// public

bool Player::falldownExeption() {
  int falldownWhere;

  static int cnt;
  static int delay;  // 모션이 끝난 이후 바로 보행을 시작하면 급발진하기 때문에
                     // 딜레이를 조금 줘야한다.

  bool rst = false;

  if (isFalldown(20, 40, falldownWhere)) {
    rst = true;
    walkStop();
    cnt++;
    cout << "cnt: " << cnt << endl;
    if (cnt > 60) {
      delay = 1;
      raisetUp(falldownWhere);
    }
  } else {
    cnt = 0;
  }

  if (delay > 0 && delay < 50) {
    rst = true;
    walkStop();
    delay++;
  }
  if (delay == 50) {
    delay = 0;
  }

  return rst;
}

void Player::selectGoalPost() {
  if (master->gameControlData.myside == RIGHT) {
    cout << "SIDE: RIGHT" << endl;
    opponent_goal = Point(0, 400);
    opponent_goal_high = Point(100, 270);
    opponent_goal_low = Point(100, 530);
    opponent_goal_middle = Point(0, 400);
    opponent_goal_middle_high = Point(100, 450);
    opponent_goal_middle_low = Point(100, 350);
  } else {
    cout << "SIDE: left" << endl;
    opponent_goal = Point(1100, 400);
    opponent_goal_high = Point(1000, 270);
    opponent_goal_low = Point(1000, 530);
    opponent_goal_middle = Point(1000, 400);
    opponent_goal_middle_high = Point(1000, 450);
    opponent_goal_middle_low = Point(1000, 350);
  }
}

void Player::penaltyControl(bool &isPenalty) {
  isPenalty = true;
  switch (master->gameControlData.penalty) {
    case HL_BALL_MANIPULATION:
      cout << "!!PENALTY: HL_BALL_MANIPULATION!!" << endl;
      break;
    case HL_ILLEGAL_ATTACK:
      cout << "!!PENALTY: HL_ILLEGAL_ATTACK!!" << endl;
      break;
    case HL_ILLEGAL_DEFENSE:
      cout << "!!PENALTY: HL_ILLEGAL_DEFENSE!!" << endl;
      break;
    case HL_PHYSICAL_CONTACT:
      cout << "!!PENALTY: HL_PHYSICAL_CONTACT!!" << endl;
      break;
    case HL_PICKUP_OR_INCAPABLE:
      cout << "!!PENALTY: HL_PICKUP_OR_INCAPABLE!!" << endl;
      walkStop();
      break;
    case HL_SERVICE:
      cout << "!!PENALTY: HL_SERVICE!!" << endl;
      break;
    case SUBSTITUTE:
      cout << "!!THIS ROBOT IS SUBSTITIUTE!!" << endl;
      walkStop();
      visionPublish(VISION_SCAN_MODE_CNTR);
      break;
    default:
      isPenalty = false;
      break;
  }

  return;
}

void Player::gameStateControl(bool control_on, int state) {
  if (!control_on) {
    return;
  }
  switch (state) {
    case STATE_INITIAL:
      stateInitial();
      break;
    case STATE_READY:
      stateReady();
      break;
    case STATE_SET:
      stateSet();
      break;
    case STATE_PLAYING:
      statePlay();
      break;
    case STATE_FINISHED:
      stateFinished();
      break;
    default:
      break;
  }
}

bool Player::compareIndices(const std::vector<double> &arr, int i, int j) {
  return arr[i] < arr[j];
}

std::vector<int> Player::sortArrayByElementSize(
    const std::vector<double> &arr) {
  std::vector<int> indices(arr.size());
  for (int i = 0; i < arr.size(); ++i) {
    indices[i] = i;  // 초기 인덱스 리스트 생성
  }

  std::sort(indices.begin(), indices.end(),
            [&](int i, int j) { return compareIndices(arr, i, j); });

  return indices;
}

void Player::selectRobotState(bool isPenalty) {
  Point ballPosition(master->local.ball_x, master->local.ball_y);
  Point myPosition(master->local.robot_x, master->local.robot_y);

  if (isPenalty) {
    cout << "robot state: ROBOT_STATE_PICKUP" << endl;
    robot_state = ROBOT_STATE_PICKUP;
    return;
  }

  // 공과 로봇의 거리 측정
  vector<double> robotDist;
  robotDist.assign(MAX_ROBOT_NUM, 10000);
  robotDist.at(master->gameControlData.robotnum - 1) =
      calcDistance(myPosition, ballPosition);
  int unavailableRobotcnt = 0;
  for (int robotIndex = 0; robotIndex < MAX_ROBOT_NUM - 1; robotIndex++) {
    cout << "udp robot " << master->udp[robotIndex].robotnum
         << " robot state :  " << master->udp[robotIndex].robotcase << endl;

    if (master->udp[robotIndex].robotnum == 0 ||
        master->udp[robotIndex].robotcase == ROBOT_STATE_PICKUP) {
      unavailableRobotcnt++;
      continue;
    }

    Point robotPosition(master->udp[robotIndex].localx,
                        master->udp[robotIndex].localy);
    robotDist.at(master->udp[robotIndex].robotnum - 1) =
        calcDistance(robotPosition, ballPosition);
  }

  std::cout << "unavailableRobotcnt : " << unavailableRobotcnt << std::endl;

  vector<int> sortedIndices = sortArrayByElementSize(robotDist);

  // 로봇끼리 거리 비교해서 컨트롤 로봇 정하기
  int tempControlRobotNum = 0;
  double dist1, dist2;
  dist1 = robotDist.at(sortedIndices.at(0));
  dist2 = robotDist.at(sortedIndices.at(1));

  for (int robotn = 0; robotn < sortedIndices.size(); robotn++) {
    // cout << "robot " << sortedIndices[robotn] + 1
    //      << " dist : " << robotDist.at(sortedIndices[robotn]) << endl;
  }

  tempControlRobotNum = sortedIndices.at(0);
  tempControlRobotNum++;

  cout << "##tempControlRobotNum: " << tempControlRobotNum << endl;
  cout << "##robotNum           : " << master->gameControlData.robotnum << endl;

  // 내 상태 결정
  if (isPenalty) {
    cout << "robot state: ROBOT_STATE_PICKUP2" << endl;
    robot_state = ROBOT_STATE_PICKUP;
  } else if (tempControlRobotNum == master->gameControlData.robotnum) {
    cout << "robot state: ROBOT_STATE_CONTROLL" << endl;
    robot_state = ROBOT_STATE_CONTROLL;
  } else {
    cout << "robot state: ROBOT_STATE_SUPPORT" << endl;
    robot_state = ROBOT_STATE_SUPPORT;
  }

  if (unavailableRobotcnt > 2) robot_state = ROBOT_STATE_CONTROLL;
  if (isPenalty) {
    cout << "robot state: ROBOT_STATE_PICKUP3" << endl;
    robot_state = ROBOT_STATE_PICKUP;
  }
}

void Player::publishMsg() {
  walkPublish();
  udpPublish();
  localPublish();
}

void Player::move(bool flag) {
  if (!flag) {
    walkStop();
  }
}

void Player::move(int x, int y, int angle_mode, int target, int targetyaw) {
  Point target_coor;
  target_coor.setX(x);
  target_coor.setY(y);
  move(target_coor, angle_mode, target, targetyaw);
}

void Player::move(Point target_coor, int angle_mode, int target,
                  int targetyaw) {
  master->master2local.targetx = target_coor.x();
  master->master2local.targety = target_coor.y();

  Point robot_coor;
  robot_coor.setX(master->local.robot_x);
  robot_coor.setY(master->local.robot_y);
  double robot_yaw;

  robot_yaw = master->imu.yaw;

  if (target == BALL) {
    Point ball_coor = target_coor;
    double ball_dist = master->vision.ball_d;

    if (0 < master->vision.ball_d && master->vision.ball_d < kick_threshold) {
      int kick_mode = FRONT_KICK;
      if (kick_mode != NONE) {
        Point ball_cam_coor;
        ball_cam_coor.setX(master->vision.ball_cam_x);
        ball_cam_coor.setY(master->vision.ball_cam_y);
        bool isCalibrate;
        calibratePosition(robot_coor, ball_cam_coor, kick_mode, isCalibrate);
        if (isCalibrate) {
          cout << "cali OK" << endl;
          if (playKickMotion(kick_mode)) {
            cout << "kick" << endl;
          }
        }
      }
    } else {
      if (moveToTarget(robot_coor, robot_yaw, ball_coor, RELATIVE, 0))
        std::cout << std::endl;
    }
  } else {
    if (isCrash(target_coor)) {
      const int obstacle_size = 50;
      Point obstacle_coor(master->local.obstacle0_x, master->local.obstacle0_y);
      Point wayPoint_coor =
          wayPointMaker(obstacle_coor, robot_coor, target_coor, obstacle_size);

      master->master2local.targetx = wayPoint_coor.x();
      master->master2local.targety = wayPoint_coor.y();

      if (obstacle_coor.y() > 700)
        wayPoint_coor = Point(wayPoint_coor.x(), 700);  //
      moveToTarget(robot_coor, robot_yaw, wayPoint_coor, angle_mode, targetyaw);
    } else {
      int dist = sqrt(pow(robot_coor.x() - target_coor.x(), 2) +
                      pow(robot_coor.y() - target_coor.y(), 2));
      if (dist > 100) {
        fixyawflag = 0;
      }
      if ((dist < 40 && !targetyaw) || fixyawflag) {
        fixyawflag = 0;
        moveFixYaw(robot_coor, robot_yaw, target_coor, true);
      } else {
        moveToTarget(robot_coor, robot_yaw, target_coor, angle_mode, targetyaw);
      }
    }
  }
}

Point Player::wayPointMaker(Point obstaclePoint, Point currentPoint,
                            Point targetPoint, double threshold) {
  obstaclePoint.setY(-1 * obstaclePoint.y());
  currentPoint.setY(-1 * currentPoint.y());
  targetPoint.setY(-1 * targetPoint.y());

  cout << "obstaclePoint.x() : " << obstaclePoint.x() << " , "
       << obstaclePoint.y() << endl;
  cout << "currentPoint.x() : " << currentPoint.x() << " , " << currentPoint.y()
       << endl;
  cout << "targetPoint.x() : " << targetPoint.x() << " , " << targetPoint.y()
       << endl;

  // cout<<endl<<endl<<targetPoint.y()-currentPoint.y()<<" ,
  // "<<(targetPoint.x()- currentPoint.x())<<endl<<endl;

  double a = -(targetPoint.x() - currentPoint.x()) /
             (float)(targetPoint.y() - currentPoint.y());
  double b = -obstaclePoint.x() * a + obstaclePoint.y();

  // cout<<endl<<endl<<-obstaclePoint.x()*a<<" ,,
  // "<<obstaclePoint.y()<<endl<<endl; cout<<endl<<endl<<a<<" ,,,
  // "<<b<<endl<<endl; cout<<endl<<endl;

  // circle eodlq asdljfadsjf
  double w = pow(a, 2) + 1;
  double l = 2 * a * (b - obstaclePoint.y()) - 2 * obstaclePoint.x();
  double m = pow(obstaclePoint.x(), 2) + pow(b - obstaclePoint.y(), 2) -
             pow(threshold, 2);

  double x1 = (-l + sqrt(l * l - 4 * w * m)) / (2 * w);
  double y1 = a * x1 + b;
  Point node1(x1, y1);

  float x2 = (-l - sqrt(l * l - 4 * w * m)) / (2 * w);
  float y2 = a * x2 + b;
  Point node2(x2, y2);

  double dis1 = calcDistance(currentPoint, node1);
  double dis2 = calcDistance(currentPoint, node2);

  if (dis1 > dis2) {
    node2.setY(-node2.y());
    return node2;
  } else {
    node1.setY(-node1.y());
    return node1;
  }
}

bool Player::isCrash(Point target) {
  Point obstaclePoint =
      Point(master->local.obstacle0_x, master->local.obstacle0_y);
  Point globalRobot(master->local.robot_x, master->local.robot_y);

  // not watch ball
  if (obstaclePoint.x() == 0 && obstaclePoint.y() == 0) {
    return false;
  }

  if ((obstaclePoint.x() > target.x() || obstaclePoint.x() < globalRobot.x()) &&
      (obstaclePoint.x() < target.x() || obstaclePoint.x() > globalRobot.x())) {
    return false;
  }
  //
  float a = -target.y() + globalRobot.y();
  float b = target.x() - globalRobot.x();
  float c = globalRobot.x() * target.y() - globalRobot.y() * target.x();

  float distance =
      abs((a * obstaclePoint.x() + b * obstaclePoint.y() + c) / hypot(a, b));
  cout << "crash distance is : " << distance << endl;
  cout << "crash distance is : " << distance << endl;
  cout << "crash distance is : " << distance << endl;
  cout << "crash distance is : " << distance << endl;

  const int crash_threshold = 100;
  if (distance < crash_threshold) return true;

  return false;
}

bool Player::playMotion(int motionnum) {
  if (publish_motion_complete) {
    cout << "playMotion" << endl;
    master->motion.motion_num = motionnum;
    cout << "motion num :" << motionnum << endl;

    publish_motion_num = motionnum;
    publish_motion_flag = true;

    // master->motionPub.publish(master->motion);

    bool motion_end = false;

    if (master->motionEnd.motion_end) {
      if (!publish_motion_complete) {
        publish_motion_flag = false;
      } else {
        publish_motion_complete = true;
      }
      motion_end = true;
    }

    return motion_end;
  } else {
    return false;
  }
}

// protected virtual

void Player::stateReady(Point init_coor) { move(init_coor, OPPONENT); }

void Player::stateSet() {}

void Player::stateFinished() {}

// private

int Player::isFalldown(const double pitch_threshold,
                       const double roll_threshold, int &falldownWhere) {
  int result = FALLDOWN;
  double pitch = master->imu.pitch;
  double roll = master->imu.roll;
  bool flag1 = true;
  bool flag2 = true;

  if (roll > roll_threshold) {
    falldownWhere = FALLDOWN_LEFT;//FALLDOWN_RIGHT;
    cout << "FALLDOWN_RIGHT" << endl;
  } else if (roll < -roll_threshold) {
    falldownWhere = FALLDOWN_RIGHT;//FALLDOWN_LEFT;
    cout << "FALLDOWN_LEFT" << endl;
  } else {
    flag1 = false;
  }

  if (pitch > pitch_threshold) {
    falldownWhere = FALLDOWN_FRONT;
    cout << "FALLDOWN_FRONT" << endl;
  } else if (pitch < -pitch_threshold) {
    falldownWhere = FALLDOWN_REAR;
    cout << "FALLDOWN_REAR" << endl;
  } else {
    flag2 = false;
  }

  if (!flag1 && !flag2) result = NOT_FALLDOWN;

  return result;
}

void Player::raisetUp(int falldownWhere) {
  switch (falldownWhere) {
    case FALLDOWN_FRONT:
      playMotion(MOTION_RASEUP_FRONT);
      break;
    case FALLDOWN_REAR:
      playMotion(MOTION_RASEUP_REAR);
      break;
    case FALLDOWN_RIGHT:
      playMotion(MOTION_RASEUP_RIGHT);
      break;
    case FALLDOWN_LEFT:
      playMotion(MOTION_RASEUP_LEFT);
      break;
    default:
      break;
  }
}

void Player::calcBallKickPosCoor(Point goal_coordinates, Point ball_coordinates,
                                 Point &OUTPUT_target_coor,
                                 double kick_radius) {
  // Refer to the formula of the exturnal section.
  Point A = goal_coordinates;
  Point B = ball_coordinates;
  Point Q;  // target coor.

  int delta_x = A.x() - B.x();
  int delta_y = A.y() - B.y();

  double m = hypot(delta_x, delta_y);  // distance of AQ
  double n = kick_radius;              // distance of BQ
  //----------------------------------------------

  if (!(m - n)) {
  } else {
    Q.setX((m * B.x() - n * A.x()) / (m - n));
    Q.setY((m * B.y() - n * B.y()) / (m - n));
  }
  OUTPUT_target_coor = Q;
}

int Player::selectKickType(Point ball_coordinates, Point robot_coordinates) {
  cout << "selectKickType" << endl;

  Point opponent_goal_high = this->opponent_goal_high;
  Point opponent_goal_low = this->opponent_goal_low;
  Point ref_point = ball_coordinates;
  opponent_goal_high -= ref_point;
  opponent_goal_low -= ref_point;
  robot_coordinates -= ref_point;

  //    cout << "opponent_goal_high: (" << opponent_goal_high.x() << ", "<<
  //    opponent_goal_high.y() << ")" << endl; cout << "opponent_goal_low:  ("
  //    << opponent_goal_low.x() << ", "<< opponent_goal_low.y() << ")" << endl;
  //    cout << "robot_coordinates:  (" << robot_coordinates.x() << ", "<<
  //    robot_coordinates.y() << ")" << endl;

  double ref_theta = 0;  // reference angle
  double com_theta = 0;  // comparative angle

  ref_theta =
      (180 / PI) * atan2(opponent_goal_high.y() * opponent_goal_low.x() -
                             opponent_goal_high.x() * opponent_goal_low.y(),
                         opponent_goal_high.x() * opponent_goal_low.x() +
                             opponent_goal_high.y() * opponent_goal_low.y());

  com_theta =
      (180 / PI) * atan2(opponent_goal_high.y() * robot_coordinates.x() -
                             opponent_goal_high.x() * robot_coordinates.y(),
                         opponent_goal_high.x() * robot_coordinates.x() +
                             opponent_goal_high.y() * robot_coordinates.y());

  int kick_mode = NONE;

  if (com_theta > 0) {
    if (com_theta > 0 && com_theta < 180 + ref_theta) {
      cout << "SIDE RIGHT" << endl;
      kick_mode = SIDE_RIGHT;
    } else if (com_theta > 180 + ref_theta && com_theta < 180) {
      cout << "FRONT KICK" << endl;
      kick_mode = FRONT_KICK;
    }
  } else if (com_theta < 0) {
    com_theta = fabs(com_theta);
    ref_theta = fabs(ref_theta);

    if (0 < com_theta && com_theta < ref_theta) {
      cout << "BANNED POSITION" << endl;
    } else if (ref_theta < com_theta && com_theta < 180) {
      cout << "SIDE LEFT" << endl;
      kick_mode = SIDE_LEFT;
    }
  }

  return kick_mode;
}

void Player::calibratePosition(Point robot_coor, Point ball_cam_coor,
                               int &kick_mode, bool &isCalibrate) {
  visionPublish(VISION_SCAN_MODE_CNTR, 0, -70);
  cout << "calibratePosition" << endl;
  isCalibrate = false;

  if (kick_mode == FRONT_KICK) {
    if (ball_cam_coor.x() > 320) {
      kick_mode = FRONT_RIGHT;
      cout << "FRONT_RIGHT" << endl;
    } else {
      kick_mode = FRONT_LEFT;
      cout << "FRONT_LEFT" << endl;
    }
  }
  Point ref_point;
  Point allowance = Point(50, 25);
  switch (kick_mode) {
    case FRONT_RIGHT:
      ref_point = Point(420, 425);
      break;
    case FRONT_LEFT:
      ref_point = Point(250, 425);
      break;
    case SIDE_RIGHT:
      ref_point = Point(320, 425);
      break;
    case SIDE_LEFT:
      ref_point = Point(320, 425);
      break;
    default:
      break;
  }

  int x = master->X_MIN;
  int y = master->Y_MIN;

  if (ball_cam_coor.x() > ref_point.x() + allowance.x()) {
    cout << "SETTING : X" << endl;
    walkStart(0, -y, 0);
  } else if (ball_cam_coor.x() < ref_point.x() - allowance.x()) {
    cout << "SETTING : -X" << endl;
    walkStart(0, y, 0);
  } else {
    if (ball_cam_coor.y() > ref_point.y() + allowance.y()) {
      cout << "SETTING : Y" << endl;
      //            walkStart(-x,0,0);
    } else if (ball_cam_coor.y() < ref_point.y() - allowance.y()) {
      cout << "SETTING : -Y" << endl;
      walkStart(x, 0, 0);
    } else {
      cout << "CALI OK" << endl;
      if (walkStop()) {
        isCalibrate = true;
      }
    }
  }
}

bool Player::playKickMotion(int kick_mode) {
  bool motion_end = false;

  switch (kick_mode) {
    case FRONT_RIGHT:
      motion_end = playMotion(MOTION_KICK_FRONT_R);
      break;
    case FRONT_LEFT:
      motion_end = playMotion(MOTION_KICK_FRONT_L);
      break;
    case SIDE_RIGHT:
      motion_end = playMotion(MOTION_KICK_SIDE_R);
      break;
    case SIDE_LEFT:
      motion_end = playMotion(MOTION_KICK_SIDE_L);
      break;
    default:
      break;
  }

  return motion_end;
}

bool Player::moveFixYaw(Point current_coordinates, double current_yaw,
                        Point target_coordinates, bool dontStop) {
  int targetyaw = 0;
  int x_param = 0;
  int y_param = 0;

  if (opponent_goal.x() < 550) {
    targetyaw = 90;
  } else {
    targetyaw = -90;
  }

  if (isArrive(current_coordinates, target_coordinates)) {
    if (abs(targetyaw - current_yaw) < 15) {
      walkStop();
    } else {
      target_angle = targetyaw - current_yaw;
      if (target_angle > 180)
        target_angle -= 360;
      else if (target_angle < -180)
        target_angle += 360;

      walkStart(x_param, y_param, static_cast<int>(target_angle));
    }
  } else {
    if (walk_switch == -1) {
      if (abs(targetyaw - current_yaw) > 15) {
        walk_switch = 2;
      } else if (abs(current_coordinates.x() - target_coordinates.x()) >
                 abs(current_coordinates.y() - target_coordinates.y())) {
        walk_switch = 0;
      } else {
        walk_switch = 1;
      }
    } else if (walk_switch == 0) {
      fixyawflag = 1;
      if (abs(current_coordinates.x() - target_coordinates.x()) <= 5) {
        walk_switch = -1;
        return false;
      }
      if (opponent_goal.x() < 550) {
        if (current_coordinates.x() - target_coordinates.x() > 0) {
          x_param = master->FRONT_MAX;
        } else {
          x_param = -10;
        }
      } else {
        if (current_coordinates.x() - target_coordinates.x() > 0) {
          x_param = -10;
        } else {
          x_param = master->FRONT_MAX;
        }
      }
      target_angle = targetyaw - current_yaw;
    } else if (walk_switch == 1) {
      fixyawflag = 1;
      if (abs(current_coordinates.y() - target_coordinates.y()) <= 5) {
        walk_switch = -1;
        return false;
      }
      if (opponent_goal.x() < 550) {
        if (current_coordinates.y() - target_coordinates.y() > 0) {
          y_param = master->LEFT_MAX;
        } else {
          y_param = master->RIGHT_MAX;
        }
      } else {
        if (current_coordinates.y() - target_coordinates.y() > 0) {
          y_param = master->RIGHT_MAX;
        } else {
          y_param = master->LEFT_MAX;
        }
      }
      target_angle = targetyaw - current_yaw;
    } else if (walk_switch == 2) {
      fixyawflag = 1;
      if (abs(targetyaw - current_yaw) < 5) {
        walk_switch = -1;
        return false;
      }
      target_angle = targetyaw - current_yaw;
    }

    if (target_angle > 180)
      target_angle -= 360;
    else if (target_angle < -180)
      target_angle += 360;

    walkStart(x_param, y_param, static_cast<int>(target_angle));
  }

  cout << "targetyaw:   " << targetyaw << endl;
  cout << "current_yaw:  " << current_yaw << endl;
  cout << "target angle: " << target_angle << endl;

  return false;
}

bool Player::moveToTarget(Point current_coordinates, double current_yaw,
                          Point target_coordinates, int angle_mode,
                          double target_angle, bool dontStop) {
  static double ref_angle;

  if (isArrive(current_coordinates, target_coordinates) &&
      master->local.ball_x == 0 && master->local.ball_y == 0) {
    cout << "+++++++++++++++++++++++++" << endl;
    walkStart(0, 0, 8);
  } else if (isArrive(current_coordinates, target_coordinates)) {
    cout << "ARRIVE" << endl;
    static int stop_once;
    if (!stop_once) {
      stop_once = walkStop();
    }

    if (angle_mode == ABSOLUTE) {
      target_angle = (target_angle - current_yaw);
    } else if (angle_mode == RELATIVE) {
      target_angle = target_angle + ref_angle;
      target_angle = target_angle - current_yaw;
    } else if (angle_mode == OPPONENT) {
      target_angle = 0;
      calcTargetAngle(opponent_goal, current_coordinates, current_yaw,
                      target_angle);
    }
    cout << "target_angle: " << target_angle << endl;
    if (alignRobot(current_yaw, target_angle)) {
      if (dontStop) {
        cout << "dontstop !!!!!!!!!" << endl;
        cout << "dontstop !!!!!!!!!" << endl;
        cout << "dontstop !!!!!!!!!" << endl;
        cout << "dontstop !!!!!!!!!" << endl;
        cout << "dontstop !!!!!!!!!" << endl;
        cout << "dontstop !!!!!!!!!" << endl;
        cout << "dontstop !!!!!!!!!" << endl;
        cout << "dontstop !!!!!!!!!" << endl;
        return true;
      }
      if (walkStop()) {
        cout << "stop where" << endl;
        cout << "stop where !!!!!!!!!" << endl;
        cout << "stop where !!!!!!!!!" << endl;
        cout << "stop where !!!!!!!!!" << endl;
        cout << "stop where !!!!!!!!!" << endl;
        cout << "stop where !!!!!!!!!" << endl;
        cout << "stop where !!!!!!!!!" << endl;
        cout << "stop where !!!!!!!!!" << endl;
        cout << "stop where !!!!!!!!!" << endl;

        return true;
      }
    }
  } else {
    ref_angle = current_yaw;
    double target_angle = 0;
    calcTargetAngle(target_coordinates, current_coordinates, current_yaw,
                    target_angle);
    cout << "target angle: " << target_angle << endl;

    int max = 30;
    int min = -30;

    double yaw;

    int x = master->FRONT_MAX *
            calcDistance(current_coordinates, target_coordinates) /
            100.0;  // 73 // first: 50 -> 53 -> 55 -> 53 -> 51 -> 52 -> 53
    if (x < master->X_MIN)
      x = master->X_MIN;
    else if (x > master->FRONT_MAX)
      x = master->FRONT_MAX;

    if (fabs(target_angle) > max) {
      yaw = target_angle > 0 ? master->R_YAW_MAX : master->L_YAW_MAX;
      x = 0;
    } else {
      double reg_err = (target_angle - min) / (max - min) * 2 - 1;

      static PIDControll yawControl(master->kp, master->ki, master->kd,
                                    master->R_YAW_MAX, 1);
      //   std::cout << "kp : " << master->kp << " ki : " << master->ki
      //             << " kd : " << master->kd << std::endl;
      std::cout << "reg_err : " << reg_err << std::endl;
      yawControl.setParams(master->kp, master->ki, master->kd, 1);
      yaw = yawControl.controller(reg_err);
      // yawControl.print();
    }
    cout << " FRONT X!! : " << x << endl;
    cout << " yaw   Z!! : " << yaw << endl;
    // cout << "current_coordinates x : " << current_coordinates.x() << endl;
    // cout << "current_coordinates y : " << current_coordinates.y() << endl;
    // cout << "target_coordinates x : " << target_coordinates.x() << endl;
    // cout << "target_coordinates y : " << target_coordinates.y() << endl;

    walkStart(x, 0, static_cast<int>(yaw));
  }
  return false;
}

void Player::calcDistance(Point starting_point, Point end_point,
                          double &result) {
  double delta_x = end_point.x() - starting_point.x();
  double delta_y = -end_point.y() + starting_point.y();

  result = hypot(delta_x, delta_y);
}

double Player::calcDistance(Point starting_point, Point end_point) {
  double result;
  calcDistance(starting_point, end_point, result);
  return result;
}

bool Player::isArrive(Point robot_coor, Point target_coor, double threshold) {
  calcDistance(robot_coor, target_coor, remaining_distance);

  bool status = remaining_distance < threshold ? true : false;
  return status;
}

void Player::calcTargetAngle(Point target_coor, Point robot_coor,
                             double robot_yaw, double &target_angle) {
  double delta_x = target_coor.x() - robot_coor.x();
  double delta_y = -target_coor.y() + robot_coor.y();

  abs_target_angle = -atan2(delta_x, delta_y);
  // abs_target_angle = -atan2(delta_y,delta_x);
  abs_target_angle = abs_target_angle * 180 / PI;

  target_angle += abs_target_angle - robot_yaw;

  if (target_angle > 180) target_angle -= 360;
  if (target_angle < -180) target_angle += 360;
}

double Player::calcTargetAngle(Point target_coor, Point robot_coor,
                               double robot_yaw) {
  double rst = 0;
  calcTargetAngle(target_coor, robot_coor, robot_yaw, rst);
  return rst;
}

bool Player::alignRobot(double robot_yaw, double target_angle, int yaw_factor,
                        int angle_mode, double align_threshold) {
  static bool off_course = false;
  if (fabs(target_angle) < align_threshold / 2) {
    off_course = false;
  }

  if ((fabs(target_angle) < align_threshold) && !off_course) {
    return true;
  } else {
    off_course = true;

    //        double yaw = yaw_factor * target_angle / fabs(target_angle);
    int max = 30;
    int min = -30;
    target_angle = target_angle > max ? max : target_angle;
    target_angle = target_angle < min ? min : target_angle;
    double yaw = 6 * ((target_angle - min) / (max - min) * 2 - 1);

    cout << "yaw: " << yaw << endl;

    walkStart(0, 0, yaw);

    return false;
  }
}

Point Player::setPoint(int point_number) {
  int n = point_number;
  Point result;
  // 중간을 기준으로 대칭 포인트
  Point p[] = {Point(550, 400),   Point(-100, 0),    Point(-100, 120),
               Point(-100, -120), Point(-150, 60),   Point(-150, 220),
               Point(-150, -60),  Point(-150, -220), Point(-200, 120),
               Point(-200, -120), Point(200, 120),   Point(200, -120),
               Point(300, 120),   Point(300, -120),  Point(0, 0)};

  if (master->gameControlData.myside == RIGHT) {
    p[n].setX(-p[n].x());
    p[n].setY(-p[n].y());
  }

  result = p[0] + p[n];

  return result;
}

Point Player::calcShootPoint(double cali_radius, Point goal_coor,
                             Point ball_coor) {
  //    Point A, B;
  //    A = goal_coor;
  //    B = ball_coor;

  double Qx, Qy;
  int x1, x2, y1, y2;

  x1 = goal_coor.x();
  x2 = ball_coor.x();
  y1 = goal_coor.y();
  y2 = ball_coor.y();

  double ball_dist = hypot(x2 - x1, y2 - y1);

  double m, n;
  m = 1;
  n = 1 + cali_radius / ball_dist;

  Qx = (x2 - x1) * n + x1;
  Qy = (y2 - y1) * n + y1;

  Point target_coor = Point(Qx, Qy);

  return target_coor;
}

void Player::dribble() {
  //
  Point globalBall(master->local.ball_x, master->local.ball_y);
  Point globalRobot(master->local.robot_x, master->local.robot_y);

  Point home(250, 400);
  move(home);
}

void Player::kick3() {
  Point globalBall(master->local.ball_x, master->local.ball_y);
  Point globalRobot(master->local.robot_x, master->local.robot_y);

  // 보정 오차범위 파라미터
  double cail_threshold = 15;
  //
  if (master->gameControlData.myside == LEFT) {
    if (master->local.robot_x > 800) {
      cail_threshold = 30;
    }
  } else
  // 상대 로봇이랑 가까이 있는 경우 - 정확도 낮추고 빠르게 걷어내기
  {
    if (master->local.robot_x < 300) {
      cail_threshold = 30;
    }
  }

  // 슛 목표 지점 선정
  Point goalPoint = opponent_goal;

  // 패스 여부 결정 - 골대와 가까운 로봇이 있으면 그 로봇의 목표 위치에 패스
  double shortestDist = calcDistance(globalRobot, opponent_goal);
  for (int i = 0; i < 3; i++) {
    Point udpRobot(master->udp[i].localx, master->udp[i].localy);
    double isCheck = true;
    if (udpRobot.x() == 0 && udpRobot.y() == 0) isCheck = false;
    double dist = calcDistance(udpRobot, opponent_goal);
    if (dist < shortestDist && isCheck) {
      shortestDist = dist;
      goalPoint = setPoint(12);

      if (master->udp[i].localy > 400)
        goalPoint.setY(520);  // 로봇 위치 기반 패스 위치 선정
      else
        goalPoint.setY(280);

      cail_threshold = 5;
      cout << "PASS" << endl;
    }
  }

  const double obstacle_threshold =
      70;  // 상대 로봇과의 거리 파라미터: 이 값보다 작으면 빠르게 보정한다.

  Point obstaclePoint =
      Point(master->local.obstacle0_x, master->local.obstacle0_y);
  double obstacle_dist = calcDistance(globalRobot, obstaclePoint);
  if (obstacle_dist < obstacle_threshold) {
    cail_threshold = 180;
  }

  static int state;

  const int refDist = 30;  // cm  30
  const int distBias = 10;

  Point pointWhereIGo = calcShootPoint(refDist, goalPoint, globalBall);

  if (state < 3) {
    if (calcDistance(globalRobot, globalBall) >
        refDist + distBias)  // 공멀어지면 다시 케이스 0으로
    {
      state = 0;
    }
    if (master->vision.ball_d == 0) {
      //            state = 0;
      walkStart(0, 0, 7);  // 한 방향으로 돌아서 공 찾기
    }
  }
  const int test = 100;
  // state = test;
  switch (state) {
    case test: {
      // 입력 변수 정의: "position"은 왼쪽과 오른쪽의 정도를 나타냄
      FuzzyVariable position("position");
      // 왼쪽, 가운데, 오른쪽, 더 왼쪽, 더 오른쪽의 다섯 가지 세트 정의
      position.add_set(FuzzySet("far_left", {{-800.0, 1.0}, {-400.0, 0.0}}));
      position.add_set(
          FuzzySet("left", {{-500.0, 0.0}, {-400.0, 1.0}, {0.0, 0.0}}));
      position.add_set(
          FuzzySet("center", {{-200.0, 0.0}, {0.0, 1.0}, {200.0, 0.0}}));
      position.add_set(
          FuzzySet("right", {{0.0, 0.0}, {0.400, 1.0}, {500.0, 0.0}}));
      position.add_set(FuzzySet("far_right", {{0.400, 0.0}, {800.0, 1.0}}));

      // 퍼지 규칙 정의
      vector<FuzzyRule> rules = {
          // 왼쪽으로 갈수록 "left"의 값이 높아지고, 오른쪽으로 갈수록 "right"의
          // 값이 높아지도록 규칙 정의
          FuzzyRule({{"position", "far_left"}},
                    {{"left", 1.0}, {"right", 0.0}}),
          FuzzyRule({{"position", "left"}}, {{"left", 0.75}, {"right", 0.25}}),
          FuzzyRule({{"position", "center"}}, {{"left", 0.5}, {"right", 0.5}}),
          FuzzyRule({{"position", "right"}}, {{"left", 0.25}, {"right", 0.75}}),
          FuzzyRule({{"position", "far_right"}},
                    {{"left", 0.0}, {"right", 1.0}})};

      // 퍼지 시스템 생성 및 규칙 추가
      FuzzySystem fuzzy_system;
      fuzzy_system.add_variable(position);
      for (auto &rule : rules) {
        fuzzy_system.add_rule(rule);
      }

      // 입력 값에 따른 출력 계산
      double position_input = -270;  // 입력 값 (왼쪽과 오른쪽의 정도)
      map<string, double> inputs = {{"position", position_input}};
      auto outputs = fuzzy_system.evaluate(inputs);
      // cout << "Position: " << position_input << " => Left: " <<
      // outputs["position"].second["left"] << ", Right: " <<
      // outputs["position"].second["right"] << endl;

      break;
    }
    case 0: {
      cout << endl << " MOVE TO TARGET " << endl << endl;
      moveToTarget(globalRobot, master->imu.yaw, globalBall, OPPONENT, 0, true);
      if (calcDistance(globalRobot, globalBall) <
          refDist + distBias)  // 오느정도 가까워 지면 다음 케이스인 어라운드
                               // 볼로 간다.￣
      {
        state = 1;
      }
      break;
    }
    case 1: {
      cout << endl << " MOVE ROBOT AROUND BALL " << endl << endl;

      if (moveRobotAroundBall(cail_threshold, refDist, pointWhereIGo,
                              goalPoint)) {
        state = 2;
      }
      break;
    }
    case 10: {
      cout << endl << " move stop and watch ball " << endl << endl;

      static int delay_previous_cnt;
      delay_previous_cnt++;
      cout << "delay_cnt: " << delay_previous_cnt << endl;
      walkStop();

      const int onesecond = 300;
      if (delay_previous_cnt > onesecond) {
        delay_previous_cnt = 0;
        state = 2;
      }
      break;
    }
    case 2: {
      cout << endl << " CALI TO BALL " << endl << endl;

      if (caliToBall(goalPoint)) {
        state = 3;
      }

      break;
    }
    case 11: {
      static int delay_previous_cnt2;
      static int check;
      delay_previous_cnt2++;

      cout << endl << " DELAY 1 " << endl << endl;
      cout << endl << delay_previous_cnt2 << endl << endl;
      cout << endl << check << endl << endl;

      walkStop();

      if (abs(master->vision.ball_2d_x) > master->In &&
          abs(master->vision.ball_2d_x) < master->Out &&
          master->vision.ball_2d_y > master->Back &&
          master->vision.ball_2d_y < master->Front) {
        check++;
      }
      if (check > 10) {
        check = 0;
        delay_previous_cnt2 = 0;
        state = 3;
      }

      if (delay_previous_cnt2 > 20) {
        delay_previous_cnt2 = 0;
        check = 0;
        state = 1;
      }

      break;
    }

    case 3: {
      cout << endl << " DELAY BEFORE MOTION" << endl << endl;

      // if (before_cnt < 5) {
      //   walkStart(0, 0, 0);
      //   before_cnt++;
      //   break;
      // }

      walkStop();

      static int cnt_delayBeforeMotion;
      cnt_delayBeforeMotion++;

      // cout << "localBall.x: " << master->vision.ball_2d_x << endl;
      // cout << "localBall.y: " << master->vision.ball_2d_y << endl;
      cout << "visionBall.x: " << master->vision.ball_cam_x << endl;
      cout << "visionBall.y: " << master->vision.ball_cam_y << endl;
      cout << "cnt_delayBeforeMotion: " << cnt_delayBeforeMotion << endl;

      if (cnt_delayBeforeMotion > 5) {  // 0.5s
        cnt_delayBeforeMotion = 0;
        state = 4;
      }

      break;
    }
    case 4:
      cout << endl << " PLAY KICK MOTION " << endl << endl;
      int kick_mode;
      if (320 - master->vision.ball_cam_x > 0) {
        kick_mode = FRONT_LEFT;
      } else {
        kick_mode = FRONT_RIGHT;
      }
      state = 5;
      playKickMotion(kick_mode);
      master->motionEnd.motion_end = 0;

      break;
    case 5:
      cout << endl << " READY KICK MOTION " << endl << endl;

      walkStop();
      if (master->motionEnd.motion_end) state = 6;

      static int motionReadyCnt;
      motionReadyCnt++;

      cout << "localBall.x: " << master->vision.ball_2d_x << endl;
      cout << "localBall.y: " << master->vision.ball_2d_y << endl;
      cout << "motionReadyCnt: " << motionReadyCnt << endl;

      if (motionReadyCnt > 70)  // 5000
      {
        motionReadyCnt = 0;

        state = 4;
      }

      break;
    case 6:
      cout << endl << " DELAY MOTION BEFORE WALK " << endl << endl;

      static int delay_cnt;
      delay_cnt++;
      cout << "delay_cnt: " << delay_cnt << endl;

      if (delay_cnt > 10) {
        delay_cnt = 0;
        state = 0;
      }

      break;
  }

  master->master2local.targetx = pointWhereIGo.x();
  master->master2local.targety = pointWhereIGo.y();
}

bool Player::moveRobotAroundBall(double cail_threshold, int refDist,
                                 Point localTarget, Point goalPoint) {
  Point localBall(static_cast<int>(master->local.ball_x),
                  static_cast<int>(master->local.ball_y));
  Point localRobot(static_cast<int>(master->local.robot_x),
                   static_cast<int>(master->local.robot_y));

  double robotyaw = static_cast<double>(master->imu.yaw);

  double ballAngle = calcTargetAngle(localBall, localRobot, robotyaw);

  double radTheta = (robotyaw + ballAngle) * (PI / 180);

  // 회전
  double EndPointX =
      cos(radTheta) * localTarget.x() - sin(radTheta) * localTarget.y();
  double StartPointX =
      cos(radTheta) * localRobot.x() - sin(radTheta) * localRobot.y();

  double O_dir = EndPointX - StartPointX;

  double goalAngle = calcTargetAngle(goalPoint, localRobot, robotyaw);

  double radius_err = refDist - calcDistance(localBall, localRobot);

  if (abs(O_dir) < 30 && abs(goalAngle) < cail_threshold /*&&*/
      //       abs(ballAngle) < 20 &&
      /*radius_err > 3*/) {
    return true;
  }

  int dir = O_dir > 0 ? -1 : 1;
  cout << "DIR: " << dir << endl;

  double y = master->ROUND_Y;
  if (fabs(O_dir) < 5) y = 0;

  cout << "DIR ORG    : " << O_dir << endl;
  cout << "GOAL ANGLE : " << goalAngle << endl;
  cout << "BALL ANGLE : " << ballAngle << endl;
  cout << "radius_err : " << radius_err << endl;

  //    if(radius_err > 0) {
  //        radius_err = 10;
  //        ballAngle = 0;
  //        y = 0;
  //    }

  if (ballAngle >= 10)
    ballAngle = 10;
  else if (ballAngle <= -10)
    ballAngle = -10;
  double reg_ballAngle = REG2(ballAngle, 0, 10);
  cout << "reg_ballAngle: " << reg_ballAngle << endl;

  int z = 5 * reg_ballAngle - dir * 3;
  if (z < -master->ROUND_YAW_MIN)
    z = -master->ROUND_YAW_MIN;
  else if (z > master->ROUND_YAW_MIN)
    z = master->ROUND_YAW_MIN;

  if (radius_err >= 5)
    radius_err = 5;
  else if (radius_err <= -20)
    radius_err = -20;
  cout << "radius_err : " << radius_err << endl;
  double reg_radiusErr =
      double(radius_err - 0) / (double)(20 - 0);  // REG2(radius_err, 0, 20);
  cout << "reg_radiusErr: " << reg_radiusErr << endl;
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_1
  walkStart(10 * reg_radiusErr, dir * y, 1.2 * z);
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_2
  walkStart(10 * reg_radiusErr, dir * y, 1.2 * z);
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_3
  walkStart(10 * reg_radiusErr, dir * y, 1.2 * z);
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_4
  walkStart(10 * reg_radiusErr, dir * y, 1.2 * z);
#endif
  return false;
}

bool Player::caliToBall(Point goalPoint)  // local base
{
  const int velX = master->X_MIN;  //-16
  const int velY = master->Y_MIN;  // 7

  Point localBall(master->vision.ball_2d_x, master->vision.ball_2d_y);
  Point visionBall(master->vision.ball_cam_x - 320, master->vision.ball_cam_y);

  cout << "vision Ball X: " << visionBall.x() << endl;
  cout << "vision Ball Y: " << visionBall.y() << endl;

  int y = 0;
  int x = 0;

  bool x_flag = false;
  bool y_flag = false;

  // 전후좌우 보정
  if (abs(visionBall.x()) > master->In && abs(visionBall.x()) < master->Out && visionBall.x() != -320) {
    y = 0;
    x_flag = true;
    cout << "X ON - IN OUT" << endl;
  } else if (visionBall.x() > master->Out && visionBall.x() != -320) {  // LEFT
    cout << "visionBall.x: " << visionBall.x() << endl;
    cout << "OUT: GO RIGHT" << endl;
    y = -(velY);
  } else if (visionBall.x() < -master->Out && visionBall.x() != -320) {  // RIGHT
    cout << "visionBall.x: " << visionBall.x() << endl;
    cout << "OUT: GO LEFT" << endl;
    y = velY;
  } else if (visionBall.x() == -320 && visionBall.x() != -320) {  // NO BALL
    cout << "visionBall.x: " << visionBall.x() << endl;
    cout << "IN: GO LEFT" << endl;
    y = 0;
    cout << "x no ball catch" << endl;
  }

  if (visionBall.y() < master->Front && visionBall.y() > 0) {
    cout << "visionBall.y: " << visionBall.y() << endl;
    cout << "GO FRONT" << endl;
    x = velX;
  } else if (visionBall.y() > master->Back) {
    cout << "visionBall.y: " << visionBall.y() << endl;
    cout << "GO BACK" << endl;
    // x = velX;
    x = -master->X_MIN;
  } else if (visionBall.y() <= 0) {
    cout << "y no ball catch" << endl;
    x = master->REAR_MAX;
  } else {
    cout << "Y ON - FRONT BACK" << endl;
    y_flag = true;
    x = 0;
  }

  if (x_flag && y_flag) {
    return true;
  }

  // yaw축 보정
  int z = 0;

  int max = 10;
  double target_angle = calcTargetAngle(
      goalPoint, Point(master->local.robot_x, master->local.robot_y),
      master->imu.yaw);
  target_angle = target_angle > max ? max : target_angle;
  if (target_angle > max)
    target_angle = max;
  else if (target_angle < -max)
    target_angle = -max;
  PIDControll yawControl(master->kp, master->ki, master->kd, master->R_YAW_MAX,
                         0);
  double yaw_err = REG2(target_angle, 0, max);
  double yaw = yawControl.controller(yaw_err);

  z = static_cast<int>(yaw);

  cout << "x , y , z : " << x << " " << y << " " << z << endl;
  walkStart(x, y, z);

  return false;
}

bool Player::walkStart(int x, int y, int yaw) {
  master->ik.flag = true;

  master->ik.x_length = x;
  master->ik.y_length = y;
  master->ik.yaw = yaw;

  return master->ikEnd.ikend;
}

bool Player::walkStop() {
  master->ik.flag = false;
  master->ik.x_length = 0;
  master->ik.y_length = 0;
  master->ik.yaw = 0;

  return master->ikEnd.ikend;
}

bool Player::walkPublish() {
  bool error = false;
  if (master->ik.x_length > master->FRONT_MAX) {
    master->ik.x_length = master->FRONT_MAX;
    error = true;
  } else if (master->ik.x_length < master->REAR_MAX) {
    master->ik.x_length = master->REAR_MAX;
    error = true;
  }

  if (master->ik.y_length > master->RIGHT_MAX) {
    master->ik.y_length = master->RIGHT_MAX;
    error = true;
  } else if (master->ik.y_length < master->LEFT_MAX) {
    master->ik.y_length = master->LEFT_MAX;
    error = true;
  }

  if (master->ik.yaw > master->R_YAW_MAX) {
    master->ik.yaw = master->R_YAW_MAX;
    error = true;
  } else if (master->ik.yaw < master->L_YAW_MAX) {
    master->ik.yaw = master->L_YAW_MAX;
    error = true;
  }

  // master->ikPub->publish(master->ik);

  return error;
}

bool Player::visionPublish(int scan_mode, int pan, int tilt) {
  master->master2vision.scanmode = scan_mode;
  master->master2vision.tilt = -45;
  master->master2vision.pan = 0;
  // master->visionPub->publish(master->master2vision);

  return true;
}

bool Player::udpPublish() {
  master->master2udp.robotnum = master->gameControlData.robotnum;
  master->master2udp.robotstate = robot_state;
  // master->master2udp.robot_state = robot_panalty_state;
  master->master2udp.robotcoorx = static_cast<int>(master->local.robot_x);
  master->master2udp.robotcoory = static_cast<int>(master->local.robot_y);
  master->master2udp.robotimuyaw = static_cast<int>(master->imu.yaw);
  master->master2udp.balldist = 0;
  master->master2udp.ballcoorx = static_cast<int>(master->local.ball_x);
  master->master2udp.ballcoory = static_cast<int>(master->local.ball_y);
  master->master2udp.myteam = static_cast<int>(master->gameControlData.myteam);

  if (static_cast<int>(master->vision.ball_d) == 0) {
    master->master2udp.ballcoorx = 0;
    master->master2udp.ballcoory = 0;
  }

  // master->udpPub.publish(master->master2udp);

  return true;
}

bool Player::localPublish() {
  if (static_cast<int>(master->ik.flag) == false) {
    master->master2local.targetx = 0;
    master->master2local.targety = 0;
  } else
    cout << "1111111111111111111pub" << endl;
  // master->localPub.publish(master->master2local);

  return true;
}

}  // namespace robocup_master25
