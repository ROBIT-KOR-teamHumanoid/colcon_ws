#ifndef PLAYER_HPP
#define PLAYER_HPP

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <vector>

#include "../master_controller_PID/pidcontroll.h"
#include "define.hpp"
#include "robocup_master25.hpp"
using namespace std;

namespace robocup_master25 {
class Player {
 public:
  Player(std::shared_ptr<robocup_master25::MasterRcko> master) {
    this->master = master;
  }

  std::shared_ptr<robocup_master25::MasterRcko> master;
  int robot_state;
  int robot_panalty_state;

  bool falldownExeption();
  void selectGoalPost();
  void penaltyControl(bool &isPenalty);
  void gameStateControl(bool control_on, int state);
  virtual void selectRobotState(bool isPenalty);
  void publishMsg();

  void move(bool flag);
  void move(int x, int y, int angle_mode = RELATIVE, int target = COORDINATE,
            int target_yaw = 0);
  void move(Point target_coor, int angle_mode = RELATIVE,
            int target = COORDINATE, int target_yaw = 0);
  bool playMotion(int motion_num);
  void setVisionTarget(int target);
  void setVisionMode(int mode);

  bool isCrash(Point target);
  Point wayPointMaker(Point obstaclePoint, Point currentPoint,
                      Point targetPoint, double threshold);

  bool isArrive(Point robot_coor, Point target_coor, double threshold = 20);  //
  int publish_motion_num;
  bool publish_motion_flag = false;
  bool publish_motion_complete = true;

  //  //walk limit
  //  #define FRONT_MAX       50
  //  #define X_MIN           25//18
  //  #define REAR_MAX        -19
  //  #define RIGHT_MAX       17//6 12
  //  #define Y_MIN           10
  //  #define ROUND_Y         14//7//12// cali ball number2: 12 -------12
  //  #define ROUND_YAW_MIN   15//7 10
  //  #define LEFT_MAX        -12 // first: -15 -> -19 -> -17 -> -15  -22  -8
  //  #define R_YAW_MAX       5 // 8
  //  #define L_YAW_MAX       -5

 protected:
  virtual void stateInitial() = 0;
  virtual void stateReady() = 0;
  virtual void stateReady(Point init_coor);
  virtual void stateSet() = 0;
  virtual void statePlay() = 0;
  virtual void stateFinished() = 0;

 protected:
  const double PI = 3.14159265;
  const double kick_threshold = 200;

  int before_cnt = 0;

  int status;
  int walk_status;
  int walk_switch = -1;
  int fixyawflag = 0;
  double target_angle;
  double abs_target_angle;
  double remaining_distance;

  size_t dribble_cnt = 0;

  Point opponent_goal_high;
  Point opponent_goal_low;
  Point opponent_goal_middle_high;
  Point opponent_goal_middle_low;
  Point opponent_goal_middle;
  Point opponent_goal;

  int isFalldown(const double pitch_threshold, const double roll_threshold,
                 int &falldownWhere);
  void raisetUp(int falldownWhere);

  void calcBallKickPosCoor(Point goal_coordinates, Point ball_coordinates,
                           Point &OUTPUT_target_coor, double kick_radius = 10);
  int selectKickType(Point ball_coordinates, Point robot_coordinates);
  void calibratePosition(Point robot_coor, Point ball_cam_coor, int &kick_mode,
                         bool &isCalibrate);
  bool playKickMotion(int kick_mode);
  bool moveFixYaw(Point current_coordinates, double current_yaw,
                  Point target_coordinates, bool dontStop);
  bool moveToTarget(Point current_coordinates, double current_yaw,
                    Point target_coordinates, int angle_mode,
                    double target_angle, bool dontStop = false);
  void calcDistance(Point starting_point, Point end_point, double &result);
  double calcDistance(Point starting_point, Point end_point);
  // bool isArrive(Point robot_coor, Point target_coor, double threshold = 20);
  void calcTargetAngle(Point target_coor, Point robot_coor, double robot_yaw,
                       double &target_angle);
  double calcTargetAngle(Point target_coor, Point robot_coor, double robot_yaw);
  bool alignRobot(double robot_yaw, double target_angle = 0, int yaw_factor = 3,
                  int angle_mode = OPPONENT, double align_threshold = 30);
  Point setPoint(int point_number);
  Point calcShootPoint(double cali_radius, Point goal_coor, Point ball_coor);
  void kick3();
  void dribble();
  bool moveRobotAroundBall(double cail_threshold, int refDist,
                           Point localTarget, Point goalPoint);
  bool caliToBall(Point goalPoint);
  bool walkStart(int x, int y, int yaw);
  bool walkStop();
  bool walkPublish();
  bool visionPublish(int scan_mode, int pan = 0, int tilt = 0);
  bool udpPublish();
  bool localPublish();
  std::vector<int> sortArrayByElementSize(const std::vector<double> &arr);
  bool compareIndices(const std::vector<double> &arr, int i, int j);

  //  //30~100, 40~120
  //  int in; = master->In; // number2: 15 -> 13 --17
  //  int out; = master->Out;//100; // first: 110 -> 100 -> 110
  //  int back; = master->Back;//40//80  --100
  //  int front; = master->Front; // number2: 120 -> 150 //least  116  190
};

// fuzzy Theory

// 퍼지 세트 정의
//- 퍼지 세트를 나타내는 클래스입니다.
//- `name`: 퍼지 세트의 이름.
//- `points`: 특정 점과 그 점의 멤버십 값을 매핑합니다.
//- `membership(double x)`: 입력 값 `x`에 대한 멤버십 값을 반환합니다.
class FuzzySet {
 public:
  string name;                 // 세트의 이름
  map<double, double> points;  // 멤버십 함수에서 사용되는 점들의 매핑

  // 생성자
  FuzzySet() {}
  FuzzySet(string _name, map<double, double> _points)
      : name(_name), points(_points) {}

  // x 값에 대한 멤버십 함수
  double membership(double x) {
    auto it = points.lower_bound(x);
    if (it == points.end()) return 0.0;
    if (it == points.begin()) return it->second;
    auto it_prev = prev(it);
    double x1 = it_prev->first, y1 = it_prev->second;
    double x2 = it->first, y2 = it->second;
    if (x1 == x2) return y1;
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
  }
};

// 퍼지 변수 정의
// 퍼지 변수를 나타내는 클래스입니다.
//    - `name`: 변수의 이름.
//    - `sets`: 변수에 포함된 퍼지 세트의 매핑.
//    - `add_set(FuzzySet fuzzy_set)`: 변수가 포함하는 퍼지 세트를 추가합니다.
//    - `membership(string set_name, double x)`: 특정 퍼지 세트의 멤버십 값을
//    반환합니다.
class FuzzyVariable {
 public:
  string name;                 // 변수의 이름
  map<string, FuzzySet> sets;  // 변수에 속한 퍼지 세트들의 매핑

  // 생성자
  FuzzyVariable() {}
  FuzzyVariable(string _name) : name(_name) {}

  // 퍼지 세트 추가
  void add_set(FuzzySet fuzzy_set) { sets[fuzzy_set.name] = fuzzy_set; }

  // 특정 퍼지 세트에 대한 멤버십 함수
  double membership(string set_name, double x) {
    return sets[set_name].membership(x);
  }
};

// 퍼지 규칙 정의

//- 퍼지 규칙을 나타내는 클래스입니다.
//   - `antecedent`: 규칙의 조건부 부분 (변수와 퍼지 세트의 매핑).
//   - `consequent`: 규칙의 결과부 부분 (변수와 그 결과 값의 매핑).
//   - `evaluate(map<string, double> inputs)`: 규칙을 평가하여 결과를
//   반환합니다.

class FuzzyRule {
 public:
  map<string, string> antecedent;  // 규칙의 조건부 매핑
  map<string, double> consequent;  // 규칙의 결과 매핑

  // 생성자
  FuzzyRule() {}
  FuzzyRule(map<string, string> _antecedent, map<string, double> _consequent)
      : antecedent(_antecedent), consequent(_consequent) {}

  // 규칙을 평가하여 결과를 반환
  pair<double, map<string, double>> evaluate(map<string, double> inputs) {
    double min_membership = 1.0;
    for (auto &[var_name, set_name] : antecedent) {
      double membership = inputs[var_name];
      min_membership = min(min_membership, membership);
    }
    return {min_membership, consequent};
  }
};

// 퍼지 시스템 정의
// 퍼지 논리 시스템을 나타내는 클래스입니다.
//  - `variables`: 시스템에 포함된 변수들의 매핑.
//  - `rules`: 시스템에 적용되는 규칙들의 벡터.
//  - `add_variable(FuzzyVariable fuzzy_variable)`: 시스템에 변수를 추가합니다.
//  - `add_rule(FuzzyRule rule)`: 시스템에 규칙을 추가합니다.
//  - `evaluate(map<string, double> inputs)`: 입력 값에 따라 시스템의 출력을
//  계산합니다.
class FuzzySystem {
 public:
  map<string, FuzzyVariable> variables;  // 시스템에 속한 변수들의 매핑
  vector<FuzzyRule> rules;               // 시스템에 적용되는 규칙들의 벡터

  FuzzySystem() {}

  // 변수 추가
  void add_variable(FuzzyVariable fuzzy_variable) {
    variables[fuzzy_variable.name] = fuzzy_variable;
  }

  // 규칙 추가
  void add_rule(FuzzyRule rule) { rules.push_back(rule); }

  // 입력 값에 따른 결과 계산
  map<string, pair<double, map<string, double>>> evaluate(
      map<string, double> inputs) {
    map<string, pair<double, map<string, double>>> outputs;
    for (auto &rule : rules) {
      map<string, double> antecedent_values;
      for (auto &[var_name, set_name] : rule.antecedent) {
        antecedent_values[var_name] =
            variables[var_name].membership(set_name, inputs[var_name]);
      }
      auto [rule_strength, consequent] = rule.evaluate(antecedent_values);
      if (outputs.find(rule.antecedent.begin()->first) == outputs.end() ||
          rule_strength > outputs[rule.antecedent.begin()->first].first) {
        outputs[rule.antecedent.begin()->first] = {rule_strength, consequent};
      }
    }
    return outputs;
  }
};
}  // namespace robocup_master25
#endif  // PLAYER_HPP
