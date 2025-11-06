#ifndef LINE_H
#define LINE_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define DEG2RAD (M_PI / 180)

using namespace std;
using namespace cv;

class LINE
{
public:
    // For Init
    LINE()
    {
    }

public:
    // 그룹1의 구조체
    struct VISION_POINT_1
    {
        // 비전에서 찾은 포인트 데이터를 저장하는 구조체

        double CONFIDENCE; // 포인트 데이터의 신뢰도
        double DISTANCE;   // 포인트 데이터의 거리

        // 특징점과 로봇 사이의 거리
        int POINT_VEC_X;
        int POINT_VEC_Y;

        // 로봇의 좌표
        int STD_X;
        int STD_Y;
    };
    VISION_POINT_1 vision_point_1;

    // 그룹3의 구조체
    struct VISION_POINT_3
    {
        // 비전에서 찾은 포인트 데이터를 저장하는 구조체

        double CONFIDENCE; // 포인트 데이터의 신뢰도
        double DISTANCE;   // 포인트 데이터의 거리

        // 특징점과 로봇 사이의 거리
        int POINT_VEC_X;
        int POINT_VEC_Y;

        // 로봇의 좌표
        int STD_X;
        int STD_Y;
    };
    VISION_POINT_3 vision_point_3;

    vector<VISION_POINT_1> vision_point_vect_1; // VISION_POINT_1 를 저장할 벡터 컨테이너
    vector<VISION_POINT_3> vision_point_vect_3; // VISION_POINT_3 를 저장할 벡터 컨테이너

    Point CIRCLE_CENTER = Point(0, 0);
    double CIRCLE_R = 0;

    // 특징점 분리, (250, 400), (550, 325), (550, 475), (550, 400), (850, 400)
    // 특징점의 로컬 좌표(1번)
    int Local_point_x_1[22] = {100, 550, 1000, 100, 300, 800, 1000, 100, 200, 900, 1000, 100, 200, 900, 1000, 100, 300, 800, 1000, 100, 550, 1000};
    int Local_point_y_1[22] = {100, 100, 100, 150, 150, 150, 150, 250, 250, 250, 250, 550, 550, 550, 550, 650, 650, 650, 650, 700, 700, 700};

    // 특징점의 로컬 좌표(3번)
    int Local_point_x_3[5] = {250, 550, 550, 550, 850};
    int Local_point_y_3[5] = {400, 325, 475, 400, 400};

    // 특징점 활성화 변수
    int Local_point_on_off_1[22] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int Local_point_check_1[22] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    int Local_point_on_off_3[5] = {0, 0, 0, 0, 0};
    int Local_point_check_3[5] = {0, 0, 0, 0, 0};

private:
    Mat Likelihood_mat;

public:
    // 그룹1
    void set_circle(double yaw, vector<VISION_POINT_1> &vect1, vector<VISION_POINT_3> &vect3)
    {
        // PRE CONDITION : YAW, VECTOR<VISION_POINT>
        // POST CONDITION :
        // PURPOSE : 로컬의 특징점 범위를 설정

        double X_MAX = -9999, X_MIN = 9999;
        double Y_MAX = -9999, Y_MIN = 9999;

        for (int i = 0; i < vect1.size(); i++)
        {
            // 포착된 포인트 중 최대 거리와 최소 거리를 계산
            int transformation_x = (vect1[i].POINT_VEC_X) * cos((-1) * yaw * DEG2RAD) - (vect1[i].POINT_VEC_Y) * sin((-1) * yaw * DEG2RAD);
            int transformation_y = (vect1[i].POINT_VEC_X) * sin((-1) * yaw * DEG2RAD) + (vect1[i].POINT_VEC_Y) * cos((-1) * yaw * DEG2RAD);
            if (transformation_x > X_MAX)
            {
                X_MAX = transformation_x;
            }
            if (transformation_x < X_MIN)
            {
                X_MIN = transformation_x;
            }
            if (transformation_y > Y_MAX)
            {
                Y_MAX = transformation_y;
            }
            if (transformation_y < Y_MIN)
            {
                Y_MIN = transformation_y;
            }
        }

        // 그룹 3 특징점 좌표 변환 및 최대/최소 값 계산
        for (int i = 0; i < vect3.size(); i++)
        {
            int transformation_x = vect3[i].POINT_VEC_X * cos(-yaw * DEG2RAD) - vect3[i].POINT_VEC_Y * sin(-yaw * DEG2RAD);
            int transformation_y = vect3[i].POINT_VEC_X * sin(-yaw * DEG2RAD) + vect3[i].POINT_VEC_Y * cos(-yaw * DEG2RAD);
            if (transformation_x > X_MAX)
            {
                X_MAX = transformation_x;
            }
            if (transformation_x < X_MIN)
            {
                X_MIN = transformation_x;
            }
            if (transformation_y > Y_MAX)
            {
                Y_MAX = transformation_y;
            }
            if (transformation_y < Y_MIN)
            {
                Y_MIN = transformation_y;
            }
        }

        if (vect1.size() > 0)
        {
            // 로봇의 좌표에서 최대 거리 값과 최소 거리 값의 평균 값을 더한 후 해당 값을 범위의 중앙으로 설정
            CIRCLE_CENTER = Point((int)(vect1[0].STD_X + ((X_MAX + X_MIN) / 2) * cos(yaw * DEG2RAD) - ((Y_MAX + Y_MIN) / 2) * sin(yaw * DEG2RAD)), (int)(vect1[0].STD_Y + ((X_MAX + X_MIN) / 2) * sin(yaw * DEG2RAD) + ((Y_MAX + Y_MIN) / 2) * cos(yaw * DEG2RAD)));
        }
        if (vect3.size() > 0)
        {
            CIRCLE_CENTER = Point((int)(vect3[0].STD_X + ((X_MAX + X_MIN) / 2) * cos(yaw * DEG2RAD) - ((Y_MAX + Y_MIN) / 2) * sin(yaw * DEG2RAD)), (int)(vect3[0].STD_Y + ((X_MAX + X_MIN) / 2) * sin(yaw * DEG2RAD) + ((Y_MAX + Y_MIN) / 2) * cos(yaw * DEG2RAD)));
        }




        // 로봇의 좌표에서 최대 거리 값과 최소 거리 값의 평균 값을 뺀 후 해당 값에서 +50 한 값을 범위의 반지름으로 설정
        CIRCLE_R = 0;

        for (int i = 0; i < vect1.size(); i++)
        {
            // cout << "CIRCLE_CENTER.x : " << CIRCLE_CENTER.x << endl;
            // cout << "CIRCLE_CENTER.y : " << CIRCLE_CENTER.y << endl;
            // cout << "vect[i].POINT_VEC_X : " << vect[i].POINT_VEC_X << endl;
            // cout << "vect[i].POINT_VEC_Y : " << vect[i].POINT_VEC_Y << endl;
            // cout << "vect[i].STD_X : " << vect[i].STD_X << endl;
            // cout << "vect[i].STD_Y : " << vect[i].STD_Y << endl;

            double dis = sqrt(pow(CIRCLE_CENTER.x - vect1[i].POINT_VEC_X - vect1[i].STD_X, 2) +
                            pow(CIRCLE_CENTER.y - vect1[i].POINT_VEC_Y - vect1[i].STD_Y, 2));
            if (CIRCLE_R < dis)
            {
                CIRCLE_R = dis;
            }
        }
        for (int i = 0; i < vect3.size(); i++)
        {
            double dis = sqrt(pow(CIRCLE_CENTER.x - vect3[i].POINT_VEC_X - vect3[i].STD_X, 2) +
                            pow(CIRCLE_CENTER.y - vect3[i].POINT_VEC_Y - vect3[i].STD_Y, 2));
            if (CIRCLE_R < dis)
            {
                CIRCLE_R = dis;
            }
        }

        CIRCLE_R += 50;
    }

    void on_local_point(int PT_X, int PT_Y, int NOW_X, int NOW_Y)
    {
        // 그룹 1 로컬 포인트 활성화 여부 계산
        for (int i = 0; i < 22; i++)  // 그룹 1: 22개 포인트
        {
            if (pow(CIRCLE_R, 2) > pow(PT_X - NOW_X + CIRCLE_CENTER.x - Local_point_x_1[i], 2) + pow(PT_Y - NOW_Y + CIRCLE_CENTER.y - Local_point_y_1[i], 2))
            {
                Local_point_on_off_1[i] = 1;  // 활성화
            }
            else
            {
                Local_point_on_off_1[i] = 0;  // 비활성화
            }
        }

        // 그룹 3 로컬 포인트 활성화 여부 계산
        for (int i = 0; i < 5; i++)  // 그룹 3: 5개 포인트
        {
            if (pow(CIRCLE_R, 2) > pow(PT_X - NOW_X + CIRCLE_CENTER.x - Local_point_x_3[i], 2) + pow(PT_Y - NOW_Y + CIRCLE_CENTER.y - Local_point_y_3[i], 2))
            {
                Local_point_on_off_3[i] = 1;  // 활성화
            }
            else
            {
                Local_point_on_off_3[i] = 0;  // 비활성화
            }
        }
    }

    void check_local_point(int ago_point_cnt, double yaw, vector<VISION_POINT_1> &vect1, vector<VISION_POINT_3> &vect3)
    {
        // PRE CONDITION : ago_point_cnt = 50, yaw, vect<VISION_POINT> = VISION_POINT_VECT
        // POST CONDITION : Local_point_check
        // PURPOSE : 로봇의 yaw값을 통해 파티클 필터 계산에 사용될 특징점 활성화

        double X_MAX = -9999, X_MIN = 9999;
        double Y_MAX = -9999, Y_MIN = 9999;
        for (int i = ago_point_cnt; i < vect1.size(); i++)
        {
            // 포착된 포인트 중 최대 거리와 최소 거리를 계산, 이때 벡터 컨테이너에서 50번째 이후 요소부터 계산
            int transformation_x = (vect1[i].POINT_VEC_X) * cos((-1) * yaw * DEG2RAD) - (vect1[i].POINT_VEC_Y) * sin((-1) * yaw * DEG2RAD);
            int transformation_y = (vect1[i].POINT_VEC_X) * sin((-1) * yaw * DEG2RAD) + (vect1[i].POINT_VEC_Y) * cos((-1) * yaw * DEG2RAD);
            if (transformation_x > X_MAX)
            {
                X_MAX = transformation_x;
            }
            if (transformation_x < X_MIN)
            {
                X_MIN = transformation_x;
            }
            if (transformation_y > Y_MAX)
            {
                Y_MAX = transformation_y;
            }
            if (transformation_y < Y_MIN)
            {
                Y_MIN = transformation_y;
            }
        }

        // 그룹 3 특징점 최대/최소 값 계산
        for (int i = ago_point_cnt; i < vect3.size(); i++)
        {
            int transformation_x = (vect3[i].POINT_VEC_X) * cos((-1) * yaw * DEG2RAD) - (vect3[i].POINT_VEC_Y) * sin((-1) * yaw * DEG2RAD);
            int transformation_y = (vect3[i].POINT_VEC_X) * sin((-1) * yaw * DEG2RAD) + (vect3[i].POINT_VEC_Y) * cos((-1) * yaw * DEG2RAD);
            if (transformation_x > X_MAX)
            {
                X_MAX = transformation_x;
            }
            if (transformation_x < X_MIN)
            {
                X_MIN = transformation_x;
            }
            if (transformation_y > Y_MAX)
            {
                Y_MAX = transformation_y;
            }
            if (transformation_y < Y_MIN)
            {
                Y_MIN = transformation_y;
            }
        }

        if (vect1.size() > 0)
        {
            // 로봇의 좌표에서 최대 거리 값과 최소 거리 값의 평균 값을 더한 후 해당 값을 범위의 중앙으로 설정
            CIRCLE_CENTER = Point((int)(vect1[0].STD_X + ((X_MAX + X_MIN) / 2) * cos(yaw * DEG2RAD) - ((Y_MAX + Y_MIN) / 2) * sin(yaw * DEG2RAD)), (int)(vect1[0].STD_Y + ((X_MAX + X_MIN) / 2) * sin(yaw * DEG2RAD) + ((Y_MAX + Y_MIN) / 2) * cos(yaw * DEG2RAD)));
        }
        else
        {
            CIRCLE_CENTER = Point((int)(vect3[0].STD_X + ((X_MAX + X_MIN) / 2) * cos(yaw * DEG2RAD) - ((Y_MAX + Y_MIN) / 2) * sin(yaw * DEG2RAD)), (int)(vect3[0].STD_Y + ((X_MAX + X_MIN) / 2) * sin(yaw * DEG2RAD) + ((Y_MAX + Y_MIN) / 2) * cos(yaw * DEG2RAD)));
        }



        // 로봇의 좌표에서 최대 거리 값과 최소 거리 값의 평균 값을 뺀 후 해당 값에서 +50 한 값을 범위의 반지름으로 설정
        CIRCLE_R = 0;

        // 그룹 1 반지름 계산
        for (int i = ago_point_cnt; i < vect1.size(); i++)
        {
            double dis = sqrt(pow(CIRCLE_CENTER.x - vect1[i].POINT_VEC_X - vect1[i].STD_X, 2) + pow(CIRCLE_CENTER.y - vect1[i].POINT_VEC_Y - vect1[i].STD_Y, 2));
            if (CIRCLE_R < dis)
            {
                CIRCLE_R = dis;
            }
        }

        // 그룹 3 반지름 계산
        for (int i = ago_point_cnt; i < vect3.size(); i++)
        {
            double dis = sqrt(pow(CIRCLE_CENTER.x - vect3[i].POINT_VEC_X - vect3[i].STD_X, 2) + pow(CIRCLE_CENTER.y - vect3[i].POINT_VEC_Y - vect3[i].STD_Y, 2));
            if (CIRCLE_R < dis)
            {
                CIRCLE_R = dis;
            }
        }

        CIRCLE_R += 50;

        // 그룹 1 특징점 활성화 상태 계산
        for (int i = 0; i < 22; i++)
        {
            if (pow(CIRCLE_R, 2) > pow(CIRCLE_CENTER.x - Local_point_x_1[i], 2) + pow(CIRCLE_CENTER.y - Local_point_y_1[i], 2))
            {
                Local_point_check_1[i] = 1;
            }
            else
            {
                Local_point_check_1[i] = 0;
            }
        }

        // 그룹 3 특징점 활성화 상태 계산
        for (int i = 0; i < 5; i++)
        {
            if (pow(CIRCLE_R, 2) > pow(CIRCLE_CENTER.x - Local_point_x_3[i], 2) + pow(CIRCLE_CENTER.y - Local_point_y_3[i], 2))
            {
                Local_point_check_3[i] = 1;
            }
            else
            {
                Local_point_check_3[i] = 0;
            }
        }
    }
    double sence(int PT_X, int PT_Y, int NOW_X, int NOW_Y, vector<VISION_POINT_1> &vect1, vector<VISION_POINT_3> &vect3)
    {
        // !!!!!!!제일 중요한 부분!!!!!!!
        // PRE CONDITION : PT_X, PT_Y, ROBOT_X, ROBOT_Y, VECTOR<VISION_POINT> = VISION_POINT_VECT
        // POST CONDITION : weight
        // PURPOSE : 가중치를 구하는 함수

        on_local_point(PT_X, PT_Y, NOW_X, NOW_Y); // 활성화 할 특징점 포인트 계산
        double weight_1 = 0.0;
        double weight_3 = 0.0;

        for (int i = 0; i < vect1.size(); i++)
        // 비전에서 포착한 포인트 수 많큼 연산
        {

            // pt = 파티클 좌표 + 계산에 사용된 로봇의 좌표 - 현 상태의 로봇 좌표 + 특징점과 로봇 사이의 거리
            int ptx = PT_X + vect1[i].STD_X - NOW_X + vect1[i].POINT_VEC_X;
            int pty = PT_Y + vect1[i].STD_Y - NOW_Y + vect1[i].POINT_VEC_Y;
            //std::cout << "vec1 : " << "ptx : " << ptx << "  pty : " << pty << std::endl;
            double min_dis = 99999999;

            for (int j = 0; j < 22; j++)
            {
                // 특징점 수 만큼 연산
                double dis = 99999999;
                if (Local_point_on_off_1[j] == 1) // 해당 특징점이 활성화 시 실행
                {
                    // 특징점의 로컬 좌표와 pt 사이의 거리
                    dis = sqrt(pow(Local_point_x_1[j] - ptx, 2) + pow(Local_point_y_1[j] - pty, 2));
                }
                // 해당 값이 전체 특징점 가운데서 가장 가까우면 해당 값의 거리 값을 저장
                if (min_dis > dis)
                {
                    min_dis = dis;
                }
            }
            if (min_dis <= 10)
            {
                min_dis = 10;
            }
            //            else if(min_dis >= 50){min_dis = (-1)*min_dis;}
            // 해당 값과 VISION_POINT의 신뢰도, 거리값을 통해 가중치 계산
            weight_1 += (10 / min_dis) * vect1[i].CONFIDENCE * abs(1 - vect1[i].DISTANCE / 10000);
            // cout << "i : " << i << endl;
            // cout << "min_dis : " << min_dis << endl;
            // cout << "vect[i].CONFIDENCE : " << vect[i].CONFIDENCE << endl;
            // cout << "vect[i].DISTANCE : " << vect[i].DISTANCE << endl;
            // cout << "weight : " << weight << endl;
        }

        // 그룹 3 가중치 계산
        for (int i = 0; i < vect3.size(); i++)
        {
            int ptx = PT_X + vect3[i].STD_X - NOW_X + vect3[i].POINT_VEC_X;
            int pty = PT_Y + vect3[i].STD_Y - NOW_Y + vect3[i].POINT_VEC_Y;

            //std::cout << "vec3 : " << "ptx : " << ptx << "  pty : " << pty << std::endl;
            double min_dis_2 = 99999999;

            for (int j = 0; j < 5; j++)  // 그룹 3 특징점
            {

                double dis = 99999999;
                if (Local_point_on_off_3[j] == 1)
                {
                    dis = sqrt(pow(Local_point_x_3[j] - ptx, 2) + pow(Local_point_y_3[j] - pty, 2));

                }
                if (min_dis_2 > dis)
                {
                    min_dis_2 = dis;
                }
            }

            if (min_dis_2 <= 10) //최소 거리 보정
            {
                min_dis_2 = 10;
            }

            //std::cout << "confidence  " << i << " : " << vect3[i].CONFIDENCE << std::endl;
            //std::cout << "distance  " << i << " : " << vect3[i].DISTANCE << std::endl;
            //std::cout << "min_dis  " << i << " : " << min_dis << std::endl;
            weight_3 += (10 / min_dis_2) * vect3[i].CONFIDENCE * abs(1 - vect3[i].DISTANCE / 10000);
        }

        // 최종 가중치 계산 (비율 적용 가능)

        double final_weight = weight_1 + weight_3;  // 그룹 1과 3의 비율 조정

        // std::cout << "weight_1 : " << weight_1 << std::endl;
        // std::cout << "weight_3 : " << weight_3 << std::endl;
        return final_weight;
    }
    //    double sence(int PT_X, int PT_Y, int NOW_X, int NOW_Y, vector<VISION_POINT> &vect)
    //    {
    //        on_local_point(PT_X, PT_Y, NOW_X, NOW_Y);
    //        double weight = 0.0;
    //        int std_R = 50;

    //        for(int i = 0; i < vect.size(); i++)
    //        {
    //            int ptx = PT_X + vect[i].STD_X - NOW_X + vect[i].POINT_VEC_X;
    //            int pty = PT_Y + vect[i].STD_Y - NOW_Y + vect[i].POINT_VEC_Y;
    //            for(int j = 0; j < 27; j++)
    //            {
    //                double dis = pow(Local_point_x[j] - ptx, 2) + pow(Local_point_y[j] - pty, 2);

    //                if(pow(std_R, 2) > dis)
    //                {
    //                    weight += ((-1) * double(sqrt(dis)) / double(std_R) + 1) * vect[i].CONFIDENCE * abs(1 - vect[i].DISTANCE / 10000);
    //                }
    //            }

    //        }
    //        return weight;

    //    }

private:
    int possibility_matching(int NUM, int N_NUM, int X_NUM, int L_NUM, int T_NUM)
    {
        if (NUM == 1)
        {
            return 0;
        }
        else if (NUM == 2)
        {
            if (T_NUM > 0)
            {
                return 1;
            }
        }
        else if (NUM == 3)
        {
            return 0;
        }
        else if (NUM == 4)
        {
            if (X_NUM > 0)
            {
                return 1;
            }
        }
        else if (NUM == 5)
        {
            if (X_NUM > 0 || L_NUM > 0)
            {
                return 1;
            }
        }
        else if (NUM == 6)
        {
            if (X_NUM > 0)
            {
                return 1;
            }
        }
        else if (NUM == 7)
        {
            return 0;
        }
        else if (NUM == 8)
        {
            if (T_NUM > 0)
            {
                return 1;
            }
        }
        else if (NUM == 9)
        {
            return 0;
        }
        return 0;
    }

    Rect get_grid_size(int Num)
    {
        int x = 0, y = 0, w = 0, h = 0;
        if (Num == 1)
        {
            x = 0;
            y = 0;
            w = 400;
            h = 300;
        }
        else if (Num == 2)
        {
            x = 400;
            y = 0;
            w = 300;
            h = 150;
        }
        else if (Num == 3)
        {
            x = 700;
            y = 0;
            w = 400;
            h = 300;
        }
        else if (Num == 4)
        {
            x = 0;
            y = 300;
            w = 400;
            h = 200;
        }
        else if (Num == 5)
        {
            x = 400;
            y = 150;
            w = 300;
            h = 500;
        }
        else if (Num == 6)
        {
            x = 700;
            y = 300;
            w = 400;
            h = 200;
        }
        else if (Num == 7)
        {
            x = 0;
            y = 500;
            w = 400;
            h = 300;
        }
        else if (Num == 8)
        {
            x = 400;
            y = 650;
            w = 300;
            h = 150;
        }
        else if (Num == 9)
        {
            x = 700;
            y = 500;
            w = 400;
            h = 300;
        }
        else
        {
            x = 0;
            y = 0;
            w = 0;
            h = 0;
        }

        Rect rect(x, y, w, h);
        return rect;
    }
    Mat get_grid_likelihood(int Num)
    {
        Rect bounds(0, 0, 1100, 800);
        Rect r = get_grid_size(Num);
        Mat roi = Likelihood_mat(r & bounds);
        return roi;
    }

    int get_grid_index(int X, int Y)
    {
        if (X > 0 && Y > 0 && X <= 400 && Y <= 300)
        {
            return 1;
        }
        else if (X > 400 && Y > 0 && X <= 700 && Y <= 150)
        {
            return 2;
        }
        else if (X > 700 && Y > 0 && X <= 1100 && Y <= 300)
        {
            return 3;
        }
        else if (X > 0 && Y > 300 && X <= 400 && Y <= 500)
        {
            return 4;
        }
        else if (X > 400 && Y > 150 && X <= 700 && Y <= 650)
        {
            return 5;
        }
        else if (X > 700 && Y > 300 && X <= 1100 && Y <= 500)
        {
            return 6;
        }
        else if (X > 0 && Y > 500 && X <= 400 && Y <= 800)
        {
            return 7;
        }
        else if (X > 400 && Y > 650 && X <= 700 && Y <= 800)
        {
            return 8;
        }
        else if (X > 700 && Y > 500 && X <= 1100 && Y <= 800)
        {
            return 9;
        }
        else
        {
            return 0;
        }
    }
};

#endif // LINE_H
