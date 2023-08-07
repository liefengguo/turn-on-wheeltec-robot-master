/* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).
   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and 
   Sorniotti et al. Path tracking for Automated Driving, 2017.
*/
#include "../../include/common/purepursuit.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include "../../include/BezierCurve.h"
#include "../../include/DWA.h"

string line;
int tmp = 0, i, gap = 0, tmp_increasement = 0;
double x;
using namespace std;


// 这个函数算得角度用来构造旋转矩阵
int get_angle(double x, double y) {
    if (x >= 0 && y >= 0) // 第一象限
        return 90 - atan2(y, x) * 180 / M_PI;
    else if (x >= 0 && y < 0)
        return 0 - atan2(y, x) * 180 / M_PI + 90;
    else if (x < 0 && y >= 0)
        return 180 - atan2(y, x) * 180 / M_PI + 270;
    else
        return 0 - atan2(y, x) * 180 / M_PI + 90;
}

// 这个函数算的值赋给给dwa中，来初始化运动
double get_start_angle(double x, double y) {
    if (x >= 0 && y >= 0) // 第一象限
        return atan2(y, x) * 180 / M_PI;
    else if (x >= 0 && y < 0) // 四
        return 360 + atan2(y, x) * 180 / M_PI;
    else if (x < 0 && y >= 0)
        return atan2(y, x) * 180 / M_PI;
    else
        return 360 + atan2(y, x) * 180 / M_PI;
}

PurePursuit::PurePursuit() : lookahead_distance_(1.0), v_max_(0.3), car_vel(v_max_), w_max_(1.0), index(0),
                             goal_reached_(false), nh_private_("~")
{
  // Get parameters from the parameter server
  nh_private_.param<bool>("log_flag", log_flag, 1);
  nh_private_.param<double>("max_rotational_velocity", w_max_, 1);
  nh_private_.param<double>("position_tolerance", position_tolerance_, 0.1);
  nh_private_.param<double>("steering_angle_limit", delta_max_, 1);
  nh_private_.param<double>("car_vel", car_vel, 0.4);
  nh_private_.param<double>("lookahead_distance", lookahead_distance_, 3);
  nh_private_.param<string>("pure_pursuit_log_path", pure_pursuit_log_path, "/home/glf/log/");
  pointNum = 0, tmp_count = 0, stop_count = 0,aim_index=9999;
  in_bizhang = false,has_barrier = false;
  load_aims();
  carPose_sub = nh_.subscribe("/fixposition/odometry", 1, &PurePursuit::poseCallback, this);

  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("coordinate_marker", 10);
  marker_car_pub= nh_.advertise<visualization_msgs::Marker>("car_marker", 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>("rvizpath", 100, true);
  marker_pub_street = nh_.advertise<visualization_msgs::Marker>("turn_angle_marker", 1);
  marker_pub_fuzhu = nh_.advertise<visualization_msgs::Marker>("fuzhu_marker", 1);
  marker_pub_ext_car = nh_.advertise<visualization_msgs::Marker>("ext_car_marker", 1);

    path.header.frame_id = "map";
    // 设置时间戳
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    // 设置参考系
    pose.header.frame_id = "map";
    will_stop = false;
    x_bei = 0;
    x_up = true, x_down = false;
    cout << "执行到了这里，初始化完成: " << pointNum << endl;
}

void PurePursuit::load_aims() {
    cout << "开始加载目标点" << endl;
    ifstream in(load_aim_path);
    int index = 0;
    while (getline(in, line)) {
        stringstream ss(line);
        tmp = 0, index++;
        while (ss >> x) {
            if (tmp == 0)
                r_x_.push_back(x);
            else if (tmp == 1)
                r_y_.push_back(x);
            else if (tmp == 2)
                r_z_.push_back(x);
            else
                r_yam_.push_back(x);
            tmp++;
        }
    }
    pointNum = index;
}

bool PurePursuit::detect_barrier() {
    tmp_count++;
    if (tmp_count == 80)
    {
        cout << "检测到了障碍物" << endl;
        return true;
    }
    return false;
}

double PurePursuit::getDegree(){return degree;}
//计算发送给模型车的转角
void PurePursuit::poseCallback(const nav_msgs::Odometry &currentWaypoint) {
  double x ,y,z;
  // GaussProjCal(currentWaypoint.pose.pose.position.x,currentWaypoint.pose.pose.position.y,&x,&y);
  ecefToEnu(currentWaypoint.pose.pose.position.x,currentWaypoint.pose.pose.position.y,currentWaypoint.pose.pose.position.z,&x,&y,&z);
  // cout<<"时间："<< currentWaypoint.header.stamp<<endl;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("FP_ENU", "ECEF", ros::Time(0),ros::Duration(1));
    geometry_msgs::QuaternionStamped ecefOrientation;
    ecefOrientation.quaternion.w = currentWaypoint.pose.pose.orientation.w;
    ecefOrientation.quaternion.x = currentWaypoint.pose.pose.orientation.x;
    ecefOrientation.quaternion.y = currentWaypoint.pose.pose.orientation.y;
    ecefOrientation.quaternion.z = currentWaypoint.pose.pose.orientation.z;
    geometry_msgs::QuaternionStamped enuOrientation; 
    tf2::doTransform(ecefOrientation, enuOrientation, transformStamped);

    auto currentPositionX = x;
    auto currentPositionY = y;
    auto currentPositionZ = z;
    auto currentQuaternionX = enuOrientation.quaternion.x;
    auto currentQuaternionY = enuOrientation.quaternion.y;
    auto currentQuaternionZ = enuOrientation.quaternion.z;
    auto currentQuaternionW = enuOrientation.quaternion.w;
    auto currentPositionYaw = tf::getYaw(enuOrientation.quaternion) ;

    /***********通过计算当前坐标和目标轨迹距离，找到一个最小距离的索引号***************************************************************************************/
    vector<double> bestPoints_;
    for (int i = 0; i < pointNum; i++) {
      // float lad = 0.0;
      double path_x = r_x_[i];
      double path_y = r_y_[i];
      // 遍历所有路径点和当前位置的距离，保存到数组中
      double lad = sqrt(pow(path_x - currentPositionX, 2) +
                      pow(path_y - currentPositionY, 2));
      bestPoints_.push_back(lad);
    }
    // 找到数组中最小横向距离
    auto smallest = min_element(bestPoints_.begin(), bestPoints_.end());
    // 找到最小横向距离的索引位置
    index = distance(bestPoints_.begin(), smallest);
    int temp_index;
    for (int i = index; i < pointNum; i++) {
      //遍历路径点和预瞄点的距离，从最小横向位置的索引开始
      float dis =
          sqrt(pow(r_y_[index] - r_y_[i], 2) + pow(r_x_[index] - r_x_[i], 2));
      //判断跟预瞄点的距离
      preview_dis = k * car_vel + lookahead_distance_;
      if (dis < preview_dis) {
        temp_index = i;
      } else {
        break;
      }
    }
    index = temp_index;
    now_pos_x = currentPositionX,now_pos_y = currentPositionY;
    cout << "下一个目标点是：" << index << " " << now_pos_x << " " << now_pos_y << endl;
    /**************************************************************************************************/
    float alpha_two_point = atan2(r_y_[index] - currentPositionY, r_x_[index] - currentPositionX); 
    float alpha = alpha_two_point - currentPositionYaw;
    alpha = (alpha > M_PI) ? (alpha - 2 * M_PI) : (alpha < -M_PI) ? (alpha + 2 * M_PI) : alpha;

    // 当前点和目标点的距离Id
    float dl = sqrt(pow(r_y_[index] - currentPositionY, 2) +
                    pow(r_x_[index] - currentPositionX, 2));
    // 发布小车运动指令及运动轨迹
    if (dl > 0.3) {
      car_vel = 0.3;
      float theta = atan(2 * Ld * sin(alpha) / dl);
      degree = theta * 180 / M_PI;
      degree = max(min(38.6, -degree), -38.6);
      double theta_send = degree * 0.5 / 38.6;
      // car_vel *= std::abs(cos(theta_send));
      double curYaw_deg = currentPositionYaw * 180 / M_PI;

      // cout<<"alpha: "<<alpha* 180 /M_PI<<" degree"<<degree<<" dis: "<<dl<<"  index:"<<index<<"x: "<<r_x_[index]<<"y: "<<r_y_[index]<<endl;
     // cout<<"curr_yaw"<< curYaw_deg <<" alpha_two_point:" << alpha_two_point * 180 / M_PI<<endl;
      geometry_msgs::Twist vel_msg;
      vel_msg.linear.x = 0.05 * x_bei;
      vel_msg.angular.z = theta_send;
      
      cout << "____________________" << theta_send << " " << degree << endl;
      pub_vel_.publish(vel_msg);

    } else {
        cout << "-------------------------------------NO: " << dl << " index=" << index << endl;
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 0;
        if (index > pointNum - 5) // 到这里确实就要停止了
        {
            vel_msg.linear.x = 0.05 * x_bei;
            pub_vel_.publish(vel_msg);
            x_down = true;
            will_stop = true;
            cout << vel_msg.linear.x << endl;
        } else // 假停止，继续走
        {
            vel_msg.linear.x = 0.05 * x_bei;
            pub_vel_.publish(vel_msg);
            cout << vel_msg.linear.x << endl;
        }
    }
      if(in_bizhang== true && index>=aim_index+15)
      {
          in_bizhang = false;
          cout << "*************************开始恢复正常带超声波的贴边" << endl;
      }

      if (detect_barrier() == true && in_bizhang == false) // 检测到障碍物
      {
          cout << "检测到障碍物!!!!!!!!!!" << endl;
          in_bizhang = true;
          x_down = true;
          // 接下来可以考虑第一次进来的时候开一个线程去生成轨迹，轨迹生成完了之后再动起来
          has_barrier = true;
      }
  }// try
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM(ex.what());
  }
}

/// 生成避障轨迹的代码
void PurePursuit::dwa_get_trace() {
    for (i = index; i < pointNum; i++) {
        float dis = sqrt(pow(now_pos_y - r_y_[i], 2) + pow(now_pos_x - r_x_[i], 2));
        if (dis > 12.0) {
            end_x = r_x_[i] - r_x_[index];
            end_y = r_y_[i] - r_y_[index];
            aim_index = i;
            break;
        }
    }
    gap = aim_index - index;
    cout << "开始生成避障轨迹！ " << index << " " << aim_index << endl;
    int angle = get_angle(end_x, end_y);
    Vector2d end;
    vector <Vector2d> barrier;

    /*躲避人的模型*/
    barrier.push_back(Vector2d(0, 3.8));
    barrier.push_back(Vector2d(0, 4.2));
    barrier.push_back(Vector2d(0, 4.6));
    barrier.push_back(Vector2d(0, 5));
    barrier.push_back(Vector2d(0, 5.4));
    barrier.push_back(Vector2d(0, 5.8));
    barrier.push_back(Vector2d(0, 6.2));
    barrier.push_back(Vector2d(0, 6.6));
    barrier.push_back(Vector2d(0, 7.0));
    barrier.push_back(Vector2d(-0.4, 3.8));
    barrier.push_back(Vector2d(-0.8, 3.8));
    barrier.push_back(Vector2d(-1.2, 3.8));
    barrier.push_back(Vector2d(-1.6, 3.8));
    barrier.push_back(Vector2d(-1.6, 4.2));
    barrier.push_back(Vector2d(-1.6, 4.6));
    barrier.push_back(Vector2d(-1.6, 5));
    barrier.push_back(Vector2d(-1.6, 5.4));
    barrier.push_back(Vector2d(-1.6, 5.8));
    barrier.push_back(Vector2d(-1.6, 6.2));
    barrier.push_back(Vector2d(-1.6, 6.6));
    barrier.push_back(Vector2d(-1.6, 7.0));
    barrier.push_back(Vector2d(-1.2, 7.0));
    barrier.push_back(Vector2d(-0.8, 7.0));
    barrier.push_back(Vector2d(-0.4, 7.0));
    barrier.push_back(Vector2d(0.4, 3));

    // 求出旋转矩阵
    MatrixXd to_y(2, 2), y_to(2, 2); // to_y表示映到y轴上  y_to表示映回去
    to_y(0, 0) = cos(angle * M_PI / 180);
    to_y(0, 1) = sin(angle * M_PI / 180);
    to_y(1, 0) = 0 - sin(angle * M_PI / 180);
    to_y(1, 1) = cos(angle * M_PI / 180);
    // 初始化两个旋转矩阵
    y_to(0, 0) = cos(angle * M_PI / 180);
    y_to(0, 1) = 0 - sin(angle * M_PI / 180);
    y_to(1, 0) = sin(angle * M_PI / 180);
    y_to(1, 1) = cos(angle * M_PI / 180);

    // --------------------------接下来是dwa的内容----------------------------
    VectorXd state(5);
    // 初始化系统状态
    state << 0.0, 0.0, 9 *PI/ 17, 0.0, 0.0;//[x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    Vector2d goal(0, 12); //目标点
    double dt = 2; //采样时间   0.25
    // double v_min=-0.5,v_max=0.5,w_min=-40*PI/180,w_max=40*PI/180; //线速度角速度边界
    double v_min = -0.4, v_max = 0.4, w_min = -20 * PI / 180, w_max = 20 * PI / 180; //线速度角速度边界
    double predict_time = 5.0;//轨迹推算时间长度
    double a_vmax = 0.2, a_wmax = 40 * PI / 180; //线加速度和角加速度最大值
    double v_sample = 0.1, w_sample = 1.2 * PI / 180; //采样分辨率，提高这两个值可以加快程序的速度，而且对结果的效果貌似也没有什么影响,nice
    double alpha = 0.15, beta = 8.0, gamma = 4; //轨迹评价函数系数
    double radius = 1.0; // 用于判断是否到达目标点
    double judge_distance = 10; //若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值
    vector <Vector2d> obstacle, Ps;//障碍物位置，生成的原始轨迹
    // 将障碍物投影到实际位置上
    for (i = 0; i < barrier.size(); i++)
        obstacle.push_back(barrier[i]);
    vector <VectorXd> trajectory;
    trajectory.push_back(state);
    DWA dwa(dt, v_min, v_max, w_min, w_max, predict_time, a_vmax, a_wmax, v_sample, w_sample, alpha, beta, gamma,
            radius, judge_distance);
    //保存画图数据
    vector<double> x_, y_, predict_x, predict_y;
    while (true) {
        pair <vector<double>, vector<VectorXd>> res = dwa.dwaControl(state, goal, obstacle);
        state = dwa.kinematicModel(state, res.first, dt);
        trajectory.push_back(state);
        x_.push_back(state(0));
        y_.push_back(state(1));
        for (VectorXd s: res.second) {//画出推算的轨迹
            predict_x.push_back(s(0));
            predict_y.push_back(s(1));
        }
        predict_x = {};
        predict_y = {};
        double dist_to_goal = (state.head(2) - goal).norm();
        if (dist_to_goal <= radius)
            break;
    }
    // -----------------------------------下面是贝塞尔曲线的内容-------------------------------
    Ps.push_back(Vector2d(0,0));
    //  注意，相关系数在这里改，整个曲线的缩放系数
    for (i = 0; i < y_.size(); i++)
        Ps.push_back(Vector2d(x_[i]/1.15, y_[i]));
    Ps.push_back(Vector2d(0,12));
    cout << "生成的轨迹为：" << endl;
    tmp_increasement = 0;
    for (i = 0; i < 100; i++) {
        Vector2d pos = bezierCommon(Ps, (double) i / 100);
        if (i % (100 / gap) == 0)  // 将原来的路标点进行替换
        {
            end = to_y * pos;
            r_x_[index + tmp_increasement] = now_pos_x + end[0];
            r_y_[index + tmp_increasement] = now_pos_y + end[1];
            cout << now_pos_x << " " << now_pos_y << " " << end[0] << " " << end[1] << endl;
            tmp_increasement++;
            if(tmp_increasement>=gap)
                break;
        }
    }
    save_new_pos();
    return;
}

void PurePursuit::save_new_pos() {
    fstream f;
    f.open(new_save_path, ios::out | ios::app);
    for (i = 0; i < r_x_.size(); i++) {
        f << r_x_[i] << " " << r_y_[i] << " " << r_z_[i] << " " << r_yam_[i] << endl;
    }
    cout << "保存新路径完成！" << endl;
    f.close();
    return;
}

PurePursuit::~PurePursuit(){
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = 0;
  vel_msg.angular.z = 0;
  pub_vel_.publish(vel_msg);

}
void PurePursuit::run()
{
  ros::spin();
}

