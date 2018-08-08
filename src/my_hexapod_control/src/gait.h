#ifndef GAI_H_
#define GAI_H_

#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <hexapod_msgs/FeetPositions.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
 
class Gait
{
public:
  Gait(void);
  void gaitCycle( const geometry_msgs::Twist &cmd_vel, hexapod_msgs::FeetPositions *feet );  //摆动腿和支撑腿切换
  int cycle_period_;
  std::vector<int> cycle_leg_number_;
private:
  void cyclePeriod( const geometry_msgs::Pose2D &base, hexapod_msgs::FeetPositions *feet);  //每条摆动腿和支撑腿一个周期内的步幅控制
  geometry_msgs::Pose2D smooth_base_;
  ros::Time current_time_, last_time_;
  int NUMBER_OF_LEGS;
  double LEG_LIFT_HEIGHT;
  bool is_travelling_ ;
  geometry_msgs::Pose2D base;
   
  int CYCLE_LENGTH;
  int MOVE_LENGTH;
  int STEP_LENGTH;
  double period_seg;
  
  std::vector<int> MOVE_GAIT_ORDER;
  std::vector<int> STEP_GAIT_ORDER_1;
  std::vector<int> STEP_GAIT_ORDER_2;
  std::vector<int> STEP_GAIT_ORDER_3;
  std::vector<int> STEP_GAIT_ORDER_4;
  std::vector<int> STEP_GAIT_ORDER_5;
  std::vector<int> STEP_GAIT_ORDER_6;
  std::vector< std::vector<int> > STEP_GAIT_ORDER;
  
  int leg_order_;
  
  bool start_cycle;
  bool stop_cycle;
  bool stop_active;

}; 

#endif





