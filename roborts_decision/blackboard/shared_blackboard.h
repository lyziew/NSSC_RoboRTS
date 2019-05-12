/****************************************************************************
 *  Copyright (C) 2019 NSSC ZIEW.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_DECISION_SHARED_BLACKBOARD_H
#define ROBORTS_DECISION_SHARED_BLACKBOARD_H

#include <ros/ros.h>
#include <vector>
#include "blackboard.h"
#include "roborts_msgs/ShootInfo.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32MultiArray.h>

namespace roborts_decision
{

class SharedBlackboard
{
public:
  typedef std::shared_ptr<SharedBlackboard> Ptr;
  explicit SharedBlackboard(const Blackboard::Ptr &blackboard_ptr) : blackboard_ptr_(blackboard_ptr)
  {
      shoot_info_pub_ = shared_nh_.advertise<roborts_msgs::ShootInfo>("robot_shoot_info", 30);
      shoot_info_sub_ = shared_nh_.subscribe<roborts_msgs::ShootInfo>("robot_shoot_info", 30, &SharedBlackboard::RobotBulletCallback, this);
      enemy_pose_pub_ = shared_nh_.advertise<geometry_msgs::PoseStamped>("enemy_pose_info", 30);
      enemy_pose_sub_ = shared_nh_.subscribe<geometry_msgs::PoseStamped>("enemy_pose_info", 30, &SharedBlackboard::EnemyPoseCallback, this);
      wounded_info_pub_ = shared_nh_.advertise<std_msgs::Int32MultiArray>("wounded_armor_info", 30);
      wounded_info_sub_ = shared_nh_.subscribe<std_msgs::Int32MultiArray>("wounded_armor_info", 30, &SharedBlackboard::WoundedArmorCallback, this);
  }

  ~SharedBlackboard() = default;

  void RobotBulletCallback(const roborts_msgs::ShootInfoConstPtr &shoot_info)
  {
    robot02_remain_bullet_ = shoot_info -> remain_bullet;
  }

  void EnemyPoseCallback(const geometry_msgs::PoseStampedConstPtr &enemyPos_info)
  {
    robot02_enemyPos_ = enemyPos_info -> pose;
  }

  void WoundedArmorCallback(const std_msgs::Int32MultiArrayConstPtr &wounded_info)
  {
    robot02_wounded_armor_vector_ = wounded_info -> data;
  }

  std::vector<unsigned int> GetRobotBullet()
  {
    remain_bullet_[0] = blackboard_ptr_ -> GetRemainBullet();
    remain_bullet_[1] = robot02_remain_bullet_;
  }

  geometry_msgs::Pose GetRobot02Enemy() const
  {
    return robot02_enemyPos_;
  }

  std::vector<int> GetRobot02WoundedArmorVector()
  {
    return robot02_wounded_armor_vector_;
  }

  void Run()
  {
    unsigned int count = 0;
    ros::Rate loop_rate(10);
    while (true) 
    {
      roborts_msgs::ShootInfo shoot_info;
      shoot_info.header.frame_id = "/bullet";
      shoot_info.header.stamp = ros::Time::now();
      shoot_info.remain_bullet = blackboard_ptr_ -> GetRemainBullet();
      shoot_info_pub_.publish(shoot_info);

      geometry_msgs::PoseStamped enemy_pose;
      enemy_pose = blackboard_ptr_ -> GetEnemy();
      enemy_pose_pub_.publish(enemy_pose);

      std_msgs::Int32MultiArray wounded_info;
      wounded_info.data = blackboard_ptr_ -> GetWoundedArmorVector();
      wounded_info_pub_.publish(wounded_info);
      ROS_INFO("%s", __FUNCTION__);

      loop_rate.sleep();
      count++;
    }
  }

protected:
  Blackboard::Ptr blackboard_ptr_;
  ros::NodeHandle shared_nh_;
  ros::Publisher shoot_info_pub_;
  ros::Subscriber shoot_info_sub_;

  ros::Publisher enemy_pose_pub_;
  ros::Subscriber enemy_pose_sub_;

  ros::Publisher wounded_info_pub_;
  ros::Subscriber wounded_info_sub_;

  std::vector<unsigned int> remain_bullet_ = {0, 0};
  unsigned int robot02_remain_bullet_;
  geometry_msgs::Pose robot02_enemyPos_;
  std::vector<int> robot02_wounded_armor_vector_;

}; // class SharedBlackboard

} // namespace roborts_decision

#endif // ROBORTS_DECISION_SHARED_BLACKBOARD_H
