/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
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
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

//Chassis
#include "roborts_msgs/TwistAccel.h"

//Gimbal
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalRate.h"
#include "roborts_msgs/GimbalMode.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"

//Referee System
#include "roborts_msgs/BonusStatus.h"
#include "roborts_msgs/GameResult.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/GameSurvivor.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/RobotBonus.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/SupplierStatus.h"
#include "roborts_msgs/ArmorDetectionAction.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

#include ""

namespace roborts_decision
{

class Blackboard
{
public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path) : enemy_detected_(false),
                                                            armor_detection_actionlib_client_("armor_detection_node_action", true)
  {

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") +
                           "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    // Enemy fake pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

    if (!decision_config.simulate())
    {

      armor_detection_actionlib_client_.waitForServer();

      ROS_INFO("Armor detection module has been connected!");

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));

      game_status_sub_ = referee_nh_.subscribe<roborts_msgs::GameStatus>("game_status", 30, &Blackboard::GameStatusCallback, this);
      game_survival_sub_ = referee_nh_.subscribe<roborts_msgs::GameSurvivor>("game_survivor", 30, &Blackboard::GameSurvivorCallback, this);
      bonus_status_sub_ = referee_nh_.subscribe<roborts_msgs::BonusStatus>("field_bonus_status", 30, &Blackboard::BonusStatusCallback, this);
      supplier_status_sub_ = referee_nh_.subscribe<roborts_msgs::SupplierStatus>("field_supplier_status", 30, &Blackboard::SupplierStatusCallback, this);
      robot_status_sub_ = referee_nh_.subscribe<roborts_msgs::RobotStatus>("robot_status", 30, &Blackboard::RobotStatusCallback, this);
      robot_heat_sub_ = referee_nh_.subscribe<roborts_msgs::RobotHeat>("robot_heat", 30, &Blackboard::RobotHeatCallback, this);
      robot_bonus_sub_ = referee_nh_.subscribe<roborts_msgs::RobotBonus>("robot_bonus", 30, &Blackboard::RobotBonusCallback, this);
      robot_damage_sub_ = referee_nh_.subscribe<roborts_msgs::RobotDamage>("robot_damage", 30, &Blackboard::RobotDamageCallback, this);
      robot_shoot_sub_ = referee_nh_.subscribe<roborts_msgs::RobotShoot>("robot_shoot", 30, &Blackboard::RobotShootCallback, this);
    }
  }

  ~Blackboard() = default;

  // 反馈比赛状态数据
  void GameStatusCallback(const roborts_msgs::GameStatusConstPtr &game_status_msg)
  {

  }

  // 反馈场上双方存活机器人状态数据
  void GameSurvivorCallback(const roborts_msgs::GameSurvivorConstPtr & game_survivor_msg)
  {

  }

  // 反馈buff区状态数据, 获取buff前应该先判断buff状态
  void BonusStatusCallback(const roborts_msgs::BonusStatusConstPtr & bonus_status_msg)
  {

  }

  // 反馈补给站状态, 补给前应该先判断补给站状态
  void SupplierStatusCallback(const roborts_msgs::SupplierStatusConstPtr &supplier_status_msg)
  {

  }

  // 反馈机器人状态
  void RobotStatusCallback(const roborts_msgs::RobotStatusConstPtr &robot_status_msg)
  {

  }

  //反馈射击热量信息
  void RobotHeatCallback(const roborts_msgs::RobotHeatConstPtr &robot_heat_msg)
  {

  }

  // 反馈获取buff信息
  void RobotBonusCallback(const roborts_msgs::RobotBonusConstPtr &robot_bonus_msg)
  {

  }

  // 反馈机器人遭受伤害信息
  void RobotDamageCallback(const roborts_msgs::RobotDamageConstPtr &robot_damage_msg)
  {

  }

  //  反馈机器人射击信息
  void RobotShootCallback(const roborts_msgs::RobotShootConstPtr &robot_shoot_msg)
  {

  }

  // Enemy
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr &feedback)
  {
    if (feedback->detected)
    {
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;

      double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
                                  camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
      double yaw = atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);

      //camera_pose_msg.pose.position.z=camera_pose_msg.pose.position.z;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                              0,
                                                              yaw);
      camera_pose_msg.pose.orientation.w = quaternion.w();
      camera_pose_msg.pose.orientation.x = quaternion.x();
      camera_pose_msg.pose.orientation.y = quaternion.y();
      camera_pose_msg.pose.orientation.z = quaternion.z();
      poseStampedMsgToTF(camera_pose_msg, tf_pose);

      tf_pose.stamp_ = ros::Time(0);
      try
      {
        tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

        if (GetDistance(global_pose_msg, enemy_pose_) > 0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2)
        {
          enemy_pose_ = global_pose_msg;
        }
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("tf error when transform enemy pose from camera to map");
      }
    }
    else
    {
      enemy_detected_ = false;
    }
  }

  geometry_msgs::PoseStamped GetEnemy() const
  {
    return enemy_pose_;
  }

  bool IsEnemyDetected() const
  {
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal)
  {
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const
  {
    return goal_;
  }

  bool IsNewGoal()
  {
    if (new_goal_)
    {
      new_goal_ = false;
      return true;
    }
    else
    {
      return false;
    }
  }
  /*---------------------------------- Tools ------------------------------------------*/

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2)
  {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2)
  {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose()
  {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap()
  {
    return costmap_ptr_;
  }

  const CostMap2D *GetCostMap2D()
  {
    return costmap_2d_;
  }

  const unsigned char *GetCharMap()
  {
    return charmap_;
  }

private:
  void UpdateRobotPose()
  {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try
    {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex)
    {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D *costmap_2d_;
  unsigned char *charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;

  //! referee handle
  ros::NodeHandle referee_nh_;
  //! referee system subscriber
  ros::Subscriber game_status_sub_;
  ros::Subscriber game_survival_sub_;
  ros::Subscriber robot_hurt_data_sub_;
  ros::Subscriber bonus_status_sub_;
  ros::Subscriber supplier_status_sub_;
  ros::Subscriber robot_status_sub_;
  ros::Subscriber robot_heat_sub_;
  ros::Subscriber robot_bonus_sub_;
  ros::Subscriber robot_damage_sub_;
  ros::Subscriber robot_shoot_sub_;
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
