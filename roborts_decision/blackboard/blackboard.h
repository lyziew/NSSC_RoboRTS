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
#include "../log.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

namespace roborts_decision
{

enum class GameStatus
{
  PRE_MATCH = 0,
  SETUP = 1,
  INIT = 2,
  FIVE_SEC_CD = 3,
  ROUND = 4,
  CALCULATION = 5
};

enum class BonusStatus
{
  UNOCCUPIED = 0,
  BEING_OCCUPIED= 1,
  OCCUPIED = 2
};

enum class SupplierStatus
{
  CLOSE = 0,
  PREPARING = 1,
  SUPPLYING = 2
};

enum class DamageType
{
  ARMOR = 0,
  OFFLINE = 1,
  EXCEED_HEAT = 2,
  EXCEED_POWER = 3
};
  
enum class ArmorAttacked
{
  NONE = 0,
  FRONT = 1,
  LEFT = 2,
  BACK = 4,
  RIGHT = 8
};

enum class RobotDetected
{
  NONE = 0,
  FRONT = 1,
  LEFT = 2,
  BACK = 4,
  RIGHT = 8
};

class Blackboard
{
public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path) : game_status_(GameStatus::PRE_MATCH),
                                                            remain_hp_(2000),
                                                            last_hp_(2000),
                                                            dmp_(0),
                                                            armor_attacked_(ArmorAttacked::NONE),
                                                            robot_bonus_aviliable_(false),
                                                            enemy_detected_(false),
                                                            armor_detection_actionlib_client_("armor_detection_node_action", true)
  {
    last_get_hp_time_ = ros::Time::now();

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") +
                           "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

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
    game_status_ = static_cast<GameStatus>(game_status_msg -> game_status);
    remaining_time_ = static_cast<unsigned int>(game_status_msg -> remaining_time);
  }

  // 反馈场上双方存活机器人状态数据
  void GameSurvivorCallback(const roborts_msgs::GameSurvivorConstPtr &game_survivor_msg)
  {
    red3_ = (bool)(game_survivor_msg -> red3);
    red4_ = (bool)(game_survivor_msg -> red4);
    blue3_ = (bool)(game_survivor_msg -> blue3);
    blue4_ = (bool)(game_survivor_msg -> blue4);
    robot_survivor_[0] = red3_;
    robot_survivor_[1] = red4_;
    robot_survivor_[2] = blue3_;
    robot_survivor_[3] = blue4_;
  }

  // 反馈buff区状态数据, 获取buff前应该先判断buff状态
  void BonusStatusCallback(const roborts_msgs::BonusStatusConstPtr &bonus_status_msg)
  {
    red_bonus_ = static_cast<BonusStatus>(bonus_status_msg -> red_bonus);
    blue_bonus_ = static_cast<BonusStatus>(bonus_status_msg -> blue_bonus);
  }

  // 反馈补给站状态, 补给前应该先判断补给站状态
  void SupplierStatusCallback(const roborts_msgs::SupplierStatusConstPtr &supplier_status_msg)
  {
    supplier_status_ = static_cast<SupplierStatus>(supplier_status_msg -> status);
  }

  // 反馈机器人状态
  void RobotStatusCallback(const roborts_msgs::RobotStatusConstPtr &robot_status_msg)
  {
    robot_id_ = static_cast<unsigned char>(robot_status_msg -> id);

    switch (robot_id_)
    {
      case 3:
        robot_color_ = 'r';
        initial_has_bullet_ = true;
        break;
      case 4:
        robot_color_ = 'r';
        break;
      case 13:
        robot_color_ = 'b';
        initial_has_bullet_ = true;
        break;
      case 14:
        robot_color_ = 'b';
        break;
    }
    remain_hp_ = static_cast<unsigned int>(robot_status_msg -> remain_hp);
    max_hp_ = static_cast<unsigned int>(robot_status_msg -> max_hp);
    heat_cooling_limit_ = static_cast<unsigned int>(robot_status_msg -> heat_cooling_limit);
    heat_cooling_rate_ = static_cast<unsigned int>(robot_status_msg -> heat_cooling_rate);
  }

  //反馈射击热量信息
  void RobotHeatCallback(const roborts_msgs::RobotHeatConstPtr &robot_heat_msg)
  {
    shooter_heat_ = static_cast<unsigned int>(robot_heat_msg -> shooter_heat);
  }

  // 反馈获取buff信息
  void RobotBonusCallback(const roborts_msgs::RobotBonusConstPtr &robot_bonus_msg)
  {
    robot_bonus_aviliable_ = static_cast<bool>(robot_bonus_msg -> bonus);
  }

  // 反馈机器人遭受伤害信息
  void RobotDamageCallback(const roborts_msgs::RobotDamageConstPtr &robot_damage_msg)
  {
    robot_damage_type_ = static_cast<DamageType>(robot_damage_msg -> damage_type);
    switch (robot_damage_msg -> damage_source)
    {
    case 0:
      armor_attacked_ = ArmorAttacked::FRONT;
      break;
    case 1:
      armor_attacked_ = ArmorAttacked::BACK;
      break;
    case 2:
      armor_attacked_ = ArmorAttacked::LEFT;
      break;
    case 3:
      armor_attacked_ = ArmorAttacked::RIGHT;
      break;
    default:
      armor_attacked_ = ArmorAttacked::NONE;
      break;
    }
  }

  //  反馈机器人射击信息
  void RobotShootCallback(const roborts_msgs::RobotShootConstPtr &robot_shoot_msg)
  {
    shoot_frequency_ = static_cast<unsigned char>(robot_shoot_msg -> frequency);
    shoot_speed_ = static_cast<double>(robot_shoot_msg -> speed);
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
  
    GameStatus GetGameStatus() const
  {
    ROS_INFO("%s: %d", __FUNCTION__, (int)game_status_);
    return game_status_;
  }

  bool* getRobotSurvivor()
  {
    return robot_survivor_;
  }
  
  // SetSelfBonusStatus修改了变量，不能设置为const
  void SetSelfBonusStatus(unsigned char robot_color_) //const
  {
    if (robot_color_ == 'r')
    {
      self_bonus_status_ = red_bonus_;
    }
    else
    {
      self_bonus_status_ = blue_bonus_;
    }
  }

  BonusStatus GetSelfBonusStatus() const
  {
    return self_bonus_status_;
  }

  BonusStatus SetEnemyBonusStatus(unsigned char robot_color_)
  {
    if (robot_color_ == 'r')
    {
      enemy_bonus_status_ = blue_bonus_;
    }
    else
    {
      enemy_bonus_status_ = red_bonus_;
    }
  }

  BonusStatus GetEnemyBonusStatus() const
  {
    return enemy_bonus_status_;
  }

  SupplierStatus GetSupplierStatus() const
  {
    ROS_INFO("%s: %d", __FUNCTION__, (int)supplier_status_);
    return supplier_status_;
  }

  unsigned int GetHP() const
  {
    return remain_hp_;
  }

  unsigned int GetRobotHeat() const
  {
    return shooter_heat_;
  }
  
  double HurtPerSecond()
  {
    if ((ros::Time::now() - last_get_hp_time_) > ros::Duration(0.5)) {
      auto hp_diff_ = last_hp_ - remain_hp_;
      dmp_ = hp_diff_ / (ros::Time::now() - last_get_hp_time_).toSec();
      last_get_hp_time_ = ros::Time::now();
      last_hp_ = remain_hp_;
      return dmp_;
    } else {
      return dmp_;
    }
  }

  bool IsBonusAviliable() {
    return robot_bonus_aviliable_;
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

  ArmorAttacked GetArmorAttacked() const
  {
    if (ros::Time::now() - last_armor_attacked__time_ > ros::Duration(0.1))
    {
      ROS_INFO("%s: %d", __FUNCTION__, (int)ArmorAttacked::NONE);
      return ArmorAttacked::NONE;
    }
    else
    {
      ROS_INFO("%s: %d", __FUNCTION__, (int)armor_attacked_);
      return armor_attacked_;
    }
  }
  
  RobotDetected GetRobotDetected() const
  {
    if (ros::Time::now() - last_robot_detected_time_ > ros::Duration(0.2))
    {
      ROS_INFO("%s: %d", __FUNCTION__, (int)RobotDetected::NONE);
      return RobotDetected::NONE;
    }
    else
    {
      ROS_INFO("%s: %d", __FUNCTION__, (int)robot_detected_);
      return robot_detected_;
    }
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
  
  //! robot self info
  ros::Publisher robot_pose_pub_;
  ros::Publisher enemy_pose_pub_;
  ros::Publisher robot_status_pub_;

  // 保存最近10次受伤的信息,用于判断受伤的装甲板
  std::queue<roborts_msgs::RobotDamageConstPtr> robot_wounded_msg_queue_;
  // 用于返回攻击的装甲板的的位置
  ArmorAttacked armor_attacked_;
  ros::Time last_armor_attacked__time_;

  //保存最近10次发现敌人的信息,用于判断敌人最可能的位置
  // std::queue<roborts_msgs::> robot_detected_msg_queue_;
  RobotDetected robot_detected_;
  ros::Time last_robot_detected_time_;

  //! Referee system info
  GameStatus game_status_;
  unsigned int remaining_time_;

  bool red3_;
  bool red4_;
  bool blue3_;
  bool blue4_;
  bool robot_survivor_[4] = {0, 0, 0, 0};
  
  BonusStatus red_bonus_;
  BonusStatus blue_bonus_;
  BonusStatus self_bonus_status_;
  BonusStatus enemy_bonus_status_;
  
  SupplierStatus supplier_status_;

  unsigned char robot_id_;
  unsigned char robot_color_;
  bool initial_has_bullet_;
  unsigned int remain_hp_;
  unsigned int max_hp_;
  unsigned int heat_cooling_limit_;
  unsigned int heat_cooling_rate_;
  double dmp_;
  unsigned int last_hp_;
  ros::Time last_get_hp_time_;

  unsigned int shooter_heat_;

  // BounsAviliable
  bool robot_bonus_aviliable_;
  
  DamageType robot_damage_type_;
  //ArmorAttacked robot_damage_source_;

  unsigned char shoot_frequency_;
  double shoot_speed_;
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
