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

#include <ros/ros.h>
#include <iostream>
#include "executor/chassis_executor.h"
#include "behavior_tree/behavior_tree.h"
#include "blackboard/blackboard.h"

#include "behavior_action/goal_factory.h"
#include "behavior_action/patrol_action.h"
#include "behavior_action/whirl_action.h"
#include "behavior_action/wait_action.h"
#include "behavior_action/turn_detected_action.h"
#include "behavior_action/turn_wounded_action.h"
#include "behavior_action/buff_action.h"
#include "behavior_action/chase_action.h"
#include "behavior_action/escape_action.h"
#include "behavior_action/search_action.h"
#include "behavior_action/shoot_action.h"
#include "behavior_action/supplier_action.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_node");
    /*
     *                                     game_status_selector
     *                                   /                      \
     *                    game_stop_condition               game_start_selector
     *                            |
     *                       wait_action
     * 
     *                                                    /                    \
     *                                 have_bullet_selector                   no_bullet_condition
     *                                 /                    \                           |
     *                     obt_buff_con          no_buff_but_bullet_sel             defense_sel
     *                          |                /                \                /  |    |    \    \
     *                    offensive_sel      emy_buff_con  search_buff_sel       emy  r b_con b_a_con P         
     *                  /   |   |   |  \         |         |  |  |     |    \     |   |     |    |      
     *                 a    d  dmp  r  SR   emy_buff_sel   a  d dmp b_a_con GB    E   AA   GB    W        
     *                 |    |   |   |       /  |   |   \   |  |  |     |                               
     *                TWA  CH   E   AA     a   d  dmp  r  TWA SH E     W                                
     *                                     |   |   |   |                        
     *                                    TWA  SH  E  AA                        
     * 
     */
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
    auto blackboard_ptr = std::make_shared<roborts_decision::Blackboard>(full_path);
    auto goal_factory_ptr = std::make_shared<roborts_decision::GoalFactory>(blackboard_ptr, full_path);

    auto chassis_executor = new roborts_decision::ChassisExecutor();
    auto chassis_executor_ptr = std::make_shared<roborts_decision::ChassisExecutor>();
    auto gimbal_executor_ptr = std::make_shared<roborts_decision::GimbalExecutor>();
    

    // action
    auto patrol_action_ptr = std::make_shared<roborts_decision::PatrolAction>(blackboard_ptr, goal_factory_ptr, chassis_executor_ptr);
    auto search_action_ptr = std::make_shared<roborts_decision::SearchAction>(blackboard_ptr, goal_factory_ptr, chassis_executor_ptr);
    auto whirl_action_ptr = std::make_shared<roborts_decision::WhirlAction>(blackboard_ptr, goal_factory_ptr, chassis_executor_ptr);
    auto shoot_action_ptr = std::make_shared<roborts_decision::ShootAction>(blackboard_ptr, goal_factory_ptr, chassis_executor_ptr);
    auto gain_buff_action_ptr = std::make_shared<roborts_decision::BuffAction>(blackboard_ptr, goal_factory_ptr, chassis_executor_ptr);
    auto chase_action_ptr = std::make_shared<roborts_decision::ChaseAction>(blackboard_ptr, goal_factory_ptr, chassis_executor_ptr);
    auto escape_action_ptr = std::make_shared<roborts_decision::EscapeAction>(blackboard_ptr, goal_factory_ptr, chassis_executor_ptr);
    //auto supplier_action_ptr = std::make_shared<roborts_decision::SupplierAction>(blackboard_ptr, goal_factory_ptr, chassis_executor_ptr);
    auto turn_to_detected_action_ptr = std::make_shared<roborts_decision::TurnDetectedAction>(blackboard_ptr, goal_factory_ptr, 
                                                                                              chassis_executor_ptr);
    auto turn_wounded_action_ptr = std::make_shared<roborts_decision::TurnWoundedAction>(blackboard_ptr, goal_factory_ptr, chassis_executor_ptr);
    auto wait_action_ptr = std::make_shared<roborts_decision::WaitAction>(blackboard_ptr, goal_factory_ptr, chassis_executor_ptr, 
                                                                          gimbal_executor_ptr);


   // 在机器人没有子弹时，当检测到敌人或者收到攻击时，执行Escape action
    auto no_bullet_find_enemy_condition_ = std::make_shared<roborts_decision::PreconditionNode>("no bullet find enemy condition",
                                                                                        blackboard_ptr,
                                                                                        [&]() {
                                                                                            if (blackboard_ptr -> GetArmorAttacked()
                                                                                               != roborts_decision::ArmorAttacked::NONE
                                                                                               || blackboard_ptr -> IsEnemyDetected()) {
                                                                                                    return true;
                                                                                            } else {
                                                                                                return false;
                                                                                            }
                                                                                        }, 
                                                                                        roborts_decision::AbortType::BOTH);
    no_bullet_find_enemy_condition_ -> SetChild(escape_action_ptr);


    // 在机器人没有子弹时，可以补给时，执行Supply action
    // auto no_bullet_supply_condition_ = std::make_shared<roborts_decision::PreconditionNode>("no bullet supply condition",
    //                                                                                     blackboard_ptr,
    //                                                                                     [&]() {
    //                                                                                         if (blackboard_ptr -> ) {
    //                                                                                                 return true;
    //                                                                                         } else {
    //                                                                                             return false;
    //                                                                                         }
    //                                                                                     }, 
    //                                                                                     roborts_decision::AbortType::LOW_PRIORITY);
    // no_bullet_supply_condition_ -> AddChildren(supplier_action_ptr);
    

    // 在机器人没有子弹时，可以获取Buff时，执行GainBuff action
    auto no_bullet_buff_condition_ = std::make_shared<roborts_decision::PreconditionNode>("no bullet buff condition",
                                                                                        blackboard_ptr,
                                                                                        [&]() {
                                                                                            if (blackboard_ptr -> IsBonusAviliable()) {
                                                                                                    return true;
                                                                                            } else {
                                                                                                return false;
                                                                                            }
                                                                                        }, 
                                                                                        roborts_decision::AbortType::LOW_PRIORITY);
    no_bullet_buff_condition_ -> SetChild(gain_buff_action_ptr);
    

    // 机器人没有子弹，并且正在Buff区获取buff时，如果受到攻击，执行Whirl action
    auto no_bullet_attack_in_buff_condition_ = std::make_shared<roborts_decision::PreconditionNode>("no bullet buff attack condition",
                                                                                        blackboard_ptr,
                                                                                        [&]() {
                                                                                            if (blackboard_ptr -> GetArmorAttacked()
                                                                                                != roborts_decision::ArmorAttacked::NONE
                                                                                                && blackboard_ptr -> GetSelfBonusStatus()
                                                                                                == roborts_decision::BonusStatus::BEING_OCCUPIED) {
                                                                                                    return true;
                                                                                            } else {
                                                                                                return false;
                                                                                            }
                                                                                        }, 
                                                                                        roborts_decision::AbortType::LOW_PRIORITY);
    no_bullet_attack_in_buff_condition_ -> SetChild(whirl_action_ptr);
    

    // defense selector
    auto defense_selector_ = std::make_shared<roborts_decision::SelectorNode>("defense selector", blackboard_ptr);
    defense_selector_ -> AddChildren(no_bullet_find_enemy_condition_);
    // defense_selector_ -> AddChildren(no_bullet_supply_condition_);
    defense_selector_ -> AddChildren(no_bullet_buff_condition_);
    defense_selector_ -> AddChildren(no_bullet_attack_in_buff_condition_);
    defense_selector_ -> AddChildren(patrol_action_ptr);
    

    // 当机器人在获取Buff的路上受到攻击时，执行 Turn Wounded Armor action
    auto search_buff_attack_condition_ = std::make_shared<roborts_decision::PreconditionNode>("search buff attack condition",
                                                                                        blackboard_ptr,
                                                                                        [&]() {
                                                                                            if (blackboard_ptr -> GetArmorAttacked()
                                                                                                != roborts_decision::ArmorAttacked::NONE) {
                                                                                                    return true;
                                                                                            } else {
                                                                                                return false;
                                                                                            }
                                                                                        }, roborts_decision::AbortType::BOTH);
    search_buff_attack_condition_ -> SetChild(turn_wounded_action_ptr);
    // search_buff_attack_condition_ -> SetChild(whirl_action_ptr);


    // 当机器人在获取Buff的路上检测到敌人时，执行 Shoot action
    auto search_buff_detect_condition_ = std::make_shared<roborts_decision::PreconditionNode>("search buff detect condition",
                                                                                        blackboard_ptr,
                                                                                        [&]() {
                                                                                            if (blackboard_ptr -> IsEnemyDetected()) {
                                                                                                    return true;
                                                                                            } else {
                                                                                                return false;
                                                                                            }
                                                                                        }, 
                                                                                        roborts_decision::AbortType::LOW_PRIORITY);
    search_buff_detect_condition_ -> SetChild(shoot_action_ptr);


    // 当机器人在获取Buff的路上，每秒受到的伤害 > 400，执行 Escape action
    auto search_buff_dmp_condition_ = std::make_shared<roborts_decision::PreconditionNode>("search buff dmp condition",
                                                                                        blackboard_ptr,
                                                                                        [&]() {
                                                                                            if (blackboard_ptr -> HurtPerSecond() > 400) {
                                                                                                    return true;
                                                                                            } else {
                                                                                                return false;
                                                                                            }
                                                                                        }, 
                                                                                        roborts_decision::AbortType::LOW_PRIORITY);
    search_buff_dmp_condition_ -> SetChild(escape_action_ptr);


    // 当机器人在Buff区时，如果受到攻击，执行 Whirl action
    auto search_buff_attack_in_buff_condition_ = std::make_shared<roborts_decision::PreconditionNode>("search buff attack in buff condition",
                                                                                                blackboard_ptr,
                                                                                                [&]() {
                                                                                                    if (blackboard_ptr -> GetArmorAttacked()
                                                                                                        != roborts_decision::ArmorAttacked::NONE
                                                                                                        && blackboard_ptr -> GetSelfBonusStatus()
                                                                                                        == roborts_decision::BonusStatus::BEING_OCCUPIED) {
                                                                                                            return true;
                                                                                                    } else {
                                                                                                            return false;
                                                                                                    }
                                                                                                }, 
                                                                                                roborts_decision::AbortType::LOW_PRIORITY);
    search_buff_attack_in_buff_condition_ -> SetChild(whirl_action_ptr);


    // search buff selector
    auto search_buff_selector_ = std::make_shared<roborts_decision::SelectorNode>("search buff selector", blackboard_ptr);
    search_buff_selector_ -> AddChildren(search_buff_attack_condition_);
    // search_buff_selector_ -> AddChildren(patrol_action_ptr);
    // search_buff_selector_ -> AddChildren(search_buff_detect_condition_);
    // search_buff_selector_ -> AddChildren(search_buff_dmp_condition_);
    // search_buff_selector_ -> AddChildren(search_buff_attack_in_buff_condition_);
    // search_buff_selector_ -> AddChildren(gain_buff_action_ptr);


    // enemy obtain buff selector
    auto emy_buff_attack_condition_ = std::make_shared<roborts_decision::PreconditionNode>("enemy buff attack condition",
                                                                                           blackboard_ptr,
                                                                                           [&]() {
                                                                                               if (blackboard_ptr -> GetArmorAttacked()
                                                                                                != roborts_decision::ArmorAttacked::NONE
                                                                                                && blackboard_ptr -> GetArmorAttacked()
                                                                                                != roborts_decision::ArmorAttacked::FRONT) {
                                                                                                   return true;
                                                                                               } else {
                                                                                                   return false;
                                                                                               }
                                                                                           }, 
                                                                                           roborts_decision::AbortType::BOTH);
    emy_buff_attack_condition_ -> SetChild(turn_wounded_action_ptr);
    auto emy_buff_detect_condition_ = std::make_shared<roborts_decision::PreconditionNode>("enemy buff detect condition",
                                                                                           blackboard_ptr,
                                                                                           [&]() {
                                                                                               if (blackboard_ptr -> IsEnemyDetected()) {
                                                                                                   return true;
                                                                                                } else {
                                                                                                    return false;
                                                                                                }
                                                                                           }, 
                                                                                           roborts_decision::AbortType::LOW_PRIORITY);
    emy_buff_detect_condition_ -> SetChild(shoot_action_ptr);
    auto emy_buff_dmp_condition_ = std::make_shared<roborts_decision::PreconditionNode>("enemy buff dmp condition",
                                                                                        blackboard_ptr,
                                                                                        [&]() {
                                                                                            if (blackboard_ptr -> HurtPerSecond() > 400) {
                                                                                                return true;
                                                                                            } else {
                                                                                                return false;
                                                                                            }
                                                                                        }, 
                                                                                        roborts_decision::AbortType::LOW_PRIORITY);
    emy_buff_dmp_condition_ -> SetChild(escape_action_ptr);
    // auto emy_buff_supply_condition_ = std::make_shared<roborts_decision::PreconditionNode>("enemy buff supply condition",
    //                                                                                        blackboard_ptr,
    //                                                                                        [&](blackboard_ptr -> ) {
    //                                                                                        if () {
    //                                                                                            return true;
    //                                                                                        } else {
    //                                                                                            return false;
    //                                                                                        }
    //                                                                                        }, roborts_decision::AbortType::LOW_PRIORITY);
    // emy_buff_supply_condition_ -> AddChildren(supplier_action_ptr);
    
    auto enemy_obtain_buff_selector = std::make_shared<roborts_decision::SelectorNode>("enemy obtain buff selector", blackboard_ptr);
    enemy_obtain_buff_selector -> AddChildren(emy_buff_attack_condition_);
    enemy_obtain_buff_selector -> AddChildren(emy_buff_detect_condition_);
    enemy_obtain_buff_selector -> AddChildren(emy_buff_dmp_condition_);
    // enemy_obtain_buff_selector -> AddChildren(emy_buff_supply_condition_);


    // no buff, but have bullet selector
    auto emy_obtain_buff_condition_ = std::make_shared<roborts_decision::PreconditionNode>("enemy obtain buff condition",
                                                                                           blackboard_ptr,
                                                                                           [&]() {
                                                                                               if (blackboard_ptr -> GetEnemyBonusStatus()
                                                                                                   == roborts_decision::BonusStatus::OCCUPIED) {
                                                                                                   return true;
                                                                                               } else {
                                                                                                   return false;
                                                                                               }
                                                                                           }, 
                                                                                           roborts_decision::AbortType::BOTH);
    emy_obtain_buff_condition_ -> SetChild(enemy_obtain_buff_selector);
    auto no_buff_but_bullet_selector = std::make_shared<roborts_decision::SelectorNode>("no buff, but bullet selector", blackboard_ptr);
    no_buff_but_bullet_selector -> AddChildren(emy_obtain_buff_condition_);
    no_buff_but_bullet_selector -> AddChildren(search_buff_selector_);


    // offensive selector
    auto offensive_attack_condition_ = std::make_shared<roborts_decision::PreconditionNode>("offensive attack condition",
                                                                                           blackboard_ptr,
                                                                                           [&]() {
                                                                                               if (blackboard_ptr -> GetArmorAttacked()
                                                                                                != roborts_decision::ArmorAttacked::NONE
                                                                                                && blackboard_ptr -> GetArmorAttacked()
                                                                                                != roborts_decision::ArmorAttacked::FRONT) {
                                                                                                   return true;
                                                                                               } else {
                                                                                                   return false;
                                                                                               }
                                                                                           }, 
                                                                                           roborts_decision::AbortType::BOTH);
    offensive_attack_condition_ -> SetChild(turn_wounded_action_ptr);
    auto offensive_detect_condition_ = std::make_shared<roborts_decision::PreconditionNode>("offensive detect condition",
                                                                                            blackboard_ptr,
                                                                                            [&]() {
                                                                                                if (blackboard_ptr -> IsEnemyDetected()) {
                                                                                                    return true;
                                                                                                } else {
                                                                                                    return false;
                                                                                                }
                                                                                           }, 
                                                                                           roborts_decision::AbortType::LOW_PRIORITY);
    offensive_detect_condition_ -> SetChild(chase_action_ptr);
    auto offensive_dmp_condition_ = std::make_shared<roborts_decision::PreconditionNode>("offensive dmp condition",
                                                                                         blackboard_ptr,
                                                                                         [&]() {
                                                                                             if (blackboard_ptr -> HurtPerSecond() > 400) {
                                                                                                 return true;
                                                                                             } else {
                                                                                                 return false;
                                                                                             }
                                                                                          }, 
                                                                                          roborts_decision::AbortType::LOW_PRIORITY);
    offensive_dmp_condition_ -> SetChild(escape_action_ptr);
    // auto offensive_supply_condition_ = std::make_shared<roborts_decision::PreconditionNode>("offensive supply condition",
    //                                                                                      blackboard_ptr,
    //                                                                                      [&](blackboard_ptr -> ) {
    //                                                                                       if () {
    //                                                                                           return true;
    //                                                                                       } else {
    //                                                                                           return false;
    //                                                                                       }
    //                                                                                       }, roborts_decision::AbortType::LOW_PRIORITY);
    // offensive_supply_condition_ -> AddChildren(supplier_action_ptr);


    auto offensive_selector_ = std::make_shared<roborts_decision::SelectorNode>("offensive selector", blackboard_ptr);
    offensive_selector_ -> AddChildren(offensive_attack_condition_);
    offensive_selector_ -> AddChildren(offensive_detect_condition_);
    offensive_selector_ -> AddChildren(offensive_dmp_condition_);
    // offensive_selector_ -> AddChildren(offensive_supply_condition_):
    offensive_selector_ -> AddChildren(search_action_ptr);


    // have bullet selector
    auto obtain_buff_condition_ = std::make_shared<roborts_decision::PreconditionNode>("obtain buff condition",
                                                                                       blackboard_ptr,
                                                                                       [&]() {
                                                                                           if (blackboard_ptr -> GetSelfBonusStatus()
                                                                                               == roborts_decision::BonusStatus::OCCUPIED) {
                                                                                               return true;
                                                                                           } else {
                                                                                               return false;
                                                                                           }
                                                                                        }, 
                                                                                        roborts_decision::AbortType::BOTH);
    obtain_buff_condition_ -> SetChild(offensive_selector_);
    auto have_bullet_selector_ = std::make_shared<roborts_decision::SelectorNode>("have bullect selector", blackboard_ptr);
    have_bullet_selector_ -> AddChildren(obtain_buff_condition_);
    have_bullet_selector_ -> AddChildren(no_buff_but_bullet_selector);


    // game_start_selector
    // auto no_bullet_condition_ = std::make_shared<roborts_decision::PreconditionNode>("no bullet condition",
    //                                                                                  blackboard_ptr,
    //                                                                                  [&]() {
    //                                                                                      if (blackboard_ptr -> ) {
    //                                                                                          return true;
    //                                                                                      } else {
    //                                                                                          return false;
    //                                                                                     }
    //                                                                                  }, 
    //                                                                                  roborts_decision::AbortType::BOTH);
    // no_bullet_condition_ -> SetChild(defense_selector_);
    auto game_start_selector_ = std::make_shared<roborts_decision::SelectorNode>("game start selector", blackboard_ptr);
    // game_start_selector_ -> AddChildren(no_bullet_condition_);
    game_start_selector_ -> AddChildren(have_bullet_selector_);


    // game_stop_condition
    auto game_stop_condition_ = std::make_shared<roborts_decision::PreconditionNode>("game stop condition", blackboard_ptr,
                                                                                     [&]() {
                                                                                         if (blackboard_ptr -> GetGameStatus()
                                                                                             != roborts_decision::GameStatus::ROUND) {
                                                                                                 return true;
                                                                                             } else {
                                                                                                 return false;
                                                                                             }
                                                                                     },
                                                                                     roborts_decision::AbortType::BOTH);
    game_stop_condition_ -> SetChild(wait_action_ptr);


    // game_status_selector
    auto game_status_selector_ = std::make_shared<roborts_decision::SelectorNode>("game status selector", blackboard_ptr);
    game_status_selector_ -> AddChildren(game_stop_condition_);
    game_status_selector_ -> AddChildren(game_start_selector_);                                                                       
    
    // auto behavior_tree_ptr = std::make_shared<roborts_decision::BehaviorTree>(patrol_action_ptr, 25);
    auto behavior_tree_ptr = std::make_shared<roborts_decision::BehaviorTree>(search_buff_selector_, 25);
    behavior_tree_ptr->Run();
    return 0;
}
