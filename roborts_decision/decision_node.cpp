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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_node");
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
    auto chassis_executor = new roborts_decision::ChassisExecutor();
    auto chassis_executor_ptr = std::make_shared<roborts_decision::ChassisExecutor>();
    auto blackboard_ptr = std::make_shared<roborts_decision::Blackboard>(full_path);
    auto goal_factory_ptr = std::make_shared<roborts_decision::GoalFactory>(blackboard_ptr, full_path);
    auto patrol_action_ptr = std::make_shared<roborts_decision::PatrolAction>(blackboard_ptr, goal_factory_ptr, chassis_executor_ptr);
    auto whirl_action_ptr = std::make_shared<roborts_decision::WhirlAction>(blackboard_ptr, goal_factory_ptr, chassis_executor_ptr);
    auto behavior_tree_ptr = std::make_shared<roborts_decision::BehaviorTree>(whirl_action_ptr, 25);
    behavior_tree_ptr->Run();
    return 0;
}
