/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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

#include "modules/decision/behavior_tree/behavior_tree.h"
#include "modules/decision/ICRA_decision/action_behavior.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "decision_node");
  auto blackboard_ptr_ = std::make_shared<rrts::decision::Blackboard>("/modules/decision/ICRA_decision/config/decision.prototxt");

  auto goal_factory_ = std::make_shared<rrts::decision::GoalFactory>(blackboard_ptr_,
                                                                     "/modules/decision/ICRA_decision/config/decision.prototxt");


  auto patrol_behavior_ptr = std::make_shared<rrts::decision::PatrolAction>(blackboard_ptr_, goal_factory_);
  rrts::decision::BehaviorTree bt(patrol_behavior_ptr, 25);
  LOG_INFO << "Execute Tree";
  bt.Execute();
}
