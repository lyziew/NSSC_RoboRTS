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
#ifndef ROBORTS_DECISION_ESCAPE_ACTION_H
#define ROBORTS_DECISION_ESCAPE_ACTION_H

#include "goal_factory.h"
#include "../behavior_tree/behavior_node.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../log.h"

namespace roborts_decision
{
class WaitAction : public ActionNode
{
  public:
    WaitAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr, ChassisExecutor::Ptr &chassis_executor_ptr) :
        ActionNode::ActionNode("wait_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr), chassis_executor_ptr_(chassis_executor_ptr)
    {
    }

    virtual ~WaitAction() = default;

  private:
    virtual void OnInitialize()
    {
        LOG_INFO << name_ << " " << __FUNCTION__;
    };

    virtual BehaviorState Update()
    {
        // update state and return
        return chassis_executor_ptr_->Update();
    }

    virtual void OnTerminate(BehaviorState state)
    {
        switch (state)
        {
        case BehaviorState::IDLE:
            chassis_executor_ptr_->Cancel();
            LOG_INFO << name_ << " " << __FUNCTION__ << " IDLE!";
            break;
        case BehaviorState::SUCCESS:
            LOG_INFO << name_ << " " << __FUNCTION__ << " SUCCESS!";
            break;
        case BehaviorState::FAILURE:
            LOG_INFO << name_ << " " << __FUNCTION__ << " FAILURE!";
            break;
        default:
            LOG_INFO << name_ << " " << __FUNCTION__ << " ERROR!";
            return;
        }
    }

    GoalFactory::GoalFactoryPtr goal_factory_ptr_;
    ChassisExecutor::Ptr chassis_executor_ptr_;
}; // class WaitAction
} // namespace roborts_decision
#endif //ROBORTS_DECISION_ESCAPE_ACTION_H
