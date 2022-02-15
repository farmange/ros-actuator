/*
 *  transition.h
 *  Copyright (C) 2019 Orthopus
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef FSM_TRANSITION_H
#define FSM_TRANSITION_H
#include "rclcpp/rclcpp.hpp"

template <typename T>
class Engine;

template <class T>
class Transition
{
public:
  Transition(T* obj, Engine<T>* engine, State<T>* final_state)
  {
    obj_ = obj;
    final_state_ = final_state;
    condition_ptr_ = nullptr;

    if (obj_ == nullptr)
    {
      printf("ERROR : Bad state machine context object !");
      return;
    }
    engine->registerTransition(this);
  }

  ~Transition(){};

  void registerConditionFcn(bool (T::*fp)(void))
  {
    condition_ptr_ = fp;
  };

  void addInitialState(State<T>* initial_state)
  {
    initial_states_.push_back(initial_state);
  };

  bool isConditionFulfilled()
  {
    if (obj_ == nullptr)
    {
      printf("ERROR : No state machine context object !");
      return false;
    }
    if (condition_ptr_ == nullptr)
    {
      RCLCPP_DEBUG(obj_->get_logger(), "Undefined transition to go to final state '%s' !",
                   final_state_->getName().c_str());
      return false;
    }
    return ((obj_->*condition_ptr_)());
  };

  std::vector<State<T>*> getInitialStates()
  {
    return initial_states_;
  };

  State<T>* getFinalState()
  {
    return final_state_;
  };

private:
  T* obj_;
  std::vector<State<T>*> initial_states_;
  State<T>* final_state_;
  bool (T::*condition_ptr_)(void);
};
#endif
