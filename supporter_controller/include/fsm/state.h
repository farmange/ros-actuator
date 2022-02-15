/*
 *  state.h
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
#ifndef FSM_STATE_H
#define FSM_STATE_H
#include "rclcpp/rclcpp.hpp"

template <typename T>
class Engine;

template <class T>
class State
{
public:
  State(T* obj, Engine<T>* engine, std::string name)
  {
    name_ = name;
    obj_ = obj;
    enter_ptr_ = nullptr;
    update_ptr_ = nullptr;
    exit_ptr_ = nullptr;

    if (obj_ == nullptr)
    {
      printf("ERROR : No state machine obj !");
      return;
    }
    RCLCPP_DEBUG(obj_->get_logger(), "Construct state %s", name.c_str());
    engine->registerState(this);
  };

  ~State(){};

  void enter()
  {
    if (obj_ == nullptr)
    {
      printf("ERROR : No state machine context object !");
      return;
    }
    if (enter_ptr_ == nullptr)
    {
      //       ROS_DEBUG("No ENTER function for the state %s !", name_.c_str());
      return;
    }
    RCLCPP_DEBUG(obj_->get_logger(), "Run enter function of state '%s'...", name_.c_str());
    (obj_->*enter_ptr_)();
  };

  void update()
  {
    if (obj_ == nullptr)
    {
      printf("ERROR : No state machine context object !");
      return;
    }
    if (update_ptr_ == nullptr)
    {
      //       ROS_DEBUG("No UPDATE function for the state %s !", name_.c_str());
      return;
    }
    RCLCPP_DEBUG(obj_->get_logger(), "Run update function of state '%s'...", name_.c_str());
    (obj_->*update_ptr_)();
  };

  void exit()
  {
    if (obj_ == nullptr)
    {
      printf("ERROR : No state machine context object !");
      return;
    }
    if (exit_ptr_ == nullptr)
    {
      //       ROS_DEBUG("No EXIT function for the state %s !", name_.c_str());
      return;
    }
    RCLCPP_DEBUG(obj_->get_logger(), "Run exit function of state '%s'...", name_.c_str());
    (obj_->*exit_ptr_)();
  };

  void registerEnterFcn(void (T::*fp)(void))
  {
    enter_ptr_ = fp;
  };

  void registerUpdateFcn(void (T::*fp)(void))
  {
    update_ptr_ = fp;
  };

  void registerExitFcn(void (T::*fp)(void))
  {
    exit_ptr_ = fp;
  };

  std::string getName()
  {
    return name_;
  };

private:
  T* obj_;
  std::string name_;
  void (T::*enter_ptr_)(void);
  void (T::*update_ptr_)(void);
  void (T::*exit_ptr_)(void);
};
#endif
