/*
 *  fsm_user_input.h
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
#ifndef FSM_USER_INPUT_H
#define FSM_USER_INPUT_H

class FsmUserInput
{
public:
  enum FsmUserInputEnum
  {
    None = 0,
    ButtonModeShortPress,
    ButtonModeDoublePress,
    ButtonModeLongPress,
    ButtonUpDownLongPress
  };

  FsmUserInput() = default;

  const std::string toString() const
  {
    std::string msg;
    switch (user_input_)
    {
      case None:
        msg = "None";
        break;
      case ButtonModeShortPress:
        msg = "ButtonModeShortPress";
        break;
      case ButtonModeDoublePress:
        msg = "ButtonModeDoublePress";
        break;
      case ButtonUpDownLongPress:
        msg = "ButtonUpDownLongPress";
        break;
      default:
        msg = "NaN";
        break;
    }
    return msg;
  }
  FsmUserInput& operator=(const FsmUserInputEnum e)
  {
    user_input_ = e;
    return *this;
  }
  friend bool operator==(const FsmUserInput& c, const FsmUserInputEnum& e)
  {
    return c.user_input_ == e;
  };
  friend bool operator!=(const FsmUserInput& c, const FsmUserInputEnum& e)
  {
    return c.user_input_ == e;
  };

private:
  FsmUserInputEnum user_input_;
};

#endif
