#ifndef ROBORTS_DECISION_GIMBAL_EXECUTOR_H
#define ROBORTS_DECISION_GIMBAL_EXECUTOR_H
#include "ros/ros.h"

#include "roborts_msgs/GimbalAngle.h"

#include "../behavior_tree/behavior_state.h"
namespace roborts_decision{
/***
 * @brief Gimbal Executor to execute different abstracted task for gimbal module
 */
class GimbalExecutor{
 public:
  /**
   * @brief Gimbal execution mode for different tasks
   */
  enum class ExcutionMode{
    IDLE_MODE,   ///< Default idle mode with no task
    ANGLE_MODE,  ///< Angle task mode
    RATE_MODE    ///< Rate task mode
  };
  /**
   * @brief Constructor of GimbalExecutor
   */
  GimbalExecutor();
  ~GimbalExecutor() = default;
  /***
   * @brief Execute the gimbal angle task with publisher
   * @param gimbal_angle Given gimbal angle
   */
  void Execute(const roborts_msgs::GimbalAngle &gimbal_angle);
  /**
   * @brief Update the current gimbal executor state
   * @return Current gimbal executor state(same with behavior state)
   */
  BehaviorState Update();
  /**
   * @brief Cancel the current task and deal with the mode transition
   */
  void Cancel();

 private:
  //! execution mode of the executor
  ExcutionMode excution_mode_;
  //! execution state of the executor (same with behavior state)
  BehaviorState execution_state_;

  //! zero gimbal angle
  roborts_msgs::GimbalAngle zero_gimbal_angle_;

  //! gimbal angle control publisher in ROS
  ros::Publisher cmd_gimbal_angle_pub_;


};
}


#endif //ROBORTS_DECISION_GIMBAL_EXECUTOR_H