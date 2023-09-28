#ifndef _ENUM_CLASS_H_
#define _ENUM_CLASS_H_

namespace ENUM_CLASS{

/**
 * @brief //* 行为执行状态
 */    
enum class BehaviorState {
  RUNNING,   ///< Running state in process
  SUCCESS,   ///< Success state as result
  FAILURE,   ///< Failure state as result
  IDLE,      ///< Idle state, state as default or after cancellation
};

/**
* @brief Chassis execution mode for different tasks
*/
enum class ChassisExcutionMode{
    IDLE_MODE,            ///< Default idle mode with no task
    GOAL_MODE,            ///< Goal-targeted task mode using global and local planner
    SPEED_MODE,           ///< Velocity task mode
    SPEED_WITH_ACCEL_MODE, ///< Velocity with acceleration task mode
};

/**
   * @brief Gimbal execution mode for different tasks
   */
  enum class GimbalExcutionMode{
    IDLE_MODE,   ///< Default idle mode with no task
    ANGLE_MODE,  ///< Angle task mode
    SHOOT_MODE    ///< Shoot task mode
  };


/**
 * @brief //* 潜伏期状态
 */
enum class LurkingStatus {
    Normal,
    Ready,
    Lurking,
};

/**
 * @brief //* 比赛阶段
 */
enum class GameStatus {
    Ready,
    Preparation,
    Initialize,
    Five_Second_CD,
    Game,
    End,
};


/**
 * @brief //* 机器人死亡情况
 */
enum class ROBORTS_DEAD {
    None,
    ROBORT_1,
    ROBORT_2,
    BOTH,
};

/**
 * @brief //* 刚与之交互过的敌方机器人
 */
enum class ROBORTS_RELATIVE {
    None,
    ROBORT_1,
    ROBORT_2,
    BOTH,
};

/**
 * @brief //* 机器人扭摆模式
 */
enum class SWING_STATUS{
    FAST,
    SLOW,
    MIDDLE,
};

/**
 * @brief //* 机器人故障情况
 */
enum class FaultType {
    None,
    Chassis_Enable,
    Gimbal_Enable,
    Shooter_Enable,
    Dead,
};

/**
 * @brief //* 视野中的机器人情况
 */
enum class EnemyDetected {
    None,
    ROBORT_1,
    ROBORT_2,
    BOTH,
};

/**
 * @brief //* 机器人颜色及编号
 */
enum class ROBORT_COLOR_NUMBER {
    RED1,
    RED2,
    BLUE1,
    BLUE2,
};

/**
 * @brief //* 场上机器人情况：死亡、是否被检测到、是否刚与之交互等等
 */
enum class ROBORTS_ {
    None,
    ROBORT_1,
    ROBORT_2,
    BOTH,
};


}
#endif