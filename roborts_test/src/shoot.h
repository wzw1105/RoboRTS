#include <cmath>
#include <algorithm>
#include <thread>

#include <ros/ros.h>
#include <ros/duration.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>

#include "costmap/costmap_interface.h"
#include "roborts_msgs/ArmorMsgs.h"
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/RobotShoot.h"

//the position information of high obstacles (higher than 15cm) (in cell): bottom left_x, bottom left_y, width, height
//玻璃信息
int grass_obstacle_info[4][4] = {
    {142, 20, 21, 5},
    {128, 70, 5, 20},
    {30, 1, 5, 21},
    {1, 66, 20, 5}
};

enum EnemyColor {
  BLUE = 0, RED  = 1
};

class Shoot{
public:
    Shoot();
    ~Shoot();

private:
    void ShootInit();
    /**
     * @brief Calculate the actual y value with air resistance
     * @param x the distanc
     * @param v Projectile velocity
     * @param angle Pitch angle
     * @return The actual y value in the gimbal coordinate
     */
    float BulletModel(float x, float v, float angle);
    /**
     * @brief Get the gimbal control angle
     * @param x Distance from enemy(the armor selected to shoot) to gimbal
     * @param y Value of y in gimbal coordinate.
     * @param v Projectile velocity
     * @return Gimbal pitch angle
     */
    float GetPitch(float x, float y, float v);
    /**
     * @brief check whether the line (start-end) cross the "high" obstacles.
     * 
     * @param startx the x value of start poin
     * @param starty the y value of start point
     * @param endx  the x value of end point
     * @param endy the y value of end point
     * @return true if the line cross high obstacles
     * @return false else
     */
    bool CheckCrossWithHighObstacle(geometry_msgs::PoseStamped start,geometry_msgs::PoseStamped end);
    /**
     * @brief calculate the y value of the line (pass point(x, y) and slope equals k) where x value equals (int)(x + 1)
     * 
     * @param x
     * @param y 
     * @param k 
     * @return double y value in (int)(x + 1)
     */
    double GetNextY(double x, double y, double k);

    /**
     * @brief check whether a vertical line cross any high obstacle
     * 
     * @param x 
     * @param y_1 
     * @param y_2 
     * @return true don't cross
     * @return false else
     */
    bool CheckVerticalLineCross(int x, int y_1, int y_2);

    /**
     * @brief armor msgs callback function
     * 
     * @param msgs the msgs of armor provided by yolox_car.cpp
     */
    void ArmorMsgsCallback(const roborts_msgs::ArmorMsgs::ConstPtr &msgs);

    /**
     * @brief choose the optimal armor for shoot and the optimal number of bullets to shoot
     * 
     * @param msgs armor msgs
     * @param choosed_armor the choosed optimal armor for shoot
     * @param shoot_num the number of bullets for shoot
     * @return true if can shoot
     * @return false else
     */
    bool ChooseArmorForShoot(std::vector<roborts_msgs::ArmorMsg> msgs, roborts_msgs::ArmorMsg &choosed_armor, int &shoot_num);

    /**
     * @brief transform the pose in camera_link frame to map frame
     * 
     * @param armor_pose_ the pose in camera frame
     * @param global_pose the pose in map frame
     * @return true if transform successfully
     * @return false 
     */
    bool GetArmorPose(geometry_msgs::Point &armor_pose_, geometry_msgs::PoseStamped &global_pose);

    /**
     * @brief calculate the distance between two points.
     * 
     * @param p_1 the first point
     * @param p_2 the second point
     * @return double 
     */
    double Distance(geometry_msgs::PoseStamped &p_1, geometry_msgs::PoseStamped &p_2);

    /**
     * @brief control the pitch and yaw of the gimbal smoothly.
     * 
     * @param next_yaw 
     * @param next_pitch 
     */
    void PublishPitchYawMsgs(const float &next_yaw, const float &next_pitch);

    /**
     * @brief call for shoot service with shoot_num bullets
     * 
     * @param shoot_num 
     */
    void CallForShootService(const int &shoot_num);

    void RobotsShootCallBack(const roborts_msgs::RobotShoot::ConstPtr &msgs);

    void ShootThread();

    ros::NodeHandle nh_;
    //armor msgs subscriber
    ros::Subscriber armor_msgs_sub_;
    //tf listener
    std::shared_ptr<tf::TransformListener> tf_ptr_;
    //Costmap pointer
    std::shared_ptr<roborts_costmap::CostmapInterface> costmap_ptr_;
    //gimbal control publisher
    ros::Publisher gimbal_cmd_pub_;
    //last armor for shoot
    geometry_msgs::PoseStamped last_armor_for_shoot_;
    //open fricWhl service
    ros::ServiceClient fric_client;
    roborts_msgs::FricWhl fricCtrl;
    //shoot service client
    ros::ServiceClient shoot_client;
    roborts_msgs::ShootCmd shootSrv;
    //subscribe the speed of the bullet
    ros::Subscriber robot_shoot_sub;
    float shoot_speed_ = 18;

    bool have_last_armor_;

    //memory ArmorMsgs
    roborts_msgs::ArmorMsgs armor_msgs_;

    int enermy_color_ = RED;

    float init_k_ = 0.026;
    float GRAVITY = 9.78;
    double PI = 3.1415926535;

    //if the area of the currently use armor is smaller than this value, then re-find the biggest armor, else use last armor for shoot;
    float min_area_for_resample = 800;
    float min_dis_for_resample = 0.4;
    float h = 0.1;

    // minimal distance between bullet path and obstacle (in cells)
    int bullet_min_cell_dis = 1; //equals cell_dis * 0.05m

    //memory last yaw last pitch;
    float cur_yaw_, cur_pitch_;

    //the threshold for changing yaw and pitch
    float yaw_change_threshold = 0.01;
    float pitch_change_threshold = 0.01;

    //the minimal step to control yaw;
    float yaw_change_unit_ = 0.05;
    float offset_x = 0;

    //
    bool enable_shoot;

    std::thread shoot_thread_;

    int shoot_num = 0;

    int update_step = 10;
    int cur_step = 0;

};