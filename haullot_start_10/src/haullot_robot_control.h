#ifndef HAULLOUT_ROBOT_CONTROL_H_
#define HAULLOUT_ROBOT_CONTROL_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#include <vector>
#include <memory>

#define DEG_TO_RAD_ (0.01745329)

class haullotRobotControl
{

public:
    haullotRobotControl(/* args */);
    ~haullotRobotControl();

    int controlCarMotion(const std::vector<std::vector<double>>& command);
    int controlLiftMechanismUpMotion(   \
        const std::vector<std::vector<double>>& command);
    int controlLiftMechanismRotateMotion(   \
        const std::vector<std::vector<double>>& command);
    int controlGuideRailMotion(const std::vector<std::vector<double>>& command);
    int controlRobotMotion(const std::vector<std::vector<double>>& command);

    int setMaxRobotJointVelocity(const double velocity);
    int setMaxLiftTranslationVelocity(const double velocity);
    int setMaxLiftRatateVelocity(const double velocity);
    int setMaxGuideRailVelocity(const double velocity);
    int setMaxCarVelocity(const double velocity);

private:
    int setModelState();
    static void subModelStateCallBack(  \
        const sensor_msgs::JointState::ConstPtr& msg);

    std::vector<std::vector<double>> getPathPlanResult( \
        const std::vector<std::vector<double>>& path_list) const;

    int EnableModelMotion(const std::vector<std::vector<double>>& path_list);
    
    int sentModelCommands(const std::vector<sensor_msgs::JointState> &command);

    int sentTransform(const std::vector<std::vector<double>>& command);

private:

    // 控制类型
    enum class CONTRL_TYPE : int{
        CAR = 0,
        LIFT_MECHANISM_UP,
        LIFT_MECHANISM_ROTATE,
        GUIDE_RAIL,
        ROBOT,
        NOTHING
    };

private:
    // ros 句柄
    ros::NodeHandle nh_;

    // ros坐标变化发布器
    std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_;
    
    // ros坐标变化监听器 
    std::unique_ptr<tf::TransformListener> tf_listener_;
    
    // 模型位置命令
    std::unique_ptr<sensor_msgs::JointState> model_command_msg_;

    // 模型状态命令
    static sensor_msgs::JointState model_state_msg_;
    static std::atomic_bool if_get_model_state_msg_; 

    // 机器人关节控制命令发布器
    std::unique_ptr<ros::Publisher> pub_model_command_;

    // 模型状态订阅器
    std::unique_ptr<ros::Subscriber> sub_model_state_;

    // 最大关节速度
    double max_robot_joint_velocity_;
    double max_lift_mechanism_up_velocity_;
    double max_lift_mechanism_rotate_velocity_;
    double max_guide_rail_velocity_;
    double max_car_velocity_;

    // 轨迹生成单位每秒次数(hz)
    int traj_rate_hz_;

    CONTRL_TYPE control_type_;
    
};

#endif