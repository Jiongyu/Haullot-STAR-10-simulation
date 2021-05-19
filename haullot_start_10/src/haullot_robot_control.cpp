#include "./haullot_robot_control.h"

sensor_msgs::JointState haullotRobotControl::model_state_msg_ =  \
    sensor_msgs::JointState();

std::atomic_bool haullotRobotControl::if_get_model_state_msg_(false);

haullotRobotControl::haullotRobotControl(/* args */)
{
    tf_broadcaster_ = std::make_unique<tf::TransformBroadcaster>();
    tf_listener_ = std::make_unique<tf::TransformListener>();
    model_command_msg_ = std::make_unique<sensor_msgs::JointState>();

    sub_model_state_ = std::make_unique<ros::Subscriber>(std::move( \
        nh_.subscribe<sensor_msgs::JointState>( \
            "/haullot_start_10/joint_states",     \
            5, subModelStateCallBack)));

    pub_model_command_ = std::make_unique<ros::Publisher>(std::move(    \
        nh_.advertise<sensor_msgs::JointState>( \
        "/haullot_start_10/joint_commands", 5)));

    model_command_msg_->name =  \
        {"Haullot_start_10_rotate_joint", "Haullot_start_10_lift_joint_1",   \
        "Haullot_start_10_lift_joint_2", "Haullot_start_10_lift_joint_3",   \
        "Haullot_start_10_lift_joint_4", "guide_rail_sliding_joint",    \
        "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"};

    model_command_msg_->position.resize(model_command_msg_->name.size());

    max_robot_joint_velocity_ = 5; /* degree */
    max_lift_mechanism_rotate_velocity_ = 5; /* degree */
    max_guide_rail_velocity_ = 0.1; /* meter */
    max_lift_mechanism_up_velocity_ = 0.1; /* meter */
    max_car_velocity_ = 0.1; /* meter */
    traj_rate_hz_ = 25;
    control_type_ = CONTRL_TYPE::NOTHING;

}

haullotRobotControl::~haullotRobotControl()
{

}

int haullotRobotControl::setMaxRobotJointVelocity(const double velocity)
{
    max_robot_joint_velocity_ = velocity;
}

int haullotRobotControl::setMaxLiftTranslationVelocity(const double velocity)
{
    max_lift_mechanism_up_velocity_ = velocity;
}

int haullotRobotControl::setMaxLiftRatateVelocity(const double velocity)
{
    max_lift_mechanism_rotate_velocity_ = velocity;
}

int haullotRobotControl::setMaxGuideRailVelocity(const double velocity)
{
    max_guide_rail_velocity_ = velocity;
}
int haullotRobotControl::setMaxCarVelocity(const double velocity)
{
    max_car_velocity_ = velocity;
}


int haullotRobotControl::controlCarMotion(  \
    const std::vector<std::vector<double>>& command)
{
    control_type_ = CONTRL_TYPE::CAR;
    std::vector<std::vector<double>>temp = getPathPlanResult(command);

    return sentTransform(temp);
}

int haullotRobotControl::controlLiftMechanismUpMotion(    \
    const std::vector<std::vector<double>>& command)
{
    if(setModelState() < 0)
    {
        ROS_WARN("controlLiftMechanismUpMotion setModelState timeout.");
        return -1;
    }
    control_type_ = CONTRL_TYPE::LIFT_MECHANISM_UP;

    return EnableModelMotion(command);
}

int haullotRobotControl::controlLiftMechanismRotateMotion(   \
    const std::vector<std::vector<double>>& command)
{
    if(setModelState() < 0)
    {
        ROS_WARN("controlLiftMechanismRotateMotion setModelState timeout.");
        return -1;
    }
    control_type_ = CONTRL_TYPE::LIFT_MECHANISM_ROTATE;

    return EnableModelMotion(command);
    
}

int haullotRobotControl::controlGuideRailMotion(    \
    const std::vector<std::vector<double>>& command)
{
    if(setModelState() < 0)
    {
        ROS_WARN("controlGuideRailMotion setModelState timeout.");
        return -1;
    }
    control_type_ = CONTRL_TYPE::GUIDE_RAIL;

    return EnableModelMotion(command);
}

int haullotRobotControl::controlRobotMotion(    \
    const std::vector<std::vector<double>>& command)
{
    if(setModelState() < 0)
    {
        ROS_WARN("controlRobotMotion setModelState timeout.");
        return -1;
    }
    control_type_ = CONTRL_TYPE::ROBOT;

    return EnableModelMotion(command);
}

int haullotRobotControl::setModelState()
{
    int count = 10;
    ros::Rate rate(count);
    // 超时3秒
    int timeout = 3 * count;
    int index = 0;
    if_get_model_state_msg_ = false;
    while(!if_get_model_state_msg_ && index < timeout){
        ros::spinOnce();
        rate.sleep();
        index ++;
    }

    if(index >= timeout)
    {
        ROS_WARN("setModelState timeout.");
        return -1;
    }

    model_command_msg_->position = model_state_msg_.position;
    if_get_model_state_msg_ = false;

    return 0;
    
}

void haullotRobotControl::subModelStateCallBack(  \
    const sensor_msgs::JointState::ConstPtr& msg)
{
    model_state_msg_ = *msg;
    if_get_model_state_msg_ = true;

}

int haullotRobotControl::EnableModelMotion( \
    const std::vector<std::vector<double>>& path_list)
{
    std::vector<std::vector<double>> path_traj =    \
        getPathPlanResult(path_list);

    int start_position = 0;
    switch (control_type_)
    {
        case CONTRL_TYPE::LIFT_MECHANISM_ROTATE:
            start_position = 0;
            break;
        case CONTRL_TYPE::LIFT_MECHANISM_UP:
            start_position = 1;
            break;
        case CONTRL_TYPE::GUIDE_RAIL:
            start_position = 5;
            break;
        case CONTRL_TYPE::ROBOT:
            start_position = 6;
            break;
        
        default:
            return -1;
    }

    std::vector<sensor_msgs::JointState>path_command;
    for(size_t i = 0; i < path_traj.size(); ++ i)
    {
        for(size_t j = 0; j < path_traj[i].size(); ++ j)
        {

            if( control_type_ == CONTRL_TYPE::ROBOT ||  \
                control_type_ == CONTRL_TYPE::LIFT_MECHANISM_ROTATE)
            {
                model_command_msg_->position[start_position + j] =  \
                    path_traj[i][j] * DEG_TO_RAD_;
            }
            else if(control_type_ == CONTRL_TYPE::GUIDE_RAIL || \
                    control_type_ == CONTRL_TYPE::LIFT_MECHANISM_UP)
            {
                model_command_msg_->position[start_position + j] =  \
                    path_traj[i][j];
            }
        }
        path_command.push_back(*model_command_msg_);
    }

    return sentModelCommands(path_command);
}


std::vector<std::vector<double>> haullotRobotControl::getPathPlanResult( \
    const std::vector<std::vector<double>>& path_list) const
{
    std::vector<std::vector<double>> traj_list;
    traj_list.clear();
    
    for(size_t i = 0; i < path_list.size() - 1; ++i)
    {
        // 1. 获取机器人关节位移,计算最大绝对位移
        // std::cout << "\n1. 获取机器人关节位移,计算最大绝对位移" << std::endl;
        double max_displacement = 0;
        std::vector<double>joint_move_distance(path_list[i].size(), 0);
        // 判断是否为小车转弯自由度
        if(control_type_ == CONTRL_TYPE::CAR)
        {
            joint_move_distance.resize(path_list[i].size() - 1);
        }
        
        for(size_t j = 0; j < path_list[i].size(); ++j)
        {

            // 判断是否为小车转弯自由度
            if(control_type_ == CONTRL_TYPE::CAR && j == path_list.size() - 1)
            {
                break;
            }

            joint_move_distance[j] = (path_list[i+1][j] - path_list[i][j]);
  
            // std::cout << joint_move_distance[j] << " ";
            if(fabs(joint_move_distance[j]) > max_displacement)
            {
                max_displacement = fabs(joint_move_distance[j]);
                // std::cout << "\n" << max_displacement << std::endl;
            }
        }
        // std::cout << "\n" << max_displacement << std::endl;

        // 2. 结合最大速度，计算运行时间, 各关节运行速度
        // std::cout << "\n2. 结合最大速度，计算运行时间, 各关节运行速度" << std::endl;

        double operation_time = 0;
        if (control_type_ == CONTRL_TYPE::ROBOT)
        {
            operation_time = max_displacement / max_robot_joint_velocity_;
        }
        else if(control_type_ == CONTRL_TYPE::LIFT_MECHANISM_ROTATE)
        {
            operation_time = max_displacement / max_lift_mechanism_rotate_velocity_;
        }
        else if(control_type_ == CONTRL_TYPE::LIFT_MECHANISM_UP)
        {
            operation_time = max_displacement / max_lift_mechanism_up_velocity_;
        }
        else if(control_type_ == CONTRL_TYPE::GUIDE_RAIL)
        {
            operation_time = max_displacement / max_guide_rail_velocity_;
        }else if(control_type_ == CONTRL_TYPE::CAR)
        {
            operation_time = max_displacement / max_car_velocity_;
        }

        // std::cout << operation_time << std::endl;

        std::vector<double>joint_move_velocity(joint_move_distance.size(), 0);
        for(size_t j = 0; j < joint_move_distance.size(); ++j)
        {
            joint_move_velocity[j] = joint_move_distance[j] / operation_time;
            // std::cout << joint_move_velocity[j] << " ";
        }

        // 3. 计算各插补增量值
        // std::cout << "\n3. 计算各插补增量值" << std::endl;
        double traj_operation_time = operation_time * traj_rate_hz_;
        // std::cout << "\n" << traj_operation_time << std::endl;
        std::vector<double>joint_derta(joint_move_distance.size(), 0);
        for(int j = 0; j < joint_derta.size(); ++j)
        {
            joint_derta[j] = joint_move_distance[j] / traj_operation_time;
            // std::cout << joint_derta[j] << " ";
        }
        
        // 4. 插值细化路径
        // std::cout << "\n4. 插值细化路径" << std::endl;
        // 判断控制类型
        int index = 1;
        std::vector<double>temp_data;
        temp_data.resize(path_list.at(0).size());

        while (index <= traj_operation_time)
        {

            for(size_t j = 0; j < joint_derta.size(); ++ j)
            {
                if(control_type_ == CONTRL_TYPE::CAR && j == joint_derta.size() - 1)
                {
                    temp_data[j + 1] = \
                        path_list[i][j + 1] + index * ( path_list[i + 1][j + 1] - path_list[i][j + 1]) / traj_operation_time;
                }
                
                temp_data[j] = \
                    path_list[i][j] + index * joint_derta[j];
            }
            // std::cout << "\n";
            traj_list.push_back(temp_data);
            index ++;
        }

        // 添加最后一点
        for(size_t j = 0; j < joint_derta.size(); ++ j){
            temp_data[j] = path_list[i + 1][j];

            if(control_type_ == CONTRL_TYPE::CAR && j == joint_derta.size() - 1)
            {
                temp_data[j + 1] =  path_list[i + 1][j + 1];
            }
        }

        traj_list.push_back(temp_data);
    }

    return std::move(traj_list);
}

int haullotRobotControl::sentModelCommands( \
    const std::vector<sensor_msgs::JointState> &command)
{

    if(command.empty())
    {
        ROS_WARN("path list is empty.");
        return -1;
    }

    int length = command.size();
    int i = 0;
    ros::Rate hz(traj_rate_hz_);
    while (i < length)
    {
        pub_model_command_->publish(command[i]);
        hz.sleep();
        i++;
    }

    // 最后一个路径点发送3次
    int j = 0;
    while(j < 3){
        pub_model_command_->publish(command[length - 1]);
        ros::spinOnce();
        hz.sleep();
        j++;
    }
    
    return 0;
}

int haullotRobotControl::sentTransform( \
    const std::vector<std::vector<double>>& command)
{
    if(command.empty())
    {
        return -1;
    }
    tf::StampedTransform temp;
    tf::Quaternion q;
    tf::Vector3 v;

    temp.child_frame_id_ = "Haullot_start_10_base_link";
    temp.frame_id_ = "world";

    ros::Rate rate(traj_rate_hz_);

    for(size_t i = 0; i < command.size(); ++ i)
    {
        v.setX(command[i][0]);
        v.setY(command[i][1]);
        v.setZ(0.31);
        temp.setOrigin(v);
        q.setRPY(1.571, 0, command[i][2] * DEG_TO_RAD_);
        temp.setRotation(q);
        temp.stamp_ = ros::Time(0);
        tf_broadcaster_->sendTransform(temp);
        rate.sleep();

    }    
    return 0;
}


