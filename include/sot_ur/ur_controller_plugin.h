#ifndef PR2_CONTROLLER_PLUGIN_H
#define PR2_CONTROLLER_PLUGIN_H

#include <sot_ur/ur_sot_controller.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/shared_ptr.hpp>

namespace sot_ur {

class UrControllerPlugin : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    explicit UrControllerPlugin();
    virtual ~UrControllerPlugin();

    virtual bool init(hardware_interface::EffortJointInterface *robot,
                      ros::NodeHandle &n);
    virtual void starting(const ros::Time& time);
    virtual void update(const ros::Time& time, const ros::Duration& dt);
    virtual void stopping(const ros::Time& time);

private:
    void fillSensors();
    void readControl(const ros::Duration &dt);

private:
    // SoT Controller
    UrSotController sot_controller_;
    SensorMap sensorsIn_;
    ControlMap controlValues_;

    std::vector<double> joint_encoder_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_control_;
    std::vector<double> error_raw;
    std::vector<double> error;


    // Ur Controller
    int loop_count_;
    ros::Time last_time_;
    static const double gain_ = 1.25;
    static const double setpoint_ = 3.00;
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<control_toolbox::Pid> pids_;
    hardware_interface::EffortJointInterface *robot_;

    // ROS interface
    //ros::NodeHandle node_;
    boost::scoped_ptr<
        realtime_tools::RealtimePublisher<
            control_msgs::JointTrajectoryControllerState> > controller_state_publisher_;

    ros::Publisher cmd_vel_pub_;

    tf::TransformListener listener_;

    double timeFromStart_;

    int _iter;
    double _mean;
};

}

#endif
