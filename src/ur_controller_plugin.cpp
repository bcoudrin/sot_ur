#include "sot_ur/ur_controller_plugin.h"
#include <pluginlib/class_list_macros.h>


namespace sot_ur {

std::ofstream logout;

UrControllerPlugin::UrControllerPlugin()
    : controller_interface::Controller<hardware_interface::EffortJointInterface>(),
      sot_controller_(),
      loop_count_(0),
      robot_(NULL) {
    logout.open("/tmp/out.log", std::ios::out);
}

UrControllerPlugin::~UrControllerPlugin() {
}

bool
UrControllerPlugin::init(hardware_interface::EffortJointInterface *robot,
                         ros::NodeHandle &n) {
    sot_controller_.node_ = n;

    // Check initialization
    if (!robot) {
        ROS_ERROR_STREAM("NULL robot pointer");
        return false;
    }
    robot_ = robot;

    // Get the joints
    XmlRpc::XmlRpcValue joint_names;
    if (!sot_controller_.node_.getParam("joints", joint_names)) {
        ROS_ERROR("No joints given. (namespace: %s)", sot_controller_.node_.getNamespace().c_str());
        return false;
    }
    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Malformed joint specification. (namespace: %s)", sot_controller_.node_.getNamespace().c_str());
        return false;
    }
    for (int i=0; i<joint_names.size(); ++i) {
        XmlRpc::XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("Array of joint names should contain all strings. (namespace: %s)", sot_controller_.node_.getNamespace().c_str());
            return false;
        }
        //std::cout << i << " : " << name_value << std::endl;
        hardware_interface::JointHandle j = robot->getHandle((std::string)name_value);
        joints_.push_back(j);
    }

    // Setup PID controllers
    std::string gains_ns;
    if (!sot_controller_.node_.getParam("gains", gains_ns))
        gains_ns = sot_controller_.node_.getNamespace() + "/gains";
    pids_.resize(joints_.size());
    for (size_t i=0; i<joints_.size(); ++i) {
        if (!pids_[i].init(ros::NodeHandle(gains_ns + "/" + joints_[i].getName()))) {
            if (!pids_[i].init(ros::NodeHandle(sot_controller_.node_,"pid_parameters"))) {
                ROS_ERROR("Failed to build PID controller");
                return false;
            }
        }
    }

    // Allocate space
    const unsigned int jsz = joints_.size();
    joint_encoder_.resize(jsz);
    joint_velocity_.resize(jsz);
    joint_effort_.resize(jsz);
    joint_control_.resize(jsz);
    error_raw.resize(jsz);
    error.resize(jsz);

    timeFromStart_ = 0.0;

    return true;
}

void
UrControllerPlugin::fillSensors() {
    // Joint values
    sensorsIn_["joints"].setName("position");
    for (unsigned int i=0; i<joints_.size(); ++i)
        joint_encoder_[i] = joints_[i].getPosition();
    sensorsIn_["joints"].setValues(joint_encoder_);

    // Joint velocities
    sensorsIn_["velocities"].setName("velocity");
    for (unsigned int i=0; i<joints_.size(); ++i)
        joint_velocity_[i] = joints_[i].getVelocity();
    sensorsIn_["velocities"].setValues(joint_velocity_);

    // Joint torques
    sensorsIn_["torques"].setName("torque");
    for (unsigned int i=0; i<joints_.size(); ++i)
        joint_effort_[i] = joints_[i].getEffort();
    sensorsIn_["torque"].setValues(joint_effort_);
}

void
UrControllerPlugin::readControl(const ros::Duration &dt) {
    // Update command
    joint_control_ = controlValues_["joints"].getValues();
    joint_velocity_ = controlValues_["velocities"].getValues();
    // 0-11 are casters and are controled by base controller
    for (unsigned int i=0; i<joints_.size(); ++i) {
        error[i] = joints_[i].getPosition() - joint_control_[i];
        double errord = joints_[i].getVelocity() - joint_velocity_[i];
        joints_[i].setCommand(pids_[i].updatePid(error[i], errord, dt));
    }
    ++loop_count_;
}


void
UrControllerPlugin::starting(const ros::Time& time) {
    std::cout << "STARTING" << std::endl;

    for (size_t i=0; i<pids_.size(); ++i)
        pids_[i].reset();

    fillSensors();
    try {
        sot_controller_.setupSetSensors(sensorsIn_);
        sot_controller_.getControl(controlValues_);
    }
    catch (std::exception &e) { throw e; }
    readControl(ros::Duration(0));

    _iter = 0;
    _mean = 0.;
}

void
UrControllerPlugin::update(const ros::Time& time, const ros::Duration& dt) {
    //std::cout << "UPDATE" << std::endl;
    fillSensors();

    struct timeval t0, t1;
    gettimeofday(&t0,0);

    try {
        sot_controller_.nominalSetSensors(sensorsIn_);
        sot_controller_.getControl(controlValues_);
    }
    catch (std::exception &e) { throw e; }

    gettimeofday(&t1, 0);
    _mean += (t1.tv_sec-t0.tv_sec)*1000000 + t1.tv_usec-t0.tv_usec;
    ++_iter;
    if (_iter == 100) {
        std::cout << "[UrControllerPlugin] : " << _mean/_iter << " µs" << std::endl;
        _iter = 0;
        _mean = 0.;
    }

    readControl(dt);
}

void
UrControllerPlugin::stopping(const ros::Time& time) {
    std::cout << "STOPPING" << std::endl;
    fillSensors();
    try {
        sot_controller_.cleanupSetSensors(sensorsIn_);
        sot_controller_.getControl(controlValues_);
    }
    catch (std::exception &e) { throw e; }
    readControl(ros::Duration(0));
}

/// Register controller to pluginlib
/*PLUGINLIB_EXPORT_CLASS(sot_ur::UrControllerPlugin,
                       controller_interface::ControllerBase)*/
PLUGINLIB_DECLARE_CLASS(sot_ur, UrControllerPlugin, sot_ur::UrControllerPlugin, controller_interface::ControllerBase);

}
