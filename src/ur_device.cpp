#include "sot_ur/ur_device.h"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

namespace sot_ur {

const double UrDevice::TIMESTEP_DEFAULT = 0.001;

UrDevice::UrDevice(const std::string &name)
: dynamicgraph::sot::Device(name),
  timestep_(TIMESTEP_DEFAULT),
  previous_state_(),
  robotState_ ("StackOfTasks(" + name + ")::output(vector)::robotState"),
  pose(),
  baseff_(),
  loop_count_(0)
{
    sotDEBUGIN(25);
    signalRegistration(robotState_);
    baseff_.resize(12);

    std::string docstring;
    docstring =
       "\n"
       "    Integrate dynamics for time step provided as input\n"
       "\n"
       "      take one floating point number as input\n"
       "\n";
     addCommand("increment",
            dynamicgraph::command::makeCommandVoid1((Device&)*this,
                     &Device::increment, docstring));

     sotDEBUGOUT(25);
}

UrDevice::~UrDevice() {
}

void
UrDevice::setSensors(SensorMap &sensorsIn) {
    sotDEBUGIN(25);
    SensorMap::iterator it;

    // Joints positions
    it = sensorsIn.find("joints");
    if (it != sensorsIn.end()) {
        const std::vector<double> &anglesIn = it->second.getValues();
        state_.resize(anglesIn.size() + 6);
        for (unsigned i=0;i<6; ++i)
            state_(i) = 0.;
        for (unsigned i=0; i<anglesIn.size(); ++i)
            state_(i+6) = anglesIn[i];
    }

    // Joint velocities
    it = sensorsIn.find("velocities");
    if (it != sensorsIn.end()) {
        const std::vector<double> &velIn = it->second.getValues();
        velocity_.resize(velIn.size() + 6);
        for (unsigned i=0;i<6; ++i)
            velocity_(i) = 0.;
        for (unsigned i=0; i<velIn.size(); ++i)
            velocity_(i+6) = velIn[i];
    }

    // Joint torques
    /*it = sensorsIn.find("torques");
    if (it != sensorsIn.end()) {
        const std::vector<double> &torqueIn = it->second.getValues();
        torque_.resize(torqueIn.size() + 6);
        for (unsigned i=0;i<6; ++i)
            torque_(i) = 0.;
        for (unsigned i=0; i<torqueIn.size(); ++i)
            torque_(i+6) = torqueIn[i];
    }*/

    sotDEBUGOUT(25);
}

void
UrDevice::setupSetSensors(SensorMap &sensorsIn) {
    setSensors(sensorsIn);
}

void
UrDevice::nominalSetSensors(SensorMap &sensorsIn) {
    setSensors(sensorsIn);
}

void
UrDevice::cleanupSetSensors(SensorMap &sensorsIn) {
    setSensors(sensorsIn);
}

void
UrDevice::getControl(ControlMap &controlOut) {
    sotDEBUGIN(25);
    std::vector<double> anglesOut;
    anglesOut.resize(state_.size());
    std::vector<double> velocitiesOut;
    velocitiesOut.resize(state_.size());

    try { increment(timestep_); }
    catch (std::exception & e) {
        std::cerr <<" UrDevice::getControl " <<  e.what()  << std::endl;
    }
     //++loop_count_;

    sotDEBUG(25) << "state = " << state_ << std::endl;
    sotDEBUG(25) << "diff = " << ((previous_state_.size() == state_.size()) ?
                                      (state_ - previous_state_) : state_ ) << std::endl;
    previous_state_ = state_;

    // Get control

    ml::Vector control;
    try {
        control = controlSIN.accessCopy();
    }
    catch (...) {
        control.resize(state_.size());
        for (unsigned i=0; i<state_.size(); ++i)
            control(i) = 0.;
    }

    // Specify joint values
    if (anglesOut.size() != state_.size() - 6)
        anglesOut.resize(state_.size() - 6);
    for (unsigned int i=6; i<state_.size(); ++i)
        anglesOut[i-6] = state_(i);
    controlOut["joints"].setValues(anglesOut);

    // Specify joint velocity
    if (velocitiesOut.size() != state_.size() - 6)
        velocitiesOut.resize(state_.size() - 6);
    for (unsigned int i=6; i<state_.size(); ++i)
        velocitiesOut[i-6] = control(i);
    controlOut["velocities"].setValues(velocitiesOut);

    sotDEBUGOUT(25);
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(UrDevice,"UrDevice");

}
