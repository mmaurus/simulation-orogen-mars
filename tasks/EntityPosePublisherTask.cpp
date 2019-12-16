/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "EntityPosePublisherTask.hpp"
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>

using namespace mars;

EntityPosePublisherTask::EntityPosePublisherTask(std::string const& name)
    : EntityPosePublisherTaskBase(name)
{
}

EntityPosePublisherTask::~EntityPosePublisherTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See EntityPosePublisherTask.hpp for more detailed
// documentation about them.

bool EntityPosePublisherTask::configureHook()
{
    if (! EntityPosePublisherTaskBase::configureHook())
        return false;

    entity_names = _entity_names.get();

    for(size_t i=0; i<out_ports_.size(); i++){
        ports()->removePort(out_ports_[i]->getName());
        delete out_ports_[i];
    }
    out_ports_.clear();
    out_ports_.resize(entity_names.size());
    
    for(unsigned int i=0; i<entity_names.size(); ++i) {
        std::string port_name = entity_names[i]+"_pose";
        LOG_INFO("Adding port %s to component", port_name.c_str());
        RTT::OutputPort<base::samples::RigidBodyState>* output_port =
                new RTT::OutputPort<base::samples::RigidBodyState>(port_name);
        ports()->addPort(port_name, *output_port);
        out_ports_[i] = output_port;
    }

    return true;
}
bool EntityPosePublisherTask::startHook()
{
    if (! EntityPosePublisherTaskBase::startHook())
        return false;
    return true;
}
void EntityPosePublisherTask::updateHook()
{
    EntityPosePublisherTaskBase::updateHook();

    for(unsigned int i=0; i<entity_names.size(); ++i) {
        const std::string& entity_name = entity_names[i];
        base::samples::RigidBodyState rbs;
        unsigned long id = control->nodes->getID(entity_name);
        if (id <= 0) {
            LOG_ERROR("No node with name '%s' found!", entity_name.c_str());
            continue;
        }

        mars::interfaces::NodeData nodedata = control->nodes->getFullNode(id);        
        rbs.position = nodedata.pos;
        rbs.orientation = nodedata.rot;
        rbs.sourceFrame = entity_name;
        rbs.targetFrame = _world_frame.get();

        out_ports_[i]->write(rbs);
    }
}

void EntityPosePublisherTask::errorHook()
{
    EntityPosePublisherTaskBase::errorHook();
}
void EntityPosePublisherTask::stopHook()
{
    EntityPosePublisherTaskBase::stopHook();
}
void EntityPosePublisherTask::cleanupHook()
{
    EntityPosePublisherTaskBase::cleanupHook();
}
