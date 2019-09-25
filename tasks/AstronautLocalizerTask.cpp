/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AstronautLocalizerTask.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/utils/misc.h>
#include <mars/sim/SimEntity.h>

using namespace mars;

AstronautLocalizerTask::AstronautLocalizerTask(std::string const& name)
    : AstronautLocalizerTaskBase(name)
{
}

AstronautLocalizerTask::~AstronautLocalizerTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AstronautLocalizerTask.hpp for more detailed
// documentation about them.

bool AstronautLocalizerTask::configureHook()
{
    if (! AstronautLocalizerTaskBase::configureHook())
        return false;
    return true;
}
bool AstronautLocalizerTask::startHook()
{
    if (! AstronautLocalizerTaskBase::startHook())
        return false;
    return true;
}
void AstronautLocalizerTask::updateHook()
{
    AstronautLocalizerTaskBase::updateHook();

    sim::SimEntity* ent = control->entities->getEntity(_entity_name.get(), false);
    if (ent == nullptr)
        ent = control->entities->getRootOfAssembly(_entity_name.get());
    if (ent != nullptr) {
        configmaps::ConfigMap cfg = ent->getConfig();
        base::samples::RigidBodyState p;
        p.position[0] = cfg["position"][0];
        p.position[1] = cfg["position"][1];
        p.position[2] = cfg["position"][2];

        p.orientation = Eigen::Quaterniond(cfg["rotation"][0], cfg["rotation"][1], cfg["rotation"][2], cfg["rotation"][3]);

        p.sourceFrame = "astronaut_feet";
        p.targetFrame = "world";

        _astronaut_pose.write(p);
    } else {
        LOG_ERROR("Neither entity nor assembly with name '%s' found!", _entity_name.get().c_str());
    }
}
void AstronautLocalizerTask::errorHook()
{
    AstronautLocalizerTaskBase::errorHook();
}
void AstronautLocalizerTask::stopHook()
{
    AstronautLocalizerTaskBase::stopHook();
}
void AstronautLocalizerTask::cleanupHook()
{
    AstronautLocalizerTaskBase::cleanupHook();
}
