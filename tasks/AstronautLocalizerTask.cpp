/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AstronautLocalizerTask.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/utils/misc.h>
#include <mars/sim/SimEntity.h>

using namespace mars;

AstronautLocalizerTask::AstronautLocalizerTask(std::string const& name)
    : AstronautLocalizerTaskBase(name), astronaut_id(0)
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
    astronaut_id = control->nodes->getID(_astronaut_name.get());
    return true;
}
void AstronautLocalizerTask::updateHook()
{
    AstronautLocalizerTaskBase::updateHook();

    if (astronaut_id > 0) {
        mars::interfaces::NodeData nodedata = control->nodes->getFullNode(astronaut_id);
        base::samples::RigidBodyState astro;
        astro.position = nodedata.pos;
        astro.orientation = nodedata.rot;

        astro.sourceFrame = "astronaut_feet";
        astro.targetFrame = "world";

        if (_coordinate_frame_id.get() == 1) {
            unsigned int robot_id = control->nodes->getID(_robot_name.get());
            if (robot_id == 0) {
                LOG_ERROR("Neither entity nor assembly with name '%s' found!", _robot_name.get().c_str());
                return;
            }

            mars::interfaces::NodeData robotnodedata = control->nodes->getFullNode(robot_id);
            base::samples::RigidBodyState rob;
            rob.position = robotnodedata.pos;
            rob.orientation = robotnodedata.rot;

            //base::Quaterniond 
            std::cout << "rob position = " << rob.position << std::endl;
            std::cout << "astronaut position = " << astro.position << std::endl;

            //astro.orientation = rob.orientation.conjugate() * astro.orientation;
            astro.position = astro.position - rob.position;

            astro.targetFrame = "map";
        }
        _astronaut_pose.write(astro);

    } else {
        LOG_ERROR("No node with name '%s' found!", _astronaut_name.get().c_str());
        astronaut_id = control->nodes->getID(_astronaut_name.get());
    }

    // sim::SimEntity* astronaut = control->entities->getEntity(_astronaut_name.get(), false);
    // if (astronaut == nullptr)
    //     astronaut = control->entities->getRootOfAssembly(_astronaut_name.get());
    // if (astronaut != nullptr) {
    //     configmaps::ConfigMap cfg = astronaut->getConfig();
    //     base::samples::RigidBodyState astro;
    //     astro.position[0] = cfg["position"][0];
    //     astro.position[1] = cfg["position"][1];
    //     astro.position[2] = cfg["position"][2];
    //     astro.orientation = Eigen::Quaterniond(cfg["rotation"][0], cfg["rotation"][1], cfg["rotation"][2], cfg["rotation"][3]);

    //     astro.sourceFrame = "astronaut_feet";
    //     astro.targetFrame = "world";

    //     if (_coordinate_frame_id.get() == 1) {
    //         sim::SimEntity* robot = control->entities->getEntity(_robot_name.get(), false);
    //         if (robot == nullptr)
    //             robot = control->entities->getRootOfAssembly(_robot_name.get());
    //         if (robot != nullptr) {
    //             configmaps::ConfigMap cfg = robot->getConfig();
    //             base::samples::RigidBodyState rob;
    //             rob.position[0] = cfg["position"][0];
    //             rob.position[1] = cfg["position"][1];
    //             rob.position[2] = cfg["position"][2];
    //             rob.orientation = Eigen::Quaterniond(cfg["rotation"][0], cfg["rotation"][1], cfg["rotation"][2], cfg["rotation"][3]);

    //             //base::Quaterniond 
    //             std::cout << "rob position = " << rob.position << std::endl;
    //             std::cout << "astronaut position = " << astro.position << std::endl;

    //             //astro.orientation = rob.orientation.conjugate() * astro.orientation;
    //             astro.position = astro.position - rob.position;

    //             astro.targetFrame = "map";

    //         } else {
    //             LOG_ERROR("Neither entity nor assembly with name '%s' found!", _robot_name.get().c_str());
    //         }
    //     }

    //     _astronaut_pose.write(astro);
    // } else {
    //     LOG_ERROR("Neither entity nor assembly with name '%s' found!", _astronaut_name.get().c_str());
    // }
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
