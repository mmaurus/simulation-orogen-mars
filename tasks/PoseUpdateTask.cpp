/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "PoseUpdateTask.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/utils/misc.h>
#include <mars/sim/SimEntity.h>
#include <unistd.h>

#include <base-logging/Logging.hpp>


namespace mars {

  using namespace interfaces;
  using namespace utils;


  PoseUpdateTask::PoseUpdateTask(std::string const& name)
      : PoseUpdateTaskBase(name)
  {
  }

  PoseUpdateTask::PoseUpdateTask(std::string const& name, RTT::ExecutionEngine* engine)
      : PoseUpdateTaskBase(name, engine)
  {
  }

  PoseUpdateTask::~PoseUpdateTask()
  {
  }

  void PoseUpdateTask::applyPose(std::string entityname, base::Pose pose) {
    sim::SimEntity* ent = control->entities->getEntity(entityname, false);
    if (ent == nullptr)
        ent = control->entities->getRootOfAssembly(entityname);
    if (ent == nullptr) {
      std::cerr << "mars::PoseUpdateTask | given entity name \"" << entityname << "\" could not be found in entity manager." << std::endl;
      return;
    }
    
    configmaps::ConfigMap cfg = ent->getConfig();
    cfg["position"][0] = pose.position[0];
    cfg["position"][1] = pose.position[1];
    cfg["position"][2] = pose.position[2];

    cfg["rotation"][0] = pose.orientation.w();
    cfg["rotation"][1] = pose.orientation.x();
    cfg["rotation"][2] = pose.orientation.y();
    cfg["rotation"][3] = pose.orientation.z();

    control->sim->physicsThreadLock();
    ent->setInitialPose(true, &cfg);
    control->sim->physicsThreadUnlock();
  }

// void PoseUpdateTask::applyRelativePose(std::string entityname, base::Pose pose) {
    
//     sim::SimEntity* ent = control->entities->getEntity(entityname);
//     if (ent == nullptr) {
//         std::cerr << "mars::PoseUpdateTask | given entity name \"" << entityname << "\" could not be found in entity manager." << std::endl;
//         return;
//     }

//     configmaps::ConfigMap cfg;
//     cfg["position"][0] = ent->getPosition() + pose.position[0];
//     cfg["position"][1] = ent->getPosition() + pose.position[1];
//     cfg["position"][2] = ent->getPosition() + pose.position[2];

//     utils::Quaternion q = ent->getOrientation();
//     utils::Quaternion new_q = q * pose.orientation;

//     cfg["rotation"][0] = new_q.w();
//     cfg["rotation"][1] = new_q.x();
//     cfg["rotation"][2] = new_q.y();
//     cfg["rotation"][3] = new_q.z();    

//     control->sim->physicsThreadLock();
//     ent->setInitialPose(true, &cfg);
//     control->sim->physicsThreadUnlock();
//   }

  void PoseUpdateTask::init()
  {
  }

  void PoseUpdateTask::update(double delta_t)
  {
  }

  /// The following lines are template definitions for the various state machine
  // hooks defined by Orocos::RTT. See PoseUpdateTask.hpp for more detailed
  // documentation about them.

  bool PoseUpdateTask::configureHook()
  {
    if (! PoseUpdateTaskBase::configureHook()) return false;
    return true;
  }

  bool PoseUpdateTask::startHook()
  {
    return PoseUpdateTaskBase::startHook();
  }

  void PoseUpdateTask::updateHook()
  {
    PoseUpdateTaskBase::updateHook();

    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

    mars::PoseUpdate update;
    if (_single_pose_update.readNewest(update)) {
        applyPose(update.entity_name, update.pose);
    }


    mars::PoseUpdates updates;
    if (_multi_pose_updates.readNewest(updates)) {
        for (auto u: updates) {
            applyPose(u.entity_name, u.pose);
        }
    }

    // mars::PoseUpdate rel_update;
    // if (_single_relatve_pose_update.readNewest(rel_update)) {
    //     applyRelativePose(update.entity_name, update.pose);
    // }


    // mars::PoseUpdates rel_updates;
    // if (_multi_relatve_pose_updates.readNewest(rel_updates)) {
    //     for (auto u: updates) {
    //         applyRelativePose(u.entity_name, u.pose);
    //     }
    // }
  }

  // void PoseUpdateTask::errorHook()
  // {
  //     PoseUpdateTaskBase::errorHook();
  // }

  void PoseUpdateTask::stopHook()
  {
    PoseUpdateTaskBase::stopHook();
  }

  // void PoseUpdateTask::cleanupHook()
  // {
  //     PoseUpdateTaskBase::cleanupHook();
  // }
}
