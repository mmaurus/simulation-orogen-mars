/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "RepositionTask.hpp"
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


  RepositionTask::RepositionTask(std::string const& name)
      : RepositionTaskBase(name)
  {
  }

  RepositionTask::RepositionTask(std::string const& name, RTT::ExecutionEngine* engine)
      : RepositionTaskBase(name, engine)
  {
  }

  RepositionTask::~RepositionTask()
  {
  }

  void RepositionTask::applyPose(std::string entityname, base::Pose pose) {
    configmaps::ConfigMap cfg;
    for (auto it: scene) {
      if (it["name"] == entityname) {
        cfg = it;
        break;
      }
    }

    cfg["position"][0] = pose.position[0];
    cfg["position"][1] = pose.position[1];
    cfg["position"][2] = pose.position[2];

    cfg["rotation"][0] = pose.orientation.w();
    cfg["rotation"][1] = pose.orientation.x();
    cfg["rotation"][2] = pose.orientation.y();
    cfg["rotation"][3] = pose.orientation.z();

    sim::SimEntity* ent = control->entities->getEntity(entityname, /*verbose*/false);
    if (!ent) ent = control->entities->getRootOfAssembly(entityname);
    if (!ent)
      LOG_ERROR("Neither entity nor assembly with name '%s' found!", entityname.c_str());
    control->sim->physicsThreadLock();
    ent->setInitialPose(true, &cfg);
    control->sim->physicsThreadUnlock();
  }

  void RepositionTask::init()
  {
  }

  void RepositionTask::update(double delta_t)
  {
  }

  /// The following lines are template definitions for the various state machine
  // hooks defined by Orocos::RTT. See RepositionTask.hpp for more detailed
  // documentation about them.

  bool RepositionTask::configureHook()
  {
    if (! RepositionTaskBase::configureHook()) return false;
    scene_path = _scene_path.get();
    entity_names = _entity_names.get();
    // Parse the cfg and save the relevant parts
    configmaps::ConfigMap scene_map = configmaps::ConfigMap::fromYamlFile(scene_path);
    LOG_DEBUG("Using scene/assembly %s\n", scene_path.c_str());
    if (scene_map.hasKey("smurfs")) {
      scene = scene_map["smurfs"];
    } else if (scene_map.hasKey("smurfa")) {
      scene = scene_map["smurfa"];
    } else if (scene_map.hasKey("entities")) {
      scene = scene_map["entities"];
    } else {
      throw std::invalid_argument ("File content ("+scene_path+") invalid!");
    }

    //find link and parent ids
    for (auto ent_it: entity_names) {
      sim::SimEntity* e = control->entities->getEntity(ent_it);
      if (e) {
        // link_ids
        link_ids[ent_it] = e->getRootestId();
        // find parent id
        for (auto scn_it: scene) {
          if (scn_it.hasKey("parent") && ((std::string) scn_it["name"])==ent_it) {
            std::string par = scn_it["parent"];
            int pos = par.find("::");
            parent_ids[link_ids[ent_it]] = control->entities->getEntityNode(par.substr(0,pos), par.substr(pos+2));
            children_ids[control->entities->getEntity(par.substr(0,pos))->getRootestId()].push_back(link_ids[ent_it]);
          }
        }
      } else {
        throw std::invalid_argument ("No such entity with name " + ent_it + " found!");
      }
    }

    return true;
  }

  bool RepositionTask::startHook()
  {
    return RepositionTaskBase::startHook();
  }

  void RepositionTask::updateHook()
  {
    RepositionTaskBase::updateHook();

    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

    std::vector<base::Pose> poses;
    if (_poses.readNewest(poses)) {
      assert(poses.size()==6);
      for (unsigned int i=0;i<6/*poses.size()*/;i++) {
          base::Pose tmp;
          if (i<poses.size()) {
            if (_ignore_position.get()) {tmp.position = Eigen::Vector3d::Zero();}
            else {tmp.position = poses[i].position;}
            tmp.orientation = poses[i].orientation;
            tmp.orientation.normalize();
            applyPose(entity_names[i], tmp);
          }
      }
    }
  }

  // void RepositionTask::errorHook()
  // {
  //     RepositionTaskBase::errorHook();
  // }

  void RepositionTask::stopHook()
  {
    RepositionTaskBase::stopHook();
  }

  // void RepositionTask::cleanupHook()
  // {
  //     RepositionTaskBase::cleanupHook();
  // }
}
