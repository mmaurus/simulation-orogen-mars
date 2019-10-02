/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "RobotTeleportation.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/JointManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/LoadCenter.h>
#include <mars/interfaces/sim/LoadSceneInterface.h>
#include <mars/utils/misc.h>
#include <mars/smurf_loader/SMURFLoader.h>
#include <mars/sim/SimEntity.h>
#include <unistd.h>

#include <base-logging/Logging.hpp>


namespace mars {

  using namespace interfaces;
  using namespace utils;


  RobotTeleportation::RobotTeleportation(std::string const& name)
      : RobotTeleportationBase(name)
  {
  }

  RobotTeleportation::RobotTeleportation(std::string const& name, RTT::ExecutionEngine* engine)
      : RobotTeleportationBase(name, engine)
  {
  }

  RobotTeleportation::~RobotTeleportation()
  {
  }

  void RobotTeleportation::init()
  {
  }

  void RobotTeleportation::update(double delta_t)
  {
  }

  /// The following lines are template definitions for the various state machine
  // hooks defined by Orocos::RTT. See RobotTeleportation.hpp for more detailed
  // documentation about them.

  bool RobotTeleportation::configureHook()
  {
    if (! RobotTeleportationBase::configureHook()) return false;
    scene_path = _scene_path.get();
    robot_name = _robot_name.get();
    pos_mode = _position_mode.get();
    // Parse the cfg and save the relevant parts
    configmaps::ConfigMap map = configmaps::ConfigMap::fromYamlFile(scene_path);
    configmaps::ConfigVector cfg;
    if (map.hasKey("smurfs")) {
      cfg = map["smurfs"];
    } else if (map.hasKey("entities")) {
      cfg = map["entities"];
    } else {
      throw std::invalid_argument ("Scene file invalid!");
    }

    bool found = false;
    for (configmaps::ConfigVector::iterator it = cfg.begin(); it!=cfg.end(); it++) {
      configmaps::ConfigMap m = *it;
      if (m.hasKey("name") && m["name"] == robot_name) {
        map = (configmaps::ConfigMap) m;
        found = true;
      }
    }
    if (!found) {
      throw std::invalid_argument ("Scene file invalid. robot_name not found!");
    }

    if (!map.hasKey("teleportation_targets") && pos_mode != 0)
      throw std::invalid_argument ("No targets given!");
    target.id = 0;
    target.cfg = map;
    reloadScene = false;
    targets.push_back(target);
    for (auto it: map["teleportation_targets"]) {
      //go through the teleportation_targets and save the changed configurations
      target.id++;
      configmaps::ConfigMap m = it;
      for (auto it2 = m.begin(); it2!=m.end(); it2++) {
        target.cfg[it2->first] = it2->second;
      }
      targets.push_back(target);
      if (it.hasKey("file") || it.hasKey("path")) reloadScene = true;
    }
    // initialize with target 0
    robot_entity = control->entities->getEntity(robot_name, false);
    if (!robot_entity) robot_entity = control->entities->getRootOfAssembly(robot_name);
    if (!robot_entity) { LOG_ERROR_S << "RobotTeleportation: "<< robot_name << " not found!"; }
    LOG_DEBUG_S << "RobotTeleportation: " << "got robot: "<< robot_name;
    reset_node_name = _reset_node_name.get();
    LOG_DEBUG_S << "RobotTeleportation: " << "got robot: "<< reset_node_name;

    LOG_DEBUG_S << "RobotTeleportation: "<< "Task configured! " << (pos_mode ? "(Manual)":"(Preconfigured)");

    return true;
  }

  bool RobotTeleportation::startHook()
  {
    return RobotTeleportationBase::startHook();
  }

  void RobotTeleportation::updateHook()
  {
    RobotTeleportationBase::updateHook();

    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

    //if triggered load new position
    old_curr_id = curr_id;
    old_pos = pos;
    old_rot = rot;
    old_anchor = anchor;
    _position_id.readNewest(curr_id);
    _position.readNewest(pos);
    _rotation.readNewest(rot);
    _anchor.readNewest(anchor);
    if (old_curr_id != curr_id||
        old_pos != pos||
        old_rot.x() != rot.x() || old_rot.y() != rot.y() || old_rot.z() != rot.z() || old_rot.w() != rot.w() ||
        old_anchor != anchor||
        _reset_node.readNewest(reset_node) == RTT::NewData
       )
    {
      LOG_DEBUG_S << "RobotTeleportation: " << "updateHook triggered!";
      if (pos_mode == 1){
        if (curr_id >=targets.size()) {
          LOG_DEBUG_S << "RobotTeleportation: " << "given to high id " << curr_id << ">=" <<targets.size()<<"!";
          curr_id = 0;
        }
        target = targets[curr_id];
      } else {
        target.id = -1;
        target.cfg["position"][0] = pos[0];
        target.cfg["position"][1] = pos[1];
        target.cfg["position"][2] = pos[2];
        sRotation rot_e = quaternionTosRotation(rot);
        target.cfg["rotation"][0] = rot_e.alpha;
        target.cfg["rotation"][1] = rot_e.beta;
        target.cfg["rotation"][2] = rot_e.gamma;
        if (anchor) {
          target.cfg["anchor"] = "world";
        } else {
          target.cfg["anchor"] = "none";
        }
        target.cfg["reset_node"] = reset_node;
      }

      if (reloadScene) {
        control->sim->physicsThreadLock();
        LOG_DEBUG_S << "RobotTeleportation: Reloading the following cfg:\n" << target.cfg.toYamlString();
        control->entities->removeEntity(robot_name, true /*remove the complete assembly*/);
        dynamic_cast<mars::smurf::SMURFLoader*>(control->loadCenter->loadScene[".smurfs"])->loadEntity(&target.cfg, getPathOfFile(scene_path));
        robot_entity = control->entities->getEntity(robot_name);
        control->sim->physicsThreadUnlock();
      } else {
        robot_entity->setInitialPose(robot_entity->hasAnchorJoint(), &target.cfg);
      }

      /* if the robot has suspension joints we have to teleport the root node to
       the new position as well*/
      NodeId id = control->nodes->getID(reset_node_name);
      if (id == INVALID_ID) {LOG_DEBUG_S << "RobotTeleportation: "<< "Reset node not found: "<< reset_node_name <<"!";}
      if (target.cfg.find("reset_node") != target.cfg.end() && target.cfg["reset_node"] && id != INVALID_ID) {
        LOG_DEBUG_S << "RobotTeleportation: "<< "Resetting node "<< reset_node_name <<"!";
        Quaternion tmpQ(1, 0, 0, 0);
        Vector tmpV;
        NodeData rootNode = control->nodes->getFullNode(id);
        if(target.cfg.find("position") != target.cfg.end()) {
          rootNode.pos.x() = target.cfg["position"][0];
          rootNode.pos.y() = target.cfg["position"][1];
          rootNode.pos.z() = target.cfg["position"][2];
          control->nodes->editNode(&rootNode, EDIT_NODE_POS | EDIT_NODE_MOVE_ALL);
        }
        if(target.cfg.find("rotation") != target.cfg.end()) {
          // check if euler angles or quaternion is provided; rotate around z
          // if only one angle is provided
          switch (target.cfg["rotation"].size()) {
          case 1: tmpV[0] = 0;
            tmpV[1] = 0;
            tmpV[2] = target.cfg["rotation"][0];
            tmpQ = eulerToQuaternion(tmpV);
            break;
          case 3: tmpV[0] = target.cfg["rotation"][0];
            tmpV[1] = target.cfg["rotation"][1];
            tmpV[2] = target.cfg["rotation"][2];
            tmpQ = utils::eulerToQuaternion(tmpV);
            break;
          case 4: tmpQ.x() = (sReal)target.cfg["rotation"][1];
            tmpQ.y() = (sReal)target.cfg["rotation"][2];
            tmpQ.z() = (sReal)target.cfg["rotation"][3];
            tmpQ.w() = (sReal)target.cfg["rotation"][0];
            break;
          }
          rootNode.rot = tmpQ;
          control->nodes->editNode(&rootNode, EDIT_NODE_ROT | EDIT_NODE_MOVE_ALL);
        }
      }

      LOG_DEBUG_S << "RobotTeleportation: "<< "Robot teleportation done!";
    }
  }

  // void RobotTeleportation::errorHook()
  // {
  //     RobotTeleportationBase::errorHook();
  // }

  void RobotTeleportation::stopHook()
  {
    RobotTeleportationBase::stopHook();
  }

  // void RobotTeleportation::cleanupHook()
  // {
  //     RobotTeleportationBase::cleanupHook();
  // }
}
