/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "EntityFakeDetection.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/sim/SimEntity.h>

#include <base-logging/Logging.hpp>

namespace mars {

  using namespace interfaces;


  EntityFakeDetection::EntityFakeDetection(std::string const& name)
      : EntityFakeDetectionBase(name)
  {
  }

  EntityFakeDetection::EntityFakeDetection(std::string const& name, RTT::ExecutionEngine* engine)
      : EntityFakeDetectionBase(name, engine)
  {
  }

  EntityFakeDetection::~EntityFakeDetection()
  {
  }

  void EntityFakeDetection::init()
  {
  }

  void EntityFakeDetection::update(double delta_t)
  {
  }

  /// The following lines are template definitions for the various state machine
  // hooks defined by Orocos::RTT. See EntityFakeDetection.hpp for more detailed
  // documentation about them.

  bool EntityFakeDetection::configureHook()
  {
    if (! EntityFakeDetectionBase::configureHook())
      return false;

    frame_id = FrameId(_frame_id.get());
    minVisibleVertices = _minVisibleVertices.get();
    use_camera = (bool) _use_camera.get();

    camera_link_name = _camera_link.get();

    if (use_camera == false || camera_link_name.empty() ) {
      frame_id = FrameId::GLOBAL;
    }
    type_prefix = _type_prefix.get();

    LOG_DEBUG_S << "EntityFakeDetection"<< "Task configured! " << (use_camera ? "(Using Camera)":"(Not Using Camera)");

    return true;
  }

  bool EntityFakeDetection::startHook()
  {
    if (use_camera == true) {
      if (!EntityFakeDetectionBase::startHook()) {
        if (!camera) {
          LOG_ERROR_S << "EntityFakeDetection"<< "Start of CameraPlugin failed! Using without camera!";
          use_camera = false;
        }
      }
    }

    //get all entities
    visible_entities = *(control->entities->subscribeToEntityCreation(nullptr));
    detectionArray = new object_detection_control::Detection3DArray(visible_entities.size());

    LOG_DEBUG_S << "EntityFakeDetection"<< "Task started!";

    return true;
  }

  void EntityFakeDetection::updateHook()
  {
      EntityFakeDetectionBase::updateHook();

      if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
      LOG_DEBUG_S << "EntityFakeDetection"<< "Running updateHook!";

      if (use_camera == true) { // then fill the buffer only with the entities seen by the camera
        camera->getEntitiesInView(visible_entities, minVisibleVertices);
        if (visible_entities.size()!=detectionArray->detections.size()) {
          delete detectionArray;
          detectionArray = new object_detection_control::Detection3DArray(visible_entities.size());
        }
      } else {
        visible_entities = *(control->entities->subscribeToEntityCreation(nullptr));
        //frame_id = FrameId::GLOBAL;
        minVisibleVertices = 0;
      }

      LOG_DEBUG_S<<"EntityFakeDetection"<< "got " << visible_entities.size() << " entities updateHook()";
      //set general header
      detectionArray->header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
      detectionArray->header.seq = seq++;
      detectionArray->header.frame_id = (int) frame_id;
      //generate detections
      utils::Vector center;
      utils::Quaternion rotation;
      unsigned int i = 0;
      for (std::map<unsigned long, sim::SimEntity*>::iterator iter = visible_entities.begin();
          iter != visible_entities.end(); ++iter) {
        LOG_DEBUG_S << "detecting";
        //Header
        detectionArray->detections[i].header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
        detectionArray->detections[i].header.seq = seq++;
        detectionArray->detections[i].header.frame_id = (int) frame_id;
        //BoundingBox3D
        iter->second->getBoundingBox(center,
                                     rotation,
                                     detectionArray->detections[i].bbox.size);

        // pose of the object relative to the camera
        // otherwise the object pose is relative to the world
        if (frame_id == FrameId::CAMERA) {

          // since the name is unique, it should be only one camera node
          int idx = 0;
          std::vector<interfaces::NodeId>  camera_nodes = control->nodes->getNodeIDs(camera_link_name);
          
          if (camera_nodes.size() == 0) {
            LOG_ERROR_S << "No camera node with the name " << camera_link_name << " was found.";
            return;
          }
          if (camera_nodes.size() > 1) {
            LOG_ERROR_S << "There are several nodes with the name " << camera_link_name << ".";
            return;
          }

          NodeId camera_id = camera_nodes.at(idx);
          utils::Vector camera_pos = control->nodes->getPosition(camera_id);
          utils::Quaternion camera_orient = control->nodes->getRotation(camera_id);

          // camera pose in world cs
          Eigen::Affine3d camera_tf;
          camera_tf.setIdentity();
          camera_tf.rotate(camera_orient);
          camera_tf.translation() = camera_pos;

          // object pose in world cs
          Eigen::Affine3d node_tf;
          node_tf.setIdentity();
          node_tf.rotate(rotation);
          node_tf.translation() = center;          

          // convert position of object from world cs into camera cs
          Eigen::Affine3d node_in_camera_cs = camera_tf.inverse() * node_tf;

          center = node_in_camera_cs.translation();
          rotation = base::Quaterniond(node_in_camera_cs.rotation());

          /*//todo: this should be done in the rock camera frame, not the camera coordinates of the mars camera sensor!
          cameraStruct cs;
          camera->getCameraInfo(&cs);

          //todo: this is not correct. Test it or delete
          center = center-cs.pos;          
          if (_use_camera_coordinate_system.get()) {
            //this works
            // if camera coordinate system should be used, we need to transform center and rotation by inverse of camera rotation
            center = cs.rot.inverse() * center; //-(cs.rot.inverse() * cs.pos) + cs.rot.inverse() * center;
            rotation = cs.rot.inverse() * rotation;
          }*/
        }

        float pos_noise = _pos_noise.get();
        if (pos_noise > 0.0) {
          LOG_DEBUG_S << "pos = " << center.transpose() << std::endl;
          float noise_x = pos_noise * (2.0 * float(std::rand()) / float(RAND_MAX) - 1);
          float noise_y = pos_noise * (2.0 * float(std::rand()) / float(RAND_MAX) - 1);
          float noise_z = pos_noise * (2.0 * float(std::rand()) / float(RAND_MAX) - 1);
          center = utils::Vector(center.x() + noise_x, center.y() + noise_y, center.z() + noise_z);
          LOG_DEBUG_S << "pos with noise = " << center.transpose() << std::endl;
        }
        float rot_deg_noise = _rot_deg_noise.get();
        if (rot_deg_noise > 0.0) {
          // add noise to quaternion 
          Eigen::Vector3d random_axis = Eigen::Vector3d::Random();
          random_axis.normalize();
          float random_angle = rot_deg_noise * (2.0 * float(std::rand()) / float(RAND_MAX) - 1.0) * M_PI/180.0;
          Eigen::Quaterniond noise_q(Eigen::AngleAxisd(random_angle, random_axis));
          LOG_DEBUG_S << "rotation = " << rotation.coeffs().transpose() << std::endl;
          rotation = rotation * noise_q;
          LOG_DEBUG_S << "rotation with noise = " << rotation.coeffs().transpose() << std::endl;
        }

        detectionArray->detections[i].bbox.center.position = center;
        detectionArray->detections[i].bbox.center.orientation = rotation;
        //ObjectHypothesisWithPose
        detectionArray->detections[i].results[0].id = iter->first;
        detectionArray->detections[i].results[0].type = type_prefix + iter->second->getName();
        detectionArray->detections[i].results[0].pose.pose.position = center;
        detectionArray->detections[i].results[0].pose.pose.orientation = rotation;
        //PointCloud
        detectionArray->detections[i].source_cloud.header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
        detectionArray->detections[i].source_cloud.header.seq = seq++;
        detectionArray->detections[i].source_cloud.header.frame_id = (int) frame_id;
        //TODO: looks like mars does not have pointclouds stored by default.
        // how would you get those?
        long unsigned int rootId = iter->second->getRootestId();
        const snmesh& mesh = control->nodes->getFullNode(rootId).mesh;
        detectionArray->detections[i].source_cloud.width = mesh.vertexcount;
        const mydVector3* vertices = mesh.vertices;
        for (unsigned int v=0; v<mesh.vertexcount; ++v) {
          base::Vector3d p(vertices[v][0], vertices[v][1], vertices[v][2]);
          detectionArray->detections[i].source_cloud.points.push_back(p);
        }
        i++;
      }

      //write to rock outputs
      _detectionArray.write(*detectionArray);
  }
  // void EntityFakeDetection::errorHook()
  // {
  //     EntityFakeDetectionBase::errorHook();
  // }
  void EntityFakeDetection::stopHook()
  {
      if (use_camera == true) {
        EntityFakeDetectionBase::stopHook();
      }
      mars::Plugin::stopHook();

  }
  // void EntityFakeDetection::cleanupHook()
  // {
  //     EntityFakeDetectionBase::cleanupHook();
  // }

  void EntityFakeDetection::getData()
  {
  }
}
