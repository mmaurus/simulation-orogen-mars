/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "ObjectHighlighter.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/JointManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/sim/SimEntity.h>

#include <base-logging/Logging.hpp>


namespace mars {

  using namespace interfaces;
  using namespace utils;


  ObjectHighlighter::ObjectHighlighter(std::string const& name)
      : ObjectHighlighterBase(name)
  {
  }

  ObjectHighlighter::ObjectHighlighter(std::string const& name, RTT::ExecutionEngine* engine)
      : ObjectHighlighterBase(name, engine)
  {
  }

  ObjectHighlighter::~ObjectHighlighter()
  {
  }

  void ObjectHighlighter::init()
  {
  }

  void ObjectHighlighter::update(double delta_t)
  {
  }

  /// The following lines are template definitions for the various state machine
  // hooks defined by Orocos::RTT. See ObjectHighlighter.hpp for more detailed
  // documentation about them.

  bool ObjectHighlighter::configureHook()
  {
    if (! ObjectHighlighterBase::configureHook()) return false;
    
    return true;
  }

  bool ObjectHighlighter::startHook()
  {
    return ObjectHighlighterBase::startHook();
  }

  void ObjectHighlighter::updateHook()
  {
    ObjectHighlighterBase::updateHook();

    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

    //if triggered load new position
    short unsigned int old_id = object_id;
    if (_object_id.readNewest(object_id) == RTT::NewData)
    {
      LOG_DEBUG_S << "ObjectHighlighter"<< "updateHook triggered!";
      
      mars::interfaces::GraphicsManagerInterface* graphics = control->graphics;
      
      if (graphics) {
        graphics->setDrawObjectSelected(old_id, false);
        graphics->setDrawObjectSelected(object_id, true);
      }
            
      LOG_DEBUG_S << "ObjectHighlighter"<< "Object highlighting done!";
    }
  }

  // void ObjectHighlighter::errorHook()
  // {
  //     ObjectHighlighterBase::errorHook();
  // }

  void ObjectHighlighter::stopHook()
  {
    ObjectHighlighterBase::stopHook();
  }

  // void ObjectHighlighter::cleanupHook()
  // {
  //     ObjectHighlighterBase::cleanupHook();
  // }
}

