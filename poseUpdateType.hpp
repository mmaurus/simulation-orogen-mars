#ifndef MARS_POSITION_UPDATE_TYPE_HH
#define MARS_POSITION_UPDATE_TYPE_HH

#include <vector>
#include <string>

#include <base/Pose.hpp>

namespace mars 
{

struct PoseUpdate {
	std::string entity_name;
	base::Pose pose;
};

typedef std::vector<PoseUpdate> PoseUpdates;

}
#endif
