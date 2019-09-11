#ifndef MARS_OBJECT_DETECTION_TYPES_HH
#define MARS_OBJECT_DETECTION_TYPES_HH

#include <vector>
#include <string>
#include <base/Pose.hpp>
#include <base/Time.hpp>
// TODO 1 #include <mars/interfaces/snmesh.h>


namespace mars
{

  struct Header {
    base::Time stamp;
    unsigned int seq = 0;
    std::string frame_id = "1";
  };

  struct PoseWithCovariance{
    base::Pose pose;
    float covariance[36] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  };

  struct ObjectHypothesisWithPose {
    long int id;
    float score = 1.0;
    std::string type;
    PoseWithCovariance pose;
  };

  struct BoundingBox3D {
    base::Pose center;
    base::Vector3d size;
  };

  struct PointCloud {
    Header header;

    unsigned int width;

    std::vector<base::Vector3d> points;
    std::vector<base::Vector4d> colors;

    /* TODO 1
    void init(interfaces:::snmesh mesh) {
      for (mesh.vertices)
    }
    */
  };

  struct Detection3D {
    Header header;
    ObjectHypothesisWithPose results[1];
    BoundingBox3D bbox;
    PointCloud source_cloud;
  };

  struct Detection3DArray {
    Header header;
    std::vector<Detection3D> detections;
    Detection3DArray(unsigned int size) : detections(size) {};
    Detection3DArray() : detections(10) {};
  };

}

// need to tell the stream_aligner/transformer that the timestamp of these types
// is hidden in "header.stamp", instead of the default "time" member variable
namespace aggregator {
    inline base::Time determineTimestamp(const mars::Detection3DArray& sample)
    {
        return sample.header.stamp;
    }

    inline base::Time determineTimestamp(const mars::Detection3D& sample)
    {
        return sample.header.stamp;
    }

    inline base::Time determineTimestamp(const mars::PointCloud& sample)
    {
        return sample.header.stamp;
    }

    inline base::Time determineTimestamp(const mars::Header& sample)
    {
        return sample.stamp;
    }
}


#endif
