#ifndef MULTI_ROBOT_PLAYGROUND_COMMON__UTILS_HPP_
#define MULTI_ROBOT_PLAYGROUND_COMMON__UTILS_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"

namespace mrp_common
{
  class TransformUtils
  {
  public:
    static tf2::Transform toLocalFrame(const tf2::Transform base_frame,
                                       const tf2::Transform child_frame);
  };

  class GeometryUtils
  {
  public:
    static double euclideanDistance(const geometry_msgs::msg::Pose &first,
                                    const geometry_msgs::msg::Pose &second);
    static double yawFromPose(const geometry_msgs::msg::Pose &pose);
  };
}

#endif