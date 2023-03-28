#ifndef __TAG_MASTER_TAG_DESCRIPTIONS_H_
#define __TAG_MASTER_TAG_DESCRIPTIONS_H_

#include <string>
#include <geometry_msgs/Transform.h>

struct TagDescription
{
  int id;
  std::string type;
  std::string pub_frame;
  std::string obj_name;
  geometry_msgs::Transform objtransform;
  double tag_size_meters;
};

#endif