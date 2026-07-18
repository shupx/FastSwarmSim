#ifndef FASTSWARM_SENSING_LOCAL_POINTCLOUD_SIM_CONFIG_HPP_
#define FASTSWARM_SENSING_LOCAL_POINTCLOUD_SIM_CONFIG_HPP_

#include <string>

#include "marsim_render/yaml_loader.hpp"

namespace fss_sensing
{

class LocalPointCloudSimConfig
{
public:
  LocalPointCloudSimConfig() = default;

  explicit LocalPointCloudSimConfig(const std::string & cfg_path)
  {
    yaml_loader::YamlLoader loader(cfg_path);
    loader.LoadParam("pose_topic", pose_topic, std::string("mavros/local_position/pose"), false);
    loader.LoadParam("odom_topic", odom_topic, std::string("mavros/local_position/odom"), false);
    loader.LoadParam("use_odom", use_odom, false, false);
    loader.LoadParam("local_pc_topic", local_pc_topic, std::string("cloud_registered"), false);
    loader.LoadParam("global_pc_topic", global_pc_topic, std::string("global_pc"), false);
    loader.LoadParam("frame_id", frame_id, std::string("map"), false);
    loader.LoadParam("sensing_rate", sensing_rate, 10, false);
  }

  std::string pose_topic{"mavros/local_position/pose"};
  std::string odom_topic{"mavros/local_position/odom"};
  bool use_odom{false};
  std::string local_pc_topic{"cloud_registered"};
  std::string global_pc_topic{"global_pc"};
  std::string frame_id{"map"};
  int sensing_rate{10};
};

}  // namespace fss_sensing

#endif  // FASTSWARM_SENSING_LOCAL_POINTCLOUD_SIM_CONFIG_HPP_
