#include <iostream>
#include <marsim_render/config.hpp>
#include <marsim_render/marsim_render.hpp>

int main() {
    // disable PCL warning
    pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ERROR);
    std::string cfg_path(CONFIG_FILE_DIR("general_360_lidar.yaml"));
    marsim::MarsimRender::Ptr render_ptr = std::make_shared<marsim::MarsimRender>(cfg_path);

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
