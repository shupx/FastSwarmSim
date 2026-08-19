#include <iostream>
#include <marsim_render/marsim_render.hpp>

int main(int argc, char ** argv) {
    // disable PCL warning
    pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ERROR);
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <config> <pcd_dir> <shader_dir>" << std::endl;
        return 1;
    }
    marsim::ResourcePaths resources{argv[2], argv[3]};
    marsim::MarsimRender::Ptr render_ptr =
        std::make_shared<marsim::MarsimRender>(argv[1], std::move(resources));

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
