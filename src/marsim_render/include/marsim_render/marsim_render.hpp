/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
// PCL 相关头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

// OpenGL 相关头文件
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// GLM 数学库
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// OpenCV 相关头文件
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// YAML
#include <yaml-cpp/yaml.h>

// C++ 标准库头文件
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <numeric>
#include <tr1/unordered_map>

// 系统头文件
#include <unistd.h>
#include <random>

// 自定义头文件
#include "marsim_render/config.hpp"
#include "marsim_render/shader_m.h"
#include "marsim_render/scope_timer.hpp"
#include "marsim_render/yaml_loader.hpp"

namespace marsim {

#define CLAMP(value, min, max) \
((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

    template<typename T>
    using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
    using Vec3f = Eigen::Matrix<decimal_t, 3, 1>;
    using Mat3f = Eigen::Matrix<decimal_t, 3, 3>;
    typedef pcl::PointXYZI PointType;


    struct BoxPointType {
        float vertex_min[3];
        float vertex_max[3];
    };


    class MarsimRender {
        Config cfg_;
        std::default_random_engine eng;
        std::uniform_real_distribution<decimal_t> uniform_01;
        std:: vector<GLfloat> red_pixels, mypixels;


    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit MarsimRender(const std::string& cfg_path_) {
            cfg_ = Config(cfg_path_);
            uniform_01 = std::uniform_real_distribution<decimal_t>(0, 1);
            eng = std::default_random_engine(419);

            red_pixels.resize(cfg_.width * cfg_.height);
            mypixels.resize(cfg_.width * cfg_.height);

            read_pointcloud_fromfile(cfg_.pcd_name);
            this->camera2world = Eigen::Matrix3f::Identity();
            this->camera = Eigen::Vector3f(0,0,0);
            this->camera_pos_world = Eigen::Vector3f(0,0,0);
            // this->camera_pos_world = Eigen::Vector3f(0, 0, 0);

            if (cfg_.lidar_type != UNDEFINED) {
                // this->pattern_matrix = Eigen::MatrixXf::Zero(height,width);
                // this->pattern_matrix.resize(width,height);
                // this->pattern_matrix.setZero();
                this->pattern_matrix = Eigen::MatrixXf::Zero(cfg_.width, cfg_.height);
            }
            else {
                // this->pattern_matrix = Eigen::MatrixXf::One(height,width);
                // this->pattern_matrix.resize(width,height);
                // this->pattern_matrix.setConstant(1);
                this->pattern_matrix = Eigen::MatrixXf::Zero(cfg_.width, cfg_.height);
                this->pattern_matrix.setConstant(1);
            }
        };


        ~MarsimRender();

        typedef std::shared_ptr<MarsimRender> Ptr;

        void getGlobalMap(pcl::PointCloud<PointType>::Ptr & global_map) {
            global_map = cloud_color_mesh.makeShared();
        }

        void renderOnceInWorld(const Eigen::Vector3f& camera_pos,
                               const Eigen::Quaternionf& camera_q,
                               const decimal_t& t_pattern_start,
                               pcl::PointCloud<PointType>::Ptr output_pointcloud) {
            render_pointcloud(camera_pos, camera_q, t_pattern_start, output_pointcloud);
        }

        void renderOnceInBody(const Eigen::Vector3f& camera_pos,
                               const Eigen::Quaternionf& camera_q,
                               const decimal_t& t_pattern_start,
                               pcl::PointCloud<PointType>::Ptr point_in_sensor) {
            pcl::PointCloud<PointType>::Ptr local_map(new pcl::PointCloud<PointType>);
            render_pointcloud(camera_pos, camera_q, t_pattern_start, local_map);
            Mat3f rot(camera_q);
            Eigen::Matrix4f sensor2world;
            sensor2world <<
                  rot(0,0), rot(0,1), rot(0,2), camera_pos.x(),
                  rot(1,0),rot(1,1), rot(1,2), camera_pos.y(),
                  rot(2,0), rot(2,1),rot(2,2), camera_pos.z(),
                  0,0,0,1;
            Eigen::Matrix4f world2sensor;
            world2sensor = sensor2world.inverse();

            //body frame pointcloud
            point_in_sensor->points.clear();
            // pcl::copyPointCloud(local_map_filled, point_in_sensor);
            pcl::transformPointCloud(*local_map, *point_in_sensor, world2sensor);
        }


    private:
        glm::mat4 projection{}, view{};
        unsigned int VBO{}, VAO{}, EBO{};
        GLFWwindow* window{};
        Shader ourShader;

        Eigen::MatrixXf pattern_matrix;
        Eigen::MatrixXf density_matrix;
        cv::Mat density_mat;

        decimal_t hash_cubesize = 5.0;

        Eigen::Matrix3f camera2world, body2world;
        Eigen::Vector3f camera;
        Eigen::Vector3f camera_pos_world;

        pcl::PointCloud<PointType> cloud_color_mesh;
        vec_E<Vec3f> depth_ptcloud_vec;
        vec_E<Vec3f> g_eigen_pt_vec;
        std::vector<unsigned int> points_index_infov;
        pcl::PointCloud<pcl::PointXYZI> dyn_clouds;

        void read_pointcloud_fromfile(const std::string& map_filename);

        void render_pointcloud(const Eigen::Vector3f& camera_pos,
                               const Eigen::Quaternionf& camera_q,
                               const decimal_t& t_pattern_start,
                               pcl::PointCloud<PointType>::Ptr output_pointcloud);

        void input_dyn_clouds(pcl::PointCloud<pcl::PointXYZI> input_cloud);


        void load_pcd_file(std::string file_name, pcl::PointCloud<PointType>& cloud_color_mesh);

        void init_pointcloud_data();

        cv::Mat colormap_depth_img(cv::Mat & depth_mat);

        void render_dynclouds_on_depthimage(cv::Mat& depth_image);

        void depth_to_pointcloud(cv::Mat& depth_image, pcl::PointCloud<PointType>::Ptr origin_cloud,
                                 vec_E<Vec3f>& depth_ptcloud_vec);

        void read_depth(int depth_buffer_precision, vec_E<Vec3f>& depth_ptcloud_vec,
                pcl::PointCloud<PointType>::Ptr rendered_cloud);
    };
};
