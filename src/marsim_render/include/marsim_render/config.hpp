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

#ifndef MARSIM_RENDER_CONFIG_HPP
#define MARSIM_RENDER_CONFIG_HPP

#include <marsim_render/yaml_loader.hpp>
#include <cmath>

#define CONFIG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/"+(name)))
#define PCD_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "pcd/"+(name)))
#define PATTERN_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/pattern/"+(name)))


namespace marsim {
    using decimal_t = float;

    enum LiDARType {
        UNDEFINED = 0,
        AVIA = 1,
        GENERAL_360 = 2,
        MID_360 = 3
    };


    class Config {
    public:
        Config() = default;

        explicit Config(const std::string& cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            // required parameters
            loader.LoadParam("sensing_blind", sensing_blind, 0.1f, true);
            loader.LoadParam("sensing_horizon", sensing_horizon, 10.0f, true);
            loader.LoadParam("sensing_rate", sensing_rate, 10, true);
            loader.LoadParam("lidar_type", lidar_type, 0, true);
            loader.LoadParam("pcd_name", pcd_name, std::string{}, false);
            pcd_name = PCD_FILE_DIR(pcd_name);
            loader.LoadParam("is_360lidar", is_360lidar, true, false);
            loader.LoadParam("polar_resolution", polar_resolution, 0.1f, false);
            loader.LoadParam("fx", fx, 250.0f, false);
            loader.LoadParam("fy", fy, 250.0f, false);
            loader.LoadParam("line_number", line_number, 64, false);
            loader.LoadParam("downsample_res", downsample_res, 0.05f, false);
            loader.LoadParam("yaw_fov", yaw_fov, 360.0f, false);
            loader.LoadParam("vertical_fov", vertical_fov, 77.4f, false);
            loader.LoadParam("depth_image_en", depth_image_en, false, false);
            loader.LoadParam("inf_point_en", inf_point_en, false, false);
            loader.LoadParam("print_time_consumption", print_time_consumption, false, false);

            effect_range = cover_dis / sin(0.5 * polar_resolution * M_PI / 180.0);
            width = is_360lidar ? ceil(360.0 / polar_resolution) : ceil(yaw_fov / polar_resolution);
            height = ceil(vertical_fov / polar_resolution);
        }

        int line_number = 64;
        bool depth_image_en = false;
        bool is_360lidar = false;
        bool inf_point_en = false;
        bool print_time_consumption = false;
        decimal_t polar_resolution;
        decimal_t fx = 250, fy = 250;
        decimal_t downsample_res = 0.01;
        decimal_t yaw_fov = 70.4;
        decimal_t vertical_fov = 77.4;
        decimal_t sensing_blind = 0.1;
        decimal_t sensing_horizon = 10;
        decimal_t cover_dis = 0.55 * 1.7321 * downsample_res;
        decimal_t effect_range = 0.0;
        decimal_t polar_resolution_max = 1.0 / fx * (cos(1) * cos(1)) * 180 / M_PI;
        int sensing_rate = 10;
        int lidar_type = LiDARType::UNDEFINED;
        std::string pcd_name{};

        int width = 350;
        int height = 350;
        decimal_t near = 0.1;
        decimal_t far = 30.0;
        decimal_t u0 = width * 0.5f;
        decimal_t v0 = width * 0.5f;
    };
}

#endif
