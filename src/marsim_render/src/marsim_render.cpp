#include <utility>

#include "marsim_render/marsim_render.hpp"

namespace marsim {
    using namespace std::chrono;
    using namespace cv;
    using std::cout;
    using std::endl;
    using PointType = pcl::PointXYZI;

    void MarsimRender::input_dyn_clouds(pcl::PointCloud<pcl::PointXYZI> input_cloud) {
        this->dyn_clouds = std::move(input_cloud);
    }

    void MarsimRender::read_pointcloud_fromfile(const std::string& map_filename) {
        // glfw: initialize and configure
        // -----------------------------
        if(map_filename.empty()) {
            std::cerr << "Map file name is empty" << std::endl;
            return;
        }

        if (!glfwInit()) {
            std::cerr << "Failed to initialize GLFW" << std::endl;
            return;
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        glfwWindowHint(GLFW_VISIBLE, cfg_.depth_image_en ? GL_TRUE : GL_FALSE); // invisible window

        // glfw window creation
        // --------------------
        window = glfwCreateWindow(cfg_.width, cfg_.height, "Opengl_sim", nullptr, nullptr);
        if (window == nullptr) {
            std::cout << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return;
        }

        glfwMakeContextCurrent(window);
        // glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
        // glfwSetCursorPosCallback(window, mouse_callback);
        // glfwSetScrollCallback(window, scroll_callback);

        // tell GLFW to capture our mouse
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        // glad: load all OpenGL function pointers
        // ---------------------------------------
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            std::cout << "Failed to initialize GLAD" << std::endl;
            return;
        }
        else {
            std::cout << "GLAD init success" << std::endl;
        }

        // configure global opengl state
        // -----------------------------
        glEnable(GL_DEPTH_TEST);
        GLenum error = glGetError();
        if (error != GL_NO_ERROR) {
            std::cerr << "Error enabling GL_DEPTH_TEST: " << error << std::endl;
        }

        glEnable(GL_PROGRAM_POINT_SIZE);
        error = glGetError();
        if (error != GL_NO_ERROR) {
            std::cerr << "Error enabling GL_PROGRAM_POINT_SIZE: " << error << std::endl;
        }

        // glEnable(GL_POINT_SMOOTH);
        // error = glGetError();
        // if (error != GL_NO_ERROR) {
        //     std::cerr << "Error enabling GL_POINT_SMOOTH: " << error << std::endl;
        // }

        // build and compile our shader zprogram
        // ------------------------------------
        // //find current program path
        // char* cwd = NULL;
        // cwd = get_current_dir_name();
        // std::string current_path = cwd;
        // //delete one folder from current path
        // // current_path = current_path.substr(0, current_path.find_last_of("/"));
        // // current_path += "/kong_ws/src/Exploration_sim/uav_simulator/local_sensing/include/";
        const auto vertex_path = PATTERN_FILE_DIR("360camera.vs");
        const auto fragment_path = PATTERN_FILE_DIR("camera.fs");
        std::cout << "Load shader frome: " << vertex_path << std::endl;;
        ourShader = Shader(vertex_path.c_str(), fragment_path.c_str());

        // std::cout << "You Pushed a button, now open file: " << a_string.Get() << endl;
        load_pcd_file(map_filename, cloud_color_mesh);

        //downsample pointcloud
        // pcl::VoxelGrid<PointType> voxel_grid;
        // voxel_grid.setInputCloud(cloud_color_mesh.makeShared());
        // voxel_grid.setLeafSize(cfg_.downsample_res, cfg_.downsample_res, cfg_.downsample_res);
        // voxel_grid.filter(cloud_color_mesh);

        //Map process
        init_pointcloud_data();
        // preprocess(cloud_color_mesh,all_normals);
        cout << "Load data finish " << endl;

        // unsigned int VBO, VAO;
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(decimal_t) * 3 * (g_eigen_pt_vec.size() + 0), g_eigen_pt_vec.data(),
                     GL_DYNAMIC_DRAW);


        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(decimal_t), (void*)0);
        glEnableVertexAttribArray(0);
        // texture coord attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(decimal_t), (void*)(3 * sizeof(decimal_t)));
        glEnableVertexAttribArray(1);

        // glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * (points_index_infov.size() + 0),
                     points_index_infov.data(), GL_DYNAMIC_DRAW);


        ourShader.use();


        // 查询并打印 OpenGL 设备信息
        const char* vendor = (const char*)glGetString(GL_VENDOR);
        const char* renderer = (const char*)glGetString(GL_RENDERER);
        const char* version = (const char*)glGetString(GL_VERSION);
        const char* shadingLanguageVersion = (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION);

        std::cout << "========== OpenGL Info ==========" << std::endl;
        std::cout << "Vendor: " << vendor << std::endl;
        std::cout << "Renderer: " << renderer << std::endl;
        std::cout << "OpenGL Version: " << version << std::endl;
        std::cout << "GLSL Version: " << shadingLanguageVersion << std::endl;
        std::cout << "========== OpenGL Info ==========" << std::endl;
    }

    MarsimRender::~MarsimRender() {
        // 确保 OpenGL 上下文有效
        if (glfwGetCurrentContext()) {
            if (VAO != 0) {
                glDeleteVertexArrays(1, &VAO);
                VAO = 0; // 防止重复删除
            }
            if (VBO != 0) {
                glDeleteBuffers(1, &VBO);
                VBO = 0; // 防止重复删除
            }
        }

        glDeleteBuffers(1, &EBO); // 在适当的地方释放

        // 终止 GLFW
        glfwTerminate();
    }


    void MarsimRender::render_pointcloud(
        const Eigen::Vector3f& camera_pos,
        const Eigen::Quaternionf& camera_q,
        const decimal_t& t_pattern_start,
        pcl::PointCloud<PointType>::Ptr output_pointcloud
    ) {
        ScopeTimer timer("TOTAL", cfg_.print_time_consumption);

        //avia pattern
        {
            ScopeTimer timer("Fill pattern matrix",  cfg_.print_time_consumption);

            /**
             * @bug The following AVIA pattern generation has a serious bug. The values become unstable as t_pattern_start increases.
             * This needs to be fixed in the future.
             * @author Peixuan Shu
             * @attention https://github.com/hku-mars/MARSIM/issues/31
             * @date 2025-12-21
             */
            // if (cfg_.lidar_type == AVIA) {
            //     pattern_matrix.setConstant(0);
            //     decimal_t w1 = 763.82589; //7294.0*2.0*3.1415926/60.0
            //     decimal_t w2 = -488.41293788; // -4664.0*2.0*3.1415926/60.0
            //     // int linestep = 2;
            //     // decimal_t point_duration = 0.000025;
            //     decimal_t point_duration = 0.000004167 * 6;

            //     decimal_t t_duration = 1.0 / cfg_.sensing_rate;
            //     decimal_t t_start = t_pattern_start;

            //     decimal_t scale_x = 0.48 * cfg_.width / 2.0;
            //     decimal_t scale_y = 0.43 * cfg_.height / 2.0;

            //     int linestep = ceil(1.4 / 0.2);
            //     // std::cout << "linestep = " << linestep << std::endl;

            //     for (decimal_t t_i = t_start; t_i < t_start + t_duration; t_i = t_i + point_duration) {
            //         int x = round(scale_x * (cos(w1 * t_i) + cos(w2 * t_i))) + round(0.5 * cfg_.width);
            //         int y = round(scale_y * (sin(w1 * t_i) + sin(w2 * t_i))) + round(0.5 * cfg_.height);

            //         if (x > (cfg_.width - 1)) {
            //             x = (cfg_.width - 1);
            //         }
            //         else if (x < 0) {
            //             x = 0;
            //         }

            //         if (y > (cfg_.height - 1)) {
            //             y = (cfg_.height - 1);
            //         }
            //         else if (y < 0) {
            //             y = 0;
            //         }

            //         pattern_matrix(x, y) = 2; //real pattern
            //         pattern_matrix(x, y + linestep) = 2;
            //         pattern_matrix(x, y + 2 * linestep) = 2;
            //         pattern_matrix(x, y + 3 * linestep) = 2;
            //         pattern_matrix(x, y - linestep) = 2;
            //         pattern_matrix(x, y - 2 * linestep) = 2;
            //     }
            // }

            /**
             * Fix the above AVIA pattern generation bug.
             * @author Peixuan Shu
             * @date 2025-12-21
             */
            if (cfg_.lidar_type == AVIA) {
                pattern_matrix.setConstant(0);

                using std::cos;
                using std::sin;
                using std::round;
                using std::fmod;
                using std::ceil;

                const decimal_t PI = 3.141592653589793;

                // ================= 参数 =================
                const decimal_t w1 = 763.82589;
                const decimal_t w2 = -488.41293788;

                const decimal_t point_duration = 0.000004167 * 6;
                const decimal_t t_duration     = 1.0 / cfg_.sensing_rate;

                const int64_t N = static_cast<int64_t>(t_duration / point_duration);

                const decimal_t scale_x = 0.48 * cfg_.width  / 2.0;
                const decimal_t scale_y = 0.43 * cfg_.height / 2.0;

                const int linestep = static_cast<int>(ceil(1.4 / 0.2));

                // ================= 周期 =================
                const decimal_t T1 = 2 * PI / std::abs(w1);
                const decimal_t T2 = 2 * PI / std::abs(w2);

                // ================= 初始相位（由起始时刻决定） =================
                decimal_t phase1 = w1 * fmod(t_pattern_start, T1);
                decimal_t phase2 = w2 * fmod(t_pattern_start, T2);

                // ================= 每步相位增量 =================
                const decimal_t dphase1 = w1 * point_duration;
                const decimal_t dphase2 = w2 * point_duration;

                // ================= 主循环 =================
                for (int64_t i = 0; i < N; ++i) {

                    int x = static_cast<int>(
                                round(
                                    scale_x * (cos(phase1) + cos(phase2))
                                )
                            ) + static_cast<int>(round(0.5 * cfg_.width));

                    int y = static_cast<int>(
                                round(
                                    scale_y * (sin(phase1) + sin(phase2))
                                )
                            ) + static_cast<int>(round(0.5 * cfg_.height));

                    // ===== 边界限制 =====
                    if (x < 0) x = 0;
                    else if (x > cfg_.width - 1) x = cfg_.width - 1;

                    if (y < 0) y = 0;
                    else if (y > cfg_.height - 1) y = cfg_.height - 1;

                    // ===== 写 pattern =====
                    pattern_matrix(x, y) = 2;
                    pattern_matrix(x, y + linestep)     = 2;
                    pattern_matrix(x, y + 2 * linestep) = 2;
                    pattern_matrix(x, y + 3 * linestep) = 2;
                    pattern_matrix(x, y - linestep)     = 2;
                    pattern_matrix(x, y - 2 * linestep) = 2;

                    // ===== 相位更新 =====
                    phase1 += dphase1;
                    phase2 += dphase2;

                    if (phase1 >= 2 * PI) phase1 -= 2 * PI;
                    if (phase1 <  0)      phase1 += 2 * PI;

                    if (phase2 >= 2 * PI) phase2 -= 2 * PI;
                    if (phase2 <  0)      phase2 += 2 * PI;
                }
            }

            else if (cfg_.lidar_type == GENERAL_360) {
                pattern_matrix.setConstant(0);
                double step = static_cast<double>(cfg_.height) / 128.0;
                double random_offset = step * uniform_01(eng);
                for (size_t i = 0; i < 128; i++) {
                    int y = round(i * step + random_offset);
                    y = CLAMP(y, 0, cfg_.height-1);
                    for (int j = 0; j < cfg_.width; j++) {
                        pattern_matrix(j, y) = 2;
                    }
                }
            }

            /**
             * @bug The following MID_360 pattern generation has a serious bug. The values become unstable as t_pattern_start increases.
             * This needs to be fixed.
             * @author Peixuan Shu
             * @attention https://github.com/hku-mars/MARSIM/issues/31
             * @date 2025-12-21
             */
            // else if (cfg_.lidar_type == MID_360) {
            //     pattern_matrix.setConstant(0);
            //     decimal_t point_duration = 1.0 / 200000.0;

            //     decimal_t t_duration = 1.0 / cfg_.sensing_rate;
            //     decimal_t t_start = t_pattern_start;

            //     //        decimal_t scale_x = 0.48 * cfg_.width / 2.0;
            //     //        decimal_t scale_y = 0.43 * cfg_.height / 2.0;
            //     decimal_t PI = 3.141519265357; // Wrong PI, Peixuan Shu 2025-12-21

            //     for (decimal_t t_i = t_start; t_i < t_start + t_duration; t_i = t_i + point_duration) {
            //         int x = (int(-round(-62050.63 * t_i + 3.11 * cos(314159.2 * t_i) * sin(628.318 * 2 * t_i))) % 360) /
            //             cfg_.polar_resolution;
            //         int y = round(
            //                 25.5 * cos(20 * PI * t_i) + 4 * cos(2 * PI / 0.006 * t_i) * cos(10000 * PI * t_i) + 22.5) /
            //             cfg_.polar_resolution + round(0.5 * cfg_.height);

            //         // ROS_INFO("X = %d, Y = %d",x,y);
            //         if (x > (cfg_.width - 1)) {
            //             x = (cfg_.width - 1);
            //         }
            //         else if (x < 0) {
            //             x = 0;
            //         }

            //         if (y > (cfg_.height - 1)) {
            //             y = (cfg_.height - 1);
            //         }
            //         else if (y < 0) {
            //             y = 0;
            //         }

            //         pattern_matrix(x, y) = 2; // real pattern
            //     }
            // }

            /**
             * Fix the above MID_360 pattern generation bug.
             * @author Peixuan Shu
             * @date 2025-12-21
             */
            else if (cfg_.lidar_type == MID_360) {
                pattern_matrix.setConstant(0);

                using std::cos;
                using std::sin;
                using std::round;
                using std::fmod;

                const decimal_t PI = M_PI;

                // ================= 时间参数 =================
                const decimal_t point_duration = 1.0 / 200000.0;
                const decimal_t t_duration     = 1.0 / cfg_.sensing_rate;

                const int64_t N = static_cast<int64_t>(t_duration / point_duration);

                // ================= 角频率 =================
                const decimal_t w1 = 314159.2;
                const decimal_t w2 = 628.318 * 2.0;
                const decimal_t w3 = 20.0 * PI;
                const decimal_t w4 = 2.0 * PI / 0.006;
                const decimal_t w5 = 10000.0 * PI;

                // ================= 周期 =================
                const decimal_t T1 = 2 * PI / w1;
                const decimal_t T2 = 2 * PI / w2;
                const decimal_t T3 = 2 * PI / w3;
                const decimal_t T4 = 2 * PI / w4;
                const decimal_t T5 = 2 * PI / w5;

                // ================= 初始相位（由起始时刻决定） =================
                decimal_t phase1 = w1 * fmod(t_pattern_start, T1);
                decimal_t phase2 = w2 * fmod(t_pattern_start, T2);
                decimal_t phase3 = w3 * fmod(t_pattern_start, T3);
                decimal_t phase4 = w4 * fmod(t_pattern_start, T4);
                decimal_t phase5 = w5 * fmod(t_pattern_start, T5);

                // ================= 每步相位增量 =================
                const decimal_t dphase1 = w1 * point_duration;
                const decimal_t dphase2 = w2 * point_duration;
                const decimal_t dphase3 = w3 * point_duration;
                const decimal_t dphase4 = w4 * point_duration;
                const decimal_t dphase5 = w5 * point_duration;

                // ================= x 线性项（只取 360 周期） =================
                const decimal_t Tx = 360.0 / 62050.63;
                decimal_t x_linear =
                    -62050.63 * fmod(t_pattern_start, Tx);

                const decimal_t x_slope = -62050.63 * point_duration;

                // ================= 主循环 =================
                for (int64_t i = 0; i < N; ++i) {

                    decimal_t x_val =
                        x_linear
                        + 3.11 * cos(phase1) * sin(phase2);

                    int x = static_cast<int>(-round(x_val));

                    x %= 360;
                    if (x < 0) x += 360; // 规范化到 [0, 360)，否则x为负数时 x %= 360也是个负数

                    x /= cfg_.polar_resolution;

                    int y = static_cast<int>(
                                round(
                                    25.5 * cos(phase3)
                                + 4.0  * cos(phase4) * cos(phase5)
                                + 22.5
                                )
                            ) / cfg_.polar_resolution
                            + static_cast<int>(round(0.5 * cfg_.height));

                    // ===== 写 pattern_matrix =====
                    // ROS_INFO("X = %d, Y = %d",x,y);
                    if (x > (cfg_.width - 1)) {
                        x = (cfg_.width - 1);
                    }
                    else if (x < 0) {
                        x = 0;
                    }

                    if (y > (cfg_.height - 1)) {
                        y = (cfg_.height - 1);
                    }
                    else if (y < 0) {
                        y = 0;
                    }

                    pattern_matrix(x, y) = 2; // real pattern

                    // ===== 相位更新 =====
                    x_linear += x_slope;

                    phase1 += dphase1; if (phase1 >= 2 * PI) phase1 -= 2 * PI;
                    phase2 += dphase2; if (phase2 >= 2 * PI) phase2 -= 2 * PI;
                    phase3 += dphase3; if (phase3 >= 2 * PI) phase3 -= 2 * PI;
                    phase4 += dphase4; if (phase4 >= 2 * PI) phase4 -= 2 * PI;
                    phase5 += dphase5; if (phase5 >= 2 * PI) phase5 -= 2 * PI;
                }
            }
        }


        //trans odom to matrix
        Eigen::Matrix3f body2world_matrix = camera_q.toRotationMatrix();
        body2world = body2world_matrix;
        glm::vec3 cameraPos = glm::vec3(camera_pos(0), camera_pos(1), camera_pos(2));
        camera = camera_pos;


        Eigen::Vector3f body_x, body_z;
        body_x << 1, 0, 0;
        body_z << 0, 0, 1;
        body_x = body2world_matrix * body_x;
        body_z = body2world_matrix * body_z;
        glm::vec3 cameraFront = glm::vec3(body_x(0), body_x(1), body_x(2));
        glm::vec3 cameraUp = glm::vec3(body_z(0), body_z(1), body_z(2));

        // std::cout << cameraUp.x << ", " << cameraUp.y << ", " << cameraUp.z << std::endl;
        // std::cout << cameraFront.x << ", " << cameraFront.y << ", " << cameraFront.z << std::endl;

        // render
        // ------
        if (!glfwGetCurrentContext()) {
            std::cerr << "OpenGL context is not current." << std::endl;
            return;
        }
        if (!glfwInit()) {
            std::cerr << "Failed to initialize GLFW." << std::endl;
            return;
        }
        GLenum err;
        while ((err = glGetError()) != GL_NO_ERROR) {
            std::cerr << "OpenGL error: " << err << std::endl;
        }

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // activate shader
        ourShader.use();

        // pass projection matrix to shader (note that in this case it could change every frame)
        projection = glm::perspective((decimal_t)atan(((decimal_t)cfg_.width) / (2 * cfg_.fy)) * 2,
                                      (decimal_t)cfg_.width / (decimal_t)cfg_.height, cfg_.sensing_blind,
                                      cfg_.sensing_horizon);
        ourShader.setMat4("projection", projection);

        // camera/view transformation
        // view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);//original licfg_.sensing_blind project
        view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        ourShader.setMat4("view", view);

        glm::vec2 sensing_range = glm::vec2(cfg_.sensing_blind, cfg_.sensing_horizon);
        ourShader.setVec2("range", sensing_range);

        glm::vec2 fov = glm::vec2(cfg_.yaw_fov, cfg_.vertical_fov);
        ourShader.setVec2("fov", fov);

        glm::vec2 res = glm::vec2(cfg_.downsample_res, cfg_.polar_resolution);
        ourShader.setVec2("res", res);

        glm::mat3 rotation_mat;
        Eigen::Matrix3f world2body = body2world_matrix.transpose(); //

        for (size_t i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rotation_mat[i][j] = world2body(j, i);
            }
        }

        // glm::mat3 rotation_mat = glm::make_mat3x3((body2world_matrix.transpose()));
        ourShader.setMat3("rot", rotation_mat);

        // glm::vec3 curpos = glm::vec3()
        ourShader.setVec3("pos", cameraPos);

        //translate matrix to eigen
        Eigen::Matrix4f eigen_view = Eigen::Matrix4f::Identity();
        for (size_t i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                eigen_view(i, j) = view[j][i];
            }
        }
        Eigen::Matrix4f eigen_proj = Eigen::Matrix4f::Identity();
        for (size_t i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                eigen_proj(i, j) = projection[i][j];
            }
        }
        camera2world = eigen_view.block<3, 3>(0, 0).transpose();
        // std::cout << eigen_view << std::endl;
        // std::cout << eigen_proj << std::endl;

        system_clock::time_point t0 = system_clock::now();

        glBindVertexArray(VAO);

        system_clock::time_point t1 = system_clock::now();

        const auto fov_check_dur = t1 - t0;

        //        duration<decimal_t> fovcheck_second(fov_check_dur);
        // std::cout << "FoV checker cost " << fovcheck_second.count() << " seconds" << std::endl;

        glPointSize(1);
        // glColor3f(1.0f,0.0f,0.0f);
        // glDrawArrays(GL_POINTS, 0, g_eigen_pt_vec.size()/2);
        // glDrawElements(GL_POINTS, g_eigen_pt_vec.size()/2, GL_UNSIGNED_INT, 0);
        glDrawElements(GL_POINTS, points_index_infov.size(), GL_UNSIGNED_INT, 0);

        glfwSwapBuffers(window);
        glfwPollEvents();

        read_depth(16, depth_ptcloud_vec,output_pointcloud);

        static int cnt = 0;
        static double total_t = timer.stop();
        cnt ++;
        total_t+=timer.stop();
        if(total_t > 0.1) {
            const auto fps = cnt / total_t;
            cnt = 0;
            total_t = 0;
            if (cfg_.print_time_consumption) { // added by Peixuan Shu
                std::cout << " -- [MARSIM] Running FPS: " << fps << std::endl;
            }
        }
    }

    void MarsimRender::load_pcd_file(std::string file_name, pcl::PointCloud<PointType>& cloud_color_mesh) {
        int status = pcl::io::loadPCDFile<PointType>(file_name, cloud_color_mesh);
        if (status == -1) {
            cout << "can't read file." << endl;
        }
    }

    void MarsimRender::render_dynclouds_on_depthimage(cv::Mat& depth_image) {
        //count running time
        system_clock::time_point t1 = system_clock::now();

        /*#pragma omp parallel default (none) \
                            shared (dyn_clouds, cfg_.effect_range,\
                            cfg_.cover_dis, cfg_.width, cfg_.height,depth_image,\
                            camera2world, camera, cfg_.yaw_fov, cfg_.vertical_fov,\
                            cfg_.downsample_res, cfg_.sensing_blind, cfg_.sensing_horizon, cfg_.polar_resolution)
        {
        #pragma omp for*/
        for (size_t i = 0; i < dyn_clouds.points.size(); i++) {
            // cout << "in dyn cloud loop: " << i << endl;

            Eigen::Vector3f temp_point;
            temp_point(0) = dyn_clouds.points[i].x;
            temp_point(1) = dyn_clouds.points[i].y;
            temp_point(2) = dyn_clouds.points[i].z;

            // fov check
            Eigen::Vector3f temp_point_cam = body2world.transpose() * (temp_point - camera);

            // project to depth image and interline
            decimal_t depth = temp_point_cam.norm();
            if (depth > cfg_.sensing_horizon || depth < cfg_.sensing_blind)
                continue;

            Eigen::Vector3f temp_point_cam_norm = temp_point_cam.normalized();
            int cen_theta_index =
                atan2(temp_point_cam_norm(1), temp_point_cam_norm(0)) / M_PI * 180.0 / cfg_.polar_resolution +
                round(0.5 * cfg_.width);
            int cen_fi_index = atan2(temp_point_cam_norm(2), sqrt(temp_point_cam_norm(0) * temp_point_cam_norm(0) +
                                         temp_point_cam_norm(1) * temp_point_cam_norm(1))) /
                M_PI *
                180.0 / cfg_.polar_resolution + round(0.5 * cfg_.height);
            if (cen_theta_index < 0 || cen_theta_index >= cfg_.width || cen_fi_index < 0 || cen_fi_index >= cfg_.height)
                continue;

            // cout << "temp_point = " << temp_point.transpose() << endl;
            // cout << "temp_point_cam = " << temp_point_cam.transpose() << endl;

            if (depth > cfg_.effect_range) {
                if (depth_image.at<decimal_t>(cen_fi_index, cen_theta_index) > depth)
                    depth_image.at<decimal_t>(cen_fi_index, cen_theta_index) = depth;
            }
            else {
                int half_cover_angle = ceil(
                    (asin(cfg_.cover_dis / depth) / (M_PI * cfg_.polar_resolution / 180.0)));

                // cout << "half_cover_angle = " << half_cover_angle << endl;
                // cout << "cen_theta_index = " << cen_theta_index << endl;
                // cout << "cen_fi_index = " << cen_fi_index << endl;

                int theta_start = cen_theta_index - half_cover_angle;
                int theta_end = cen_theta_index + half_cover_angle;
                int fi_start = cen_fi_index - half_cover_angle;
                int fi_end = cen_fi_index + half_cover_angle;

                for (int theta_index_o = theta_start;
                     theta_index_o <= theta_end; theta_index_o++) {
                    for (int fi_index_o = fi_start;
                         fi_index_o <= fi_end; fi_index_o++) {
                        int fi_index_inver = cfg_.height - 1 - fi_index_o;
                        int theta_index_inver = cfg_.width - 1 - theta_index_o;
                        if ((theta_index_inver > (cfg_.width - 1)) || (theta_index_inver < 0) ||
                            (fi_index_inver > (cfg_.height - 1)) || (fi_index_inver < 0)) {
                            continue;
                        }

                        if (depth_image.at<decimal_t>(fi_index_inver, theta_index_inver) > depth) {
                            depth_image.at<decimal_t>(fi_index_inver, theta_index_inver) = depth;
                            density_matrix(fi_index_inver, theta_index_inver) = dyn_clouds.points[i].intensity;
                        }
                    }
                }
            }
        }
        // }

        system_clock::time_point t2 = system_clock::now();
        auto dur = t2 - t1;
        duration<decimal_t> second(dur);
        // cout << "dynamic interline costs " << second.count() << " seconds" << endl;
        // std::cout << "One frame interline costs " << second.count() << " seconds, " << "first for loop costs "<< second2.count() << endl;
    }


    //from depth image to point cloud
    void MarsimRender::depth_to_pointcloud(cv::Mat& depth_image, pcl::PointCloud<PointType>::Ptr origin_cloud,
                                           vec_E<Vec3f>& depth_ptcloud_vec) {
        origin_cloud->points.clear();
        depth_ptcloud_vec.clear();
        const auto u0 = cfg_.width * 0.5;
        const auto v0 = cfg_.height * 0.5;

        cv::Mat pattern_image = cv::Mat::zeros(cfg_.height, cfg_.width, CV_32F);

        const auto polar_resolution_rad = cfg_.polar_resolution / 180.0 * M_PI;
        for (int v = 0; v < cfg_.height; v++) {
            for (int u = 0; u < cfg_.width; u++) {
                pattern_image.at<float>(v, u) = 30;
                if (pattern_matrix(u, (cfg_.height - 1 - v)) > 0) {
                    float depth = depth_image.at<float>(v, u);
                    // if(v == v0 && u == (int)(u0))
                    // {
                    //     std::cout << "depth = " << depth << std::endl;
                    // }
                    if (depth > cfg_.sensing_blind && depth < (cfg_.sensing_horizon - 0.1)) {
                        Eigen::Vector3f temp_point, temp_point_world;
                        temp_point(0) = depth * cos(polar_resolution_rad * (v - v0)) * sin(
                            polar_resolution_rad * (u - u0));
                        temp_point(1) = -depth * sin(polar_resolution_rad * (v - v0));
                        temp_point(2) = -depth * cos(polar_resolution_rad * (v - v0)) * cos(
                            polar_resolution_rad * (u - u0));

                        temp_point_world = camera2world * (temp_point) + camera;

                        PointType temp_pclpt;
                        temp_pclpt.x = temp_point_world(0);
                        temp_pclpt.y = temp_point_world(1);
                        temp_pclpt.z = temp_point_world(2);
                        temp_pclpt.intensity = density_matrix(v, u);
                        origin_cloud->points.push_back(temp_pclpt);
                        depth_ptcloud_vec.push_back(temp_point_world);

                        pattern_image.at<float>(v, u) = depth;
                    }
                    else if (depth >= (cfg_.sensing_horizon - 0.1)) {
                        Eigen::Vector3f temp_point, temp_point_world;

                        temp_point(0) = depth * cos(polar_resolution_rad * (v - v0)) * sin(
                            polar_resolution_rad * (u - u0));
                        temp_point(1) = -depth * sin(polar_resolution_rad * (v - v0));
                        temp_point(2) = -depth * cos(polar_resolution_rad * (v - v0)) * cos(
                            polar_resolution_rad * (u - u0));

                        temp_point.normalize();
                        temp_point = 2 * cfg_.sensing_horizon * temp_point;

                        //interface with ros
                        temp_point_world = camera2world * (temp_point) + camera;

                        if (cfg_.inf_point_en) {
                            PointType temp_pclpt;
                            temp_pclpt.x = temp_point_world(0);
                            temp_pclpt.y = temp_point_world(1);
                            temp_pclpt.z = temp_point_world(2);
                            temp_pclpt.intensity = density_matrix(v, u);
                            origin_cloud->points.push_back(temp_pclpt);
                            depth_ptcloud_vec.push_back(temp_point_world);
                        }
                    }
                }
            }
        }

        if(cfg_.depth_image_en) {
            double min;
            double max;
            minMaxLoc(pattern_image, &min, &max, nullptr, nullptr);
            // cout << "final_min = " << min << ", final_max = " << max << endl ;
            pattern_image.convertTo(pattern_image, CV_8UC1, 255.0 / (max - min), -min);
            cv::Mat show_pattern;
            cv::applyColorMap(pattern_image, show_pattern, cv::COLORMAP_HOT);
            cv::imshow("pattern", show_pattern);
            cv::waitKey(1);
        }
    }

    cv::Mat MarsimRender::colormap_depth_img(cv::Mat& depth_mat) {
        cv::Mat cm_img0;
        cv::Mat adjMap;
        double min;
        double max;
        for (int i = 0; i < depth_mat.rows; ++i) {
            for (int j = 0; j < depth_mat.cols; ++j) {
                float depth = depth_mat.at<float>(i, j);
                // depth_mat.at<float>(i, j) = Licfg_.sensing_blindizeDepth(depth,cfg_.sensing_blind,cfg_.sensing_horizon);
                // depth_mat.at<float>(i, j) = regainrealdepth(depth,cfg_.sensing_blind,cfg_.sensing_horizon);
                depth_mat.at<float>(i, j) = 2 * (depth - 0.5) * cfg_.sensing_horizon; //
            }
        }
        minMaxLoc(depth_mat, &min, &max, 0, 0);
        depth_mat.convertTo(adjMap, CV_8UC1, 255.0 / (max - min), -min);
        cm_img0 = adjMap; //adjMap
        return cm_img0;
    }

    void MarsimRender::read_depth(int depth_buffer_precision,
                                  vec_E<Vec3f>& depth_ptcloud_vec,
                                  pcl::PointCloud<PointType>::Ptr rendered_cloud) {
        if (depth_buffer_precision == 16) {
            const int& image_width = cfg_.width;
            const int& image_height = cfg_.height;
            const int size = image_width * image_height;
            std::memset(mypixels.data(), 0, size * sizeof(GL_FLOAT));
            std::memset(red_pixels.data(), 0, size * sizeof(GL_FLOAT));
            {
                ScopeTimer timer("Read depth buffer",  cfg_.print_time_consumption);
                glReadPixels(0, 0, cfg_.width, cfg_.height, GL_DEPTH_COMPONENT, GL_FLOAT, mypixels.data());
            }
            {
                ScopeTimer timer("Read RED depth buffer",  cfg_.print_time_consumption);
                glReadPixels(0, 0, image_width, image_height, GL_RED, GL_FLOAT, red_pixels.data());
            }

            cv::Mat red_image_mat(image_height, image_width, CV_32FC1, red_pixels.data());
            cv::flip(red_image_mat, red_image_mat, 0);

            density_matrix = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            (red_image_mat.ptr<float>(), image_height, image_width);

            cv::Mat image_mat(image_height, image_width, CV_32F, mypixels.data());
            cv::Mat image_aft_flip;
            {
                ScopeTimer timer("Render depth image",  cfg_.print_time_consumption);
                cv::flip(image_mat, image_aft_flip, 0);
                colormap_depth_img(image_aft_flip);
                render_dynclouds_on_depthimage(image_aft_flip);
                depth_to_pointcloud(image_aft_flip, rendered_cloud, depth_ptcloud_vec);
            }


            if(cfg_.depth_image_en) {
                double min;
                double max;
                minMaxLoc(image_aft_flip, &min, &max, 0, 0);
                // cout << "final_min = " << min << ", final_max = " << max << endl ;
                image_aft_flip.convertTo(image_aft_flip, CV_8UC1, 255.0 / (max - min), -min);
                cv::applyColorMap(image_aft_flip, image_aft_flip, cv::COLORMAP_JET);
                cv::imshow("interline Depth", (image_aft_flip)); //
                cv::waitKey(1);
            }
        }
        else if (depth_buffer_precision == 24) {
            GLuint mypixels[cfg_.width * cfg_.height];
            // There is no 24 bit variable, so we'll have to settle for 32 bit
            glReadPixels(0, 0, cfg_.width, cfg_.height, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT_24_8, mypixels);
            // No upconversion.
            cv::Mat image_mat(cfg_.height, cfg_.width, CV_16U, mypixels);

            cv::imshow("Depth", colormap_depth_img(image_mat));
            cv::waitKey(10);
        }
        else if (depth_buffer_precision == 32) {
            GLuint mypixels[cfg_.width * cfg_.height];
            glReadPixels(0, 0, cfg_.width, cfg_.height, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, mypixels);
            cv::Mat image_mat(cfg_.height, cfg_.width, CV_16U, mypixels);
            cv::imshow("Depth", colormap_depth_img(image_mat));
            cv::waitKey(10);
        }
        else if (depth_buffer_precision == 48) {
            glReadPixels(0, 0, cfg_.width, cfg_.height, GL_DEPTH_COMPONENT, GL_FLOAT, mypixels.data());
            cout << mypixels[(int)(cfg_.width * cfg_.height * 0.5 + cfg_.width * 0.5)] << endl;
            cv::Mat image_mat(cfg_.height, cfg_.width, CV_32F, mypixels.data());
            cv::Mat image_aft_flip;
            cv::flip(image_mat, image_aft_flip, 0);
            cv::imshow("Depth", colormap_depth_img(image_aft_flip));
            cv::waitKey(1);
        }
    }


    void MarsimRender::init_pointcloud_data() {
        PointType global_mapmin;
        PointType global_mapmax;
        pcl::getMinMax3D(cloud_color_mesh, global_mapmin, global_mapmax);
        cout << "Now initing the point cloud data: " << endl;
        g_eigen_pt_vec.clear();
        Eigen::Matrix<decimal_t, 3, 1> eigen_pt;
        Eigen::Matrix<decimal_t, 3, 1> rgb_pt;
        g_eigen_pt_vec.resize(cloud_color_mesh.size() * 2);
        for (size_t i = 0; i < cloud_color_mesh.size(); i++) {
            eigen_pt << cloud_color_mesh.points[i].x, cloud_color_mesh.points[i].y, cloud_color_mesh.points[i].z;
            rgb_pt << 0, 0, 1;
            g_eigen_pt_vec[2 * i] = eigen_pt;
            g_eigen_pt_vec[2 * i + 1] = rgb_pt;
            points_index_infov.push_back(i);
        }
        cout << "Number of points = " << g_eigen_pt_vec.size() / 20000.0 << "X 10000" << endl;
    }
}
