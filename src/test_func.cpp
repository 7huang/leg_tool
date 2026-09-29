#include "test_func.hpp"
extern "C" double sin(double);

void test_id_init(){
    for (int i = 0; i < 12; ++i) {
        cmd_test.motor_id[i] = i + 1;
    }
    posCheck();
}

void test_print(){
    for (int i = 0; i < 12; ++i){
        printf("|id= %d, ", motor_state_output.motor_id[i]);
        printf("q= %f, ", motor_state_output.q[i]);
        printf("qd= %f, ", motor_state_output.qd[i]);
        printf("tau= %f |\n", motor_state_output.tau[i]);
    }
}

void test_pos_one_by_one(){

    cmd_test.mode[0] = 1;
    while (true) {
        
        // 开始测试关节旋转
        cmd_test.msg_cmd += 1;
        if (cmd_test.msg_cmd >= 48000){
            printf("结束关节依次旋转测试！");
            cmd_test.msg_cmd = 0;
            break;
        }

        if (cmd_test.mode[0] == 1){
            int i;
            float limit_max[12] = {10,9,20,4,20,20,10,9,20,4,20,20};
            float limit_min[12] = {-10,-9,-20,-24,-20,-20,-10,-9,-20,-24,-20,-20};
            i = cmd_test.msg_cmd / time_factor; //4秒
            if ((cmd_test.msg_cmd - i*time_factor) / 1000 == 0){
                cmd_test.qd_des[i] = 10;

                cmd_test.q_des[i] = limit_max[i];
                
            }
            if ((cmd_test.msg_cmd - i*time_factor) / 1000 == 1 || cmd_test.msg_cmd / 1000 == 2){
                cmd_test.q_des[i] = limit_min[i];
            }
            if ((cmd_test.msg_cmd - i*time_factor) / 1000 == 3){
                cmd_test.q_des[i] = 0;
                cmd_test.qd_des[i] = 2;
            }
        }
        cmdCallback(&cmd_test);
        usleep(2000);
    }
}

void test_sin_wave_all(){
//通信实时性测试
    auto start_time = std::chrono::steady_clock::now();
        
    std::cout << std::fixed << std::setprecision(2);  // 输出保留2位小数

    while (true) {
        // 计算当前时间（秒）
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        
        if (elapsed >= duration_sec) {
            std::cout << "Control finished." << std::endl;
            break;
        }

        // 计算当前角度（正弦值，单位：弧度）
        angle_rad = amplitude_rad * sin(2 * PI * sin_frequency_hz * elapsed);
        
        // 输出结果
        std::cout << "Time: " << std::setw(5) << elapsed << "s | "
                << "Angle: " << std::setw(6) << angle_rad << "" << std::endl;
        
            cmd_test.mode[0] = 2;
            cmd_test.kp[0] =  50;
            cmd_test.kp[1] =  50;
            cmd_test.kp[2] =  50;
            cmd_test.kp[3] =  50;
            cmd_test.kp[4] =  50;
            cmd_test.kp[5] =  50;
            cmd_test.kp[6] =  50;
            cmd_test.kp[7] =  50;
            cmd_test.kp[8] =  50;
            cmd_test.kp[9] =  50;
            cmd_test.kp[10] = 50;
            cmd_test.kp[11] = 50;
        
            cmd_test.kd[0] =  10;
            cmd_test.kd[1] =  1;
            cmd_test.kd[2] =  10;
            cmd_test.kd[3] =  10;
            cmd_test.kd[4] =  1;
            cmd_test.kd[5] =  1;
            cmd_test.kd[6] =  10;
            cmd_test.kd[7] =  1;
            cmd_test.kd[8] =  10;
            cmd_test.kd[9] =  10;
            cmd_test.kd[10] = 1;
            cmd_test.kd[11] = 1;
        
            cmd_test.q_des[0] =   0.2 + angle_rad;
            cmd_test.q_des[1] =   -0.2 + angle_rad;
            cmd_test.q_des[2] =   0.2 + angle_rad;
            cmd_test.q_des[3] =   -0.4 + angle_rad;
            cmd_test.q_des[4] =   angle_rad;
            cmd_test.q_des[5] =   -angle_rad;
            cmd_test.q_des[6] =   -0.2 + angle_rad;
            cmd_test.q_des[7] =   0.2 + angle_rad;
            cmd_test.q_des[8] =   -0.2 + angle_rad;
            cmd_test.q_des[9] =   0.4 + angle_rad;
            cmd_test.q_des[10] =  -angle_rad;
            cmd_test.q_des[11] =  angle_rad;

            cmdCallback(&cmd_test);
            usleep(2000);
    }
}

void test_stand_pd(){
    cmd_test.mode[0] = 2;
    cmd_test.kp[0] =  6;
    cmd_test.kp[1] =  6;
    cmd_test.kp[2] =  6;
    cmd_test.kp[3] =  6;
    cmd_test.kp[4] =  6;
    cmd_test.kp[5] =  6;
    cmd_test.kp[6] =  6;
    cmd_test.kp[7] =  6;
    cmd_test.kp[8] =  6;
    cmd_test.kp[9] =  6;
    cmd_test.kp[10] = 6;
    cmd_test.kp[11] = 6;

    cmd_test.kd[0] =  2;
    cmd_test.kd[1] =  2;
    cmd_test.kd[2] =  2;
    cmd_test.kd[3] =  2;
    cmd_test.kd[4] =  2;
    cmd_test.kd[5] =  2;
    cmd_test.kd[6] =  2;
    cmd_test.kd[7] =  2;
    cmd_test.kd[8] =  2;
    cmd_test.kd[9] =  2;
    cmd_test.kd[10] = 2;
    cmd_test.kd[11] = 2;

    cmd_test.q_des[0] =   0;
    cmd_test.q_des[1] =   0;
    cmd_test.q_des[2] =   0;
    cmd_test.q_des[3] =   0;
    cmd_test.q_des[4] =   0.2;
    cmd_test.q_des[5] =   0.;
    cmd_test.q_des[6] =   0;
    cmd_test.q_des[7] =   0;
    cmd_test.q_des[8] =   0;
    cmd_test.q_des[9] =   0;
    cmd_test.q_des[10] =  0.;
    cmd_test.q_des[11] =  -0.2;
    cmdCallback(&cmd_test);
}

void setting_tool(){

    while(1){
        int setting_input;
        std::cout << "设置此工具，输入数字： \n \
        1: 切换debug消息输出（部分模式下） \n \
        9: 退出设置 \n ";

        std::cin >> setting_input;

        switch (setting_input)
        {
        case 1:
            printf("正在切换debug消息输出！\n");
            show_debug_info = !show_debug_info;
            if (show_debug_info == true){
                printf("debug消息：输出\n");
            }
            if (show_debug_info == false){
                printf("debug消息：不输出\n");
            }
        break;

        case 9:
            printf("退出设置！ \n");
            return;
        break;

        default:
            printf("\033[1;33m invalid input! \033[0m \n\n");
        break;
        }
    }

}

void test_ankel_solver(){
    {
        FastAnkelSolver _ankle_joint_mapping(0.0);
        AnkleParallelMechanism mech;
        std::vector<JointData> motor_data(12);

        double q[12] = {0,0,0,0,-0.16,0.26,
                        0,0,0,0,0.46,-0.26};
        // double q[12] = {0,0,0,0,0.8,-0.8,
        //                 0,0,0,0,-0.915,0.827};
        double qd[12] = {0,0,0,0,1,2,
                        0,0,0,0,3,-1};
        double tau[12] = {0,0,0,0,1,4,
                        0,0,0,0,10,-6};
        motor_data[4].pos_ = q[4];
        motor_data[5].pos_ = q[5];
        motor_data[10].pos_ = q[10];
        motor_data[11].pos_ = q[11];
        motor_data[4].vel_ = qd[4];
        motor_data[5].vel_ = qd[5];
        motor_data[10].vel_ = qd[10];
        motor_data[11].vel_ = qd[11];
        motor_data[4].tau_ = tau[4];
        motor_data[5].tau_ = tau[5];
        motor_data[10].tau_ = tau[10];
        motor_data[11].tau_ = tau[11];

        struct timespec start, end;
        clock_gettime(CLOCK_MONOTONIC, &start);
        motor_data = _ankle_joint_mapping.motorToJointAll(motor_data);
        clock_gettime(CLOCK_MONOTONIC, &end);
        long delta_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
        printf("cmdCallback: %ldμs\n", delta_ns / 1000);


        std::vector<double> q_d = {0,0,0,0,0.0,0.0,
                                    0,0,0,0,-0.,-0.};
        std::vector<double> v_d = {0,0,0,0,0.4,1,
                                    0,0,0,0,-0.4,-2};
        std::vector<double> tq_d = {0,0,0,0,-100,100,
                                    0,0,0,0,-100,100};

        struct timespec start2, end2;
        clock_gettime(CLOCK_MONOTONIC, &start2);
        _ankle_joint_mapping.jointToMotorAll(q_d, v_d, tq_d);
        clock_gettime(CLOCK_MONOTONIC, &end2);
        long delta_ns2 = (end2.tv_sec - start2.tv_sec) * 1e9 + (end2.tv_nsec - start2.tv_nsec);
        printf("cmdCallback: %ldμs\n", delta_ns2 / 1000);
        
        for (int i = 0; i < 12; ++i){
            if (i == 4 || i == 5 || i == 10 || i == 11){
                printf("joint---- q = %.3f, qd = %.3f, tau = %.3f\n",
                            motor_data[i].pos_,motor_data[i].vel_,motor_data[i].tau_);
            }
        }
        for (int i = 0; i < 12; ++i){
            if (i == 4 || i == 5 || i == 10 || i == 11){
                printf("motor---- q = %.3f, qd = %.3f, tau = %.3f\n",
                            q_d[i],v_d[i],tq_d[i]);
            }
        }
    }
    //test_solver
    {
        double q[12] = {0,0,0,0,-0.16,0.26,
                        0,0,0,0,0.46,-0.26};
        double qd[12] = {0,0,0,0,1,2,
                        0,0,0,0,3,-1};
        double tau[12] = {0,0,0,0,1,4,
                        0,0,0,0,10,-6};
        
        // q[4] = q[4]+0.261;
        // q[5] = q[5]+M_PI-0.261;
        // q[10] = q[10]+M_PI-0.261;
        // q[11] = q[11]+0.261;

        std::vector<double> q_d = {0,0,0,0,-0.4,0.0,
                                    0,0,0,0,-0.,-0.};
        std::vector<double> v_d = {0,0,0,0,0.4,1,
                                    0,0,0,0,-0.4,-2};
        std::vector<double> tq_d = {0,0,0,0,-100,100,
                                    0,0,0,0,-100,100};

        ParallelMechanism test_solver;

        double alpha[12];
        double v[12];
        double tq[12];
        double j_p[12];
        double j_v[12];
        double j_t[12];
        struct timespec start, end;
        clock_gettime(CLOCK_MONOTONIC, &start);
        std::tie(j_p[5], j_p[4], j_v[5], j_v[4], j_t[5], j_t[4]) = test_solver.motorToJointLeft(q[4],q[5],qd[4],qd[5],tau[4],tau[5]);
        std::tie(j_p[11], j_p[10], j_v[11], j_v[10], j_t[11], j_t[10]) = test_solver.motorToJointRight(q[10],q[11],qd[10],qd[11],tau[10],tau[11]);
        clock_gettime(CLOCK_MONOTONIC, &end);
        long delta_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
        printf("cmdCallback: %ldμs\n", delta_ns / 1000);
        struct timespec start2, end2;
        clock_gettime(CLOCK_MONOTONIC, &start2);
        std::tie(alpha[4], alpha[5], v[4], v[5], tq[4], tq[5]) = test_solver.jointToMotorLeft(q_d[4],q_d[5],v_d[4],v_d[5],tq_d[4],tq_d[5]);
        std::tie(alpha[10], alpha[11], v[10], v[11], tq[10], tq[11]) =test_solver.jointToMotorRight(q_d[10],q_d[11],v_d[10],v_d[11],tq_d[10],tq_d[11]);
        clock_gettime(CLOCK_MONOTONIC, &end2);
        long delta_ns2 = (end2.tv_sec - start2.tv_sec) * 1e9 + (end2.tv_nsec - start2.tv_nsec);
        printf("cmdCallback: %ldμs\n", delta_ns2 / 1000);
        printf("\n");
            for (int i = 0; i < 12; ++i){
            if (i == 4 || i == 5 || i == 10 || i == 11){
                printf("joint---- q = %.3f, qd = %.3f, tau = %.3f\n",
                            j_p[i],j_v[i],j_t[i]);
            }
        }
        for (int i = 0; i < 12; ++i){
            if (i == 4 || i == 5 || i == 10 || i == 11){
                printf("motor---- q = %.3f, qd = %.3f, tau = %.3f\n",
                            alpha[i],v[i],tq[i]);
            }
        }
    }
}

void test_waist_solver(){
    {
        FastWaistSolver _waist_joint_mapping(0.0);
        WaistParallelMechanism mech;
        std::vector<JointData> motor_data(15);
        double m[8][2] = {
            // {0.3573,-0.3573},
            // {0.9399,-0.9399}, //前倾?
            // {-0.3654,0.3654},
            // {-0.9399,0.9399}, //后仰?

            // {-0.1109,-0.1010},
            // {-0.2919,-0.2428},
            // {0.1109,0.1010},
            // {0.2308,0.1918},
            {0.0,-0.0},
            {0.9399,-0.9399}, //前倾?
            {-0.3654,0.3654},
            {-0.9399,0.9399}, //后仰?

            {-0.1109,-0.1010},
            {-0.2919,-0.2428},
            {0.1109,0.1010},
            {0.2308,0.1918},
            };
        double j[4][2] = {
            {0, 0},
            {-0.44, -0.0},
            {0, 0.4},
            {0, -0.4},
        };
        for (int i = 0; i < 8; ++i){
            double q[15] = {0,0,0,0,0,0,
                            0,0,0,0,0,0,
                        0,m[i][0],m[i][1]};
            double qd[15] = {0,0,0,0,0,0,
                            0,0,0,0,0,0,
                        0,1,0.5};
            double tau[15] = {0,0,0,0,0,0,
                            0,0,0,0,0,0,
                        0,10,10};

            motor_data[13].pos_ = q[13];
            motor_data[14].pos_ = q[14];

            motor_data[13].vel_ = qd[13];
            motor_data[14].vel_ = qd[14];

            motor_data[13].tau_ = tau[13];
            motor_data[14].tau_ = tau[14];

            struct timespec start, end;
            clock_gettime(CLOCK_MONOTONIC, &start);
            motor_data = _waist_joint_mapping.motorToJointAll(motor_data);
            clock_gettime(CLOCK_MONOTONIC, &end);
            long delta_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
            printf("\n");
            printf("\n");
            printf("cmdCallback: %ldμs\n", delta_ns / 1000);
            printf("\n");
            printf("input motor rad1 = %f ,rad2 = %f\n", m[i][0], m[i][1]);
            for (int i = 0; i < 15; ++i){
                if (i == 13 || i == 14){
                    printf("m2j---- q = %.3f,    qd = %.3f, tau = %.3f\n",
                                motor_data[i].pos_,motor_data[i].vel_,motor_data[i].tau_);
                }
            }
        }
        printf("----------------------------------\n");
        for (int i = 0; i < 4; ++i){

            std::vector<double> q_d = {0,0,0,0,0,0,
                                        0,0,0,0,0,0,
                                    0,j[i][0],j[i][1]};
            std::vector<double> v_d = {0,0,0,0,0,0,
                                        0,0,0,0,0,0,
                                    0,1,0};
            std::vector<double> tq_d = {0,0,0,0,10,0,
                                        0,0,0,0,10,0,
                                    0,10,10};
            struct timespec start2, end2;
            clock_gettime(CLOCK_MONOTONIC, &start2);
            _waist_joint_mapping.jointToMotorAll(q_d, v_d, tq_d);
            clock_gettime(CLOCK_MONOTONIC, &end2);
            long delta_ns2 = (end2.tv_sec - start2.tv_sec) * 1e9 + (end2.tv_nsec - start2.tv_nsec);
            printf("cmdCallback: %ldμs\n", delta_ns2 / 1000);
            printf("\n");
            printf("input joint pitch = %f , roll = %f\n", j[i][0], j[i][1]);
            for (int i = 0; i < 15; ++i){
                if (i == 13 || i == 14){
                    printf("j2m---- q = %.3f,    qd = %.3f, tau = %.3f\n",
                                q_d[i],v_d[i],tq_d[i]);
                }
            }
        }
        printf("\n");
    }
    printf("parallal mechanisim------------------------\n");
    printf("parallal mechanisim------------------------\n");
    printf("parallal mechanisim------------------------\n");
    {
        ParallelMechanism test_solver;
        std::vector<JointData> motor_data(15);
        double m[8][2] = {
            {0.0,-0.0},
            {0.9399,-0.9399}, //前倾?
            {-0.3654,0.3654},
            {-0.9399,0.9399}, //后仰?

            {-0.1109,-0.1010},
            {-0.2919,-0.2428},
            {0.1109,0.1010},
            {0.2308,0.1918},
            };
        double j[4][2] = {
            {0, 0},
            {-0.44, -0.0},
            {0, 0.4},
            {0, -0.4},
        };
            double alpha[15];
            double v[15];
            double tq[15];
            double j_p[15];
            double j_v[15];
            double j_t[15];
        for (int i = 0; i < 8; ++i){
            double q[15] = {0,0,0,0,0,0,
                            0,0,0,0,0,0,
                        0,m[i][0],m[i][1]};
            double qd[15] = {0,0,0,0,0,0,
                            0,0,0,0,0,0,
                        0,1,0.5};
            double tau[15] = {0,0,0,0,0,0,
                            0,0,0,0,0,0,
                        0,10,10};

            motor_data[13].pos_ = q[13];
            motor_data[14].pos_ = q[14];

            motor_data[13].vel_ = qd[13];
            motor_data[14].vel_ = qd[14];

            motor_data[13].tau_ = tau[13];
            motor_data[14].tau_ = tau[14];
            
            struct timespec start, end;
            clock_gettime(CLOCK_MONOTONIC, &start);
            std::tie(j_p[14], j_p[13], j_v[14], j_v[13], j_t[14], j_t[13]) = test_solver.motorToJointW(q[13],q[14],qd[13],qd[14],tau[13],tau[14]);
            clock_gettime(CLOCK_MONOTONIC, &end);
            long delta_ns = (end.tv_sec - start.tv_sec) * 1e9 + (end.tv_nsec - start.tv_nsec);
            printf("\n");
            printf("\n");
            printf("cmdCallback: %ldμs\n", delta_ns / 1000);
            printf("\n");
            printf("input motor rad1 = %f ,rad2 = %f\n", m[i][0], m[i][1]);
            for (int i = 0; i < 15; ++i){
                if (i == 13 || i == 14){
                    printf("m2j---- q = %.3f,    qd = %.3f, tau = %.3f\n",
                                j_p[i],j_v[i],j_t[i]);
                }
            }
        }
        printf("----------------------------------\n");
        for (int i = 0; i < 4; ++i){

            std::vector<double> q_d = {0,0,0,0,0,0,
                                        0,0,0,0,0,0,
                                    0,j[i][0],j[i][1]};
            std::vector<double> v_d = {0,0,0,0,0,0,
                                        0,0,0,0,0,0,
                                    0,1,0};
            std::vector<double> tq_d = {0,0,0,0,10,0,
                                        0,0,0,0,10,0,
                                    0,10,10};
            struct timespec start2, end2;
            clock_gettime(CLOCK_MONOTONIC, &start2);
            std::tie(alpha[13], alpha[14], v[13], v[14], tq[13], tq[14]) = test_solver.jointToMotorW(q_d[13],q_d[14],v_d[13],v_d[14],tq_d[13],tq_d[14]);
            clock_gettime(CLOCK_MONOTONIC, &end2);
            long delta_ns2 = (end2.tv_sec - start2.tv_sec) * 1e9 + (end2.tv_nsec - start2.tv_nsec);
            printf("cmdCallback: %ldμs\n", delta_ns2 / 1000);
            printf("\n");
            printf("input joint pitch = %f , roll = %f\n", j[i][0], j[i][1]);
            for (int i = 0; i < 15; ++i){
                if (i == 13 || i == 14){
                    printf("j2m---- q = %.3f,    qd = %.3f, tau = %.3f\n",
                                alpha[i],v[i],tq[i]);
                }
            }
        }
    }
}
