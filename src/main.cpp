#include "test_func.hpp"
#include "ec_api_without_ros.hpp"

#include <sched.h>
#include <stdio.h>

int main(int argc, char** argv) {

    if (motor_init() != 0) { //CAN init
        printf("[CAN-BUS] motor_init 失败: CAN 总线已被其它进程占用,leg_tool 直接退出。\n");
        return -1;
    }
    test_id_init(); //
    std::signal(SIGINT, signal_handler);

    printf("\n ------欢迎使用双足电机测试工具vDD-preview------ \n");
    
    if(HUMANOID_TYPE == 2){
        while(1){
            int mode_input;
            std::cout << "\n 请输入数字指令 : \n \
            666: 检查-双足电机所有角度 \n \
            7890: 测试-回零 \n \
            4321: debug-标零 \n \
            4399: debug-设置id \n \
            1688: 读取pvt参数(kp/kd) \n \
            2345: 读取电机力位模式参数(pvt_kp/pvt_kd/max_pos/max_vel等) \n \
            7531: 设置电机力位混控(PVT)参数(未测试) \n \
             \n";

            std::cin >> mode_input;
            switch (mode_input)
            {

                case 666:
                    printf("开始检查双足电机所有角度！\n");
                    posCheck();
                    break;

                case 4321:
                    printf("开始标零！\n");
                    motorSetZero();
                    usleep(500 * 1000);
                    posCheck();
                    break;

                case 7890:
                    printf("开始回零！\n");
                    while(1){
                        motorBackToZero();
                        usleep(2000);
                    }
                    break;
                case 4399:
                    printf("开始设置id号！\n");
                    idSet();
                    break;

                case 1688:
                    printf("开始查询PVT！\n");
                    printf("输入电机ID！\n");
                    uint16_t id;
                    std::cin >> id;
                    get_pvt_kp(id);
                    usleep(50 * 1000);
                    get_pvt_kd(id);
                    usleep(50 * 1000);
                    break;

                case 2345:
                    printf("开始读取电机力位模式参数！\n");
                    printf("输入电机ID！\n");
                    uint16_t pvt_id;
                    std::cin >> pvt_id;
                    get_motor_pvt_params(pvt_id);
                    break;

                case 7531:
                    printf("开始设置电机力位混控(PVT)参数！\n");
                    set_motor_pvt_params();
                    break;
                    
                case 8:
                    test_ankel_solver();
                    break;

                case 9:
                    test_waist_solver();
                    break;

                default:
                    printf("\033[1;33m invalid input! \033[0m \n\n");
                    break;

                
            }
        }
    }
    return 0;
}
