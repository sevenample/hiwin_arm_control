#include <iostream>
#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <chrono>

#include "hiwin_interfaces/srv/motioncmd.hpp"
#include "hiwin_interfaces/srv/readcmd.hpp"
#include "hiwin_interfaces/srv/digitalcmd.hpp"

#include "rclcpp/rclcpp.hpp"

#include "hiwin_libmodbus/hiwin_libmodbus.hpp"

#include "hiwin_libmodbus//visibility_control.h"

HiwinLibmodbus hiwinlibmodbus;
class HiwinlibmodbusServiceServer : public rclcpp::Node
{
    
    public:
    HiwinlibmodbusServiceServer() : Node("hiwinmodbus_server")
    {

        motion_server_ = create_service<hiwin_interfaces::srv::Motioncmd>(
            "motioncmd", std::bind(&HiwinlibmodbusServiceServer::hiwinmodbus_motioncmd, this,
                                    std::placeholders::_1, std::placeholders::_2));
                                    
        digital_server_ = create_service<hiwin_interfaces::srv::Digitalcmd>(
            "digitalcmd", std::bind(&HiwinlibmodbusServiceServer::hiwinmodbus_digitalcmd, this,
                                    std::placeholders::_1, std::placeholders::_2));

        read_server_ = create_service<hiwin_interfaces::srv::Readcmd>(
            "readcmd", std::bind(&HiwinlibmodbusServiceServer::hiwinmodbus_readcmd, this,
                                    std::placeholders::_1, std::placeholders::_2));







    }

    private:
                
        int arm_state;
        int digital_state;
        std::vector<double> current_pos;
        std::vector<double> command;
        int command_type;
        int digital_output = 0;
        std::mutex mutex;

        void DO_Timer(int d_o, int time) {
            std::cout<<"waiting";
            std::this_thread::sleep_for(std::chrono::milliseconds(time*100));
            digital_output = d_o;
            std::unique_lock<std::mutex> lock(mutex);
            if (digital_output){
                hiwinlibmodbus.DO(digital_output, 0);
                digital_output = 0;
            }
        };

        rclcpp::Service<hiwin_interfaces::srv::Motioncmd>::SharedPtr motion_server_;
        rclcpp::Service<hiwin_interfaces::srv::Digitalcmd>::SharedPtr digital_server_;
        rclcpp::Service<hiwin_interfaces::srv::Readcmd>::SharedPtr read_server_;

        

        void hiwinmodbus_motioncmd(const std::shared_ptr<hiwin_interfaces::srv::Motioncmd::Request> request,    
            std::shared_ptr<hiwin_interfaces::srv::Motioncmd::Response>     response)  
        {

            mutex.lock(); 

            if (request->cmd_mode == 1){
                hiwinlibmodbus.MOTOR_EXCITE();
            }
            else if (request->cmd_mode == 2){
                if(request->cmd_type==0){
                    command = {request->joints[0], request->joints[1], request->joints[2],
                    request->joints[3], request->joints[4], request->joints[5]};
                }
                else if(request->cmd_type==1){
                    command={request->pose.linear.x, request->pose.linear.y, request->pose.linear.z,
                    request->pose.angular.x, request->pose.angular.y, request->pose.angular.z};
                }

                hiwinlibmodbus.PTP(request->cmd_type, request->velocity, request->acceleration, request->tool, request->base, command);
            }
            else if (request->cmd_mode == 3){
                if(request->cmd_type==0){
                    command = {request->joints[0], request->joints[1], request->joints[2],
                    request->joints[3], request->joints[4], request->joints[5]};
                }
                else if(request->cmd_type==1){
                    command={request->pose.linear.x, request->pose.linear.y, request->pose.linear.z,
                    request->pose.angular.x, request->pose.angular.y, request->pose.angular.z};
                }                hiwinlibmodbus.LIN(request->cmd_type, request->velocity, request->acceleration, request->tool, request->base, command);
            }
            else if (request->cmd_mode == 4){
                hiwinlibmodbus.CIRC(request->velocity, request->acceleration, request->tool, request->base, request->circ_s, request->circ_end); 
            }
            else if (request->cmd_mode == 5){
                hiwinlibmodbus.HOME();
            }
            else if (request->cmd_mode == 6){
                hiwinlibmodbus.JOG(request->jog_joint, request->jog_dir); 
            }
            else if (request->cmd_mode == 7){
                hiwinlibmodbus.Modbus_Close();
                rclcpp::shutdown();
            }
            else if (request->cmd_mode == 8){
                request->holding = true;
            }
            else if (request->cmd_mode == 9){
                command={request->pose.linear.x, request->pose.linear.y, request->pose.linear.z,
                request->pose.angular.x, request->pose.angular.y, request->pose.angular.z};
                hiwinlibmodbus.SET_BASE(request->base_num, command);
                request->holding == false;
            }
            else if (request->cmd_mode == 10){
                command={request->pose.linear.x, request->pose.linear.y, request->pose.linear.z,
                request->pose.angular.x, request->pose.angular.y, request->pose.angular.z};
                hiwinlibmodbus.SET_TOOL(request->tool_num, command);
                request->holding == false;
            }
            else if (request->cmd_mode == 11){
                hiwinlibmodbus.Motion_Stop();
                request->holding == false;
            }
            else if (request->cmd_mode == 12){
                if (request->move_dir != "x" && request->move_dir != "y" && request->move_dir != "z") {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "error command");
                }
                else{
                    const int move_distance = request->move_dis;
                    hiwinlibmodbus.moveFlange(current_pos, request->move_dir, move_distance); 
                    response->current_position = current_pos;
                    // command = {current_pos[0], current_pos[1], current_pos[2],  
                    //             current_pos[3], current_pos[4], current_pos[5]};
                    // std::cout<<current_pos[0]<<std::endl;
                    // std::cout<<current_pos[1]<<std::endl;
                    // std::cout<<current_pos[2]<<std::endl;
                    // std::cout<<current_pos[3]<<std::endl;
                    // std::cout<<current_pos[4]<<std::endl;
                    // std::cout<<current_pos[5]<<std::endl;
                    hiwinlibmodbus.LIN(request->cmd_type, request->velocity, request->acceleration, request->tool, request->base, current_pos);
                    std::cout<<"?HELLOHELLOHELLO"<<std::endl;
                    request->holding == true;
                }
                
            }
            if (request->holding == true){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                while(1){
                    hiwinlibmodbus.Arm_State_REGISTERS(arm_state); // return arm_state
                    if (digital_output){
                        hiwinlibmodbus.DO(digital_output, 0);
                        digital_output = 0;
                    }
                    if (arm_state == 1){
                        response->arm_state = arm_state;
                        break;
                    }

                }
            }
            else{
                hiwinlibmodbus.Arm_State_REGISTERS(arm_state); // return arm_state{
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "arm_state=%d",arm_state);
                response->arm_state = arm_state;
                }
            mutex.unlock();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response");
        }



        void hiwinmodbus_digitalcmd(const std::shared_ptr<hiwin_interfaces::srv::Digitalcmd::Request> request,    
            std::shared_ptr<hiwin_interfaces::srv::Digitalcmd::Response>     response)  
        {
            mutex.lock();
            /************* Discret_e Input *************/
              /*********************************
                    S0
              value:0 ~ 255 -> S0[1] ~ [256]
                0 or 1    -> R 
              ----------------------------------
                    DI
              value:300 ~ 555 -> DI[1] ~ [256]
                0 or 1      -> R
              **********************************/
         
              /************* Coil *************/
              /*********************************
                    DI
              value:0 ~ 255  -> DI[1] ~ [256]
                0 or 65280 -> R/W 
              ----------------------------------
                    DO
              value:300 ~ 555 -> DO[1] ~ [256]
                0 or 65280  -> R/W 
              **********************************/
            if (request->cmd_mode == 1){
                const int d_o = request->digital_output_pin+299;
                hiwinlibmodbus.DO(d_o, request->digital_output_cmd); 
                if (request->do_timer!=0){
                    std::thread t(&HiwinlibmodbusServiceServer::DO_Timer, this, d_o, request->do_timer);
                    t.detach(); 
                }
                request->holding == false;
            }
            else if (request->cmd_mode == 2){
                hiwinlibmodbus.Read_DI(299+request->digital_input_pin, digital_state);
                response->digital_state = digital_state;
                request->holding == false;
            }
            if (request->holding == true){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                while(1){
                    hiwinlibmodbus.Arm_State_REGISTERS(arm_state); // return arm_state
                    // int arm_state = hiwinlibmodbus.Check_Arm_State();
                    if (digital_output){
                        hiwinlibmodbus.DO(digital_output, 0);
                        digital_output = 0;
                    }
                    if (arm_state == 1){
                        response->arm_state = arm_state;
                        break;
                    }

                }
            }
            else{
                hiwinlibmodbus.Arm_State_REGISTERS(arm_state); // return arm_state{
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "arm_state=%d",arm_state);
                response->arm_state = arm_state;
                }
            mutex.unlock();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response");
        }


        void hiwinmodbus_readcmd(const std::shared_ptr<hiwin_interfaces::srv::Readcmd::Request> request,    
            std::shared_ptr<hiwin_interfaces::srv::Readcmd::Response>     response)  
        {
            mutex.lock();
            if (request->cmd_mode == 1){
                hiwinlibmodbus.getArmJoints(current_pos); 
                response->current_position = current_pos;
            }
            else if (request->cmd_mode == 2){
                hiwinlibmodbus.getArmPose(current_pos); 
                std::cout<<"----------------------------------"<<std::endl;
                response->current_position = current_pos;
            }
            if (request->holding == true){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                while(1){
                    hiwinlibmodbus.Arm_State_REGISTERS(arm_state); // return arm_state
                    if (digital_output){
                        hiwinlibmodbus.DO(digital_output, 0);
                        digital_output = 0;
                    }
                    if (arm_state == 1){
                        response->arm_state = arm_state;
                        break;
                    }

                }
            }
            else{
                hiwinlibmodbus.Arm_State_REGISTERS(arm_state); // return arm_state{
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "arm_state=%d",arm_state);
                response->arm_state = arm_state;
                }
            mutex.unlock();   
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response");
        }
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  hiwinlibmodbus.libModbus_Connect("192.168.0.1");
  hiwinlibmodbus.Holding_Registers_init();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to recieve commands.");

  rclcpp::spin(std::make_shared<HiwinlibmodbusServiceServer>());
  // rclcpp::shutdown();
}
