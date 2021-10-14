#ifndef kimm_multi_husky_gui_H
#define kimm_multi_husky_gui_H

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>


#include <rqt_gui_cpp/plugin.h>
#include "kimm_multi_husky_gui/ui_kimm_multi_husky_gui.h"

#include <ros/macros.h>

#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>
#include <QObject>
#include <QStateMachine>
#include <QState>
#include <QEventTransition>
#include <QMetaType>
#include <QGraphicsRectItem>
#include <QGraphicsSceneWheelEvent>
#include <QStringListModel>
#include <QSignalMapper>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include "mujoco_ros_msgs/JointSet.h"
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose2D.h>

#include "kimm_joint_planner_ros_interface/plan_joint_path.h"
#include "kimm_joint_planner_ros_interface/JointAction.h"
#include "kimm_path_planner_ros_interface/plan_mobile_path.h"
#include "kimm_path_planner_ros_interface/MobileTrajectory.h"
#include "kimm_path_planner_ros_interface/Obstacle2D.h"
#include "kimm_se3_planner_ros_interface/plan_se3_path.h"
#include "kimm_se3_planner_ros_interface/SE3Action.h"
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#define DEGREE 180.0/M_PI
#define RAD M_PI/180.0

using namespace std;
using namespace pinocchio;

namespace kimm_multi_husky_gui
{
    class HuskyGui : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT
    public:
        HuskyGui();
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin(){};
        virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const {};
        virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings) {};

    private:
        double time_;
        Ui::MultiHuskyGuiWidget ui_;
        QWidget *widget_;

        ros::NodeHandle nh_;
        

    protected slots:
        protected slots:
        virtual void EnableSimulCallback(){
            for (int k=0; k<2; k++) {
                bool_msg_[k].data = true;
                run_pub_[k].publish(bool_msg_[k]);
            }
        };
        virtual void DisableSimulCallback(){
            int16_msg_[0].data = 0;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void InitCallback(){
            int16_msg_[0].data = 1;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void QuitCallback(){
            bool_msg_[0].data = true;
            quit_pub_[0].publish(bool_msg_[0]);
            quit_pub_[1].publish(bool_msg_[0]);
        }
        virtual void GraspCallback(){
            int16_msg_[0].data = 899;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };

        virtual void DisableSimulCallback1(){
            int16_msg_[0].data = 0;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
        };
        virtual void InitCallback1(){
            int16_msg_[0].data = 1;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
        };
        virtual void GraspCallback1(){
            int16_msg_[0].data = 899;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
        };
        virtual void DisableSimulCallback2(){
            int16_msg_[0].data = 0;
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void InitCallback2(){
            int16_msg_[0].data = 1;
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void GraspCallback2(){
            int16_msg_[0].data = 899;
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };


        virtual void CustomCtrl1Callback(){
            int16_msg_[0].data = 1;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void CustomCtrl2Callback(){
            int16_msg_[0].data = 2;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void CustomCtrl3Callback(){
            int16_msg_[0].data = 3;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void CustomCtrl4Callback(){
            int16_msg_[0].data = 4;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void CustomCtrl5Callback(){
            int16_msg_[0].data = 5;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void CustomCtrl6Callback(){
            int16_msg_[0].data = 6;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void CustomCtrl7Callback(){
            int16_msg_[0].data = 7;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void CustomCtrl8Callback(){
            int16_msg_[0].data = 8;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void CustomCtrl9Callback(){
            int16_msg_[0].data = 9;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };
        virtual void CustomCtrl10Callback(){
            int16_msg_[0].data = 10;
            custom_ctrl_pub_[0].publish(int16_msg_[0]);
            custom_ctrl_pub_[1].publish(int16_msg_[0]);
        };


    //     virtual void Timercb(const std_msgs::Float32ConstPtr &msg){
    //         time_ = msg->data;
    //         ui_.currenttime->setText(QString::number(msg->data, 'f', 3));
    //     };
        virtual void Jointcb0(const sensor_msgs::JointStateConstPtr &msg){
            if (issimulation_[0]){
                ui_.robot1_p1->setText(QString::number(msg->position[7] * DEGREE , 'f', 3));
                ui_.robot1_p2->setText(QString::number(msg->position[8] * DEGREE , 'f', 3));
                ui_.robot1_p3->setText(QString::number(msg->position[11] * DEGREE , 'f', 3));
                ui_.robot1_p4->setText(QString::number(msg->position[12] * DEGREE , 'f', 3));
                ui_.robot1_p5->setText(QString::number(msg->position[13] * DEGREE , 'f', 3));
                ui_.robot1_p6->setText(QString::number(msg->position[14] * DEGREE , 'f', 3));
                ui_.robot1_p7->setText(QString::number(msg->position[15] * DEGREE , 'f', 3));
                ui_.robot1_p8->setText(QString::number(msg->position[16] * DEGREE , 'f', 3));
                ui_.robot1_p9->setText(QString::number(msg->position[17] * DEGREE , 'f', 3));

                q_[0](0) = 0;
                q_[0](1) = 0;
                for (int i=0; i<7; i++)
                    q_[0](i+2) = msg->position[11+i];

            }
            else{
                ui_.robot1_p1->setText(QString::number(msg->position[0] * DEGREE , 'f', 3));
                ui_.robot1_p2->setText(QString::number(msg->position[1] * DEGREE , 'f', 3));
                ui_.robot1_p3->setText(QString::number(msg->position[2] * DEGREE , 'f', 3));
                ui_.robot1_p4->setText(QString::number(msg->position[3] * DEGREE , 'f', 3));
                ui_.robot1_p5->setText(QString::number(msg->position[4] * DEGREE , 'f', 3));
                ui_.robot1_p6->setText(QString::number(msg->position[5] * DEGREE , 'f', 3));
                ui_.robot1_p7->setText(QString::number(msg->position[6] * DEGREE , 'f', 3));
                ui_.robot1_p8->setText(QString::number(msg->position[7] * DEGREE , 'f', 3));
                ui_.robot1_p9->setText(QString::number(msg->position[8] * DEGREE , 'f', 3));

                q_[0](0) = 0;
                q_[0](1) = 0;
                for (int i=0; i<7; i++)
                    q_[0](i+2) = msg->position[2+i];
            }

            if (issimulation_[0]){
                for (int i=0; i<4; i++)
                    joint_state_msg_[0].position[i] = msg->position[i+7];

                for (int i=0; i<7; i++)
                    joint_state_msg_[0].position[i+4] = msg->position[i+11];

                for (int i=0; i<2; i++)
                    joint_state_msg_[0].position[i+11] = msg->position[i+18];
            }
            else{
                joint_state_msg_[0].position[0] = msg->position[0];
                joint_state_msg_[0].position[2] = msg->position[0];
                joint_state_msg_[0].position[1] = msg->position[1];
                joint_state_msg_[0].position[3] = msg->position[1];

                for (int i=0; i<7; i++)
                    joint_state_msg_[0].position[i+4] = msg->position[i+2];

                for (int i=0; i<2; i++)
                    joint_state_msg_[0].position[i+11] = msg->position[i+9];
            }
                         
            joint_state_msg_[0].header.stamp = ros::Time::now();            
            joint_state_pub_[0].publish(joint_state_msg_[0]);

        }; 
        virtual void Jointcb1(const sensor_msgs::JointStateConstPtr &msg){
            if (issimulation_[1]){
                ui_.robot2_p1->setText(QString::number(msg->position[7] * DEGREE , 'f', 3));
                ui_.robot2_p2->setText(QString::number(msg->position[8] * DEGREE , 'f', 3));
                ui_.robot2_p3->setText(QString::number(msg->position[11] * DEGREE , 'f', 3));
                ui_.robot2_p4->setText(QString::number(msg->position[12] * DEGREE , 'f', 3));
                ui_.robot2_p5->setText(QString::number(msg->position[13] * DEGREE , 'f', 3));
                ui_.robot2_p6->setText(QString::number(msg->position[14] * DEGREE , 'f', 3));
                ui_.robot2_p7->setText(QString::number(msg->position[15] * DEGREE , 'f', 3));
                ui_.robot2_p8->setText(QString::number(msg->position[16] * DEGREE , 'f', 3));
                ui_.robot2_p9->setText(QString::number(msg->position[17] * DEGREE , 'f', 3));

                q_[1](0) = 0;
                q_[1](1) = 0;
                for (int i=0; i<7; i++)
                    q_[1](i+2) = msg->position[11+i];
            }
            else{
                ui_.robot2_p1->setText(QString::number(msg->position[0] * DEGREE , 'f', 3));
                ui_.robot2_p2->setText(QString::number(msg->position[1] * DEGREE , 'f', 3));
                ui_.robot2_p3->setText(QString::number(msg->position[2] * DEGREE , 'f', 3));
                ui_.robot2_p4->setText(QString::number(msg->position[3] * DEGREE , 'f', 3));
                ui_.robot2_p5->setText(QString::number(msg->position[4] * DEGREE , 'f', 3));
                ui_.robot2_p6->setText(QString::number(msg->position[5] * DEGREE , 'f', 3));
                ui_.robot2_p7->setText(QString::number(msg->position[6] * DEGREE , 'f', 3));
                ui_.robot2_p8->setText(QString::number(msg->position[7] * DEGREE , 'f', 3));
                ui_.robot2_p9->setText(QString::number(msg->position[8] * DEGREE , 'f', 3));

                q_[1](0) = 0;
                q_[1](1) = 0;
                for (int i=0; i<7; i++)
                    q_[1](i+2) = msg->position[2+i];
            }

            if (issimulation_[1]){
                for (int i=0; i<4; i++)
                    joint_state_msg_[1].position[i] = msg->position[i+7];

                for (int i=0; i<7; i++)
                    joint_state_msg_[1].position[i+4] = msg->position[i+11];

                for (int i=0; i<2; i++)
                    joint_state_msg_[1].position[i+11] = msg->position[i+18];
            }
            else{
                joint_state_msg_[1].position[0] = msg->position[0];
                joint_state_msg_[1].position[2] = msg->position[0];
                joint_state_msg_[1].position[1] = msg->position[1];
                joint_state_msg_[1].position[3] = msg->position[1];

                for (int i=0; i<7; i++)
                    joint_state_msg_[1].position[i+4] = msg->position[i+2];

                for (int i=0; i<2; i++)
                    joint_state_msg_[1].position[i+11] = msg->position[i+9];
            }
                         
            joint_state_msg_[1].header.stamp = ros::Time::now();            
            joint_state_pub_[1].publish(joint_state_msg_[1]);

        }; 
   
        virtual void Basecb0(const sensor_msgs::JointStateConstPtr& msg){
            int j = 0;
            ui_.base0_p1->setText(QString::number(msg->position[0] , 'f', 3));
            ui_.base0_p2->setText(QString::number(msg->position[1] , 'f', 3));
            ui_.base0_p3->setText(QString::number(msg->position[2] * DEGREE, 'f', 3));

            for (int i=0; i<3; i++)
                base_[j](i) = msg->position[i];
        };
        virtual void Basecb1(const sensor_msgs::JointStateConstPtr& msg){
            int j = 1;
            ui_.base1_p1->setText(QString::number(msg->position[0] , 'f', 3));
            ui_.base1_p2->setText(QString::number(msg->position[1] , 'f', 3));
            ui_.base1_p3->setText(QString::number(msg->position[2] * DEGREE, 'f', 3));

            for (int i=0; i<3; i++)
                base_[j](i) = msg->position[i];
        };

        virtual void EEcb0(const geometry_msgs::TransformConstPtr& msg){
            int i = 0;
            ui_.ee0_p1->setText(QString::number(msg->translation.x , 'f', 3));
            ui_.ee0_p2->setText(QString::number(msg->translation.y , 'f', 3));
            ui_.ee0_p3->setText(QString::number(msg->translation.z , 'f', 3));

            _quat[i].x() = msg->rotation.x;
            _quat[i].y() = msg->rotation.y;
            _quat[i].z() = msg->rotation.z;
            _quat[i].w() = msg->rotation.w;

            _angles[i] = _quat[i].toRotationMatrix().eulerAngles(0,1,2);
            ui_.ee0_p4->setText(QString::number(_angles[i](0) * DEGREE , 'f', 3));
            ui_.ee0_p5->setText(QString::number(_angles[i](1) * DEGREE , 'f', 3));
            ui_.ee0_p6->setText(QString::number(_angles[i](2) * DEGREE , 'f', 3));
            
            x_[i](0) = msg->translation.x;
            x_[i](1) = msg->translation.y;
            x_[i](2) = msg->translation.z;
            x_[i].tail(3) = _angles[i];
        };

        virtual void EEcb1(const geometry_msgs::TransformConstPtr& msg){
            int i = 1;
            ui_.ee1_p1->setText(QString::number(msg->translation.x , 'f', 3));
            ui_.ee1_p2->setText(QString::number(msg->translation.y , 'f', 3));
            ui_.ee1_p3->setText(QString::number(msg->translation.z , 'f', 3));

            _quat[i].x() = msg->rotation.x;
            _quat[i].y() = msg->rotation.y;
            _quat[i].z() = msg->rotation.z;
            _quat[i].w() = msg->rotation.w;

            _angles[i] = _quat[i].toRotationMatrix().eulerAngles(0,1,2);
            ui_.ee1_p4->setText(QString::number(_angles[i](0) * DEGREE , 'f', 3));
            ui_.ee1_p5->setText(QString::number(_angles[i](1) * DEGREE , 'f', 3));
            ui_.ee1_p6->setText(QString::number(_angles[i](2) * DEGREE , 'f', 3));
            
            x_[i](0) = msg->translation.x;
            x_[i](1) = msg->translation.y;
            x_[i](2) = msg->translation.z;
            x_[i].tail(3) = _angles[i];
        };

    signals:
        void timerCallback(const std_msgs::Float32ConstPtr &msg);
        void jointStateCallback0(const sensor_msgs::JointStateConstPtr& msg);
        void jointStateCallback1(const sensor_msgs::JointStateConstPtr& msg);
        void baseStateCallback0(const sensor_msgs::JointStateConstPtr& msg);
        void baseStateCallback1(const sensor_msgs::JointStateConstPtr& msg);
        void eeStateCallback0(const geometry_msgs::TransformConstPtr& msg);
        void eeStateCallback1(const geometry_msgs::TransformConstPtr& msg);

    protected slots:
        virtual void customctrlbtn(){
            ui_.stackedWidget->setCurrentIndex(0);
        };
        virtual void jointctrlbtn(){
            ui_.stackedWidget->setCurrentIndex(1);
        };
        virtual void se3ctrlbtn(){
            ui_.stackedWidget->setCurrentIndex(2);
        };
        virtual void basectrlbtn(){
            ui_.stackedWidget->setCurrentIndex(3);
        };
        virtual void jointctrlcb(){
            // joint target
            sensor_msgs::JointState c_joint, t_joint;
            int robot = ui_.husky_num->currentIndex(); 
            c_joint.position.resize(7);
            t_joint.position.resize(7);
            
            for (int i=0; i<7; i++){
                c_joint.position[i] = q_[robot](i+2);
            }
            t_joint.position[0] = ui_.joint_t1->text().toFloat() * RAD;
            t_joint.position[1] = ui_.joint_t2->text().toFloat() * RAD;
            t_joint.position[2] = ui_.joint_t3->text().toFloat() * RAD;
            t_joint.position[3] = ui_.joint_t4->text().toFloat() * RAD;
            t_joint.position[4] = ui_.joint_t5->text().toFloat() * RAD;
            t_joint.position[5] = ui_.joint_t6->text().toFloat() * RAD;
            t_joint.position[6] = ui_.joint_t7->text().toFloat() * RAD;

            // joint kp, kv
            std::vector<double> kp, kv; 
             
            kp.push_back(ui_.joint_kp1->text().toFloat());
            kp.push_back(ui_.joint_kp2->text().toFloat());
            kp.push_back(ui_.joint_kp3->text().toFloat());
            kp.push_back(ui_.joint_kp4->text().toFloat());
            kp.push_back(ui_.joint_kp5->text().toFloat());
            kp.push_back(ui_.joint_kp6->text().toFloat());
            kp.push_back(ui_.joint_kp7->text().toFloat());

            if (ui_.ideal_kv->isChecked()){
                ui_.joint_kv1->setText(QString::number(2.0*std::sqrt(kp[0]), 'f', 3));
                ui_.joint_kv2->setText(QString::number(2.0*std::sqrt(kp[1]), 'f', 3));
                ui_.joint_kv3->setText(QString::number(2.0*std::sqrt(kp[2]), 'f', 3));
                ui_.joint_kv4->setText(QString::number(2.0*std::sqrt(kp[3]), 'f', 3));
                ui_.joint_kv5->setText(QString::number(2.0*std::sqrt(kp[4]), 'f', 3));
                ui_.joint_kv6->setText(QString::number(2.0*std::sqrt(kp[5]), 'f', 3));
                ui_.joint_kv7->setText(QString::number(2.0*std::sqrt(kp[6]), 'f', 3));                
            }

            kv.push_back(ui_.joint_kv1->text().toFloat());
            kv.push_back(ui_.joint_kv2->text().toFloat());
            kv.push_back(ui_.joint_kv3->text().toFloat());
            kv.push_back(ui_.joint_kv4->text().toFloat());
            kv.push_back(ui_.joint_kv5->text().toFloat());
            kv.push_back(ui_.joint_kv6->text().toFloat());
            kv.push_back(ui_.joint_kv7->text().toFloat());

            // joint masking
            std::vector<bool> mask;
            mask.push_back(ui_.joint_m1->isChecked());
            mask.push_back(ui_.joint_m2->isChecked());
            mask.push_back(ui_.joint_m3->isChecked());
            mask.push_back(ui_.joint_m4->isChecked());
            mask.push_back(ui_.joint_m5->isChecked());
            mask.push_back(ui_.joint_m6->isChecked());
            mask.push_back(ui_.joint_m7->isChecked());
            
            for (int i=0; i<7; i++)
                if (mask[i] == 0)
                    t_joint.position[i] = c_joint.position[i];
             
            plan_joint_srv_[robot].request.current_joint = c_joint;
            plan_joint_srv_[robot].request.target_joint.clear();
            plan_joint_srv_[robot].request.target_joint.push_back(t_joint);
            plan_joint_srv_[robot].request.traj_type = ui_.joint_trajectory_mode->currentIndex();
            plan_joint_srv_[robot].request.kp = kp;
            plan_joint_srv_[robot].request.kv = kv;
            
            plan_joint_srv_[robot].request.mask.resize(7);
            for (int i=0; i<7; i++)
                plan_joint_srv_[robot].request.mask[i].data = mask[i];

            string traj_type;
            if (ui_.joint_trajectory_mode->currentIndex() == 0){
                traj_type = "Constant";
                plan_joint_srv_[robot].request.duration= 0.0;
                plan_joint_srv_[robot].request.vel_limit = 0;
                plan_joint_srv_[robot].request.acc_limit = 0;

            }
            else if (ui_.joint_trajectory_mode->currentIndex() == 1){
                traj_type = "Cubic Spline";
                plan_joint_srv_[robot].request.duration= ui_.joint_trajectory_duration->text().toFloat();
                plan_joint_srv_[robot].request.vel_limit = 0;
                plan_joint_srv_[robot].request.acc_limit = 0;

            }
            else{
                traj_type = "Time Optimal Spline";
                plan_joint_srv_[robot].request.duration= 0.0;
                if (target_q_vec_.size() > 0)
                    plan_joint_srv_[robot].request.target_joint = target_q_vec_;

                plan_joint_srv_[robot].request.vel_limit = ui_.joint_trajectory_tvel->text().toFloat();
                plan_joint_srv_[robot].request.acc_limit = ui_.joint_trajectory_tacc->text().toFloat();

            }

            joint_plan_client_[robot].call(plan_joint_srv_[robot]);
            ROS_WARN("%f", t_joint.position[0]);
            ROS_WARN("%d", ui_.joint_trajectory_mode->currentIndex());

            string log;
            log = "Time: " + to_string(time_) + "\n";
            log += "Trajectory Type: " + traj_type + "\n";
            
            if (ui_.joint_trajectory_mode->currentIndex() == 2 && target_q_vec_.size() > 0){                
                for (int j=0; j<target_q_vec_.size(); j++){
                    log += "Desired Joint Targets: Waypoint #" + to_string(j) +"\t";
                    for (int i=0;i<7;i++)
                        log += to_string(target_q_vec_[j].position[i] *DEGREE) + "\t";
                }
            }
            else{
                log += "Desired Joint Target:\t";
                for (int i=0;i<7;i++)
                    log += to_string(t_joint.position[i] *DEGREE) + "\t";
            }
            ui_.joint_trajectory_log->appendPlainText(QString::fromStdString(log));
            target_q_vec_.clear();

            kimm_joint_planner_ros_interface::JointAction joint_action_msg;
            joint_action_msg.kp = plan_joint_srv_[robot].request.kp;
            joint_action_msg.kv = plan_joint_srv_[robot].request.kv;
            joint_action_msg.duration = plan_joint_srv_[robot].request.duration;
            joint_action_msg.target_joint = plan_joint_srv_[robot].request.target_joint;
            joint_action_msg.traj_type = plan_joint_srv_[robot].request.traj_type;

            joint_action_pub_[robot].publish(joint_action_msg);



        }
        virtual void jointctrlcb2(){
            int i = ui_.husky_num->currentIndex(); 
            int16_msg_[i].data = 900;
            custom_ctrl_pub_[i].publish(int16_msg_[i]);
        }
        virtual void jointctrlcb3(){
            int i = ui_.husky_num->currentIndex();           
            ui_.joint_t1->setText(QString::number(q_[i](2) * DEGREE, 'f', 3));
            ui_.joint_t2->setText(QString::number(q_[i](3) * DEGREE, 'f', 3));
            ui_.joint_t3->setText(QString::number(q_[i](4) * DEGREE, 'f', 3));
            ui_.joint_t4->setText(QString::number(q_[i](5) * DEGREE, 'f', 3));
            ui_.joint_t5->setText(QString::number(q_[i](6) * DEGREE, 'f', 3));
            ui_.joint_t6->setText(QString::number(q_[i](7) * DEGREE, 'f', 3));
            ui_.joint_t7->setText(QString::number(q_[i](8) * DEGREE, 'f', 3));            
        }
        virtual void jointctrlcb4(){
            sensor_msgs::JointState t_joint;
            t_joint.position.resize(7);
  
            t_joint.position[0] = ui_.joint_t1->text().toFloat() * RAD;
            t_joint.position[1] = ui_.joint_t2->text().toFloat() * RAD;
            t_joint.position[2] = ui_.joint_t3->text().toFloat() * RAD;
            t_joint.position[3] = ui_.joint_t4->text().toFloat() * RAD;
            t_joint.position[4] = ui_.joint_t5->text().toFloat() * RAD;
            t_joint.position[5] = ui_.joint_t6->text().toFloat() * RAD;
            t_joint.position[6] = ui_.joint_t7->text().toFloat() * RAD;
            
            string log;
            log = "Add Waypoint:\t" ;

            for (int i=0;i<7;i++)
                log += to_string(t_joint.position[i] *DEGREE) + "\t";
            ui_.joint_trajectory_log->appendPlainText(QString::fromStdString(log));
            target_q_vec_.push_back(t_joint);
        }
        virtual void jointctrlcb5(){
            target_q_vec_.clear();
            string log;
            log = "Clear Waypoints" ;
            ui_.joint_trajectory_log->appendPlainText(QString::fromStdString(log));
        }
        virtual void basectrlcb1(){
            // base target
            int robot = ui_.husky_num->currentIndex(); 
            geometry_msgs::Pose2D c_pose, t_pose;
                        
            plan_mobile_srv_[robot].request.current_mobile_state.x = base_[robot](0);
            plan_mobile_srv_[robot].request.current_mobile_state.y = base_[robot](1);
            plan_mobile_srv_[robot].request.current_mobile_state.theta = base_[robot](2);

            plan_mobile_srv_[robot].request.target_mobile_pose.x = ui_.base_d1->text().toFloat();
            plan_mobile_srv_[robot].request.target_mobile_pose.y = ui_.base_d2->text().toFloat();
            plan_mobile_srv_[robot].request.target_mobile_pose.theta = ui_.base_d3->text().toFloat() * RAD;

            // obstacle
            obs_vec_.clear();
            int n_obs = ui_.obs_list->count(); 

            Eigen::VectorXd obs(4);
            for (int i=0; i< n_obs; i++){
                QListWidgetItem *item = ui_.obs_list->item(i);
                string obs_inf = item->text().toUtf8().constData();
                istringstream ss(obs_inf);
                string buffer;
                int j=0;
                while (getline(ss, buffer, ',')){
                    obs(j) = stod(buffer);
                    j++;
                }
                kimm_path_planner_ros_interface::Obstacle2D obs_2d;
                obs_2d.x1.data = obs(0);
                obs_2d.y1.data = obs(1);
                obs_2d.x2.data = obs(2);
                obs_2d.y2.data = obs(3);
                obs_vec_.push_back(obs_2d);
            }
            plan_mobile_srv_[robot].request.Obstacles2D = obs_vec_;
            mobile_plan_client_[robot].call(plan_mobile_srv_[robot]);
            
            
            string log;
            log = "Time: " + to_string(time_) + "\n";
            log += "Current Position: " + to_string(base_[robot](0)) + " " + to_string(base_[robot](1)) + " " + to_string(base_[robot](2)) + "\n";
            log += "Target Position: " + to_string(ui_.base_d1->text().toFloat()) + " " + to_string(ui_.base_d2->text().toFloat()) + " " + to_string(ui_.base_d3->text().toFloat()) + "\n";
            if (plan_mobile_srv_[robot].response.mobile_path.points.size() >= 1)
                log += "RRT based planner: Succeed\n";
            else{
                log += "RRT based planner: Failed\n";
            }
            
            ui_.base_log->appendPlainText(QString::fromStdString(log));

            //display
            visualization_msgs::MarkerArray response_list, req_list;
            int id = 0;

            visualization_msgs::Marker marker;
            marker.header.frame_id = "husky_odom";
            marker.ns = "my_namespace";
            marker.pose.position.z = 0;
            
            marker.scale.x = 0.1;
            marker.scale.y = 0.015;
            marker.scale.z = 0.015;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker.type = visualization_msgs::Marker::ARROW;

            for (uint32_t i = 0; i < plan_mobile_srv_[robot].response.mobile_path.points.size(); i=i+2)
            {
                marker.id = id;
                                
                marker.pose.position.x = plan_mobile_srv_[robot].response.mobile_path.points[i].x;
                marker.pose.position.y = plan_mobile_srv_[robot].response.mobile_path.points[i].y;

                double theta = plan_mobile_srv_[robot].response.mobile_path.points[i].theta;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = sin(theta/2.);
                marker.pose.orientation.w = cos(theta/2.);
                
                id++;
                response_list.markers.push_back(marker);
            }

            base_traj_resp_pub_[robot].publish(response_list);

            id = 0;
            marker.id = id;       
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;   
            marker.pose.position.x = plan_mobile_srv_[robot].request.target_mobile_pose.x;
            marker.pose.position.y = plan_mobile_srv_[robot].request.target_mobile_pose.y;

            double theta = plan_mobile_srv_[robot].request.target_mobile_pose.theta;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = sin(theta/2.);
            marker.pose.orientation.w = cos(theta/2.);
            id++;
            req_list.markers.push_back(marker);
            
            marker.type = visualization_msgs::Marker::CUBE;
            for (int i=0; i< n_obs; i++){
                marker.id = id;  
                marker.pose.position.x = (obs_vec_[i].x1.data + obs_vec_[i].x2.data)/2.0;
                marker.pose.position.y = (obs_vec_[i].y1.data + obs_vec_[i].y2.data)/2.0;
                marker.pose.position.z = 0.2;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = abs(obs_vec_[i].x1.data - obs_vec_[i].x2.data);
                marker.scale.y = abs(obs_vec_[i].y1.data - obs_vec_[i].y2.data);
                marker.scale.z = 0.4;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                id++;
                req_list.markers.push_back(marker);
            }

            base_traj_req_pub_[robot].publish(req_list);

            kimm_path_planner_ros_interface::MobileTrajectory mobile_msg;
            mobile_msg.points = plan_mobile_srv_[robot].response.mobile_path.points;
            mobile_action_pub_[robot].publish(mobile_msg);
        }
        virtual void basectrlcb2(){
            int robot = ui_.husky_num->currentIndex(); 
            int16_msg_[robot].data = 901;
            custom_ctrl_pub_[robot].publish(int16_msg_[robot]);
        }
        virtual void basectrlcb3(){
            int robot = ui_.husky_num->currentIndex(); 
            int16_msg_[robot].data = 902;
            custom_ctrl_pub_[robot].publish(int16_msg_[robot]);
        }
        virtual void basectrlcb4(){
            int robot = ui_.husky_num->currentIndex(); 
            int16_msg_[robot].data = 903;
            custom_ctrl_pub_[robot].publish(int16_msg_[robot]);
        }
        virtual void basectrlcb5(){
            string obs_msg;

            obs_msg = to_string(ui_.obs_x1->text().toFloat());
            obs_msg += "," + to_string(ui_.obs_y1->text().toFloat());
            obs_msg += "," + to_string(ui_.obs_x2->text().toFloat());
            obs_msg += "," + to_string(ui_.obs_y2->text().toFloat());

            ui_.obs_list->addItem(obs_msg.c_str());
        }
        virtual void basectrlcb6(){
            QListWidgetItem *item = ui_.obs_list->takeItem(ui_.obs_list->currentIndex().row());
            ui_.obs_list->removeItemWidget(item);
        }
        virtual void se3ctrlcb1(){
            int robot = ui_.husky_num->currentIndex(); 

            // se3 target
            geometry_msgs::Transform c_se3, t_se3;
            c_se3.translation.x = x_[robot](0);
            c_se3.translation.y = x_[robot](1);
            c_se3.translation.z = x_[robot](2);

            Eigen::Quaternion<double> quat_tmp;
            Eigen::AngleAxisd rollAngle(x_[robot](3), Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(x_[robot](4), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(x_[robot](5), Eigen::Vector3d::UnitZ());
            quat_tmp = rollAngle * pitchAngle * yawAngle;
            c_se3.rotation.x = quat_tmp.x();
            c_se3.rotation.y = quat_tmp.y();
            c_se3.rotation.z = quat_tmp.z();
            c_se3.rotation.w = quat_tmp.w();

            std::vector<bool> mask;
            mask.push_back(ui_.se3_m1->isChecked());
            mask.push_back(ui_.se3_m2->isChecked());
            mask.push_back(ui_.se3_m3->isChecked());
            mask.push_back(ui_.se3_m4->isChecked());
            mask.push_back(ui_.se3_m5->isChecked());
            mask.push_back(ui_.se3_m6->isChecked());

            if (ui_.use_rel->isChecked()){
                t_se3 = c_se3;
                if (mask[0] == 1)
                    t_se3.translation.x += ui_.se3_x->text().toFloat();
                if (mask[1] == 1)
                    t_se3.translation.y += ui_.se3_y->text().toFloat();
                if (mask[2] == 1)
                    t_se3.translation.z += ui_.se3_z->text().toFloat();
                if (mask[3] == 1)
                    rollAngle = Eigen::AngleAxisd(x_[robot](3) + ui_.se3_roll->text().toFloat() * RAD, Eigen::Vector3d::UnitX());
                else
                    rollAngle = Eigen::AngleAxisd(x_[robot](3), Eigen::Vector3d::UnitX());
                if (mask[4] == 1)
                    pitchAngle = Eigen::AngleAxisd(x_[robot](4) + ui_.se3_pitch->text().toFloat() * RAD, Eigen::Vector3d::UnitY());
                else
                    pitchAngle = Eigen::AngleAxisd(x_[robot](4), Eigen::Vector3d::UnitY());
                if (mask[5] == 1)
                    yawAngle = Eigen::AngleAxisd(x_[robot](5) + ui_.se3_yaw->text().toFloat() * RAD, Eigen::Vector3d::UnitZ());
                else
                    yawAngle = Eigen::AngleAxisd(x_[robot](5), Eigen::Vector3d::UnitZ());


               // quat_tmp = rollAngle * pitchAngle * yawAngle;
                t_se3.rotation.x = quat_tmp.x();
                t_se3.rotation.y = quat_tmp.y();
                t_se3.rotation.z = quat_tmp.z();
                t_se3.rotation.w = quat_tmp.w();
            }
            else{
                t_se3 = c_se3;
                if (mask[0] == 1)
                    t_se3.translation.x = ui_.se3_x->text().toFloat();
                if (mask[1] == 1)
                    t_se3.translation.y = ui_.se3_y->text().toFloat();
                if (mask[2] == 1)
                    t_se3.translation.z = ui_.se3_z->text().toFloat();
                if (mask[3] == 1)
                    rollAngle = Eigen::AngleAxisd(ui_.se3_roll->text().toFloat() * RAD, Eigen::Vector3d::UnitX());
                else
                    rollAngle = Eigen::AngleAxisd(x_[robot](3), Eigen::Vector3d::UnitX());
                if (mask[4] == 1)
                    pitchAngle = Eigen::AngleAxisd(ui_.se3_pitch->text().toFloat() * RAD, Eigen::Vector3d::UnitY());
                else
                    pitchAngle = Eigen::AngleAxisd(x_[robot](4), Eigen::Vector3d::UnitY());
                if (mask[5] == 1)
                    yawAngle = Eigen::AngleAxisd(ui_.se3_yaw->text().toFloat() * RAD, Eigen::Vector3d::UnitZ());
                else
                    yawAngle = Eigen::AngleAxisd(x_[robot](5), Eigen::Vector3d::UnitZ());

               //quat_tmp = rollAngle * pitchAngle * yawAngle;
                t_se3.rotation.x = quat_tmp.x();
                t_se3.rotation.y = quat_tmp.y();
                t_se3.rotation.z = quat_tmp.z();
                t_se3.rotation.w = quat_tmp.w();
            }

            std::vector<double> kp, kv; 
            kp.push_back(ui_.se3_kp1->text().toFloat());
            kp.push_back(ui_.se3_kp2->text().toFloat());
            kp.push_back(ui_.se3_kp3->text().toFloat());
            kp.push_back(ui_.se3_kp4->text().toFloat());
            kp.push_back(ui_.se3_kp5->text().toFloat());
            kp.push_back(ui_.se3_kp6->text().toFloat());

            if (ui_.ideal_kv->isChecked()){
                ui_.se3_kv1->setText(QString::number(2.0*std::sqrt(kp[0]), 'f', 3));
                ui_.se3_kv2->setText(QString::number(2.0*std::sqrt(kp[1]), 'f', 3));
                ui_.se3_kv3->setText(QString::number(2.0*std::sqrt(kp[2]), 'f', 3));
                ui_.se3_kv4->setText(QString::number(2.0*std::sqrt(kp[3]), 'f', 3));
                ui_.se3_kv5->setText(QString::number(2.0*std::sqrt(kp[4]), 'f', 3));
                ui_.se3_kv6->setText(QString::number(2.0*std::sqrt(kp[5]), 'f', 3));             
            }          

            kv.push_back(ui_.se3_kv1->text().toFloat());
            kv.push_back(ui_.se3_kv2->text().toFloat());
            kv.push_back(ui_.se3_kv3->text().toFloat());
            kv.push_back(ui_.se3_kv4->text().toFloat());
            kv.push_back(ui_.se3_kv5->text().toFloat());
            kv.push_back(ui_.se3_kv6->text().toFloat());

            plan_se3_srv_[robot].request.current_se3 = c_se3;
            plan_se3_srv_[robot].request.target_se3.clear();
            plan_se3_srv_[robot].request.target_se3.push_back(t_se3);
            plan_se3_srv_[robot].request.traj_type = ui_.se3_trajectory_mode->currentIndex();
            plan_se3_srv_[robot].request.iswholebody.data = ui_.wholebody_check->isChecked();
            plan_se3_srv_[robot].request.kp = kp;
            plan_se3_srv_[robot].request.kv = kv;
            
            plan_se3_srv_[robot].request.mask.resize(6);
            for (int i=0; i<6; i++)
                plan_se3_srv_[robot].request.mask[i].data = mask[i];

            string traj_type;
            if (ui_.se3_trajectory_mode->currentIndex() == 0){
                traj_type = "Constant";
                plan_se3_srv_[robot].request.duration= 0.0;
                plan_se3_srv_[robot].request.vel_limit = 0;
                plan_se3_srv_[robot].request.acc_limit = 0;

            }
            else if (ui_.se3_trajectory_mode->currentIndex() == 1){
                traj_type = "Cubic Spline";
                plan_se3_srv_[robot].request.duration= ui_.se3_trajectory_duration->text().toFloat();
                plan_se3_srv_[robot].request.vel_limit = 0;
                plan_se3_srv_[robot].request.acc_limit = 0;

            }
            else{
                traj_type = "Time Optimal Spline";
                plan_se3_srv_[robot].request.duration= 0.0;
                if (target_x_vec_.size() > 0)
                    plan_se3_srv_[robot].request.target_se3 = target_x_vec_;
                plan_se3_srv_[robot].request.vel_limit = ui_.se3_trajectory_tvel->text().toFloat();
                plan_se3_srv_[robot].request.acc_limit = ui_.se3_trajectory_tvel->text().toFloat();
            }

            se3_plan_client_[robot].call(plan_se3_srv_[robot]);
            
            string log;
            log = "Time: " + to_string(time_) + "\n";
            log += "Trajectory Type: " + traj_type + "\n";
            
            if (ui_.se3_trajectory_mode->currentIndex() == 2){                
                for (int j=0; j<target_x_vec_.size(); j++){
                    log += "Desired SE3 Targets: Waypoint #" + to_string(j) +"\t";
                    log += to_string(target_x_vec_[j].translation.x) + "\t" + to_string(target_x_vec_[j].translation.y) + "\t" +to_string(target_x_vec_[j].translation.z) + "\t" ;
                }
            }
            else{
                log += "Desired SE3 Target:\t";
                log += to_string(t_se3.translation.x) + "\t" + to_string(t_se3.translation.y) + "\t" +to_string(t_se3.translation.z) 
                       + "\t" + to_string(t_se3.rotation.x) + "\t" + to_string(t_se3.rotation.y) + "\t" +to_string(t_se3.rotation.z) + "\t"   + to_string(t_se3.rotation.w) + "\t"   ;
            }
            ui_.se3_trajectory_log->appendPlainText(QString::fromStdString(log));
            target_x_vec_.clear();

            //display
            visualization_msgs::MarkerArray response_list;
            int id = 0;

            visualization_msgs::Marker marker;
            marker.header.frame_id = "husky_odom";
            marker.ns = "my_namespace";
            marker.pose.position.z = 0;
            
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            marker.type = visualization_msgs::Marker::SPHERE;

            for (uint32_t i = 0; i < plan_se3_srv_[robot].response.res_traj.size(); i=i+10)
            {
                marker.id = id;
                                
                marker.pose.position.x = plan_se3_srv_[robot].response.res_traj[i].translation.x;
                marker.pose.position.y = plan_se3_srv_[robot].response.res_traj[i].translation.y;
                marker.pose.position.z = plan_se3_srv_[robot].response.res_traj[i].translation.z + 0.25;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                
                id++;
                response_list.markers.push_back(marker);
            }

            ee_traj_resp_pub_[robot].publish(response_list);

            kimm_se3_planner_ros_interface::SE3Action se3_action_msg;
            se3_action_msg.kp = plan_se3_srv_[robot].request.kp;
            se3_action_msg.kv = plan_se3_srv_[robot].request.kv;
            se3_action_msg.duration = plan_se3_srv_[robot].request.duration;
            se3_action_msg.target_se3 = plan_se3_srv_[robot].request.target_se3;
            se3_action_msg.traj_type = plan_se3_srv_[robot].request.traj_type;
            se3_action_msg.iswholebody = plan_se3_srv_[robot].request.iswholebody;

            se3_action_pub_[robot].publish(se3_action_msg);


        }
        virtual void se3ctrlcb2(){
            int robot = ui_.husky_num->currentIndex(); 
            int16_msg_[robot].data = 904;
            custom_ctrl_pub_[robot].publish(int16_msg_[robot]);
        }
        virtual void se3ctrlcb3(){
            int robot = ui_.husky_num->currentIndex(); 
            ui_.se3_x->setText(QString::number(x_[robot](0), 'f', 3));
            ui_.se3_y->setText(QString::number(x_[robot](1), 'f', 3));
            ui_.se3_z->setText(QString::number(x_[robot](2), 'f', 3));
            ui_.se3_roll->setText(QString::number(x_[robot](3) * DEGREE, 'f', 3));
            ui_.se3_pitch->setText(QString::number(x_[robot](4) * DEGREE, 'f', 3));
            ui_.se3_yaw->setText(QString::number(x_[robot](5) * DEGREE, 'f', 3));
        }
        virtual void se3ctrlcb4(){
            int robot = ui_.husky_num->currentIndex(); 
            geometry_msgs::Transform t_x;
            t_x.translation.x = ui_.se3_x->text().toFloat();
            t_x.translation.y = ui_.se3_y->text().toFloat();
            t_x.translation.z = ui_.se3_z->text().toFloat();

            Eigen::Quaternion<double> quat_tmp;
            Eigen::AngleAxisd rollAngle(x_[robot](3), Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(x_[robot](4), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(x_[robot](5), Eigen::Vector3d::UnitZ());
            quat_tmp = yawAngle * pitchAngle * rollAngle;
            
            t_x.rotation.x = quat_tmp.x();
            t_x.rotation.y = quat_tmp.y();
            t_x.rotation.z = quat_tmp.z();
            t_x.rotation.w = quat_tmp.w();
                        
            string log;
            log = "Add Waypoint:\t" ;
            log += to_string(t_x.translation.x) + "\t" + to_string(t_x.translation.y) + "\t" +to_string(t_x.translation.z) ;
            ui_.se3_trajectory_log->appendPlainText(QString::fromStdString(log));
            target_x_vec_.push_back(t_x);
        }
        virtual void se3ctrlcb5(){
            target_x_vec_.clear();
            string log;
            log = "Clear Waypoints" ;
            ui_.se3_trajectory_log->appendPlainText(QString::fromStdString(log));
        }

    public:
        ros::Publisher run_pub_[2], quit_pub_[2], custom_ctrl_pub_[2], base_traj_resp_pub_[2], base_traj_req_pub_[2], obs_pub_[2], ee_traj_resp_pub_[2];
        ros::Publisher joint_action_pub_[2], se3_action_pub_[2], mobile_action_pub_[2];
        ros::Subscriber simtime_sub_[2], jointstate_sub_[2], torquestate_sub_[2], base_state_sub_[2], ee_state_sub_[2];
        ros::ServiceClient joint_plan_client_[2], mobile_plan_client_[2], se3_plan_client_[2];

        kimm_path_planner_ros_interface::plan_mobile_path plan_mobile_srv_[2];
        kimm_se3_planner_ros_interface::plan_se3_path plan_se3_srv_[2];
        kimm_joint_planner_ros_interface::plan_joint_path plan_joint_srv_[2];


        Eigen::VectorXd q_[2], x_[2];
        Eigen::Vector3d base_[2];
        Eigen::MatrixXd rot_[2];
        std::vector<sensor_msgs::JointState> target_q_vec_;
        std::vector<geometry_msgs::Transform>  target_x_vec_;
        std::vector<kimm_path_planner_ros_interface::Obstacle2D> obs_vec_;
        bool issimulation_[2];       
        string group_name_[2];

        std_msgs::Bool bool_msg_[2];
        std_msgs::Int16 int16_msg_[2];
        Eigen::Quaternion<double> _quat[2];
        Eigen::Vector3d _angles[2];
        Model model_[2];    

        ros::Publisher joint_state_pub_[2];
        sensor_msgs::JointState joint_state_msg_[2];
    };

}
 // namespace kimm_husky_gui
Q_DECLARE_METATYPE(std_msgs::StringConstPtr);
Q_DECLARE_METATYPE(std_msgs::Float32ConstPtr);
Q_DECLARE_METATYPE(std_msgs::Float64ConstPtr);
Q_DECLARE_METATYPE(sensor_msgs::JointStateConstPtr);
Q_DECLARE_METATYPE(mujoco_ros_msgs::JointSetConstPtr);
Q_DECLARE_METATYPE(geometry_msgs::TransformConstPtr);


#endif
