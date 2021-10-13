
#include "kimm_multi_husky_gui/kimm_multi_husky_gui.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <vector>
#include <QDebug>
#include "qxtglobalshortcut.h"

namespace kimm_multi_husky_gui
{
   HuskyGui::HuskyGui()
   : rqt_gui_cpp::Plugin(), widget_(0)
    {
        qRegisterMetaType<std_msgs::Float32ConstPtr>();
        qRegisterMetaType<std_msgs::Float64ConstPtr>();
        qRegisterMetaType<sensor_msgs::JointStateConstPtr>();
        qRegisterMetaType<mujoco_ros_msgs::JointSetConstPtr>();
        qRegisterMetaType<geometry_msgs::TransformConstPtr>();

        setObjectName("MultiHuskyGui");

        string model_path, urdf_name;
        int nq = 4 + 9;

        for (int i=0; i<2; i++){
            if (i == 0)
                nh_.getParam("/robot_group0", group_name_[0]);
            else
                nh_.getParam("/robot_group1", group_name_[1]);    //check

            nh_.getParam("/" + group_name_[i] +"/issimulation", issimulation_[i]);

            nh_.getParam("/" + group_name_[i] +"_gui/rviz_urdf_path", model_path);    
            nh_.getParam("/" + group_name_[i] +"_gui/rviz_urdf", urdf_name);  
            

            vector<string> package_dirs;
            package_dirs.push_back(model_path);
            string urdfFileName = package_dirs[0] + urdf_name;
            ROS_INFO_STREAM(urdfFileName);
            pinocchio::urdf::buildModel(urdfFileName, model_[i], false);       
           
            for (int j=1; j<14; j++)
                ROS_INFO_STREAM(model_[i].names[j]);

            run_pub_[i] = nh_.advertise<std_msgs::Bool>("/" + group_name_[i] + "/mujoco_ros/mujoco_ros_interface/sim_run", 100);
            quit_pub_[i] = nh_.advertise<std_msgs::Bool>("/" + group_name_[i] + "/mujoco_ros/mujoco_ros_interface/sim_quit", 100);

            base_traj_resp_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>("/" + group_name_[i]  + "_gui/kimm_mobile_plan_markers/mobile/response", 100);
            base_traj_req_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>("/" + group_name_[i]  + "_gui/kimm_mobile_plan_markers/mobile/request", 100);
            ee_traj_resp_pub_[i] = nh_.advertise<visualization_msgs::MarkerArray>("/" + group_name_[i]  + "_gui/kimm_se3_plan_markers/robot/response", 100);
            joint_state_pub_[i] = nh_.advertise<sensor_msgs::JointState>("/" + group_name_[i]  +"_gui/joint_states", 100);

            if (issimulation_[i]){
                custom_ctrl_pub_[i] = nh_.advertise<std_msgs::Int16>("/" + group_name_[i] + "/mujoco_ros/mujoco_ros_interface/ctrl_type", 100);
                simtime_sub_[i] = nh_.subscribe("/" + group_name_[i] + "/mujoco_ros_interface/sim_time", 100, &HuskyGui::timerCallback, this);
                
            }
            else {
                custom_ctrl_pub_[i] = nh_.advertise<std_msgs::Int16>("/" + group_name_[i] + "/real_robot/ctrl_type", 100);
                simtime_sub_[i] = nh_.subscribe("/" + group_name_[i] + "/time", 100, &HuskyGui::timerCallback, this);                
            }

            joint_plan_client_[i] = nh_.serviceClient<kimm_joint_planner_ros_interface::plan_joint_path>("/" + group_name_[i] + "_gui/kimm_joint_planner_ros_interface_server/plan_joint_path");
            mobile_plan_client_[i] = nh_.serviceClient<kimm_path_planner_ros_interface::plan_mobile_path>("/" + group_name_[i] + "_gui/kimm_path_planner_ros_interface_server/plan_mobile_path");
            se3_plan_client_[i] = nh_.serviceClient<kimm_se3_planner_ros_interface::plan_se3_path>("/" + group_name_[i] + "_gui/kimm_se3_planner_ros_interface_server/plan_se3_path");

            joint_state_msg_[i].name.resize(nq);
            joint_state_msg_[i].position.resize(nq);
            joint_state_msg_[i].velocity.resize(nq);
            joint_state_msg_[i].effort.resize(nq);
            
            for (int j=0; j<nq; j++){
                joint_state_msg_[i].name[j] = model_[i].names[j+1];
                joint_state_msg_[i].position[j] = 0.0;
                joint_state_msg_[i].velocity[j] = 0.0;
                joint_state_msg_[i].effort[j] = 0.0;
            }

            q_[i].setZero(9);
            x_[i].setZero(6);
            ROS_INFO_STREAM(group_name_[i]);
        }     

        if (issimulation_[0]){  
            jointstate_sub_[0] = nh_.subscribe("/" + group_name_[0] + "/mujoco_ros/mujoco_ros_interface/joint_states", 100, &HuskyGui::jointStateCallback0, this);
            ee_state_sub_[0] = nh_.subscribe("/" + group_name_[0] + "/mujoco_ros//mujoco_ros_interface/ee_state", 100, &HuskyGui::eeStateCallback0, this);
            base_state_sub_[0] = nh_.subscribe("/" + group_name_[0] + "/mujoco_ros/mujoco_ros_interface/base_state", 100, &HuskyGui::baseStateCallback0, this);
        }
        else{
            jointstate_sub_[0] = nh_.subscribe("/" + group_name_[0] + "/real_robot/joint_states", 100, &HuskyGui::jointStateCallback0, this);
            ee_state_sub_[0] = nh_.subscribe("/" + group_name_[0] + "/real_robot/ee_state", 100, &HuskyGui::eeStateCallback0, this);
            base_state_sub_[0] = nh_.subscribe("/" + group_name_[0] + "/real_robot/base_state", 100, &HuskyGui::baseStateCallback0, this);
        }
        if (issimulation_[1]){  
            jointstate_sub_[1] = nh_.subscribe("/" + group_name_[1] + "/mujoco_ros/mujoco_ros_interface/joint_states", 100, &HuskyGui::jointStateCallback1, this);
            ee_state_sub_[1] = nh_.subscribe("/" + group_name_[1] + "/mujoco_ros/mujoco_ros_interface/ee_state", 100, &HuskyGui::eeStateCallback1, this);
            base_state_sub_[1] = nh_.subscribe("/" + group_name_[1] + "/mujoco_ros//mujoco_ros_interface/base_state", 100, &HuskyGui::baseStateCallback1, this);
        }
        else{
            jointstate_sub_[1] = nh_.subscribe("/" + group_name_[1] + "/real_robot/joint_states", 100, &HuskyGui::jointStateCallback1, this);
            ee_state_sub_[1] = nh_.subscribe("/" + group_name_[1] + "/real_robot/ee_state", 100, &HuskyGui::eeStateCallback1, this);
            base_state_sub_[1] = nh_.subscribe("/" + group_name_[1] + "/real_robot/base_state", 100, &HuskyGui::baseStateCallback1, this);
        }
    }

    void HuskyGui::initPlugin(qt_gui_cpp::PluginContext& context)
    {
    	QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
        widget_ = new QWidget();
        int horizontalDpi = widget_->logicalDpiX(); 
        cout << horizontalDpi << endl;
        ui_.setupUi(widget_);

        if (context.serialNumber() > 1)
        {
            widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }
        context.addWidget(widget_);

        // ui_.simulon_button->setShortcut(QKeySequence(Qt::Key_R));
        ui_.simuloff_button->setShortcut(QKeySequence(Qt::Key_P));
        ui_.grasp_button->setShortcut(QKeySequence(Qt::Key_G));
        ui_.quit_button->setShortcut(QKeySequence(Qt::Key_Q));

        connect(this, &HuskyGui::jointStateCallback0, this, &HuskyGui::Jointcb0);
        connect(this, &HuskyGui::jointStateCallback1, this, &HuskyGui::Jointcb1);
        connect(this, &HuskyGui::baseStateCallback0, this, &HuskyGui::Basecb0);
        connect(this, &HuskyGui::baseStateCallback1, this, &HuskyGui::Basecb1);
        connect(this, &HuskyGui::eeStateCallback0, this, &HuskyGui::EEcb0);
        connect(this, &HuskyGui::eeStateCallback1, this, &HuskyGui::EEcb1);

         // Basic Interface to Mujoco
        connect(ui_.quit_button, SIGNAL(pressed()), this, SLOT(QuitCallback()));
        connect(ui_.simulon_button, SIGNAL(pressed()), this, SLOT(EnableSimulCallback()));
        connect(ui_.simuloff_button, SIGNAL(pressed()), this, SLOT(DisableSimulCallback()));
        connect(ui_.init_button, SIGNAL(pressed()), this, SLOT(InitCallback()));
        connect(ui_.grasp_button, SIGNAL(pressed()), this, SLOT(GraspCallback()));
        
        connect(ui_.husky1_estop, SIGNAL(pressed()), this, SLOT(DisableSimulCallback1()));
        connect(ui_.husky1_init, SIGNAL(pressed()), this, SLOT(InitCallback1()));
        connect(ui_.husky1_grasp, SIGNAL(pressed()), this, SLOT(GraspCallback1()));

        connect(ui_.husky2_estop, SIGNAL(pressed()), this, SLOT(DisableSimulCallback2()));
        connect(ui_.husky2_init, SIGNAL(pressed()), this, SLOT(InitCallback2()));
        connect(ui_.husky2_grasp, SIGNAL(pressed()), this, SLOT(GraspCallback2()));
        
       
        // // Custom Controller Interface to Mujoco
        connect(ui_.custom_ctrl_btn, SIGNAL(pressed()), this, SLOT(customctrlbtn()));
        ui_.custom_ctrl_btn->setShortcut(QKeySequence(Qt::Key_1));
        connect(ui_.Customctrlbtn_1, SIGNAL(pressed()), this, SLOT(CustomCtrl1Callback()));
        connect(ui_.Customctrlbtn_2, SIGNAL(pressed()), this, SLOT(CustomCtrl2Callback()));
        connect(ui_.Customctrlbtn_3, SIGNAL(pressed()), this, SLOT(CustomCtrl3Callback()));
        connect(ui_.Customctrlbtn_4, SIGNAL(pressed()), this, SLOT(CustomCtrl4Callback()));
        connect(ui_.Customctrlbtn_5, SIGNAL(pressed()), this, SLOT(CustomCtrl5Callback()));
        connect(ui_.Customctrlbtn_6, SIGNAL(pressed()), this, SLOT(CustomCtrl6Callback()));
        connect(ui_.Customctrlbtn_7, SIGNAL(pressed()), this, SLOT(CustomCtrl7Callback()));
        connect(ui_.Customctrlbtn_8, SIGNAL(pressed()), this, SLOT(CustomCtrl8Callback()));
        connect(ui_.Customctrlbtn_9, SIGNAL(pressed()), this, SLOT(CustomCtrl9Callback()));
        connect(ui_.Customctrlbtn_10, SIGNAL(pressed()), this, SLOT(CustomCtrl10Callback()));

        // Joint Controller Interface to Mujoco
        // connect(ui_.joint_ctrl_btn, SIGNAL(pressed()), this, SLOT(jointctrlbtn()));
        // ui_.joint_ctrl_btn->setShortcut(QKeySequence(Qt::Key_2));
        // connect(ui_.joint_trajectory_send, SIGNAL(pressed()), this, SLOT(jointctrlcb()));
        // connect(ui_.joint_trajectory_send_2, SIGNAL(pressed()), this, SLOT(jointctrlcb2()));
        // connect(ui_.c_joint, SIGNAL(pressed()), this, SLOT(jointctrlcb3()));
        // connect(ui_.waypoint_add_btn, SIGNAL(pressed()), this, SLOT(jointctrlcb4()));
        // connect(ui_.waypoint_clear_btn, SIGNAL(pressed()), this, SLOT(jointctrlcb5()));

        // // SE3 Controller Interface to Mujoco
        // connect(ui_.se3_ctrl_btn, SIGNAL(pressed()), this, SLOT(se3ctrlbtn()));
        // ui_.se3_ctrl_btn->setShortcut(QKeySequence(Qt::Key_3));
        // connect(ui_.se3_trajectory_send_1, SIGNAL(pressed()), this, SLOT(se3ctrlcb1()));
        // connect(ui_.se3_trajectory_send_2, SIGNAL(pressed()), this, SLOT(se3ctrlcb2()));
        // connect(ui_.c_joint_2, SIGNAL(pressed()), this, SLOT(se3ctrlcb3()));
        // connect(ui_.waypoint_add_btn_2, SIGNAL(pressed()), this, SLOT(se3ctrlcb4()));
        // connect(ui_.waypoint_clear_btn_2, SIGNAL(pressed()), this, SLOT(se3ctrlcb5()));

        // // Base Controller Interface to Mujoco
        // connect(ui_.base_ctrl_btn, SIGNAL(pressed()), this, SLOT(basectrlbtn()));
        // ui_.base_ctrl_btn->setShortcut(QKeySequence(Qt::Key_4));
        // connect(ui_.base_p_set_btn, SIGNAL(pressed()), this, SLOT(basectrlcb1())); // set configuration
        // connect(ui_.base_p_run, SIGNAL(pressed()), this, SLOT(basectrlcb2())); // run planner
        // connect(ui_.forward_btn, SIGNAL(pressed()), this, SLOT(basectrlcb3())); // run forward
        // connect(ui_.backward_btn, SIGNAL(pressed()), this, SLOT(basectrlcb4())); // run backward
        // connect(ui_.obs_add_btn, SIGNAL(pressed()), this, SLOT(basectrlcb5())); // obs add
        // connect(ui_.obs_del_btn, SIGNAL(pressed()), this, SLOT(basectrlcb6())); // obs del
    }

} // namespace kimm_husky_gui

PLUGINLIB_EXPORT_CLASS(kimm_multi_husky_gui::HuskyGui, rqt_gui_cpp::Plugin)
