/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
int USE_GPU;
int USE_GPU_ACC_FLOW;
int PUB_RECTIFY;
Eigen::Matrix3d rectify_R_left;
Eigen::Matrix3d rectify_R_right;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    IMAGE0_TOPIC = "/camera/image_raw";
    IMAGE1_TOPIC = "";
    MAX_CNT = 150;
    MIN_DIST = 30;
    F_THRESHOLD = 1.0;
    SHOW_TRACK = 1;
    FLOW_BACK = 1;

    MULTIPLE_THREAD = 4;

    USE_GPU = 1;
    USE_GPU_ACC_FLOW = 0;

    USE_IMU = 1;
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        IMU_TOPIC = "/imu";
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = 0.04334389206191802;
        ACC_W = 6.226271873858886e-04;
        GYR_N = 0.0019792625912192652;
        GYR_W = 1.0866894432315462e-05;
        G.z() = 9.81007;
    }

    SOLVER_TIME = 0.04;
    NUM_ITERATIONS = 8;
    MIN_PARALLAX = 10.0;
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    OUTPUT_FOLDER =  "/home/itmo/vins_lite_gpu/src/VINS_Lite_GPU/output/";
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ESTIMATE_EXTRINSIC = 0;
    // if (ESTIMATE_EXTRINSIC == 2)
    // {
    //     ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
    //     RIC.push_back(Eigen::Matrix3d::Identity());
    //     TIC.push_back(Eigen::Vector3d::Zero());
    //     EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    // }
    // else 
    // {
        // if ( ESTIMATE_EXTRINSIC == 1)
        // {
        //     ROS_WARN(" Optimize extrinsic param around initial guess!");
        //     EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        // }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        // fs["body_T_cam0"] >> cv_T;
        // Eigen::Matrix4d T;
        // cv::cv2eigen(cv_T, T);
        Eigen::Matrix3d Ric;
        Ric << 1.3541311096539888e-02, 0.999908302752944, 0.000137766160856,
                                0.999902134214296, -0.013540742971475, -0.003517139957148,
                                -0.003514951988922, 0.000185379364592, -9.9999380535431692e-01;
        Eigen::Vector3d tic;
        tic << 2.6870690119900075e-02, 0.020740935736938, -4.3251750752052849e-02;
        
        RIC.push_back(Ric);
        TIC.push_back(tic); 
    // } 
    
    NUM_OF_CAM = 1;
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    cam0Calib = "imx477.yaml";
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    // if(NUM_OF_CAM == 2)
    // {
    //     STEREO = 1;
    //     std::string cam1Calib;
    //     fs["cam1_calib"] >> cam1Calib;
    //     std::string cam1Path = configPath + "/" + cam1Calib; 
    //     //printf("%s cam1 path\n", cam1Path.c_str() );
    //     CAM_NAMES.push_back(cam1Path);
        
    //     cv::Mat cv_T;
    //     fs["body_T_cam1"] >> cv_T;
    //     Eigen::Matrix4d T;
    //     cv::cv2eigen(cv_T, T);
    //     RIC.push_back(T.block<3, 3>(0, 0));
    //     TIC.push_back(T.block<3, 1>(0, 3));
    //     fs["publish_rectify"] >> PUB_RECTIFY;
    // }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = 0.03356973236020965;
    ESTIMATE_TD = 1;
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = 480;
    COL = 848;
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }
    // if(PUB_RECTIFY)
    // {
    //     cv::Mat rectify_left;
    //     cv::Mat rectify_right;
    //     fs["cam0_rectify"] >> rectify_left;
    //     fs["cam1_rectify"] >> rectify_right;
    //     cv::cv2eigen(rectify_left, rectify_R_left);
    //     cv::cv2eigen(rectify_right, rectify_R_right);

    // }

    // fs.release();
}
