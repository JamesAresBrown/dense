//
// Created by hzx on 2022/9/12.
//

#ifndef DENSE_PDENSE_H
#define DENSE_PDENSE_H
#include <iostream>
#include <string>
//#include "../estimator/estimator.h"
//#include "base/pmvs/findMatch.h"
//#include "base/pmvs/option.h"
#include "../../global_fusion/ThirdParty/GeographicLib/include/LocalCartesian.hpp"

#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;

class Pdense {
public:
    Pdense() = default;

    explicit Pdense(const string &config_path);

    Pdense(const string &config_path, const string &option_name);

    string root_path_;
    string config_path_;
    string option_name_ = "option-0000";
    int max_num_ = 0;
    int num_image_ = 0;
    GeographicLib::LocalCartesian geoConverter;
    Eigen::Vector3d origin_T_;

//    void write(const Estimator& estimator, cv::Mat img);
    void write(const Eigen::Matrix3d& R, const Eigen::Vector3d& T, const cv::Mat& img);

    void pmvs2() const;

    static cv::Mat Gray2Color(cv::Mat &phase);

    void PLY2PCD() const;

    void generateOption();

    static void Combine();

//    void SetGlobal(const Estimator& estimator, const double& latitude, const double& longitude, const double& altitude);
    bool initGPS_ = false;

//    double latitude_;
//    double longitude_;
//    double altitude_;
    void Global() const;

private:
};

#endif //DENSE_PDENSE_H
