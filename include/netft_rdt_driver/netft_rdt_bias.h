#ifndef NETFT_RDT_BIAS_H_
#define NETFT_RDT_BIAS_H_

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Bool.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "netft_rdt_driver/String_cmd.h"

namespace netft_rdt_driver
{

class NetFTRDTDriverBias{

public:

    NetFTRDTDriverBias(ros::NodeHandle& nh, double rot, const tf::Vector3 &scale_F,double alpha, std::size_t num_points=50);

    void update(geometry_msgs::Wrench& wrench);

    void compute_bias(const geometry_msgs::Wrench& wrench);

    void set_compute_bias(bool val=true);

private:

    void print_bias() const;

    bool service_callback(netft_rdt_driver::String_cmd::Request& request, netft_rdt_driver::String_cmd::Response& response);

    /// taken from control_toolbox filter.h
    static inline double exponentialSmoothing(double current_raw_value, double last_smoothed_value, double alpha)
    {
        return alpha*current_raw_value + (1-alpha)*last_smoothed_value;
    }

private:


    geometry_msgs::Vector3  force_b,  force_b_tmp;
    geometry_msgs::Vector3  torque_b, torque_b_tmp;

    geometry_msgs::Wrench   wrench_tmp;
    geometry_msgs::Wrench   bias_msg;
    double                  alpha;

    ros::ServiceServer    service_server;
    ros::Publisher        pub_bias_status;
    ros::Publisher        pub_bias;
    std_msgs::Bool        bias_status;

    tf::Vector3           scale_F;

    tf::Matrix3x3         Rot;
    tf::Vector3           tmp;

    std::size_t           num_points;
    std::size_t           count;
    bool                  bComputeBias;
    bool                  bSmooth;

};

}


#endif
