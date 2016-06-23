#include "netft_rdt_driver/netft_rdt_bias.h"

namespace netft_rdt_driver
{

NetFTRDTDriverBias::NetFTRDTDriverBias(ros::NodeHandle& nh, double rot,
                                       const tf::Vector3& scale_F, double alpha, std::size_t num_points):
  num_points(num_points),
  scale_F(scale_F),
  alpha(alpha)
{
  count      = 0;
  force_b.x  = 0;
  force_b.y  = 0;
  force_b.z  = 0;
  torque_b.x = 0;
  torque_b.y = 0;
  torque_b.z = 0;

  force_b_tmp  = force_b;
  torque_b_tmp = torque_b;

  if(alpha==0) {
    bSmooth=false;
  } else {
    bSmooth=true;
  }

  service_server  = nh.advertiseService("bias_cmd",
                                        &NetFTRDTDriverBias::service_callback,this);
  pub_bias_status = nh.advertise<std_msgs::Bool>("bias_status",1);
  pub_bias = nh.advertise<geometry_msgs::Wrench>("bias",1);
  bias_status.data = false; // bias not set
  bComputeBias    = false;

  Rot.setRPY(0,0,rot);

}

void NetFTRDTDriverBias::update(geometry_msgs::Wrench& wrench)
{

  // filter the signal
  if(bSmooth) {
    exponentialSmoothing(wrench.force.x,wrench_tmp.force.x,alpha);
    exponentialSmoothing(wrench.force.y,wrench_tmp.force.y,alpha);
    exponentialSmoothing(wrench.force.z,wrench_tmp.force.z,alpha);

    exponentialSmoothing(wrench.torque.x,wrench_tmp.torque.x,alpha);
    exponentialSmoothing(wrench.torque.y,wrench_tmp.torque.y,alpha);
    exponentialSmoothing(wrench.torque.z,wrench_tmp.torque.z,alpha);
    wrench_tmp = wrench;
  }

  // remove the bias
  wrench.force.x = wrench.force.x - force_b.x;
  wrench.force.y = wrench.force.y - force_b.y;
  wrench.force.z = wrench.force.z - force_b.z;

  wrench.torque.x = wrench.torque.x - torque_b.x;
  wrench.torque.y = wrench.torque.y - torque_b.y;
  wrench.torque.z = wrench.torque.z - torque_b.z;

    bias_msg.force.x = force_b.x;
    bias_msg.force.y = force_b.y;
    bias_msg.force.z = force_b.z;

    bias_msg.torque.x = torque_b.x;
    bias_msg.torque.y = torque_b.y;
    bias_msg.torque.z = torque_b.z;


  // Rotate the force vector
  tmp[0] = wrench.force.x;
  tmp[1] = wrench.force.y;
  tmp[2] = wrench.force.z;
  tmp    = Rot * tmp;
  wrench.force.x = tmp[0] * scale_F[0];
  wrench.force.y = tmp[1] * scale_F[1];
  wrench.force.z = tmp[2] * scale_F[2];

  // Rotate the torque vector
  tmp[0] = wrench.torque.x;
  tmp[1] = wrench.torque.y;
  tmp[2] = wrench.torque.z;
  tmp    = Rot * tmp;
  wrench.torque.x = tmp[0];
  wrench.torque.y = tmp[1];
  wrench.torque.z = tmp[2];

  pub_bias_status.publish(bias_status);
  pub_bias.publish(bias_msg);
}

void NetFTRDTDriverBias::set_compute_bias(bool val)
{
  this->bComputeBias = val;
}
void NetFTRDTDriverBias::compute_bias(const geometry_msgs::Wrench& wrench)
{
  if(bComputeBias) {
    if(count < num_points) {
      force_b_tmp.x  = force_b_tmp.x + wrench.force.x;
      force_b_tmp.y  = force_b_tmp.y + wrench.force.y;
      force_b_tmp.z  = force_b_tmp.z + wrench.force.z;

      torque_b_tmp.x = torque_b_tmp.x + wrench.torque.x;
      torque_b_tmp.y = torque_b_tmp.y + wrench.torque.y;
      torque_b_tmp.z = torque_b_tmp.z + wrench.torque.z;
      count++;
    } else {

      force_b_tmp.x = (1/(double)num_points) * force_b_tmp.x;
      force_b_tmp.y = (1/(double)num_points) * force_b_tmp.y;
      force_b_tmp.z = (1/(double)num_points) * force_b_tmp.z;

      torque_b_tmp.x = (1/(double)num_points) * torque_b_tmp.x;
      torque_b_tmp.y = (1/(double)num_points) * torque_b_tmp.y;
      torque_b_tmp.z = (1/(double)num_points) * torque_b_tmp.z;

      force_b  = force_b_tmp;
      torque_b = torque_b_tmp;

      print_bias();
      bComputeBias     = false;
      bias_status.data = true;
    }
  }
}


bool NetFTRDTDriverBias::service_callback(netft_rdt_driver::String_cmd::Request&
    request, netft_rdt_driver::String_cmd::Response& response)
{
  std::string cmd = request.cmd;

  if(cmd == "bias") {
    bComputeBias=true;
    bias_status.data=false;
    force_b_tmp.x  = force_b_tmp.y  = force_b_tmp.z  = 0;
    torque_b_tmp.x = torque_b_tmp.y = torque_b_tmp.z = 0;
    count = 0;
    response.res = " command [" + cmd + "] sucessfully called";
    return true;
  } else if (cmd == "print") {
    response.res = "commands : bias | print";
    return true;
  } else {
    std::string res =  "no such cmd [" +  cmd +
                       "] defined        NetFTRDTDriverBias::service_callback";
    response.res = res;
    return false;
  }

}

void NetFTRDTDriverBias::print_bias() const
{
  std::cout<< "=== bias (mean) ===" <<std::endl;
  std::cout<< "F:         " << force_b.x << "\t" << force_b.y << "\t"   <<
           force_b.z << std::endl;
  std::cout<< "T:         " << torque_b.x << "\t" << torque_b.y << "\t" <<
           torque_b.z << std::endl;
  std::cout<< "nbSamples: " << num_points << std::endl;
}


}
