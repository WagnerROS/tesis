#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

bool zero_orientation_set = false;

sensor_msgs::Imu imu;
int16_t w = 0;
int16_t x = 0;
int16_t y = 0;
int16_t z = 0;

int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;

int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;

bool set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}

void chatterCallback(const sensor_msgs::Imu& msg)
{

  x = msg.orientation.x;
  y = msg.orientation.y;
  z = msg.orientation.z;
  w = msg.orientation.w;

  gx = msg.angular_velocity.x;
  gy = msg.angular_velocity.y;
  gz = msg.angular_velocity.z;

  ax = msg.linear_acceleration.x;
  ay = msg.linear_acceleration.y;
  az = msg.linear_acceleration.z;
  
}

int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;
  std::string tf_parent_frame_id;
  std::string tf_frame_id;
  std::string frame_id;
  double time_offset_in_seconds;
  bool broadcast_tf;
  double linear_acceleration_stddev;
  double angular_velocity_stddev;
  double orientation_stddev;
  uint8_t last_received_message_number;
  bool received_message = false;

  tf::Quaternion orientation;
  tf::Quaternion zero_orientation;

  ros::init(argc, argv, "mpu6050_serial");

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
  private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
  private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
  private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);
  private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
  private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
  private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);

  imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

  imu.angular_velocity_covariance[0] = angular_velocity_stddev;
  imu.angular_velocity_covariance[4] = angular_velocity_stddev;
  imu.angular_velocity_covariance[8] = angular_velocity_stddev;

  imu.orientation_covariance[0] = orientation_stddev;
  imu.orientation_covariance[4] = orientation_stddev;
  imu.orientation_covariance[8] = orientation_stddev;
  
  
  ros::NodeHandle nh("imu");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 50);
  ros::Publisher imu_temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 50);
  ros::ServiceServer service = nh.advertiseService("set_zero_orientation", set_zero_orientation);
  
  ros::Subscriber sub_imu1 = nh.subscribe("imu1",100, imuCallback);

  ros::Rate r(200); // 200 hz

  static tf::TransformBroadcaster tf_br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0,0,0));

  std::string input;
  std::string read;

  // get quaternion values            
  double wf = w/16384.0;
  double xf = x/16384.0;
  double yf = y/16384.0;
  double zf = z/16384.0;

  tf::Quaternion orientation(xf, yf, zf, wf);

  if (!zero_orientation_set)
  {
    zero_orientation = orientation;
    zero_orientation_set = true;
  }

  //http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
  tf::Quaternion differential_rotation;
  differential_rotation = zero_orientation.inverse() * orientation;

  // get gyro values
  // calculate rotational velocities in rad/s
  // without the last factor the velocities were too small
  // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
  // FIFO frequency 100 Hz -> factor 10 ?
  // seems 25 is the right factor
  //TODO: check / test if rotational velocities are correct
  double gxf = gx * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
  double gyf = gy * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
  double gzf = gz * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

  // get acelerometer values
  // calculate accelerations in m/s²
  double axf = ax * (8.0 / 65536.0) * 9.81;
  double ayf = ay * (8.0 / 65536.0) * 9.81;
  double azf = az * (8.0 / 65536.0) * 9.81;

  // get temperature
  // calculate measurement time
  ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

  // publish imu message
  imu.header.stamp = measurement_time;
  imu.header.frame_id = frame_id;

  quaternionTFToMsg(differential_rotation, imu.orientation);
  imu.angular_velocity.x = gxf;
  imu.angular_velocity.y = gyf;
  imu.angular_velocity.z = gzf;

  imu.linear_acceleration.x = axf;
  imu.linear_acceleration.y = ayf;
  imu.linear_acceleration.z = azf;

  imu_pub.publish(imu);

  // publish tf transform
  if (broadcast_tf)
  {
    transform.setRotation(differential_rotation);
    tf_br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
  }
  ros::spinOnce();
  r.sleep();

}
