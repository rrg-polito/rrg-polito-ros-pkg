#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


class Odometry_node
{
private:
	void imuCallback (const sensor_msgs::ImuPtr& imu_msg);
	void odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg);
    double getYawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
    double getYawFromQuaternion(const tf::Quaternion& quaternion);
    void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);
    double calcCartMismatch(tf::Transform a, tf::Transform b );
    double calcAngMismatch(tf::Transform a, tf::Transform b );
    double normalizeAngle(double angle);
	void getOdometry();

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber odom_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Publisher  odom_publisher_;

    std::string fixed_frame_, robot_frame_;

    tf::TransformBroadcaster tf_broadcaster_;

    bool use_imu_;
    bool use_odom_;

    bool received_imu_;
    bool received_odom_;

    double latest_imu_yaw_;
    double last_imu_yaw_;

    double bias_;

    double latest_imu_yaw_rate;

    int time_steady;
    double cumulative_bias;

    ros::Time latest_imu_timestamp;
    ros::Time last_imu_timestamp;

    geometry_msgs::Quaternion latest_imu_q_;
    geometry_msgs::Quaternion last_imu_q_;
    nav_msgs::Odometry latest_odom_;
    nav_msgs::Odometry last_odom_;

    nav_msgs::Odometry latest_corrected_odom_;
    nav_msgs::Odometry last_corrected_odom_;

    ros::Timer timer;

public:
	Odometry_node(ros::NodeHandle nh, ros::NodeHandle nh_private);
};


Odometry_node::Odometry_node(ros::NodeHandle nh, ros::NodeHandle nh_private):
		  nh_(nh),
		  nh_private_(nh_private)
{

    received_imu_=false;
    received_odom_=false;

    latest_imu_yaw_ = 0.0;
    bias_=0.0;
    last_imu_yaw_=0.0;
    latest_imu_yaw_rate=0.0;
    time_steady=0.0;
    cumulative_bias=0.0;

    fixed_frame_ = "odom";
    robot_frame_ = "base_link";

	 if (!nh_private_.getParam ("use_imu", use_imu_))
	    use_imu_ = true;
	 if (!nh_private_.getParam ("use_odom",use_odom_))
		 use_odom_ = true;

	 if (use_imu_)
	  {
	    imu_subscriber_ = nh_.subscribe("imu/data", 1, &Odometry_node::imuCallback, this);
	  }

	  if (use_odom_)
	  {
	    odom_subscriber_ = nh_.subscribe("/odometry", 1, &Odometry_node::odomCallback, this);
	  }

	  odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("odom" , 5);
}



void Odometry_node::imuCallback (const sensor_msgs::ImuPtr& imu_msg)
{
  double dt;

  boost::mutex::scoped_lock(mutex_);
  latest_imu_yaw_rate = imu_msg->angular_velocity.z;
  latest_imu_timestamp = imu_msg->header.stamp;
  latest_imu_q_ = imu_msg->orientation;
  // integrate yaw rate
  dt = (latest_imu_timestamp - last_imu_timestamp).toSec();
  latest_imu_yaw_ = latest_imu_yaw_ +  (latest_imu_yaw_rate - bias_) * dt;
  last_imu_timestamp = latest_imu_timestamp;

  //ROS_INFO("imuCALL latest_imu_yaw_deltas = %f, time_steady = %d, bias = %f", latest_imu_yaw_,time_steady, bias_);

  if (!received_imu_)
  {
    bias_ = 0.0;
    latest_imu_yaw_ = 0.0;
    last_imu_yaw_ = latest_imu_yaw_;
    last_imu_q_ = latest_imu_q_;
    received_imu_ = true;
  }

}

void Odometry_node::odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  boost::mutex::scoped_lock(mutex_);
  latest_odom_ = *odom_msg;
  if (!received_odom_)
  {
    last_odom_ = *odom_msg;
    //last_corrected_odom_ = *odom_msg; 

    last_corrected_odom_.pose.pose.position.x = 0.0;
    last_corrected_odom_.pose.pose.position.y = 0.0;
    tf::Quaternion q = tf::createQuaternionFromYaw(0.0);
    tf::quaternionTFToMsg( 	q ,  last_corrected_odom_.pose.pose.orientation  );

    received_odom_ = true;
  }
  getOdometry();
}


double Odometry_node::getYawFromQuaternion(
  const tf::Quaternion& quaternion)
{
  double temp, yaw;
  tf::Matrix3x3 m(quaternion);
  m.getRPY(temp, temp, yaw);
  return yaw;
}

double Odometry_node::getYawFromQuaternion(
  const geometry_msgs::Quaternion& quaternion)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(quaternion, q);
  return getYawFromQuaternion(q);
}

void Odometry_node::createTfFromXYTheta(
  double x, double y, double theta, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

double Odometry_node::normalizeAngle(double angle)
{
	while(angle > M_PI)
		angle -= 2 * M_PI;
	while(angle < -M_PI)
		angle += 2 * M_PI;

	return angle;
}

double Odometry_node::calcCartMismatch(tf::Transform a, tf::Transform b )
{
	return a.getOrigin().distance(b.getOrigin());
}

double Odometry_node::calcAngMismatch(tf::Transform a, tf::Transform b  )
{
	tf::Transform temp;
	temp=a.inverse()*b;

	double diff= fabs( getYawFromQuaternion(temp.getRotation()));

	return diff;
}

void Odometry_node::getOdometry()
{
  boost::mutex::scoped_lock(mutex_);
  tf::Transform tf_latest;
  tf::Transform tf_last;

  // **** base case - no input available, use zero-motion model
  double pr_ch_x = 0.0;
  double pr_ch_y = 0.0;
  double pr_ch_a = 0.0;
double latest_corrected_a;

  // **** use wheel odometry
  if (use_odom_ && received_odom_)
  {
    // OK: we obtain the deltas from wheels
	//------------------------------------------------------------------------
    pr_ch_x = latest_odom_.pose.pose.position.x -
              last_odom_.pose.pose.position.x;

    pr_ch_y = latest_odom_.pose.pose.position.y -
              last_odom_.pose.pose.position.y;

    pr_ch_a = getYawFromQuaternion(latest_odom_.pose.pose.orientation) -
              getYawFromQuaternion(last_odom_.pose.pose.orientation);
    pr_ch_a = normalizeAngle(pr_ch_a);

    double theta = getYawFromQuaternion(last_odom_.pose.pose.orientation);
    double temp[2];
    temp[0] =    cos(theta)*pr_ch_x + sin(theta)*pr_ch_y;
    temp[1] =   - sin(theta)*pr_ch_x + cos(theta)*pr_ch_y;
    pr_ch_x= temp[0];
    pr_ch_y= temp[1];
    // ------------------------------------------------------------------------
    // tf_latest - from wheels
	createTfFromXYTheta(latest_odom_.pose.pose.position.x, latest_odom_.pose.pose.position.y, getYawFromQuaternion(latest_odom_.pose.pose.orientation), tf_latest);
	// tf_last - from wheels
	createTfFromXYTheta(last_odom_.pose.pose.position.x, last_odom_.pose.pose.position.y, getYawFromQuaternion(last_odom_.pose.pose.orientation), tf_last);

	// ROS_WARN("deltas %f %f, time_steady = %d, bias = %f",calcCartMismatch(tf_latest,tf_last), calcAngMismatch(tf_latest,tf_last),time_steady, bias_);
  // **** use imu
	  if (use_imu_ && received_imu_)
	  {
		  // if the robot is not moving
		  if( (calcCartMismatch(tf_latest,tf_last) <= 0.000001 && calcAngMismatch(tf_latest,tf_last) <= 0.000001))
		  {
			  // Estimate gyro bias when the robot is not moving
			  time_steady = time_steady+1;
			  cumulative_bias = cumulative_bias + latest_imu_yaw_rate;
			  if (time_steady>100 && time_steady<10000) // only at the beginning
			  {
				  bias_ = cumulative_bias/time_steady;
			  }

		  }
		  else
		  {
	// Fuse imu with wheel odometry: in the current version we trust the delta_yaw of the IMU and the deltaxy of the wheels
			  time_steady = 0;
			  cumulative_bias = 0;
			  //ROS_INFO("using orientation from IMU, wheel delta yaw %f", pr_ch_a);
			  pr_ch_a = latest_imu_yaw_ - last_imu_yaw_;
			  pr_ch_a = normalizeAngle(pr_ch_a);
			  //ROS_INFO("using orientation from IMU, imu delta yaw %f", pr_ch_a);
		  }
	  }
  }

 // pr_ch contains both imu and wheels
  //------------------------------------------------------------------------
  double theta_corrected = getYawFromQuaternion(last_corrected_odom_.pose.pose.orientation);
  // these are the Cartesian displacement in the global frame
  double temp_corrected[2];
  temp_corrected[0] =    cos(theta_corrected)*pr_ch_x - sin(theta_corrected)*pr_ch_y;
  temp_corrected[1] =    sin(theta_corrected)*pr_ch_x + cos(theta_corrected)*pr_ch_y;

  latest_corrected_odom_.pose.pose.position.x = last_corrected_odom_.pose.pose.position.x + temp_corrected[0];
  latest_corrected_odom_.pose.pose.position.y = last_corrected_odom_.pose.pose.position.y + temp_corrected[1];
  if(received_imu_)
 	latest_corrected_a = latest_imu_yaw_; 
  else
	latest_corrected_a = getYawFromQuaternion(last_corrected_odom_.pose.pose.orientation) + pr_ch_a;

  latest_corrected_a = normalizeAngle(latest_corrected_a);
  tf::Quaternion q = tf::createQuaternionFromYaw(latest_corrected_a);
  tf::quaternionTFToMsg( 	q ,  latest_corrected_odom_.pose.pose.orientation  );
  

  // latest_corrected_odom_tf_
  tf::Transform latest_corrected_odom_tf_;
  createTfFromXYTheta(latest_corrected_odom_.pose.pose.position.x, latest_corrected_odom_.pose.pose.position.y, getYawFromQuaternion(latest_corrected_odom_.pose.pose.orientation), latest_corrected_odom_tf_);

  //if (time_steady>100) // only at the beginning
//	  ROS_WARN("correcting bias %f",bias_);
 // else
//ROS_WARN("orientation from imu - latest_corrected_a =  %f",latest_corrected_a);
  // ------------------------------------------------------------------------

  nav_msgs::Odometry odom_msg = latest_corrected_odom_;
  /*
  ROS_INFO("estimated yaw of the robot (imu - post) %f", getYawFromQuaternion(q));

  ROS_INFO("last imu yaw %f", getYawFromQuaternion(last_odom_.pose.pose.orientation));

  tf::quaternionTFToMsg( 	q ,odom_msg.pose.pose.orientation);
  */
  odom_publisher_.publish(odom_msg);

  tf::StampedTransform transform_msg (latest_corrected_odom_tf_, ros::Time::now(), fixed_frame_, robot_frame_);
  tf_broadcaster_.sendTransform (transform_msg);

// imu + wheel odometry
  last_corrected_odom_ = latest_corrected_odom_;

  // wheel odometry
  last_odom_  = latest_odom_;

  // imu
  last_imu_q_ = latest_imu_q_;
  last_imu_yaw_ = latest_imu_yaw_;

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Odometry_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  Odometry_node odometry_node(nh, nh_private);
  ros::spin();
  return 0;
}
