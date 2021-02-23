#include "../include/ros2-marvelmind/marvelmindROSWrapper.h"



marvelmindWrapper::marvelmindWrapper(std::shared_ptr<rclcpp::Node> nh) {


	this->nh = nh;
	pose_publisher = nh->create_publisher<geometry_msgs::msg::Pose>("pose_marvelmind_readings",10);
	accel_publisher = nh->create_publisher<geometry_msgs::msg::Accel>("accel_marvelmind_readings",10);
	twist_publisher = nh->create_publisher<geometry_msgs::msg::Twist>("twist_marvelmind_readings",10);

	std::chrono::milliseconds sensor_update_ms(static_cast<int>(1000.0/200)); //200Hz

	timer_marvelmind_publish = nh->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(sensor_update_ms), std::bind(&marvelmindWrapper::publishMarvelmindReadings,this));

	RCLCPP_INFO(nh->get_logger(),"Initializing Marvelmind");
}

/*Callback method that publishes Marvelmind data when valid data is retreived from Hedgehog*/
void marvelmindWrapper::publishMarvelmindReadings(){

	geometry_msgs::msg::Pose pose_msg;
	geometry_msgs::msg::Accel accel_msg;
	geometry_msgs::msg::Twist twist_msg;

	struct FusionIMUValue data; //All data that hedge can get

	getFusionIMUFromMarvelmindHedge(this->hedge,&data);

	if (data.updated){
		RCLCPP_INFO_ONCE(nh->get_logger(),"Started publishing Marvelmind readings");

		// coordinates in mm
		pose_msg.position.x = (float) data.x;
		pose_msg.position.y = (float) data.y;
		pose_msg.position.z = (float) data.z;

		//Convert to meters
		pose_msg.position.x /= 1000;
		pose_msg.position.y /= 1000;
		pose_msg.position.z /= 1000;

		// quaternion, normalized to 10000
		pose_msg.orientation.x = data.qx;
		pose_msg.orientation.y = data.qy;
		pose_msg.orientation.z = data.qz;
		pose_msg.orientation.w = data.qw;

		// acceleration in mm/s^2
		accel_msg.linear.x = (float) data.ax;
		accel_msg.linear.y = (float) data.ay;
		accel_msg.linear.z = (float) data.az;

		//Convert to m/s^2
		accel_msg.linear.x /= 1000;
		accel_msg.linear.y /= 1000;
		accel_msg.linear.z /= 1000;

		// velocity in mm/s
		twist_msg.linear.x = (float) data.vx;
		twist_msg.linear.y = (float) data.vy;
		twist_msg.linear.z = (float) data.vz;

		//Convert to m/s
		twist_msg.linear.x /= 1000;
		twist_msg.linear.y /= 1000;
		twist_msg.linear.z /= 1000;


		pose_publisher->publish(pose_msg);
		accel_publisher->publish(accel_msg);
		twist_publisher->publish(twist_msg);
	}
}

bool marvelmindWrapper::connect()
{
	RCLCPP_INFO(nh->get_logger(),"Connecting to Marvelmind system");

	this->hedge=createMarvelmindHedge();

	//Creation test
  if (this->hedge==NULL)
  {
      RCLCPP_ERROR(nh->get_logger(),"Unable to create MarvelmindHedge");
			return false;
  }

  this->hedge->ttyFileName=DEFAULT_TTY_FILENAME;
  this->hedge->verbose=true; // show errors and warnings
	this->hedge->baudRate=115200;
	// this->hedge->anyInputPacketCallback=(void (*)()) &marvelmindWrapper::publishMarvelmindReadings;

  startMarvelmindHedge (this->hedge);

	return true;
}

bool marvelmindWrapper::disconnect()
{
	RCLCPP_INFO(nh->get_logger(),"Disconnecting Marvelmind system");

	stopMarvelmindHedge (this->hedge);
  destroyMarvelmindHedge (this->hedge);


	return true;
}
