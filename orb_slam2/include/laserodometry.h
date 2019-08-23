#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>


namespace Lidar
{

struct RobotPose
{   
    RobotPose()
    :x(0),y(0),z(0)
    {

    }

    double x,y,z;

};


class InputOdometry


{
public:
    InputOdometry()
    {
    // transformStamped_.header.frame_id="base_footprint";
    
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_(buffer_);
    get_transform(buffer_);
      
    pose_.x=transformStamped_.transform.translation.x;
    pose_.y=transformStamped_.transform.translation.y;
    pose_.z=transformStamped_.transform.translation.z;

            // int argc;char**argv;
        // ros::init(argc, argv, "my_tf2_listener");


    }


    void get_transform(tf2_ros::Buffer& buffer)
    {
    try{
      transformStamped_ = buffer.lookupTransform("odom", "base_footprint",
                               ros::Time(0),ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    }


const RobotPose GetPose() const 
{
    return pose_;
}

    private:
    RobotPose pose_;

    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped transformStamped_;
       
};



}
