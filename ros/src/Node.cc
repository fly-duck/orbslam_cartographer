#include "Node.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>

Node::Node (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) {
  name_of_node_ = ros::this_node::getName();
  node_handle_ = node_handle;
  min_observations_per_point_ = 2;

  //static parameters
  node_handle_.param(name_of_node_+ "/publish_pointcloud", publish_pointcloud_param_, true);
  node_handle_.param(name_of_node_+ "/publish_pose", publish_pose_param_, true);
  node_handle_.param<std::string>(name_of_node_+ "/pointcloud_frame_id", map_frame_id_param_, "map");
  node_handle_.param<std::string>(name_of_node_+ "/camera_frame_id", camera_frame_id_param_, "camera_link");
  node_handle_.param<std::string>(name_of_node_ + "/map_file", map_file_name_param_, "map.bin");
  node_handle_.param<std::string>(name_of_node_ + "/voc_file", voc_file_name_param_, "file_not_set");
  node_handle_.param<std::string>(name_of_node_ + "/settings_file", settings_file_name_param_, "file_not_set");
  node_handle_.param(name_of_node_ + "/load_map", load_map_param_, false);

  orb_slam_ = new ORB_SLAM2::System (voc_file_name_param_, settings_file_name_param_, sensor, map_file_name_param_, load_map_param_);
  initial_pose_pub_=node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1);

  service_server_ = node_handle_.advertiseService(name_of_node_+"/save_map", &Node::SaveMapSrv, this);

  //Setup dynamic reconfigure
  dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig>::CallbackType dynamic_param_callback;
  dynamic_param_callback = boost::bind(&Node::ParamsChangedCallback, this, _1, _2);
  dynamic_param_server_.setCallback(dynamic_param_callback);

  rendered_image_publisher_ = image_transport.advertise (name_of_node_+"/debug_image", 1);
  if (publish_pointcloud_param_) {
    map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> (name_of_node_+"/map_points", 1);
  }

  // Enable publishing camera's pose as PoseStamped message
  if (publish_pose_param_) {
    pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped> (name_of_node_+"/pose", 1);
  }

}


Node::~Node () {
  // Stop all threads
  orb_slam_->Shutdown();

  // Save camera trajectory
  orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  delete orb_slam_;
}


void Node::Update () {
  cv::Mat position = orb_slam_->GetCurrentPosition();

  if (!position.empty()) {
    PublishPositionAsTransform (position);

    if (publish_pose_param_) {
      PublishPositionAsPoseStamped (position);
    }
  }

  PublishRenderedImage (orb_slam_->DrawCurrentFrame());

  if (publish_pointcloud_param_) {
    PublishMapPoints (orb_slam_->GetAllMapPoints());
  }

}


void Node::PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points);
  map_points_publisher_.publish (cloud);
}


void Node::PublishPositionAsTransform (cv::Mat position) {
  tf::Transform transform = TransformFromMat (position);
  static tf::TransformBroadcaster tf_broadcaster;
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, current_frame_time_, map_frame_id_param_, camera_frame_id_param_));
}

inline void PoseMsgToTF(const geometry_msgs::Pose& msg, tf::Transform& bt)
{
  bt = tf::Transform(tf::Quaternion(msg.orientation.x,
                            msg.orientation.y,
                            msg.orientation.z,
                            msg.orientation.w),
                 tf::Vector3(msg.position.x, msg.position.y, msg.position.z));
}

inline void ShowTFTransform( const tf::Transform& transform, const std::string & string ) 
{

    std::cout<< string << "\n";
    std::cout << "row1" << transform.getBasis().getRow(1).x()<<" "<<transform.getBasis().getRow(1).y()<<" "<<transform.getBasis().getRow(1).z()<< "\n";
    std::cout << "row2" << transform.getBasis().getRow(2).x()<<" "<<transform.getBasis().getRow(2).y()<<" "<<transform.getBasis().getRow(2).z()<< "\n";
    std::cout << "row3" << transform.getBasis().getRow(3).x()<<" "<<transform.getBasis().getRow(3).y()<<" "<<transform.getBasis().getRow(3).z()<< "\n";

    std::cout <<  "origin" << transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<transform.getOrigin().z()<< "\n";

}


void Node::PublishPositionAsPoseStamped (cv::Mat position) {
  // std::cout<< position << "\n";
  tf::Transform grasp_tf = TransformFromMat (position);
  // ShowTFTransform(grasp_tf,"before");
  tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time_, map_frame_id_param_);
  geometry_msgs::PoseStamped pose_msg;
  
  tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg);
  

  // tf::Transform after;
  // PoseMsgToTF(pose_msg.pose,after);
  // ShowTFTransform(after,"after");

  // pose_msg.
  geometry_msgs::PoseWithCovarianceStamped initial_pose;
  // initial_pose.header
  initial_pose.pose.pose.orientation=pose_msg.pose.orientation;
  initial_pose.pose.pose.position=pose_msg.pose.position;

  // for( auto & ele :initial_pose.pose.covariance)
  // {
  //   ele = static_cast<float>(0);
  // }
  initial_pose.header=pose_msg.header;
  std::string frame_id = "map";
  initial_pose.header.frame_id=frame_id;
  initial_pose_pub_.publish(initial_pose);
  pose_publisher_.publish(pose_msg);
}


void Node::PublishRenderedImage (cv::Mat image) {
  std_msgs::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}

template<typename T> 
T Cal360(const T & number)
{
  return number*180/3.1415;
}

void ShowRPY(tf::Matrix3x3 matrix,const string & string)

{
  double x,y,z ;
  matrix.getRPY(x,y,z);
  x=Cal360(x);
  y=Cal360(y);
  z=Cal360(z);

  std::cout<<string<<": "<<"x" << x<< "y"<< y<< "z"<< z<< "\n";
  

}


tf::Transform Node::TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);

  tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  // x--> -90 z-->-90

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  ShowRPY(tf_camera_rotation,"before");
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;
  ShowRPY(tf_camera_rotation,"After");



  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  ShowRPY(tf_camera_rotation,"Transpose");

  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);



  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}


sensor_msgs::PointCloud2 Node::MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i=0; i<cloud.width; i++) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {
      data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
    }
  }

  return cloud;
}


void Node::ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level) {
  orb_slam_->EnableLocalizationOnly (config.localize_only);
  min_observations_per_point_ = config.min_observations_for_ros_map;

  if (config.reset_map) {
    orb_slam_->Reset();
    config.reset_map = false;
  }

  orb_slam_->SetMinimumKeyFrames (config.min_num_kf_in_map);
}


bool Node::SaveMapSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res) {
  res.success = orb_slam_->SaveMap(req.name);

  if (res.success) {
    ROS_INFO_STREAM ("Map was saved as " << req.name);
  } else {
    ROS_ERROR ("Map could not be saved.");
  }

  return res.success;
}