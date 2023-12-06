#include <h264_bag_playback/h264_bag_playback.hpp>
#include <custom_point_types/point_xyzir.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <h264_bag_playback/helper_functions.hpp>
#include <lidar_camera_projection/lidar_camera_node.hpp>

#include <nav_msgs/Odometry.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <cv_bridge/cv_bridge.h>

#include <py_trees_msgs/Behaviour.h>

class DirectPlayback : public dataset_toolkit::h264_bag_playback {

public:

  DirectPlayback(std::string bag_file, std::vector <std::string> _camera_topics, std::string _lidar_topic)
      : h264_bag_playback(),
        lidar_topic(_lidar_topic),
        border_r(0),
        border_g(0),
        border_b(254){

    test_change_to_transform = 10;

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    for (auto prefix: _camera_topics) {
      std::string camera_topic = prefix + "/image_color";

      camera_topics[camera_topic] = camera_topic;
      lidar_image_publisher[camera_topic] = it.advertise(prefix + "/lidar_projected", 1);

      // subscribe to the camera topic to force the playback tool to use these images
      subscribers.push_back(it.subscribe(camera_topic, 1, DirectPlayback::image_callback));

      // create the projector object
      std::shared_ptr <lidar_camera_projection::Projector> projector = std::make_shared<lidar_camera_projection::Projector>(
          camera_topic);

      // get the camera_lidar projector to publish images with range point cloud
      projector->data_products->flag_range_image = true;
      projector->data_products->flag_rgb_pointcloud = true;

      camera_info_topics[prefix + "/camera_info"] = projector;
      projectors[camera_topic] = projector;
    }

    init_playback();

    // look up the transform for the camera-lidar
    for (auto projector: projectors) {
      ROS_INFO_STREAM("Looking up projection for " << projector.second->camera_frame);
      projector.second->lookupLidarCameraTF(transformer_);
    }
  }


  void ApplyTransformCorrection(std::string from_frame, std::string to_frame, ros::Time &message_time, double pitch, double roll, double yaw) {
    geometry_msgs::TransformStamped lidar_baselink_transform;
    lidar_baselink_transform = transformer_->lookupTransform(from_frame, to_frame, message_time);

    tf2::Quaternion rotation_to_apply, existing_rotation, new_rotation;
    // create a quaternion for the change in orientation
    rotation_to_apply.setRPY(pitch, roll, yaw);

    // get the original quaternion from the tf transform
    tf2::convert(lidar_baselink_transform.transform.rotation, existing_rotation);

    // apply the change transform
    new_rotation = rotation_to_apply * existing_rotation;

    tf2::convert(new_rotation, lidar_baselink_transform.transform.rotation);
    transformer_->setTransform(lidar_baselink_transform, "zio");
  }

  // an empty callback to allow the h264 publisher to start publishing images
  static void image_callback(const sensor_msgs::ImageConstPtr &msg) {}


  void ImagePublisher(image_transport::Publisher &publisher, const sensor_msgs::ImageConstPtr &message) {

    // check if this topic is being used
    auto projector = projectors.find(publisher.getTopic());

    if (projector != projectors.end()) {


      if (!projector->second->camera_info_msg) {
        std::cout << "no camera info yet, not using image" << std::endl;
        return;
      }

      ros::Time message_time = message->header.stamp;

      std::string new_camera_name = publisher.getTopic();

      // get the camera name from the dataset camera topic and add "_lidar"
      new_camera_name = remove_last_of_string(new_camera_name, "/");
      new_camera_name += "_lidar";

      std::string new_data_bag = "colour_pc";
      std::string colour_lidar_topic = publisher.getTopic() + "/colour_lidar";
      std::string uv_lidar_topic = publisher.getTopic() + "/uv_lidar";

      cv_bridge::CvImagePtr cv_ptr;

      if (!latest_pc) {
        ROS_INFO_STREAM_THROTTLE(5., "No lidar yet");
        try {
          cv_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
      }
      else {

        if (std::shared_ptr < lidar_camera_projection::DataProducts > projected_data = projector->second->callback_lidar_camera(latest_pc, message)) {

          // write the image with lidar superimposed to a new video
          if (projected_data->range_image) {

            this->bag_writer.WriteMessage<sensor_msgs::PointCloud2::Ptr>(new_data_bag, colour_lidar_topic,
                                                                         message_time,
                                                                         projector->second->data_products->rgb_pointcloud_msg);
            
           // this->bag_writer.WriteMessage<sensor_msgs::PointCloud2::Ptr>(new_data_bag, uv_lidar_topic,
           //                                                              message_time,
           //                                                              projector->second->data_products->uv_pointcloud_msg);

            try {
              cv_ptr = cv_bridge::toCvCopy(projected_data->range_image, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception &e) {
              ROS_ERROR("cv_bridge exception: %s", e.what());
              return;
            }
          }
        }
      }

      cv::Mat overlay = cv_ptr->image.clone();
      overlay.setTo(cv::Scalar(0, 0, 0, 0));

      cv::Mat additional_overlay = overlay.clone();
        cv::Mat additional_overlay_secondary = overlay.clone();
      double weighting = 0.7;


      cv::addWeighted(cv_ptr->image, 1., overlay, 0.95, 0, cv_ptr->image);

      cv::addWeighted(overlay, 1., additional_overlay, 0.85, 0, overlay);
      additional_overlay.setTo(cv::Scalar(0, 0, 0, 0));


      this->bag_writer.WriteToVideo(new_data_bag, new_camera_name, message_time, cv_ptr->toImageMsg(),
                                  projector->second->camera_info_msg);

      // uncomment the following section if you want to publish the images with lidar data superimposed
      //if (projected_data->range_image) {
      //  lidar_image_publisher[publisher.getTopic()].publish(projected_data->range_image);
      //}
    }

    // if you want to publish all of the images, uncomment the following line
    //publisher.publish(message);
  }

  void CameraInfoPublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message,
                           const sensor_msgs::CameraInfoConstPtr &scaled_info_msg) {

    // check if the camera_info message is being used
    auto camera_info_topic = camera_info_topics.find(publisher.getTopic());

    if (camera_info_topic != camera_info_topics.end()) {

      camera_info_topic->second->callback_camerainfo(scaled_info_msg);

      sensor_msgs::CameraInfoPtr copy_msg(new sensor_msgs::CameraInfo(*scaled_info_msg));
      camera_info_topic->second->camera_info_msg = copy_msg;
    }
  }

  void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {

    auto lidar_msg = message.instantiate<sensor_msgs::PointCloud2>();
    if (lidar_msg && publisher.getTopic() == lidar_topic) {
      latest_pc = lidar_msg;
    }
    // if you want to publish all of the bag topics, include the following line
    publisher.publish(message);
  }

  // map the camera topic to the projector, and the camera info topic to the projector
  std::map <std::string, std::shared_ptr<lidar_camera_projection::Projector>> projectors;
  std::map <std::string, std::shared_ptr<lidar_camera_projection::Projector>> camera_info_topics;

  // map the camera name to the camera topic
  std::map <std::string, std::string> camera_topics;

  // the lidar topic used to superimpose onto the images
  std::string lidar_topic;

  // store a pointer to the latest pointcloud
  sensor_msgs::PointCloud2ConstPtr latest_pc;


  int test_change_to_transform;

  int border_r, border_g, border_b;
  // a publisher for the new images with lidar superimposed
  std::map <std::string, image_transport::Publisher> lidar_image_publisher;

  // subscribe to the topic only to trigger the h264 playback - it will only publish if someone is subscribed
  std::vector <image_transport::Subscriber> subscribers;

};


int main(int argc, char **argv) {

  ros::init(argc, argv, "lidar_camera_bag_playback");
  ros::NodeHandle private_nh("~");

  std::string bag_file_name = "";
  private_nh.getParam("bag_file", bag_file_name);

  std::string camera_topic_prefix;
  private_nh.getParam("camera_topic_prefix", camera_topic_prefix);

  if (bag_file_name.empty()) {
    ROS_INFO_STREAM("Could not find bagfile " << bag_file_name);
    return -1;
  }

  std::vector <std::string> camera_topics;
  private_nh.getParam("camera_topics", camera_topics);

  // set up the input topics
  std::string lidar_topic = "/ouster/points";

  // create the playback object
  DirectPlayback playback(bag_file_name, camera_topics, lidar_topic);

  // Start the playback
  playback.ReadFromBag();

  return 0;
}
