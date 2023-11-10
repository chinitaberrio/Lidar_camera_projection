#undef BOOST_BIND_NO_PLACEHOLDERS

#include "lidar_camera_node.hpp"

#include <Eigen/Dense>

#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace lidar_camera_projection {

  Projector::Projector() : camera_info_msg(NULL),
      MIN_X_VALUE(1.35),
      MIN_Y_VALUE(1.35),
      MIN_Z_VALUE(0.3)
 {

/*
    camera_matrix = MatrixXf(3,3); // 3x3 camera matrix
    distortion_coefficients = MatrixXf(1,4);  // 1x4 distortion coefficients
*/
    data_products = std::make_shared<DataProducts>();

    nh = ros::NodeHandle("~");

    std::string camera_frame_param;
    nh.getParam("camera_frame", camera_frame_param);
    Initialise(camera_frame_param);

  }

  Projector::Projector(std::string _camera_frame) {

    data_products = std::make_shared<DataProducts>();

    Initialise(_camera_frame);
  }


  std::string
  Projector::VideoFileName() {

    std::string output_file_name = camera_frame;
    std::replace(output_file_name.begin(), output_file_name.end(), '/', '_');

    output_file_name.erase(0, output_file_name.find_first_not_of('_'));
    output_file_name.erase(output_file_name.find_last_not_of('_') + 1);
    return output_file_name + ".avi";
  }


  void DataProducts::Reset() {
    rgb_pointcloud = NULL;
    labelled_pointcloud = NULL;
    uv_pointcloud = NULL;

    labelled_pointcloud_msg = NULL;
    rgb_pointcloud_msg = NULL;
    uv_pointcloud_msg = NULL;

    range_image = NULL;
    intensity_image = NULL;
    ring_image = NULL;
  }


  void
  Projector::Initialise(std::string _camera_frame) {

    nh = ros::NodeHandle("~");

    camera_frame = _camera_frame;
    //nh.getParam("camera_frame", camera_frame);
    nh.getParam("lidar_frame", lidar_frame);
    nh.getParam("polygon_frame", polygon_frame);

    nh.getParam("publish_rgb_cloud", data_products->flag_rgb_pointcloud);
    nh.getParam("publish_uv_cloud", data_products->flag_uv_pointcloud);
    nh.getParam("publish_full_cloud", full);

    nh.getParam("save_range_image", data_products->flag_range_image);
    nh.getParam("save_intensity_image", data_products->flag_intensity_image);
    nh.getParam("save_ring_image", data_products->flag_ring_image);

    nh.getParam("path", path);

    pub_rgb = nh.advertise<sensor_msgs::PointCloud2>(camera_frame + "/colored_pointcloud", 10);
    pub_labels = nh.advertise<sensor_msgs::PointCloud2>(camera_frame + "/labelled_pointcloud", 10);
    pub_uv = nh.advertise<sensor_msgs::PointCloud2>(camera_frame + "/pixel_pointcloud", 10);

    cameramat = cv::Mat::zeros(3, 3, CV_64F);
    distcoeff = cv::Mat::eye(1, 4, CV_64F);

    valid_camera_info = false;
    valid_transform = false;

    if (camera_frame == "/sekonix_camera/port_a_cam_0/image_color")
      cam_frame = "port_a_camera_0";
    else if (camera_frame == "/sekonix_camera/port_a_cam_1/image_color")
      cam_frame = "port_a_camera_1";
    else if (camera_frame == "/sekonix_camera/port_b_cam_0/image_color")
      cam_frame = "port_b_camera_0";
    else if (camera_frame == "/sekonix_camera/port_a_cam_1/image_color")
      cam_frame = "port_b_camera_1";
    else if (camera_frame == "//sekonix_camera/port_c_cam_0/image_color")
      cam_frame = "port_c_camera_0";
    else if (camera_frame == "/sekonix_camera/port_a_cam_1/image_color")
      cam_frame = "port_c_camera_1";
    else if (camera_frame == "/sekonix_camera/port_d_cam_0/image_color")
      cam_frame = "port_d_camera_0";
    else if (camera_frame == "/sekonix_camera/port_a_cam_1/image_color")
      cam_frame = "port_d_camera_1";


  }


  void Projector::callback_camerainfo(const sensor_msgs::CameraInfo::ConstPtr &msg) {

/*
    camera_matrix << msg->K[0], 0,         msg->K[2],
                     0.,        msg->K[4], msg->K[5],
                     0.,        0.,        1.;

    distortion_coefficients << msg->D[0], msg->D[1], msg->D[2], msg->D[3];
*/

    cameramat.at<double>(0, 0) = msg->K[0];
    cameramat.at<double>(0, 2) = msg->K[2];
    cameramat.at<double>(1, 1) = msg->K[4];
    cameramat.at<double>(1, 2) = msg->K[5];

    cameramat.at<double>(2, 2) = 1;

    distcoeff.at<double>(0) = msg->D[0];
    distcoeff.at<double>(1) = msg->D[1];
    distcoeff.at<double>(2) = msg->D[2];
    distcoeff.at<double>(3) = msg->D[3];

    height = msg->height;
    width = msg->width;

    valid_camera_info = true;
  }

  bool
  Projector::CartesianToImage(geometry_msgs::Vector3 &vector, int &u, int &v, float &distance)
  {
    double u_float = 0., v_float = 0.;

    double tmpxC = vector.x / vector.z;
    double tmpyC = vector.y / vector.z;
    double tmpzC = vector.z;

    distance = pow(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z, 0.5);
    cv::Point2d planepointsC;
//    int range = std::min(float(round((dis / 50) * 149)), (float) 149.0);
//    int intensity = std::min(float(round((it->intensity / 100)) * 149), (float) 149.0);
//    int ring = std::min(float(round((it->ring % 16) * 9)), (float) 149.0);

    // applying the distortion
    double r2 = tmpxC * tmpxC + tmpyC * tmpyC;
    double r1 = pow(r2, 0.5);
    double a0 = std::atan(r1);
    double a1;

    a1 = a0 * (1 + distcoeff.at<double>(0) * pow(a0, 2) + distcoeff.at<double>(1) * pow(a0, 4) +
               distcoeff.at<double>(2) * pow(a0, 6) + distcoeff.at<double>(3) * pow(a0, 8));

    u_float = (a1 / r1) * tmpxC;
    v_float = (a1 / r1) * tmpyC;

    u_float = cameramat.at<double>(0, 0) * u_float + cameramat.at<double>(0, 2);
    v_float = cameramat.at<double>(1, 1) * v_float + cameramat.at<double>(1, 2);

    u = u_float;
    v = v_float;

    if (tmpzC >= 0.3 && std::abs(tmpxC) <= 1.35 && std::abs(tmpyC) <= 1.35)
      return true;

    return false;
  }

/*
  std::shared_ptr<DataProducts>
  Projector::pointcloud_camera(const pcl::PointCloud<pcl::PointXYZIR>::Ptr &msg,
                                const sensor_msgs::ImageConstPtr &img) {

*/
  std::shared_ptr<DataProducts>
  Projector::callback_lidar_camera(const sensor_msgs::PointCloud2ConstPtr &msg,
                                   const sensor_msgs::ImageConstPtr &img) {

    if (valid_camera_info and valid_transform) {
      data_products->Reset();


      // convert from PC2 msg to pointcloud with correct transform
      pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZIR>());
      sensor_msgs::PointCloud2 cloud_tf;
      tf2::doTransform(*msg, cloud_tf, transform);
      pcl::fromROSMsg(cloud_tf, *cloud);

      // create the pointclouds
      if (data_products->flag_rgb_pointcloud && sensor_msgs::image_encodings::isColor(img->encoding)) {
        data_products->rgb_pointcloud = pcl::PointCloud<pcl::PointXYZIRRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZIRRGB>());
      }
      else if (data_products->flag_rgb_pointcloud && !sensor_msgs::image_encodings::isColor(img->encoding)) {
        data_products->labelled_pointcloud = pcl::PointCloud<pcl::PointXYZIRL>::Ptr(new pcl::PointCloud<pcl::PointXYZIRL>());
      }

      if (data_products->flag_uv_pointcloud) {
        data_products->uv_pointcloud = pcl::PointCloud<pcl::PointXYZIRUV>::Ptr(new pcl::PointCloud<pcl::PointXYZIRUV>());
      }

      // convert image_msg to cv pointer
      cv_bridge::CvImageConstPtr cv_ptr;
      try
      {
        if (sensor_msgs::image_encodings::isColor(img->encoding))
          cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        else
          cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return data_products;
      }

      // create the required images
      if (data_products->flag_range_image) {
        data_products->image_range = cv_ptr->image.clone();
      }

      if (data_products->flag_intensity_image) {
        data_products->image_intensity = cv_ptr->image.clone();
      }

      if (data_products->flag_ring_image) {
        data_products->image_ring = cv_ptr->image.clone();
      }

      // for each point, convert to image coords and draw if necessary
      for (pcl::PointCloud<pcl::PointXYZIR>::const_iterator it = cloud->begin(); it != cloud->end(); it++) {

        int u = 0,v = 0;
        float distance = 0.;

        geometry_msgs::Vector3 vector;
        vector.x = it->x; vector.y = it->y; vector.z = it->z;

        // if the transform is successful and the point is on the image
        if (CartesianToImage(vector, u, v, distance) && v >= 0 && v < height && u >= 0 && u < width ) {

          // if the pointers to the pointclouds are valid, add the point
          if (data_products->rgb_pointcloud) {
            pcl::PointXYZIRRGB point;
            point.x = it->x;
            point.y = it->y;
            point.z = it->z;
            point.intensity = it->intensity;
            point.ring = it->ring;

            uint32_t rgb;

            int b = int(cv_ptr->image.at<cv::Vec3b>(v, u)[0]);
            int g = int(cv_ptr->image.at<cv::Vec3b>(v, u)[1]);
            int r = int(cv_ptr->image.at<cv::Vec3b>(v, u)[2]);

            rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(g) << 8 |
                            static_cast<uint32_t>(b));

            point.rgb = *reinterpret_cast<float *>(&rgb);
            data_products->rgb_pointcloud->points.push_back(point);
          }


          if (data_products->labelled_pointcloud) {
            pcl::PointXYZIRL point;
            point.x = it->x;
            point.y = it->y;
            point.z = it->z;
            point.intensity = it->intensity;
            point.ring = it->ring;

            point.label = uint16_t(cv_ptr->image.at<cv::Vec3b>(v, u)[0]);
            data_products->labelled_pointcloud->points.push_back(point);
          }

          if (data_products->uv_pointcloud) {

            pcl::PointXYZIRUV point;
            point.x = it->x;
            point.y = it->y;
            point.z = it->z;
            point.intensity = it->intensity;
            point.ring = it->ring;
            point.U = u;
            point.V = v;

            data_products->uv_pointcloud->points.push_back(point);
          }



          if (data_products->flag_range_image) {

            int range = std::min(float(round((distance / 50) * 149)), (float) 149.0);
            cv::circle(data_products->image_range,
                       cv::Point(u, v), 4,
                       CV_RGB(255 * colmap[range][0], 255 * colmap[range][1], 255 * colmap[range][2]), -1);
          }


          if (data_products->flag_intensity_image) {

            int intensity = std::min(float(round((it->intensity / 100)) * 149), (float) 149.0);

            cv::circle(data_products->image_intensity,
                       cv::Point(u, v), 2,
                       CV_RGB(255 * colmap[intensity][0], 255 * colmap[intensity][1], 255 * colmap[intensity][2]), -1);
          }


          if (data_products->flag_ring_image) {

            int ring = std::min(float(round((it->ring % 16) * 9)), (float) 149.0);

            cv::circle(data_products->image_ring,
                       cv::Point(u, v), 2,
                       CV_RGB(255 * colmap[ring][0], 255 * colmap[ring][1], 255 * colmap[ring][2]), -1);
          }
        }

        // the pointcloud couldn't be set to the image, might want to add it to the output pointcloud anyway
        else {

          // if the pointers to the pointclouds are valid, add the point
          if (data_products->rgb_pointcloud and full) {
            pcl::PointXYZIRRGB point;
            point.x = it->x;
            point.y = it->y;
            point.z = it->z;
            point.intensity = it->intensity;
            point.ring = it->ring;

            uint32_t rgb = (static_cast<uint32_t>(255) << 16 |
                            static_cast<uint32_t>(255) << 8 | static_cast<uint32_t>(255));

            point.rgb = *reinterpret_cast<float *>(&rgb);
            data_products->rgb_pointcloud->points.push_back(point);
          }
          // if the pointers to the pointclouds are valid, add the point
          if (data_products->labelled_pointcloud and full) {
            pcl::PointXYZIRL point;
            point.x = it->x;
            point.y = it->y;
            point.z = it->z;
            point.intensity = it->intensity;
            point.ring = it->ring;

            uint32_t rgb = (static_cast<uint32_t>(255) << 16 |
                            static_cast<uint32_t>(255) << 8 | static_cast<uint32_t>(255));

            point.label = 0;
            data_products->labelled_pointcloud->points.push_back(point);
          }
          if (data_products->uv_pointcloud and full) {
            pcl::PointXYZIRUV point;
            point.x = it->x;
            point.y = it->y;
            point.z = it->z;
            point.intensity = it->intensity;
            point.ring = it->ring;
            point.U = 3000;
            point.V = 3000;

            data_products->uv_pointcloud->points.push_back(point);
          }
        }
      }


      // if required, create the image messages from the cv pointers
      std_msgs::Header header; // empty header
      header.seq = 0; // user defined counter
      header.stamp = img->header.stamp; // time

      if (data_products->flag_range_image) {
        cv_bridge::CvImage cv_image = cv_bridge::CvImage(header,
                                                         img->encoding,
                                                         data_products->image_range);

        data_products->range_image = sensor_msgs::Image::Ptr(new sensor_msgs::Image);
        cv_image.toImageMsg(*(data_products->range_image));
      }
      if (data_products->flag_intensity_image) {
        cv_bridge::CvImage cv_image = cv_bridge::CvImage(header,
                                                         img->encoding,
                                                         data_products->image_intensity);

        data_products->intensity_image = sensor_msgs::Image::Ptr(new sensor_msgs::Image);
        cv_image.toImageMsg(*(data_products->intensity_image));
      }
      if (data_products->flag_ring_image) {
        cv_bridge::CvImage cv_image = cv_bridge::CvImage(header,
                                                         img->encoding,
                                                         data_products->image_ring);

        data_products->ring_image = sensor_msgs::Image::Ptr(new sensor_msgs::Image);
        cv_image.toImageMsg(*(data_products->ring_image));
      }




      /*

std::string s_v, s_i;
std::stringstream out;

out << msg->header.seq;
s_v = out.str();
out << img->header.seq;
s_i = out.str();

if (data_products->flag_range_image) {
  if (!path.empty() && !output_range_video.isOpened()) {
    output_range_video = cv::VideoWriter(path + "/range_" + VideoFileName(), CV_FOURCC('M', 'J', 'P', 'G'), 10,
                                         cv::Size(width, height));
  }
  if (output_range_video.isOpened()) {
    output_range_video << data_products->image_range;
  }
}
if (data_products->flag_intensity_image) {
  if (!path.empty() && !output_intensity_video.isOpened()) {
    output_intensity_video = cv::VideoWriter(path + "/intensity_" + VideoFileName(),
                                             CV_FOURCC('M', 'J', 'P', 'G'), 10,
                                             cv::Size(width, height));
  }
  if (output_intensity_video.isOpened()) {
    output_intensity_video << data_products->image_intensity;
  }
}
if (data_products->flag_ring_image) {
  if (!path.empty() && !output_ring_video.isOpened()) {
    output_ring_video = cv::VideoWriter(path + "/ring_" + VideoFileName(), CV_FOURCC('M', 'J', 'P', 'G'), 10,
                                        cv::Size(width, height));
  }
  if (output_ring_video.isOpened()) {
    output_ring_video << data_products->image_ring;
  }
}
*/


      // if required. create the resulting PC2 messages from the point clouds
      if (data_products->rgb_pointcloud) {

        data_products->rgb_pointcloud_msg = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*(data_products->rgb_pointcloud), *(data_products->rgb_pointcloud_msg));
        data_products->rgb_pointcloud_msg->header = msg->header;
        data_products->rgb_pointcloud_msg->header.frame_id = cam_frame;
        pub_rgb.publish(*(data_products->rgb_pointcloud_msg));
      }

      if (data_products->labelled_pointcloud) {
        data_products->labelled_pointcloud_msg = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*(data_products->labelled_pointcloud), *(data_products->labelled_pointcloud_msg));
        data_products->labelled_pointcloud_msg->header = msg->header;
        data_products->labelled_pointcloud_msg->header.frame_id = cam_frame;
        pub_labels.publish(*(data_products->labelled_pointcloud_msg));
      }

      if (data_products->uv_pointcloud) {
        //sensor_msgs::PointCloud2 ros_cloud;
        data_products->uv_pointcloud_msg = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*(data_products->uv_pointcloud), *(data_products->uv_pointcloud_msg));
        data_products->uv_pointcloud_msg->header = msg->header;
        data_products->uv_pointcloud_msg->header.frame_id = cam_frame;
        pub_uv.publish(*(data_products->uv_pointcloud_msg));
      }

    } else {
      std::string missing_text = "missing transforms for ";
      if (!valid_camera_info) {
        missing_text += "camera info";
        if (!valid_transform) {
          missing_text += " AND ";
        }
      }
      if (!valid_transform) {
        missing_text += "tf";
      }
      ROS_INFO_STREAM_THROTTLE(1, missing_text);
    }

    return data_products;
  }




  bool
  Projector::draw_pose_shape(std::string frame_id,
                       const std::shared_ptr <tf2::BufferCore> &tf_buffer,
                       ros::Time &message_time,
                       geometry_msgs::Pose pose,
                       double shape_size, // in meters
                       cv::Mat &img,
                       uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha) {


    float delta_x = pose.position.x;
    float delta_y = pose.position.y;
    float delta_z = pose.position.z;

    double r, y, p;
    geometry_msgs::Quaternion q = pose.orientation;
    
    tf::Quaternion tfq;
    
    //// removed because it was causing warnings for un-normalised conversions 
    //tf::quaternionMsgToTF(q, tfq);

    tfq.setW(q.w); tfq.setX(q.x); tfq.setY(q.y); tfq.setZ(q.z);
    tfq.normalize();

    tf::Matrix3x3(tfq).getEulerYPR(y, p, r);

    pcl::PointCloud<pcl::PointXYZIR>::Ptr polygon_original_frame(new pcl::PointCloud <pcl::PointXYZIR>);

    std::vector <geometry_msgs::Vector3Stamped> polygon; //, cv::Mat &img, uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha

    geometry_msgs::Vector3Stamped untransformed_point;
    untransformed_point.vector.z = 0;

    untransformed_point.vector.x = -.5*shape_size;
    untransformed_point.vector.y = -.5*shape_size;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = -.7*shape_size;
    untransformed_point.vector.y = 0;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = -.5*shape_size;
    untransformed_point.vector.y = +.5*shape_size;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = 0;
    untransformed_point.vector.y = +.7*shape_size;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = +.5*shape_size;
    untransformed_point.vector.y = +0.5*shape_size;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = +.7*shape_size;
    untransformed_point.vector.y = 0;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = +0.5*shape_size;
    untransformed_point.vector.y = -0.5*shape_size;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = 0;
    untransformed_point.vector.y = -.7*shape_size;
    polygon.push_back(untransformed_point);

//    double x_offset = 1.2;
    double x_offset = 0;

    for (auto &arrow_point: polygon) {

      double new_x = arrow_point.vector.x * cos(y) - arrow_point.vector.y * sin(y);
      double new_y = arrow_point.vector.y * cos(y) + arrow_point.vector.x * sin(y);

      pcl::PointXYZIR new_point_arrow;
      new_point_arrow.x = x_offset + delta_x + new_x;
      new_point_arrow.y = delta_y + new_y;
      new_point_arrow.z = delta_z;
      new_point_arrow.intensity = 1.;
      new_point_arrow.ring = 1;

      polygon_original_frame->push_back(new_point_arrow);
    }

    return draw_polygon(frame_id, tf_buffer, message_time, polygon_original_frame, img, red, green, blue, alpha);
  }


  bool
  Projector::draw_pose_arrow(std::string frame_id,
                             const std::shared_ptr <tf2::BufferCore> &tf_buffer,
                             ros::Time &message_time,
                             geometry_msgs::Pose pose,
                             double arrow_size, // in meters
                             cv::Mat &img,
                             uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha) {


    float delta_x = pose.position.x;
    float delta_y = pose.position.y;
    float delta_z = pose.position.z;

    double r, y, p;
    geometry_msgs::Quaternion q = pose.orientation;
    tf::Quaternion tfq;
    tf::quaternionMsgToTF(q, tfq);
    tf::Matrix3x3(tfq).getEulerYPR(y, p, r);

    pcl::PointCloud<pcl::PointXYZIR>::Ptr polygon_original_frame(new pcl::PointCloud <pcl::PointXYZIR>);

    std::vector <geometry_msgs::Vector3Stamped> polygon; //, cv::Mat &img, uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha

    geometry_msgs::Vector3Stamped untransformed_point;
    untransformed_point.vector.z = 0;

    untransformed_point.vector.x = -.1*arrow_size;
    untransformed_point.vector.y = -0.30*arrow_size;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = +0.65*arrow_size;
    untransformed_point.vector.y = -0.30*arrow_size;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = 0.65*arrow_size;
    untransformed_point.vector.y = -0.5*arrow_size;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = arrow_size;
    untransformed_point.vector.y = 0;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = +0.65*arrow_size;
    untransformed_point.vector.y = +0.5*arrow_size;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = +0.65*arrow_size;
    untransformed_point.vector.y = +0.3*arrow_size;
    polygon.push_back(untransformed_point);
    untransformed_point.vector.x = -.1*arrow_size;
    untransformed_point.vector.y = +0.30*arrow_size;
    polygon.push_back(untransformed_point);

//    double x_offset = 1.2;
    double x_offset = 0;

    for (auto &arrow_point: polygon) {

      double new_x = arrow_point.vector.x * cos(y) - arrow_point.vector.y * sin(y);
      double new_y = arrow_point.vector.y * cos(y) + arrow_point.vector.x * sin(y);

      pcl::PointXYZIR new_point_arrow;
      new_point_arrow.x = x_offset + delta_x + new_x;
      new_point_arrow.y = delta_y + new_y;
      new_point_arrow.z = delta_z;
      new_point_arrow.intensity = 1.;
      new_point_arrow.ring = 1;

      polygon_original_frame->push_back(new_point_arrow);
    }

    return draw_polygon(frame_id, tf_buffer, message_time, polygon_original_frame, img, red, green, blue, alpha);
  }


  bool
  Projector::draw_dots(const sensor_msgs::PointCloud2ConstPtr &msg,
                          const std::shared_ptr <tf2::BufferCore> &tf_buffer,
                        cv::Mat &img,
                        uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, int thickness) {

    geometry_msgs::TransformStamped image_transform;
    
    try {
      // attempt lookup of transform
      image_transform = tf_buffer->lookupTransform(cam_frame, msg->header.frame_id, msg->header.stamp);

      sensor_msgs::PointCloud2Ptr pc_camera_frame(new sensor_msgs::PointCloud2);
      pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZIR>());
      
      // convert to pc message and do the conversion
      tf2::doTransform(*msg, *pc_camera_frame, image_transform);
      pcl::fromROSMsg(*pc_camera_frame, *cloud);

      // for each point, convert to image coords and draw if necessary
      for (pcl::PointCloud<pcl::PointXYZIR>::const_iterator it = cloud->begin(); it != cloud->end(); it++) {

        int u = 0,v = 0;
        float distance = 0.;

        geometry_msgs::Vector3 vector;
        vector.x = it->x; vector.y = it->y; vector.z = it->z;

        // if the transform is successful and the point is on the image
        if (CartesianToImage(vector, u, v, distance) && v >= 0 && v < height && u >= 0 && u < width ) {

          int intensity = std::min(float(round((it->intensity / 100)) * 149), (float) 149.0);

          cv::circle(img,
                     cv::Point(u, v), thickness,
                     CV_RGB(255 * colmap[intensity][0], 255 * colmap[intensity][1], 255 * colmap[intensity][2]), -1);
          
        }
      }

    }
    catch (tf2::TransformException &ex) {
      ROS_INFO_STREAM_THROTTLE(1., "marker transform "<<  cam_frame << " -> " << msg->header.frame_id << " NOT FOUND");
      return false;
    }


    return true;
  }

/*

  bool
  Projector::draw_dots_new(const sensor_msgs::PointCloud2ConstPtr &msg,
                          const std::shared_ptr <tf2::BufferCore> &tf_buffer,
                        cv::Mat &img,
                        uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, int thickness) {

    geometry_msgs::TransformStamped image_transform;
    
    try {
      // attempt lookup of transform
      image_transform = tf_buffer->lookupTransform(cam_frame, "base_link", msg->header.stamp);

      sensor_msgs::PointCloud2Ptr pc_camera_frame(new sensor_msgs::PointCloud2);
      pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZIR>());
      
      // convert to pc message and do the conversion
      tf2::doTransform(*msg, *pc_camera_frame, image_transform);
      pcl::fromROSMsg(*pc_camera_frame, *cloud);



      //ArrayBoolean NewCartesianToImage(MatrixXf &world_coordinates, MatrixXf &image_coordinates, MatrixXf &camera_matrix, MatrixXf &distortion_coefficients);

      MatrixXf image_coordinates(cloud->size(), 2);
      MatrixXf world_coordinates = cloud->getMatrixXfMap();

      auto valid_points = NewCartesianToImage(world_coordinates, image_coordinates, camera_matrix, distortion_coefficients);

      int intensity = 0;

      for (int i=0; i < world_coordinates.size(); i++) {
        if (valid_points(i) == true) {
          // third field represents intensity
          intensity = std::min(float(round((world_coordinates(i,3) / 100)) * 149), (float) 149.0);

          cv::circle(img, cv::Point(image_coordinates(i,0), image_coordinates(i,1)), thickness,
                     CV_RGB(255 * colmap[intensity][0], 255 * colmap[intensity][1], 255 * colmap[intensity][2]), -1);
        }
      }


    }
    catch (tf2::TransformException &ex) {
      ROS_INFO_STREAM_THROTTLE(1., "marker transform "<<  cam_frame << " -> " << msg->header.frame_id << " NOT FOUND");
      return false;
    }


    return true;
  }


*/

  bool
  Projector::draw_polygon(std::string frame_id,
                          const std::shared_ptr <tf2::BufferCore> &tf_buffer,
                          ros::Time &message_time,
                          const pcl::PointCloud<pcl::PointXYZIR>::Ptr &polygon,
                          cv::Mat &img,
                          uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, int thickness) {

    geometry_msgs::TransformStamped polygon_transform;
    try {
      // attempt lookup of transform
      polygon_transform = tf_buffer->lookupTransform(cam_frame, frame_id, message_time);

      sensor_msgs::PointCloud2Ptr pc_original_frame(new sensor_msgs::PointCloud2);
      sensor_msgs::PointCloud2Ptr pc_camera_frame(new sensor_msgs::PointCloud2);

      pcl::PointCloud<pcl::PointXYZIR>::Ptr polygon_camera_frame(new pcl::PointCloud<pcl::PointXYZIR>());

      // convert to pc message and do the conversion
      pcl::toROSMsg(*polygon.get(), *pc_original_frame);
      tf2::doTransform(*pc_original_frame, *pc_camera_frame, polygon_transform);
      pcl::fromROSMsg(*pc_camera_frame, *polygon_camera_frame);

      // draw the converted polygon into the image
      return draw_transformed_polygon(polygon_camera_frame, img, red, green, blue, alpha, thickness);
    }
    catch (tf2::TransformException &ex) {
      ROS_INFO_STREAM_THROTTLE(1., "marker transform "<<  cam_frame << " -> " << frame_id << " NOT FOUND");
      return false;
    }
  }


  bool
  Projector::draw_polyline(std::string frame_id,
                          const std::shared_ptr <tf2::BufferCore> &tf_buffer,
                          ros::Time &message_time,
                          const pcl::PointCloud<pcl::PointXYZIR>::Ptr &polygon,
                          cv::Mat &img,
                          uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, int thickness) {

    geometry_msgs::TransformStamped polygon_transform;
    try {
      // attempt lookup of transform
      polygon_transform = tf_buffer->lookupTransform(cam_frame, frame_id, message_time);

      sensor_msgs::PointCloud2Ptr pc_original_frame(new sensor_msgs::PointCloud2);
      sensor_msgs::PointCloud2Ptr pc_camera_frame(new sensor_msgs::PointCloud2);

      pcl::PointCloud<pcl::PointXYZIR>::Ptr polygon_camera_frame(new pcl::PointCloud<pcl::PointXYZIR>());

      // convert to pc message and do the conversion
      pcl::toROSMsg(*polygon.get(), *pc_original_frame);
      tf2::doTransform(*pc_original_frame, *pc_camera_frame, polygon_transform);
      pcl::fromROSMsg(*pc_camera_frame, *polygon_camera_frame);

      // draw the converted polygon into the image
      return draw_transformed_polyline(polygon_camera_frame, img, red, green, blue, alpha, thickness);
    }
    catch (tf2::TransformException &ex) {
      ROS_INFO_STREAM_THROTTLE(1., "marker transform "<<  cam_frame << " -> " << frame_id << " NOT FOUND");
      return false;
    }
  }



  bool
  Projector::draw_polygon_direct_to_screen(const pcl::PointCloud<pcl::PointXYZIR>::Ptr &polygon, cv::Mat &img, uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha) {

    if (!polygon->empty()) {
  
      std::vector<cv::Point> cv_polygon_points;
  
      // for each point, convert to image coords and draw if necessary
      for (pcl::PointCloud<pcl::PointXYZIR>::const_iterator it = polygon->begin(); it != polygon->end(); it++) {
        cv_polygon_points.push_back(cv::Point(it->x, it->y));
      }

      if (cv_polygon_points.size() > 0) {
        const cv::Point* cv_polygon[1] = { &cv_polygon_points[0] };
        int number_of_points[] = { (int)cv_polygon_points.size() };

        cv::fillPoly(img, cv_polygon, number_of_points, 1.5, CV_RGB(0.7*(float)red, 0.7*(float)green, 0.7*(float)blue), cv::LINE_8);
        //      cv::polylines(img, cv_polygon_points, true, cv::Scalar(red,green,blue),1,150,0);
      }
    }

    return true;
  }



  bool
  Projector::draw_transformed_polygon(const pcl::PointCloud<pcl::PointXYZIR>::Ptr &polygon, cv::Mat &img, uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, int thickness) {

      std::vector<cv::Point> cv_polygon_points;

      if (!polygon->empty()) {

      geometry_msgs::Vector3 previous_point;
      previous_point.x = polygon->back().x;
      previous_point.y = polygon->back().y;
      previous_point.z = polygon->back().z;

      auto first_point = polygon->front();
        polygon->push_back(first_point);

      bool previous_off_screen = true;

      // for each point, convert to image coords and draw if necessary
      for (pcl::PointCloud<pcl::PointXYZIR>::const_iterator it = polygon->begin(); it != polygon->end(); it++) {

        int u = 0, v = 0;
        float distance = 0.;

        geometry_msgs::Vector3 vector;
        vector.x = it->x;
        vector.y = it->y;
        vector.z = it->z;

        bool POLYGON_TO_IMAGE_BORDER = true;

        bool found_any_points = false;

        // if the transform is successful and the point is on the image
        if (CartesianToImage(vector, u, v, distance)) {

          if (CartesianToImage(previous_point, u, v, distance)) {

            cv_polygon_points.push_back(cv::Point(u, v));
            found_any_points = true;
            previous_point = vector;
          }
          else {
            // find a point along the line that is on the image, replace with this
            auto point_difference = vector;
            point_difference.x -= previous_point.x;
            point_difference.y -= previous_point.y;
            point_difference.z -= previous_point.z;

            for (float interpolate = 0.0; interpolate < 1.0; interpolate += 0.05) {
              auto intermediate_point = previous_point;
              intermediate_point.x += interpolate * point_difference.x;
              intermediate_point.y += interpolate * point_difference.y;
              intermediate_point.z += interpolate * point_difference.z;

              if (CartesianToImage(intermediate_point, u, v, distance)) {
                cv_polygon_points.push_back(cv::Point(u, v));
                found_any_points = true;
                //            previous_point = intermediate_point;
              }
            }

          }
        }
        else if (POLYGON_TO_IMAGE_BORDER) {

          //check if prev is also off screen
          if (CartesianToImage(previous_point, u, v, distance)) {



            // find a point along the line that is on the image, replace with this
            auto point_difference = vector;
            point_difference.x -= previous_point.x;
            point_difference.y -= previous_point.y;
            point_difference.z -= previous_point.z;

            for (float interpolate = 0.0; interpolate < 1.0; interpolate += 0.05) {
              auto intermediate_point = previous_point;
              intermediate_point.x += interpolate * point_difference.x;
              intermediate_point.y += interpolate * point_difference.y;
              intermediate_point.z += interpolate * point_difference.z;

              if (CartesianToImage(intermediate_point, u, v, distance)) {
                cv_polygon_points.push_back(cv::Point(u, v));
                found_any_points = true;
                //            previous_point = intermediate_point;
              }
            }
          }
//          if (!found_any_points)

          previous_off_screen = true;
        }

        previous_point = vector;

      }
    }

/*
      for (auto &point: polygon) {

        geometry_msgs::Vector3Stamped point_transformed;
//        point.vector.x = std::get<0>(polygon_point);
//        point.vector.y = std::get<1>(polygon_point);
//        point.vector.z = std::get<2>(polygon_point);
        //tf2::doTransform(point, point_transformed, polygon_transform);
        tf2::doTransform(point, point_transformed, transform);

        int u = 0,v = 0;
        float distance = 0.;

        geometry_msgs::Vector3 vector(point_transformed.vector);
        //vector.x = it->x; vector.y = it->y; vector.z = it->z;
        // if the transform is successful and the point is on the image
        if (CartesianToImage(vector, u, v, distance) && v >= 0 && v < height && u >= 0 && u < width ) {
std::cout << "test: " << vector.x << ", " << vector.y << ", " << vector.z << std::endl;
          std::cout << ": " << ", " << u << ", " << v << std::endl;

          cv_polygon_points.push_back(cv::Point(v, u));
        }
      }
*/
/*      for (auto &point: cv_polygon_points) {
        cv::circle(img,
                   cv::Point(point.x, point.y), 1,
                   cv::Scalar(255, 10, 255));

      }
*/

      if (cv_polygon_points.size() > 0) {
        const cv::Point* cv_polygon[1] = { &cv_polygon_points[0] };
        int number_of_points[] = { (int)cv_polygon_points.size() };

//        std::vector<cv::Point> point_v;
//        for (int i = 0; i < (int)cv_polygon_points.size(); i++){
 //         point_v.push_back(cv_polygon)
 //       }

        if (thickness < 0) 
          cv::fillPoly(img, cv_polygon, number_of_points, 1.5, CV_RGB(0.7*(float)red, 0.7*(float)green, 0.7*(float)blue), cv::LINE_8);

        //const cv::Point* ppt[1] = { cv_polygon[0] };
        //int npt[] = { 20 };

        if (thickness < 0)
          cv::polylines(img, cv_polygon_points, true, cv::Scalar(red,green,blue),1,150,0);
        else {
          // the first point was included in the transform process to complete the polygon (should be reviewed)
          //  so it needs to be removed so the polygon is not closed.
          cv_polygon_points.erase(cv_polygon_points.begin());
          
          cv::circle(img, cv_polygon_points.front(), 4, cv::Scalar(0,0,255),2,cv::LINE_AA);
          cv::circle(img, cv_polygon_points.back(), 9, cv::Scalar(0,255,0),2,cv::LINE_AA);

          cv::polylines(img, cv_polygon_points, false, cv::Scalar(red,green,blue),thickness,cv::LINE_AA);
        }
        
        //cv::polylines(img, ppt, number_of_points, false, CV_RGB(red, green, blue), 8);

////        cv::drawLine(img, cv_polygon, number_of_points, 1, CV_RGB(red, green, blue), cv::LINE_8);
      }

    return true;
  }


bool
Projector::draw_transformed_polyline(const pcl::PointCloud<pcl::PointXYZIR>::Ptr &polygon, cv::Mat &img, uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, int thickness) {

  if (polygon->size() < 2)
    return false;

  bool found_any_points = false;

  int u_prev = 0, v_prev = 0, u_next = 0, v_next = 0;
  float distance_prev = 0., distance_next = 0.;

  std::vector<cv::Point> cv_polygon_points;

  // check if the first point is in the image
  auto first_point = polygon->front();

  pcl::PointCloud<pcl::PointXYZIR>::const_iterator prev, next;
  geometry_msgs::Vector3 prev_vector, next_vector;

  prev = polygon->begin();
  prev_vector.x = prev->x; prev_vector.y = prev->y; prev_vector.z = prev->z;
  bool prev_in = CartesianToImage(prev_vector, u_prev, v_prev, distance_prev); 

  if (prev_in) {
      found_any_points = true;
      cv_polygon_points.push_back(cv::Point(u_prev, v_prev));
  }

  for (next = ++polygon->begin(); next != polygon->end(); prev++, next++) {

    next_vector.x = next->x; next_vector.y = next->y; next_vector.z = next->z;
    bool next_in = CartesianToImage(next_vector, u_next, v_next, distance_next); 

    if (!next_in && !prev_in) {
      // neither are in, so ignore both points
    }
    else if (next_in && prev_in){
      // both are in, so use both (only need to add the next point as the prev point is already included)
      found_any_points = true;
      cv_polygon_points.push_back(cv::Point(u_next, v_next));
    } 
    else {
      // one only of the points are in, so interpolate and add as required

      // find a point along the line that is on the image, replace with this
      auto point_difference = next_vector;
      point_difference.x -= prev_vector.x;
      point_difference.y -= prev_vector.y;
      point_difference.z -= prev_vector.z;

      for (float interpolate = 0.0; interpolate < 1.0; interpolate += 0.05) {
        auto intermediate_point = prev_vector;
        intermediate_point.x += interpolate * point_difference.x;
        intermediate_point.y += interpolate * point_difference.y;
        intermediate_point.z += interpolate * point_difference.z;

        if (CartesianToImage(intermediate_point, u_prev, v_prev, distance_prev)) {
          cv_polygon_points.push_back(cv::Point(u_prev, v_prev));
          found_any_points = true;
        }
      }

      if (next_in){
        // include the next point if it is in
        found_any_points = true;
        cv_polygon_points.push_back(cv::Point(u_next, v_next));
      } 
    }

    prev_in = next_in;
    prev_vector = next_vector;
    u_prev = u_next;
    v_prev = v_next;
    distance_prev = distance_next;
  }
  if (cv_polygon_points.size() > 0) {
    //cv::circle(img, cv_polygon_points.front(), 4, cv::Scalar(0,0,255),2,cv::LINE_AA);
    //cv::circle(img, cv_polygon_points.back(), 9, cv::Scalar(0,255,0),2,cv::LINE_AA);

    cv::polylines(img, cv_polygon_points, false, cv::Scalar(red,green,blue),thickness,cv::LINE_AA);
  }

  return true;
}




  bool
  Projector::lookupLidarCameraTF(const std::shared_ptr <tf2::BufferCore> &tf_buffer) {

    ROS_INFO_STREAM("Looking for projection TF between " << cam_frame << " and " << lidar_frame);
    ROS_INFO_STREAM("Looking for polygon TF between " << cam_frame << " and " << polygon_frame);

    if (!valid_transform) {
      try {
        transform = tf_buffer->lookupTransform(cam_frame, lidar_frame, ros::Time(0));
        ROS_INFO_STREAM("lidar camera transform FOUND");
        valid_transform = true;

      }
      catch (tf2::TransformException &ex) {
        ROS_INFO_STREAM("lidar camera transform NOT FOUND");
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
      }

      try {
        polygon_transform = tf_buffer->lookupTransform(cam_frame, "base_link", ros::Time(0));
        ROS_INFO_STREAM("polygon transform FOUND");
        valid_transform = true;

      }
      catch (tf2::TransformException &ex) {
        ROS_INFO_STREAM("polygon transform NOT FOUND");
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
      }

      try {
        lidar_baselink_transform = tf_buffer->lookupTransform(lidar_frame, "base_link", ros::Time(0));
        ROS_INFO_STREAM("lidar_baselink_transform transform FOUND");
        valid_transform = true;

      }
      catch (tf2::TransformException &ex) {
        ROS_INFO_STREAM("lidar_baselink_transform NOT FOUND");
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
      }



    }
  }


/*/
ArrayBoolean Projector::NewCartesianToImage(MatrixXf &world_coordinates, MatrixXf &image_coordinates, MatrixXf &camera_matrix, MatrixXf &distortion_coefficients)  {

    auto valid_z_pixels = world_coordinates(Eigen::placeholders::all, 2).array() >= MIN_Z_VALUE;
    
    auto div_z = world_coordinates.rowwise().hnormalized();

    // applying the distortion
    auto r1 = div_z.array().square().rowwise().sum().cwiseSqrt();
    
    auto a0 = r1.array().atan();

    auto a1 = a0 * (1 + distortion_coefficients(0,0) * a0.square() + distortion_coefficients(0,1) * a0.square().square() +
               distortion_coefficients(0,2) * a0.pow(6) + distortion_coefficients(0,3) * a0.square().square().square());

    auto a1_r1 = (a1 / r1);

    MatrixXf transformed_points = div_z.array().colwise() * a1_r1.array();

    MatrixXf cam(2,2);
    cam << camera_matrix(0, 0), 0, 0, camera_matrix(1, 1);

    MatrixXf cam_offset(1,2);
    cam_offset << camera_matrix(0, 2), camera_matrix(1, 2);

    image_coordinates = (transformed_points * cam).rowwise() + cam_offset.array();

    auto valid_xy_pixels = div_z(Eigen::placeholders::all, 0).array() <= MIN_X_VALUE && 
                            div_z(Eigen::placeholders::all, 1).array() <= MIN_Y_VALUE &&
                            valid_z_pixels;

    return valid_xy_pixels;
  }

*/

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "LidarCameraProjector");

  lidar_camera_projection::Projector projector;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener listener(tfBuffer);

  ROS_INFO_STREAM("looking for transform " << projector.cam_frame << " to " << projector.lidar_frame);

  while (true) {
    try {
      projector.transform = tfBuffer.lookupTransform(projector.cam_frame,
                                                     projector.lidar_frame,
                                                     ros::Time(0));

      projector.valid_transform = true;
      break;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();

      continue;
    }

  }

  message_filters::Subscriber <sensor_msgs::Image> image_sub(projector.nh, "image", 1);
  message_filters::Subscriber <sensor_msgs::PointCloud2> lidar_sub(projector.nh, "raw_lidar", 1);

  typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

  message_filters::Synchronizer <MySyncPolicy> sync(MySyncPolicy(10), lidar_sub, image_sub);
  sync.registerCallback(boost::bind(&lidar_camera_projection::Projector::callback_lidar_camera, &projector, _1, _2));

  ros::Subscriber sub = projector.nh.subscribe("camera_info", 20,
                                               &lidar_camera_projection::Projector::callback_camerainfo,
                                               &projector);

  ros::spin();

  return 0;
}
