#ifndef MAIN_HPP
#define MAIN_HPP
#define PCL_NO_PRECOMPILE

#include <tuple>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <ros/console.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <pcl/PointIndices.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>
#include "pcl_ros/transforms.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf_conversions/tf_eigen.h>
#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/package.h>
#include <custom_point_types/point_xyzir.h>
#include <custom_point_types/point_xyzirl.h>
#include <custom_point_types/point_xyziruv.h>
#include <custom_point_types/point_xyzirrgb.h>
#include "colour_map.hpp"

#include <Eigen/Dense>

/* 
using Eigen::MatrixXf;

typedef Eigen::Array<bool,Eigen::Dynamic,1> ArrayBoolean;
*/


namespace lidar_camera_projection {

  class DataProducts {

  public:

    DataProducts() : flag_rgb_pointcloud(false),
                     flag_uv_pointcloud(false),
                     flag_range_image(false),
                     flag_intensity_image(false),
                     flag_ring_image(false) {

      Reset();
    }

    sensor_msgs::PointCloud2::Ptr rgb_pointcloud_msg, uv_pointcloud_msg, labelled_pointcloud_msg;

    pcl::PointCloud<pcl::PointXYZIRL>::Ptr labelled_pointcloud;
    pcl::PointCloud<pcl::PointXYZIRUV>::Ptr uv_pointcloud;
    pcl::PointCloud<pcl::PointXYZIRRGB>::Ptr rgb_pointcloud;

    sensor_msgs::Image::Ptr range_image, intensity_image, ring_image;

    cv::Mat image_range, image_intensity, image_ring;

    bool flag_rgb_pointcloud, flag_uv_pointcloud, flag_range_image, flag_intensity_image, flag_ring_image;

    void Reset();
  };


  class Projector {

  public:

    Projector();

    Projector(std::string _camera_frame);

    void Initialise(std::string _camera_frame);

    std::string VideoFileName();

//todo: reduce scope of variables to be
// private:

    std::shared_ptr<DataProducts> callback_lidar_camera(const sensor_msgs::PointCloud2ConstPtr &msg, const sensor_msgs::ImageConstPtr &img);

    bool draw_dots(const sensor_msgs::PointCloud2ConstPtr &msg,
                        const std::shared_ptr <tf2::BufferCore> &tf_buffer,
                        cv::Mat &img,
                        uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, int thickness);

//    bool draw_polygon(const std::vector<geometry_msgs::Vector3Stamped> &polygon, cv::Mat &img, uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha);
    bool draw_polygon(std::string frame_id,
                      const std::shared_ptr <tf2::BufferCore> &tf_buffer,
                      ros::Time &message_time,
                      const pcl::PointCloud<pcl::PointXYZIR>::Ptr &polygon, cv::Mat &img, uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, int thickness = -1);

    bool draw_polyline(std::string frame_id,
                      const std::shared_ptr <tf2::BufferCore> &tf_buffer,
                      ros::Time &message_time,
                      const pcl::PointCloud<pcl::PointXYZIR>::Ptr &polygon, cv::Mat &img, uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, int thickness = -1);


    bool draw_polygon_direct_to_screen(const pcl::PointCloud<pcl::PointXYZIR>::Ptr &polygon, cv::Mat &img, uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha);



    bool draw_pose_arrow(std::string frame_id,
                         const std::shared_ptr <tf2::BufferCore> &tf_buffer,
                          ros::Time &message_time,
                          geometry_msgs::Pose pose,
                          double arrow_size, // in meters
                          cv::Mat &img,
                          uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha);


    bool draw_pose_shape(std::string frame_id,
                         const std::shared_ptr <tf2::BufferCore> &tf_buffer,
                         ros::Time &message_time,
                         geometry_msgs::Pose pose,
                         double shape_size, // in meters
                         cv::Mat &img,
                         uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha);



    void overlayImage(cv::Mat* src, cv::Mat* overlay, const cv::Point& location)
    {
      for (int y = std::max(location.y, 0); y < src->rows; ++y)
      {
        int fY = y - location.y;

        if (fY >= overlay->rows)
          break;

        for (int x = std::max(location.x, 0); x < src->cols; ++x)
        {
          int fX = x - location.x;

          if (fX >= overlay->cols)
            break;

          double opacity = ((double)overlay->data[fY * overlay->step + fX * overlay->channels() + 3]) / 255;

          for (int c = 0; opacity > 0 && c < src->channels(); ++c)
          {
            unsigned char overlayPx = overlay->data[fY * overlay->step + fX * overlay->channels() + c];
            unsigned char srcPx = src->data[y * src->step + x * src->channels() + c];
            src->data[y * src->step + src->channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
          }
        }
      }
    }

    bool draw_transformed_polygon(const pcl::PointCloud<pcl::PointXYZIR>::Ptr &polygon, cv::Mat &img, uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, int thickness = -1);
    
    bool draw_transformed_polyline(const pcl::PointCloud<pcl::PointXYZIR>::Ptr &polygon, cv::Mat &img, uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, int thickness);

    void callback_camerainfo(const sensor_msgs::CameraInfo::ConstPtr &msg);

    bool lookupLidarCameraTF(const std::shared_ptr<tf2::BufferCore> &tf_buffer);

    bool CartesianToImage(geometry_msgs::Vector3 &vector, int &u, int &v, float &distance);

/*
    bool draw_dots_new(const sensor_msgs::PointCloud2ConstPtr &msg,
                      const std::shared_ptr <tf2::BufferCore> &tf_buffer,
                      cv::Mat &img,
                      uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, int thickness);


    ArrayBoolean NewCartesianToImage(MatrixXf &world_coordinates, MatrixXf &image_coordinates, MatrixXf &camera_matrix, MatrixXf &distortion_coefficients);
*/


    //bool rgb, uv, full, save_range, save_intensity, save_ring;
    bool full;

    std::string camera_frame, lidar_frame, cam_frame, polygon_frame;

    sensor_msgs::CameraInfoPtr camera_info_msg;

    cv::Mat cameramat;
    cv::Mat distcoeff;

/*
    MatrixXf camera_matrix; // 3x3 camera matrix
    MatrixXf distortion_coefficients;  // 1x4 distortion coefficients
*/


    int height, width;
    bool valid_camera_info, valid_transform;

    /* TO REMOVE */
    geometry_msgs::TransformStamped transform, polygon_transform, lidar_baselink_transform;

    ros::NodeHandle nh;
    ros::Publisher pub_labels, pub_rgb, pub_uv;

    std::string path;

    double MIN_X_VALUE; // = 1.35;
    double MIN_Y_VALUE; // = 1.35;
    double MIN_Z_VALUE; // = 0.3;

    cv::VideoWriter output_range_video, output_intensity_video, output_ring_video;

    std::shared_ptr<DataProducts> data_products;
  };

}

#endif
