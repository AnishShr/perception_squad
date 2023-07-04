#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/io/ply_io.h"
#include "pcl/common/transforms.h"
#include "pcl/registration/icp.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

using namespace std::chrono_literals;
std::string out_directory;
std::string bag_file;

class ExtrinsicCalibration : public rclcpp::Node
{
    public:

        pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pc_;   
        pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pc_downsampled_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr depthcam_pc_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr depthcam_pc_downsampled_;

        std::string lidar_target_frame = "lidar_link";
        std::string lidar_source_frame = "lidar_link_laser";

        std::string depthCam_target_frame = "camera_link";
        std::string depthCam_source_frame = "camera_depth_optical_frame";

        geometry_msgs::msg::TransformStamped lidar_tf;
        geometry_msgs::msg::TransformStamped depthCam_tf;

        

        ExtrinsicCalibration() : Node("evaluate_transform",
                                    rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
        {

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "points",
                rclcpp::QoS(rclcpp::KeepAll()).best_effort(),
                std::bind(&ExtrinsicCalibration::lidarCallback, this, std::placeholders::_1));

            depth_cam_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "camera/depth/color/points",
                10,
                std::bind(&ExtrinsicCalibration::depthCamCallback, this, std::placeholders::_1)
            );
            
            lidar_pc_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            lidar_pc_downsampled_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            depthcam_pc_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            depthcam_pc_downsampled_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            get_parameter("pointcloud_out_dir", out_directory);
            std::cout << "Pointcloud output directory set to: " << out_directory << std::endl;

            get_parameter("bag_file", bag_file);
            std::cout << "Bag file: " << bag_file << std::endl;



        }


    private:
        
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cam_subscription_;
        int count_depthCam_msgs = 0;

        void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
        {

            if (count_depthCam_msgs > 0)
                rclcpp::shutdown();

            RCLCPP_INFO(this->get_logger(),
                "Receiving lidar pointcloud");
            
            pcl::fromROSMsg(*lidar_msg, *lidar_pc_);

            try
            {
                lidar_tf = tf_buffer_->lookupTransform(
                    lidar_target_frame,
                    lidar_source_frame,
                    tf2::TimePointZero
                );

                depthCam_tf = tf_buffer_->lookupTransform(
                    depthCam_target_frame,
                    depthCam_source_frame,
                    tf2::TimePointZero
                );

            }

            catch(const tf2::TransformException & ex)
            {
                RCLCPP_INFO(
                    this->get_logger(), "Could not transform : %s",ex.what());
                    return;
            }

        }

        void depthCamCallback(const sensor_msgs::msg::PointCloud2::SharedPtr depthcam_msg)
        {

            RCLCPP_INFO(this->get_logger(),
                "Receiving depthcam pointcloud");

            pcl::fromROSMsg(*depthcam_msg, *depthcam_pc_); 
            count_depthCam_msgs++;
            
        }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExtrinsicCalibration>();

    rclcpp::spin(node);
    

    // Retrieving transfrom from lidar link laser to lidar link
    // std::cout << "Transform from "<< node->lidar_source_frame <<  " to " << node->lidar_target_frame << ": " << std::endl; 
    // std::cout << "Translation: " << "[" << node->lidar_tf.transform.translation.x 
    //             << " " << node->lidar_tf.transform.translation.y
    //             << " " << node->lidar_tf.transform.translation.z   
    //             << "]" <<std::endl;
    // std::cout << "Rotation: " << "[" 
    //             << node->lidar_tf.transform.rotation.x 
    //             << " " << node->lidar_tf.transform.rotation.y
    //             << " " << node->lidar_tf.transform.rotation.z
    //             << " " << node->lidar_tf.transform.rotation.w   
    //             << "]" <<std::endl;

    // Eigen::Quaternionf lidar_rotation_quat(node->lidar_tf.transform.rotation.w,
    //                                  node->lidar_tf.transform.rotation.x,
    //                                  node->lidar_tf.transform.rotation.y,
    //                                  node->lidar_tf.transform.rotation.z
    //                                  );
    // Eigen::Vector3f lidar_translation(node->lidar_tf.transform.translation.x,
    //                             node->lidar_tf.transform.translation.y,
    //                             node->lidar_tf.transform.translation.z
    //                             );                            
    // Eigen::Matrix3f lidar_rotation_matrix = lidar_rotation_quat.toRotationMatrix();


    // Eigen::Matrix4f lidar_transformation = Eigen::Matrix4f::Identity();
    // lidar_transformation.block<3,3>(0,0) = lidar_rotation_matrix;
    // lidar_transformation.block<3,1>(0,3) = lidar_translation;

    // std::cout << "Lidar transformation matrix: " << std::endl;
    // std::cout << lidar_transformation << std::endl; 


    // Retrieving transform from depth optical frame to camera link
    // std::cout << "Transform from "<< node->depthCam_source_frame <<  " to " << node->depthCam_target_frame << ": " << std::endl; 
    // std::cout << "Translation: " << "[" << node->depthCam_tf.transform.translation.x 
    //           << " " << node->depthCam_tf.transform.translation.y
    //           << " " << node->depthCam_tf.transform.translation.z   
    //           << "]" <<std::endl;
    // std::cout << "Rotation: " << "[" << node->depthCam_tf.transform.rotation.x 
    //           << " " << node->depthCam_tf.transform.rotation.y
    //           << " " << node->depthCam_tf.transform.rotation.z
    //           << " " << node->depthCam_tf.transform.rotation.w   
    //           << "]" <<std::endl;


    Eigen::Quaternionf depthCam_rotation_quat(node->depthCam_tf.transform.rotation.w,
                                     node->depthCam_tf.transform.rotation.x,
                                     node->depthCam_tf.transform.rotation.y,
                                     node->depthCam_tf.transform.rotation.z
                                     );
    Eigen::Vector3f depthCam_translation(node->depthCam_tf.transform.translation.x,
                                node->depthCam_tf.transform.translation.y,
                                node->depthCam_tf.transform.translation.z
                                );                            
    Eigen::Matrix3f depthCam_rotation_matrix = depthCam_rotation_quat.toRotationMatrix();


    Eigen::Matrix4f depthCam_transformation = Eigen::Matrix4f::Identity();
    depthCam_transformation.block<3,3>(0,0) = depthCam_rotation_matrix;
    depthCam_transformation.block<3,1>(0,3) = depthCam_translation;


    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*(node->depthcam_pc_), *out_cloud, depthCam_transformation);

    std::cout << "Computing ICP ..." << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(50);
    icp.setInputSource(out_cloud);
    icp.setInputTarget(node->lidar_pc_);

    pcl::PointCloud<pcl::PointXYZ> registered_pc;
    icp.align(registered_pc);

    Eigen::Matrix4f icp_transformation;
    
    float x, y, z;
    float roll, pitch, yaw;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_depthcam_pc(new pcl::PointCloud<pcl::PointXYZ>);

    

    if (icp.hasConverged())
    {
        icp_transformation = icp.getFinalTransformation();
        std::cout << "ICP Transformation: " << std::endl << icp_transformation << std::endl;
        std::cout << "ICP fitness score: " << icp.getFitnessScore() << std::endl;
        
        pcl::transformPointCloud(*out_cloud, *transformed_depthcam_pc, icp_transformation);

        // std::cout << "Rotation in RPY: " <<std::endl;
        Eigen::Matrix3f rotation_matrix = icp_transformation.block<3, 3>(0,0);
        Eigen::Vector3f translation_vector = icp_transformation.block<3, 1>(0, 3);

        std::cout << "Translation Vector: " << std::endl;
        std::cout << translation_vector << std::endl;

        x = translation_vector(0);
        y = translation_vector(1);
        z = translation_vector(2);

        Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(0, 1, 2);

        roll = euler_angles(0);
        pitch = euler_angles(1);
        yaw = euler_angles(2);

        // roll = std::atan2(-rotation_matrix(2, 1), rotation_matrix(2, 2));
        // pitch = std::asin(rotation_matrix(2, 0));
        // yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

        std::cout << "Roll, Pitch, Yaw: " << std::endl;
        std::cout << roll << " " << pitch << " " << yaw << std::endl;
    }

    else
    {
        std::cout << "ICP convergence failed . " << std::endl;
    }

    // Lidar downsampling
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_lidar;
    voxel_grid_lidar.setInputCloud(node->lidar_pc_);
    voxel_grid_lidar.setLeafSize(0.02f, 0.02f, 0.04f);
    voxel_grid_lidar.filter(*(node->lidar_pc_downsampled_));

    // Lidar Segmentation
    pcl::ModelCoefficients::Ptr lidar_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr lidar_inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> lidar_segment;
    lidar_segment.setOptimizeCoefficients(true);
    lidar_segment.setModelType(pcl::SACMODEL_PLANE);
    lidar_segment.setMethodType(pcl::SAC_RANSAC);
    lidar_segment.setMaxIterations(1000);
    lidar_segment.setDistanceThreshold(0.03);
    lidar_segment.setInputCloud(node->lidar_pc_downsampled_);
    lidar_segment.segment(*lidar_inliers, *lidar_coefficients);
    Eigen::Vector3f lidar_planeNormal(lidar_coefficients->values[0], lidar_coefficients->values[1], lidar_coefficients->values[2]);

    // For Ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_ground_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> lidar_extract_indices_ground;
    lidar_extract_indices_ground.setInputCloud(node->lidar_pc_downsampled_);
    lidar_extract_indices_ground.setIndices(lidar_inliers);
    lidar_extract_indices_ground.setNegative(false);
    lidar_extract_indices_ground.filter(*lidar_ground_points);

    pcl::io::savePLYFile(out_directory + "lidar_ground_points.ply", *lidar_ground_points);

    // For Non Ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_non_ground_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> lidar_extract_indices_non_ground;
    lidar_extract_indices_non_ground.setInputCloud(node->lidar_pc_downsampled_);
    lidar_extract_indices_non_ground.setIndices(lidar_inliers);
    lidar_extract_indices_non_ground.setNegative(true);
    lidar_extract_indices_non_ground.filter(*lidar_non_ground_points);

    pcl::io::savePLYFile(out_directory + "lidar_non_ground_points.ply", *lidar_non_ground_points);

    // Depth Camera Downsampling
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_depth_cam;
    voxel_grid_depth_cam.setInputCloud(transformed_depthcam_pc);
    voxel_grid_depth_cam.setLeafSize(0.03f, 0.03f, 0.03f);
    voxel_grid_depth_cam.filter(*(node->depthcam_pc_downsampled_));

    // Depth Camera Segmentation
    pcl::ModelCoefficients::Ptr depthcam_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr depthcam_inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> depthcam_segment;
    depthcam_segment.setOptimizeCoefficients(true);
    depthcam_segment.setModelType(pcl::SACMODEL_PLANE);
    depthcam_segment.setMethodType(pcl::SAC_RANSAC);
    depthcam_segment.setMaxIterations(1000);
    depthcam_segment.setDistanceThreshold(0.05);
    depthcam_segment.setInputCloud(node->depthcam_pc_downsampled_);
    depthcam_segment.segment(*depthcam_inliers, *depthcam_coefficients);
    Eigen::Vector3f depthcam_planeNormal(depthcam_coefficients->values[0], depthcam_coefficients->values[1], depthcam_coefficients->values[2]);

    // For Ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr depthcam_ground_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> depthcam_extract_indices_ground;
    depthcam_extract_indices_ground.setInputCloud(node->depthcam_pc_downsampled_);
    depthcam_extract_indices_ground.setIndices(depthcam_inliers);
    depthcam_extract_indices_ground.setNegative(false);
    depthcam_extract_indices_ground.filter(*depthcam_ground_points);

    pcl::io::savePLYFile(out_directory + "depthcam_ground_points.ply", *depthcam_ground_points);

    // For Non Ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr depthcam_non_ground_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> depthcam_extract_indices_non_ground;
    depthcam_extract_indices_non_ground.setInputCloud(node->depthcam_pc_downsampled_);
    depthcam_extract_indices_non_ground.setIndices(depthcam_inliers);
    depthcam_extract_indices_non_ground.setNegative(true);
    depthcam_extract_indices_non_ground.filter(*depthcam_non_ground_points);

    pcl::io::savePLYFile(out_directory + "depthcam_non_ground_points.ply", *depthcam_non_ground_points);
    
    std::ofstream file(out_directory + "output.txt");

    if (file.is_open())
    {
        file << "Assuming tha Lidar is at (0, 0, 0)" << std::endl;
        file << "Relative position of Camera (XYZ): " << x << ", " << y << ", " << z << std::endl;
        file << "Relative orientation of Camera (RPY): " << roll << ", " << pitch << ", " << yaw << std::endl;
        file << "Distance to floor plane (directly under lidar): " << lidar_coefficients->values[3] << std::endl;
        file <<  "Floor plane Orientation: X, Y, Z: " << lidar_planeNormal.x() << ", " << lidar_planeNormal.y() << ", " << lidar_planeNormal.z() << std::endl;

        file.close();
    }

    return 0;
}