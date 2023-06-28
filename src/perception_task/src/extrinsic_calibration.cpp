#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <fstream>
#include <filesystem>
#include <iostream>

std::string out_dir;
std::string out_dir_arg;
std::string bag_file;
std::string bag_file_arg;

class ExtrinsicCalibrationNode : public rclcpp::Node
{

    public:

        pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pc_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr depthcam_pc_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_downsampled;
        pcl::PointCloud<pcl::PointXYZ>::Ptr depthcam_cloud_downsampled;

        ExtrinsicCalibrationNode() : Node("extrinsic_calibration")
        {
            // Subscribe to the point clouds topic
            lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points",
            rclcpp::QoS(rclcpp::KeepAll()).best_effort(),
            std::bind(&ExtrinsicCalibrationNode::lidarCallback, this, std::placeholders::_1));

            depth_cam_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "camera/depth/color/points", 10, 
                std::bind(&ExtrinsicCalibrationNode::depthcamCallback, this, std::placeholders::_1));

            lidar_pc_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            lidar_cloud_downsampled = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            depthcam_pc_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            depthcam_cloud_downsampled = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            // declare_parameter("point_cloud_directory", "/home/anish/squad_ws/src/clouds/");
            declare_parameter("point_cloud_directory", out_dir_arg);
            get_parameter("point_cloud_directory", out_dir);

            // declare_parameter("bag_file", "/home/anish/squad_ws/src/perception_task/bags/test_data/rosbag2_2023_06_19-18_17_14/rosbag2_2023_06_19-18_17_14_0.db3");
            declare_parameter("bag_file", bag_file_arg);
            get_parameter("bag_file", bag_file);

            RCLCPP_INFO(get_logger(), "output_file_directory: %s", out_dir.c_str());
            RCLCPP_INFO(get_logger(), "bag_file: %s", bag_file.c_str());

        }



    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cam_subscription_;
        
        
        int count_depthcam_msgs = 0;

        void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
        {
            RCLCPP_INFO(this->get_logger(),
            "Receiving lidar pointcloud");

            pcl::fromROSMsg(*lidar_msg, *lidar_pc_);
            if (count_depthcam_msgs > 0)
                rclcpp::shutdown();

        }

        void depthcamCallback(const sensor_msgs::msg::PointCloud2::SharedPtr depthcam_msg)
        {
            RCLCPP_INFO(this->get_logger(),
            "Receiving depthcam pointcloud");

            pcl::fromROSMsg(*depthcam_msg, *depthcam_pc_);
            count_depthcam_msgs++;
        }



};

int main(int argc, char** argv)
{
    out_dir_arg = argv[1];
    bag_file_arg= argv[2];

  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExtrinsicCalibrationNode>();
  
    

  rclcpp::spin(node);


    
    // pcl::io::savePLYFile(pcl_dir + "lidar_cloud.ply", *(node->lidar_pc_));
    // pcl::io::savePLYFile("./clouds/lidar_cloud.ply", *(node->lidar_pc_));
    // pcl::io::savePLYFile(pcl_dir + "/depth_cam_cloud.ply", *(node->depthcam_pc_));

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_lidar;
    voxel_grid_lidar.setInputCloud(node->lidar_pc_);
    voxel_grid_lidar.setLeafSize(0.02f, 0.02f, 0.04f);
    voxel_grid_lidar.filter(*(node->lidar_cloud_downsampled));

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_depth_cam;
    voxel_grid_depth_cam.setInputCloud(node->depthcam_pc_);
    voxel_grid_depth_cam.setLeafSize(0.03f, 0.03f, 0.03f);
    voxel_grid_depth_cam.filter(*(node->depthcam_cloud_downsampled));

    // pcl::io::savePLYFile("./clouds/lidar_cloud_downsampled.ply", *(node->lidar_cloud_downsampled));
    // pcl::io::savePLYFile("./clouds/depth_cam_cloud_downsampled.ply", *(node->depthcam_cloud_downsampled));

    // Lidar Segmentation
    pcl::ModelCoefficients::Ptr lidar_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr lidar_inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> lidar_segment;
    lidar_segment.setOptimizeCoefficients(true);
    lidar_segment.setModelType(pcl::SACMODEL_PLANE);
    lidar_segment.setMethodType(pcl::SAC_RANSAC);
    lidar_segment.setMaxIterations(1000);
    lidar_segment.setDistanceThreshold(0.03);
    // seg.setInputCloud(pclCloud);
    lidar_segment.setInputCloud(node->lidar_cloud_downsampled);
    lidar_segment.segment(*lidar_inliers, *lidar_coefficients);
    Eigen::Vector3f lidar_planeNormal(lidar_coefficients->values[0], lidar_coefficients->values[1], lidar_coefficients->values[2]);

    std::cout << "Lidar transforms: " << std::endl;
    std::cout << "Orientation_x, Orientation_y, Orientation_z: " << lidar_planeNormal.x() 
        << " " << lidar_planeNormal.y() << " " << lidar_planeNormal.z() << std::endl; 
    std::cout << "d: " << lidar_coefficients->values[3] << std::endl; 

    // Find the minimum Z-coordinate (distance to floor plane)
    float minZ_lidar = std::numeric_limits<float>::max();
    for (const auto& point_lidar : node->lidar_pc_->points)
    {
      if (point_lidar.z < minZ_lidar)
        minZ_lidar = point_lidar.z;
    }

    std::cout << "minZ lidar: " << minZ_lidar << std::endl;


    // For Ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_ground_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> lidar_extract_indices_ground;
    // extract_indices_ground.setInputCloud(pclCloud);
    lidar_extract_indices_ground.setInputCloud(node->lidar_cloud_downsampled);
    lidar_extract_indices_ground.setIndices(lidar_inliers);
    lidar_extract_indices_ground.setNegative(false);
    lidar_extract_indices_ground.filter(*lidar_ground_points);

    pcl::io::savePLYFile(out_dir + "lidar_ground_points.ply", *lidar_ground_points);

    // For Non Ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_non_ground_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> lidar_extract_indices_non_ground;
    // extract_indices_ground.setInputCloud(pclCloud);
    lidar_extract_indices_non_ground.setInputCloud(node->lidar_cloud_downsampled);
    lidar_extract_indices_non_ground.setIndices(lidar_inliers);
    lidar_extract_indices_non_ground.setNegative(true);
    lidar_extract_indices_non_ground.filter(*lidar_non_ground_points);

  pcl::io::savePLYFile(out_dir + "lidar_non_ground_points.ply", *lidar_non_ground_points);

    // Depth Camera Segmentation
    pcl::ModelCoefficients::Ptr depthcam_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr depthcam_inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> depthcam_segment;
    depthcam_segment.setOptimizeCoefficients(true);
    depthcam_segment.setModelType(pcl::SACMODEL_PLANE);
    depthcam_segment.setMethodType(pcl::SAC_RANSAC);
    depthcam_segment.setMaxIterations(1000);
    depthcam_segment.setDistanceThreshold(0.05);
    // seg.setInputCloud(pclCloud);
    depthcam_segment.setInputCloud(node->depthcam_cloud_downsampled);
    depthcam_segment.segment(*depthcam_inliers, *depthcam_coefficients);
    Eigen::Vector3f depthcam_planeNormal(depthcam_coefficients->values[0], depthcam_coefficients->values[1], depthcam_coefficients->values[2]);

    std::cout << "Depth Camera transforms: " << std::endl;
    std::cout << "Orientation_x, Orientation_y, Orientation_z: " << depthcam_planeNormal.x() 
        << " " << depthcam_planeNormal.y() << " " << depthcam_planeNormal.z() << std::endl; 
    std::cout << "d: " << depthcam_coefficients->values[3] << std::endl; 

    // For Ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr depthcam_ground_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> depthcam_extract_indices_ground;
    // extract_indices_ground.setInputCloud(pclCloud);
    depthcam_extract_indices_ground.setInputCloud(node->depthcam_cloud_downsampled);
    depthcam_extract_indices_ground.setIndices(depthcam_inliers);
    depthcam_extract_indices_ground.setNegative(false);
    depthcam_extract_indices_ground.filter(*depthcam_ground_points);

    pcl::io::savePLYFile(out_dir + "depthcam_ground_points.ply", *depthcam_ground_points);

    // For Non Ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr depthcam_non_ground_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> depthcam_extract_indices_non_ground;
    // extract_indices_ground.setInputCloud(pclCloud);
    depthcam_extract_indices_non_ground.setInputCloud(node->depthcam_cloud_downsampled);
    depthcam_extract_indices_non_ground.setIndices(depthcam_inliers);
    depthcam_extract_indices_non_ground.setNegative(true);
    depthcam_extract_indices_non_ground.filter(*depthcam_non_ground_points);

  pcl::io::savePLYFile(out_dir + "depthcam_non_ground_points.ply", *depthcam_non_ground_points);

  float minZ_depthcam = std::numeric_limits<float>::max();
    for (const auto& point_depthcam : node->depthcam_pc_->points)
    {
      if (point_depthcam.z < minZ_depthcam)
        minZ_depthcam = point_depthcam.z;
    }

    std::cout << "minZ depth cam: " << minZ_depthcam << std::endl;

    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(500);
    icp.setInputSource(depthcam_ground_points);
    icp.setInputTarget(lidar_ground_points);
    // icp.setInputSource(node->depthcam_cloud_downsampled);
    // icp.setInputTarget(node->lidar_cloud_downsampled);
    pcl::PointCloud<pcl::PointXYZ> registered_pc;
    icp.align(registered_pc);

    Eigen::Matrix4f transformation;
    if (icp.hasConverged())
    {
        transformation = transformation = icp.getFinalTransformation();
        std::cout << "Transformation: " << std::endl << transformation << std::endl;
        std::cout << "ICP fitness score: " << icp.getFitnessScore() << std::endl;

        Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);
        Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
        
        float x = translation[0];
        float y = translation[1];
        float z = translation[2];

        float roll, pitch, yaw;
        roll = std::atan2(rotation(2, 1), rotation(2, 2));
        pitch = std::atan2(-rotation(2, 0), std::sqrt(rotation(2, 1) * rotation(2, 1) + rotation(2, 2) * rotation(2, 2)));
        yaw = std::atan2(rotation(1, 0), rotation(0, 0));

        std::ofstream file(out_dir + "output.txt");

        if (file.is_open())
        {
            file << "Relative position of camera: XYZ: " << x << ", " << y << ", " << z << std::endl;
            file << "Relative orientation of camera: RPY: " << roll << ", " << pitch << ", " << yaw << std::endl;
            file << "Distance to florr plane: " << -lidar_coefficients->values[3] << std::endl;
            file << "Floor Plane Orientation_x, Orientation_y, Orientation_z: " << lidar_planeNormal.x() 
                    << " " << lidar_planeNormal.y() << " " << lidar_planeNormal.z() << std::endl; 
            file.close();
        }
        else
        {
            std::cout << "Could not open file" << std::endl;
        }
        
        std::cout << "Source point cloud XYZ: " << x << ", " << y << ", " << z << std::endl;
        std::cout << "Source point cloud RPY: " << roll << ", " << pitch << ", " << yaw << std::endl;
        
    }
    else
    {
        std::cout << "ICP convergence failed . " << std::endl;
    }

    

  return 0;
}
