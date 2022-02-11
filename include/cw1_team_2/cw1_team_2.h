/* Software License Agreement (MIT License)
 *
 *  Copyright (c) 2019-, Dimitrios Kanoulas
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CW1_TEAM_2_H_
#define CW1_TEAM_2_H_

#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>

// ROS includes
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// standard c++ library includes (std::string, std::vector)
#include <string>
#include <vector>

// headers generated by catkin for the custom services we have made
#include <cw1_team_2/set_arm.h>
#include <cw1_team_2/set_gripper.h>
#include <cw1_team_2/add_collision.h>
#include <cw1_team_2/remove_collision.h>
#include <cw1_team_2/pick.h>
#include <cw1_world_spawner/Task1Service.h>
#include <cw1_world_spawner/Task2Service.h>
#include <cw1_world_spawner/Task3Service.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

/** \brief CW1 Solution.
  *
  * \author Ahmed Adamjee, Abdulbaasit Sanusi, Kennedy Dike
  */
class Cw1Solution
{
  public:

    /** \brief  Class constructor. 
      *
      * \input[in] nh ROS node handle
      */
    Cw1Solution(ros::NodeHandle& nh);


    /** \brief Service callback function for moving the arm. 
      *
      * \input[in] request service request message 
      * \input[in] response service response message
      *  
      * \return true if service succeeds
      */
    bool 
    setArmCallback(cw1_team_2::set_arm::Request &request,
      cw1_team_2::set_arm::Response &response);
    
    /** \brief Service callback function for setting the gripper fingers to a specific width. 
      *
      * \input[in] request service request message 
      * \input[in] response service response message
      *  
      * \return true if service succeeds
      */
    bool 
    setGripperCallback(cw1_team_2::set_gripper::Request &request,
      cw1_team_2::set_gripper::Response &response);
    
    /** \brief Service callback function for adding collision objects. 
      *
      * \input[in] request service request message 
      * \input[in] response service response message
      *  
      * \return true if service succeeds
      */
    bool 
    addCollisionCallback(cw1_team_2::add_collision::Request &request,
      cw1_team_2::add_collision::Response &response);

    /** \brief Service callback function for removing collision objects. 
      *
      * \input[in] request service request message 
      * \input[in] response service response message
      *  
      * \return true if service succeeds
      */
    bool 
    removeCollisionCallback(cw1_team_2::remove_collision::Request &request,
      cw1_team_2::remove_collision::Response &response);

    /** \brief Service callback function for picking up an object. 
      *
      * \input[in] request service request message 
      * \input[in] response service response message
      *  
      * \return true if service succeeds
      */
    bool 
    pickCallback(cw1_team_2::pick::Request &request,
      cw1_team_2::pick::Response &response);

    /** \brief Service callback function for ............. 
      *
      * \input[in] request service request ...............
      *  
      * \return true if .....................
      */
    bool 
    task1Callback(cw1_world_spawner::Task1Service::Request &request,
      cw1_world_spawner::Task1Service::Response &response);

    /** \brief Service callback function for ............. 
      *
      * \input[in] request service request ...............
      *  
      * \return true if .....................
      */
    bool 
    task2Callback(cw1_world_spawner::Task2Service::Request &request,
      cw1_world_spawner::Task2Service::Response &response);


    /** \brief Service callback function for ............. 
      *
      * \input[in] request service request ...............
      *  
      * \return true if .....................
      */
    bool 
    task3Callback(cw1_world_spawner::Task3Service::Request &request,
      cw1_world_spawner::Task3Service::Response &response);


    /** \brief MoveIt function for moving the move_group to the target position.
      *
      * \input[in] target_pose pose to move the arm to
      *
      * \return true if moved to target position 
      */
    bool 
    moveArm(geometry_msgs::Pose target_pose);

    /** \brief MoveIt function for moving the gripper fingers to a new position. 
      *
      * \input[in] width desired gripper finger width
      *
      * \return true if gripper fingers are moved to the new position
      */
    bool 
    moveGripper(float width);

    /** \brief MoveIt function for adding a cuboid collision object in RViz
      * and the MoveIt planning scene.
      *
      * \input[in] object_name name for the new object to be added
      * \input[in] centre point at which to add the new object
      * \input[in] dimensions dimensions of the cuboid to add in x,y,z
      * \input[in] orientation rotation to apply to the cuboid before adding
      */
    void
    addCollisionObject(std::string object_name, geometry_msgs::Point centre, 
      geometry_msgs::Vector3 dimensions, geometry_msgs::Quaternion orientation);

    /** \brief Remove a collision object from the planning scene.
      * 
      * \input[in] object_name for the object to be removed
      */
    void 
    removeCollisionObject(std::string object_name);

    /** \brief Pick an object up with a given position.
      * 
      * \input[in] position the xyz coordinates where the gripper converges
      */
    bool
    pick(geometry_msgs::Point position);

    /** \brief Place an object up with a given position.
      * 
      * \input[in] position the xyz coordinates where the gripper converges
      */
    bool
    place(geometry_msgs::Point position);

    // FROM PCL TUTORIAL -->

    /** \brief Point Cloud CallBack function.
      * 
      * \input[in] cloud_input a PointCloud2 sensor_msgs const pointer
      */
    void
    cloudCallBackOne (const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);
    
    /** \brief Apply Voxel Grid filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
    void
    applyVX (PointCPtr &in_cloud_ptr,
             PointCPtr &out_cloud_ptr);

    /** \brief Apply Pass Through filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
    void
    applyPT (PointCPtr &in_cloud_ptr,
             PointCPtr &out_cloud_ptr);

    /** \brief Apply Color filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
    void
    applyCF (PointCPtr &in_cloud_ptr,
             PointCPtr &out_cloud_ptr);
    
    /** \brief Normal estimation.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    findNormals (PointCPtr &in_cloud_ptr);
    
    /** \brief Segment Plane from point cloud.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    segPlane (PointCPtr &in_cloud_ptr);

    /** \brief Segment clusters from point cloud.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    segClusters (PointCPtr &in_cloud_ptr);
    
    /** \brief Segment Cylinder from point cloud.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    segCylind (PointCPtr &in_cloud_ptr);
    
    
    /** \brief Find the Pose of Cylinder.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    findCylPose (PointCPtr &in_cloud_ptr);
    
    /** \brief Point Cloud publisher.
      * 
      *  \input pc_pub ROS publisher
      *  \input pc point cloud to be published
      */
    void
    pubFilteredPCMsg (ros::Publisher &pc_pub, PointC &pc);
    
    /** \brief Publish the cylinder point.
      * 
      *  \input[in] cyl_pt_msg Cylinder's geometry point
      *  
      *  \output true if three numbers are added
      */
    void
    publishPose (geometry_msgs::PointStamped &cyl_pt_msg);
    


      
    /* Variables */

    /** \brief Define object and goal locations of objects to manipulate */
    //geometry_msgs/PoseStamped object_loc;

    /** \brief Define some useful constant values */
    std::string base_frame_ = "panda_link0";
    double gripper_open_ = 80e-3;
    double gripper_closed_ = 0.0;

    /** \brief Parameters to define the pick operation */
    double z_offset_ = 0.125;
    double angle_offset_ = 3.14159 / 4.0;
    double approach_distance_ = 0.1;
    
    /** \brief Node handle. */
    ros::NodeHandle g_nh;

    /** \brief  service servers for advertising ROS services  */
    ros::ServiceServer set_arm_srv_;
    ros::ServiceServer set_gripper_srv_;
    ros::ServiceServer add_collision_srv_;
    ros::ServiceServer remove_collision_srv_;
    ros::ServiceServer pick_srv_;
    ros::ServiceServer task1_srv_;
    ros::ServiceServer task2_srv_;
    ros::ServiceServer task3_srv_;


    /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
      * these are defined in urdf. */
    moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
    moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

    /** \brief MoveIt interface to interact with the moveit planning scene 
      * (eg collision objects). */
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;


    /** \brief The input point cloud frame id. */
    std::string g_input_pc_frame_id_;

    /** \brief ROS publishers. */
    ros::Publisher g_pub_cloud;

    /** \brief ROS subscribers. */
    ros::Subscriber g_sub_cloud;
    
    /** \brief ROS geometry message point. */
    geometry_msgs::PointStamped g_cyl_pt_msg;


    /** \brief Current centroid found .................... */
    geometry_msgs::PointStamped g_current_centroid;
    
    /** \brief ROS pose publishers. */
    ros::Publisher g_pub_pose;
    
    /** \brief Voxel Grid filter's leaf size. */
    double g_vg_leaf_sz;
    
    /** \brief Point Cloud (input) pointer. */
    PointCPtr g_cloud_ptr;
    
    /** \brief Point Cloud (filtered) pointer. */
    PointCPtr g_cloud_filtered, g_cloud_filtered2;
    
    /** \brief Point Cloud (filtered) sensros_msg for publ. */
    sensor_msgs::PointCloud2 g_cloud_filtered_msg;
    
    /** \brief Point Cloud (input). */
    pcl::PCLPointCloud2 g_pcl_pc;
    
    /** \brief Voxel Grid filter. */
    pcl::VoxelGrid<PointT> g_vx;
    
    /** \brief Pass Through filter. */
    pcl::PassThrough<PointT> g_pt;
    
    /** \brief Pass Through min and max threshold sizes. */
    double g_pt_thrs_min, g_pt_thrs_max;
    
    /** \brief Color filter. */
    pcl::ConditionalRemoval<PointT> g_cf;

    /** \brief Color filter rgb filter values. */
    double g_cf_red, g_cf_green, g_cf_blue;
    
    /** \brief KDTree for nearest neighborhood search. */
    pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
    
    /** \brief Normal estimation. */
    pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
    
    /** \brief Cloud of normals. */
    pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals, g_cloud_normals2;
    
    /** \brief Nearest neighborhooh size for normal estimation. */
    double g_k_nn;
    
    /** \brief SAC segmentation. */
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg; 
    
    /** \brief Extract point cloud indices. */
    pcl::ExtractIndices<PointT> g_extract_pc;
  
    /** \brief Extract point cloud normal indices. */
    pcl::ExtractIndices<pcl::Normal> g_extract_normals;
    
    /** \brief Point indices for plane. */
    pcl::PointIndices::Ptr g_inliers_plane;
      
    /** \brief Point indices for cylinder. */
    pcl::PointIndices::Ptr g_inliers_cylinder;
    
    /** \brief Model coefficients for the plane segmentation. */
    pcl::ModelCoefficients::Ptr g_coeff_plane;
    
    /** \brief Model coefficients for the culinder segmentation. */
    pcl::ModelCoefficients::Ptr g_coeff_cylinder;
    
    /** \brief Point cloud to hold plane and cylinder points. */
    PointCPtr g_cloud_plane, g_cloud_cylinder;
    
    /** \brief cw1Q1: TF listener definition. */
    tf::TransformListener g_listener_;


  protected:
    /** \brief Debug mode. */
    bool debug_;
};
#endif