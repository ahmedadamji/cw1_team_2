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

#include <cw1_team_2/cw1_team_2.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

////////////////////////////////////////////////////////////////////////////////
Cw1Solution::Cw1Solution (ros::NodeHandle &nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_plane (new PointC), // plane point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  debug_ (false)
{
  g_nh = nh;

  // Define the publishers
  g_pub_cloud = g_nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  g_pub_pose = g_nh.advertise<geometry_msgs::PointStamped> ("cube_pt", 1, true);
  
  // Define public variables
  g_vg_leaf_sz = 0.01; // VoxelGrid leaf size: Better in a config file
  g_x_thrs_min = -0.7; // PassThrough min thres: Better in a config file
  g_x_thrs_max = -0.5; // PassThrough max thres: Better in a config file
  g_y_thrs_min = 0.0; // PassThrough min thres: Better in a config file
  g_y_thrs_max = 0.4; // PassThrough max thres: Better in a config file
  g_cf_red = 25.5; // Colour Filter Red Value: Better in a config file
  g_cf_blue = 204; // Colour Filter Blue Value: Better in a config file
  g_cf_green = 25.5; // Colour Filter Green Value: Better in a config file
  g_k_nn = 50; // Normals nn size: Better in a config file

  // namespace for our ROS services, they will appear as "/namespace/srv_name"
  std::string service_ns = "/cw1_team_2";

  // advertise the services available from this node
  task1_srv_ = g_nh.advertiseService("/task1_start",
    &Cw1Solution::task1Callback, this);
  task2_srv_ = g_nh.advertiseService("/task2_start",
    &Cw1Solution::task2Callback, this);
  task3_srv_ = g_nh.advertiseService("/task3_start",
    &Cw1Solution::task3Callback, this);

  ROS_INFO("MoveIt! services initialisation finished, namespace: %s", 
    service_ns.c_str());


  // Create a ROS subscriber for the input point cloud
  g_sub_cloud = g_nh.subscribe("/r200/camera/depth_registered/points",1,
    &Cw1Solution::cloudCallBackOne,this);
}


///////////////////////////////////////////////////////////////////////////////

bool
Cw1Solution::task1Callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response)
{
  /* This service picks an object with a given pose and places it at a given pose */

// this is used in defining the origin of the floor collision object
  geometry_msgs::Point floor_origin;
  floor_origin.x = 0.0;
  floor_origin.y = 0.0;
  floor_origin.z = 0.0;

// this is used in defining the dimension of the floor collision object
  geometry_msgs::Vector3 floor_dimension;
  floor_dimension.x = 3;
  floor_dimension.y = 3;
  floor_dimension.z = 0.001;

// this is used in defining the orientation of the floor collision object
  geometry_msgs::Quaternion floor_orientation;
  
  floor_orientation.x = 0.0;
  floor_orientation.y = 0.0;
  floor_orientation.z = 0.0;
  floor_orientation.w = 1.0;

  // function call to add a floor collision object with the arguments defined above/
  addCollisionObject("cube_1",floor_origin,floor_dimension,floor_orientation);

  // this is used in defining the origin of box collision object
  geometry_msgs::Point box_origin;
  box_origin.x = request.goal_loc.point.x;
  box_origin.y = request.goal_loc.point.y;
  box_origin.z = 0;

  // this is used in defining the dimension of the box collision object
  geometry_msgs::Vector3 box_dimension;
  box_dimension.x = 0.22;
  box_dimension.y = 0.22;
  box_dimension.z = 0.45;

  // this is used in defining the orientation of the box collision object
  geometry_msgs::Quaternion box_orientation;
  
  box_orientation.x = 0.0;
  box_orientation.y = 0.0;
  box_orientation.z = 0.0;
  box_orientation.w = 1.0;

  // function call to add a box collision object with the arguments defined above
  addCollisionObject("cube_2",box_origin,box_dimension,box_orientation);

  // function call to pick an object at the desired coordinate
  bool pick_success = pick(request.object_loc.pose.position);

  if (not pick_success) 
  {
    ROS_ERROR("Object Pick up  failed");

    return false;
  }

  // This is used to set the desired location to place the object picked
  geometry_msgs::Point target_point;
  target_point.x = request.goal_loc.point.x;
  target_point.y = request.goal_loc.point.y;
  target_point.z = request.goal_loc.point.z+0.1;

  // Function call to place the object picked inside the basket
  bool move_success = place(target_point);

  if (not move_success)
  {
    ROS_ERROR("Placing the object failed");
    
    return false;
  }
  
  
  return true;
}


///////////////////////////////////////////////////////////////////////////////

bool
Cw1Solution::task2Callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  
  /* This service is used to detect object of particular colour
    * among different colours
  
  */
  
  // clearing the list that store centroids of any previous centroid values
  centroids.clear();

  int size = 0;


  g_cf_red = request.r.data*255;
  g_cf_green = request.g.data*255;
  g_cf_blue = request.b.data*255;

  std::cout << "red: " << g_cf_red << std::endl;
  std::cout << "green: " << g_cf_green << std::endl;
  std::cout << "blue: " << g_cf_blue << std::endl;



  //initializing variable to scan an area of the robot arm environment
  float x_scan = 0.50;
  float x_thrs_min = 0.20;
  float y_scan = 0.35;
  float y_thrs_min = 0.15;


  //initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan1;

  for (int i = 0; i < 3; i++)
  {
    //function call setting the scan area to specific coordinate
    scan1 = scan(scan1, x_scan, y_scan, 0.6);

    //setting a threshold to store a centroid within a particular scan area
    g_x_thrs_min = x_thrs_min;
    g_y_thrs_min = y_thrs_min;
    g_x_thrs_max = g_x_thrs_min + 0.6;
    g_y_thrs_max = g_y_thrs_min + 0.3;
    
    //function call to move arm towards scan coordinates
    bool scan1_success = moveArm(scan1);
    
    //storing the centroids founds in scan area to the initialized centroids variable
    centroids = findCentroidsAtScanLocation(centroids);
    
    //updating the scan area for the next iteration
    y_scan -= 0.30;
    y_thrs_min -= 0.35;
  }

  response.centroids = centroids;


  return true;
}


///////////////////////////////////////////////////////////////////////////////

bool
Cw1Solution::task3Callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{

  /* This service picks and places cubes of blue colour into a target location(basket)*/
  
  // clearing the list that store centroids of any previous centroid values
  centroids.clear();

  g_cf_red = request.r.data*255;
  g_cf_green = request.g.data*255;
  g_cf_blue = request.b.data*255;

  std::cout << "red: " << g_cf_red << std::endl;
  std::cout << "green: " << g_cf_green << std::endl;
  std::cout << "blue: " << g_cf_blue << std::endl;


  // this is used in defining the origin of the floor collision object
  geometry_msgs::Point floor_origin;
  floor_origin.x = 0.0;
  floor_origin.y = 0.0;
  floor_origin.z = 0.0;

  // this is used in defining the dimension of the floor collision object
  geometry_msgs::Vector3 floor_dimension;
  floor_dimension.x = 3;
  floor_dimension.y = 3;
  floor_dimension.z = 0.001;

  // this is used in defining the orientation of the floor collision object
  geometry_msgs::Quaternion floor_orientation;
  
  floor_orientation.x = 0.0;
  floor_orientation.y = 0.0;
  floor_orientation.z = 0.0;
  floor_orientation.w = 1.0;

  // function call to add a floor collision object with the arguments defined above  
  addCollisionObject("cube_1",floor_origin,floor_dimension,floor_orientation);

  // this is used in defining the origin of box collision object
  geometry_msgs::Point box_origin;
  box_origin.x = request.goal_loc.point.x;
  box_origin.y = request.goal_loc.point.y;
  box_origin.z = 0;

  // this is used in defining the dimension of the box collision object
  geometry_msgs::Vector3 box_dimension;
  box_dimension.x = 0.22;
  box_dimension.y = 0.22;
  box_dimension.z = 0.45;

  // this is used in defining the orientation of the box collision object
  geometry_msgs::Quaternion box_orientation;
  
  box_orientation.x = 0.0;
  box_orientation.y = 0.0;
  box_orientation.z = 0.0;
  box_orientation.w = 1.0;

  // function call to add a box collision object with the arguments defined above  
  addCollisionObject("cube_2",box_origin,box_dimension,box_orientation);


  int size = 0;
  
  //initializing variable to scan an area of the robot arm environment
  float x_scan = 0.5;
  float x_thrs_min = 0.20;
  float y_scan = 0.3;
  float y_thrs_min = 0.15;

  // Scanning for the blue boxes at the first 3 scan locations:

  //initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan1;

  for (int i = 0; i < 3; i++)
  {
    //function call setting the scan area to specific coordinate
    scan1 = scan(scan1, x_scan, y_scan, 0.6);

    //setting a threshold to store a centroid within a particular scan area
    g_x_thrs_min = x_thrs_min;
    g_y_thrs_min = y_thrs_min;
    g_x_thrs_max = g_x_thrs_min + 0.6;
    g_y_thrs_max = g_y_thrs_min + 0.3;
    
    //function call to move arm towards scan coordinates
    bool scan1_success = moveArm(scan1);
    
    //storing the centroids founds in scan area to the initialized centroids variable
    centroids = findCentroidsAtScanLocation(centroids);
    
    //updating the scan area for the next iteration
    y_scan -= 0.3;
    y_thrs_min -= 0.3;
  }
  
  // Scanning for the blue boxes at the 4th scan location:

  //initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan4;

  //function call setting the scan area to specific coordinate
  scan4 = scan(scan4, 0.233, -0.3, 0.6);


  //setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = 0.1;
  g_y_thrs_min = -0.45;
  g_x_thrs_max = g_x_thrs_min + 0.1;
  g_y_thrs_max = g_y_thrs_min + 0.3;

  //function call to move arm towards scan coordinates  
  bool scan4_success = moveArm(scan4);
  
  //storing the centroids founds in scan area to the initialized centroids variable
  centroids = findCentroidsAtScanLocation(centroids);

  
  // Scanning for the blue boxes at the 5th scan location:

  //initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan5;

  //function call setting the scan area to specific coordinate
  scan5 = scan(scan5, -0.033, -0.3, 0.6); 

  //setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = -0.1;
  g_y_thrs_min = -0.45;
  g_x_thrs_max = g_x_thrs_min + 0.2;
  g_y_thrs_max = g_y_thrs_min + 0.3;


  //function call to move arm towards scan coordinates  
  bool scan5_success = moveArm(scan5);


  //storing the centroids founds in scan area to the initialized centroids variable
  centroids = findCentroidsAtScanLocation(centroids);
  
  
  // Scanning for the blue boxes at the 6th scan location:

  //initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan6;

  //function call setting the scan area to specific coordinate
  scan6 = scan(scan6, -0.3, -0.3, 0.6); 

  //setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = -0.8;
  g_y_thrs_min = -0.45;
  g_x_thrs_max = g_x_thrs_min + 0.70;
  g_y_thrs_max = g_y_thrs_min + 0.30;


  //function call to move arm towards scan coordinates 
  bool scan6_success = moveArm(scan6);

  //storing the centroids founds in scan area to the initialized centroids variable
  centroids = findCentroidsAtScanLocation(centroids);


  
  // Scanning for the blue boxes at the 7th scan location:

  //initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan7;

  //function call setting the scan area to specific coordinate
  scan7 = scan(scan7, -0.3, 0.0, 0.6);

  
  //setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = -0.8;
  g_y_thrs_min = -0.15;
  g_x_thrs_max = g_x_thrs_min + 0.70;
  g_y_thrs_max = g_y_thrs_min + 0.30;


  //function call to move arm towards scan coordinates
  bool scan7_success = moveArm(scan7);
  

  //storing the centroids founds in scan area to the initialized centroids variable
  centroids = findCentroidsAtScanLocation(centroids);

  
  // Scanning for the blue boxes at the 8th scan location:

  //initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan8;

  //function call setting the scan area to specific coordinate
  scan8 = scan(scan8, -0.3, 0.3, 0.6); 

  //setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = -0.8;
  g_y_thrs_min = 0.15;
  g_x_thrs_max = g_x_thrs_min + 0.70;
  g_y_thrs_max = g_y_thrs_min + 0.30;

  //function call to move arm towards scan coordinates
  bool scan8_success = moveArm(scan8);
  
  //storing the centroids founds in scan area to the initialized centroids variable
  centroids = findCentroidsAtScanLocation(centroids);
  

  // Scanning for the blue boxes at the 9th scan location:

  //initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan9;

  //function call setting the scan area to specific coordinate
  scan9 = scan(scan9, -0.033, 0.3, 0.6); 

  //setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = -0.1;
  g_y_thrs_min = 0.15;
  g_x_thrs_max = g_x_thrs_min + 0.20;
  g_y_thrs_max = g_y_thrs_min + 0.30;
    

  //function call to move arm towards scan coordinates
  bool scan9_success = moveArm(scan9);


  //storing the centroids founds in scan area to the initialized centroids variable
  centroids = findCentroidsAtScanLocation(centroids);


  // Scanning for the blue boxes at the 10th scan location:

  //initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan10;

  //function call setting the scan area to specific coordinate
  scan10 = scan(scan10, 0.233, 0.3, 0.6); 

  //setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = 0.1;
  g_y_thrs_min = 0.15;
  g_x_thrs_max = g_x_thrs_min + 0.10;
  g_y_thrs_max = g_y_thrs_min + 0.30;


  //function call to move arm towards scan coordinates 
  bool scan10_success = moveArm(scan10);


  //storing the centroids founds in scan area to the initialized centroids variable
  centroids = findCentroidsAtScanLocation(centroids);

  //store the number of centroids in centroid list
  size = centroids.size();

  //setting a condition to go through all the centroids as long as there are centroids in the list
  if (size > 0)
  {
      //looping through all the centroids
      for (int i = 0; i < size; i++)
      {
        std::cout << "We are now trying to pick cube:  " + std::to_string(i)  << std::endl;;
        
        //initializing a variable to store the coordinates of each centroid found
        geometry_msgs::Point position;
        position.x = (round(centroids[i].point.x * pow(10.0f, (2.0))) / pow(10.0f, (2.0)));
        position.y = (round(centroids[i].point.y * pow(10.0f, (2.0))) / pow(10.0f, (2.0)));
        position.z = 0.02; 

        // function call to pick an object at the desired coordinate
        bool pick_success = pick(position);

        if (not pick_success) 
        {
          ROS_ERROR("Object Pick up  failed");

          return false;
        }

        //initializing a variable to store the coordinates of the goal location(where the cube is to be placed)
        geometry_msgs::Point target_point;
        target_point.x = request.goal_loc.point.x;
        target_point.y = request.goal_loc.point.y;
        target_point.z = request.goal_loc.point.z+0.1;

        // Function call to place the object picked inside the basket
        bool move_success = place(target_point);

        if (not move_success)
        {
          ROS_ERROR("Placing the object failed");
          
          return false;
        }


      }
  }

  
  return true;
}
///////////////////////////////////////////////////////////////////////////////

std::vector<geometry_msgs::PointStamped>
Cw1Solution::findCentroidsAtScanLocation(std::vector<geometry_msgs::PointStamped> centroids)
{
  /*this function is used to find centroids of object particular colour within
    * a particular scan area
    * If centroid is found, it is appended to the centroids list created
    * and this is then returned
   */
  
  geometry_msgs::PointStamped centroid;

  g_sub_cloud;

  int size = g_centroids.size();

  if (size > 0)
  {
    for (int i = 0; i < size; i++)
    {
      centroid = g_centroids[i];
      double x = centroid.point.x;
      double y = centroid.point.y;

      //check if the centroid found is within a particular scan area to avoid duplication
      if (((g_x_thrs_min <= x ) && (x < g_x_thrs_max)) && ((g_y_thrs_min <= y ) && (y < g_y_thrs_max)))
      {
        ROS_INFO("A centroid was found at this location");

        //append centroid found to the centroids list
        centroids.push_back(centroid);
      }
    }

  }
  return centroids;

}

///////////////////////////////////////////////////////////////////////////////
geometry_msgs::Pose
Cw1Solution::scan(geometry_msgs::Pose scan_num, float x, float y, float z)
{
  /* This function is used to set coordinate for the desired scan area */
  
  //setting the values for the scan orientation
  scan_num.orientation.x = -1.0;
  scan_num.orientation.y = 0.0;
  scan_num.orientation.z = 0.0;
  scan_num.orientation.w = 0.0;

  //setting the values for the scan position
  scan_num.position.x = x;
  scan_num.position.y = y;
  scan_num.position.z = z;
  
  return scan_num;
}

///////////////////////////////////////////////////////////////////////////////

bool
Cw1Solution::moveArm(geometry_msgs::Pose target_pose)
{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
Cw1Solution::moveGripper(float width)
{
  /* this function moves the gripper fingers to a new position. Joints are:
      - panda_finger_joint1
      - panda_finger_joint2
  */

  // safety checks
  if (width > gripper_open_) width = gripper_open_;
  if (width < gripper_closed_) width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  hand_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

void
Cw1Solution::addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */

  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  // hint: what about collision_object.REMOVE?
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

///////////////////////////////////////////////////////////////////////////////

void
Cw1Solution::removeCollisionObject(std::string object_name)
{
  /* remove a collision object from the planning scene */

  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input the name and specify we want it removed
  collision_object.id = object_name;
  collision_object.operation = collision_object.REMOVE;

  // apply this collision object removal to the scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

///////////////////////////////////////////////////////////////////////////////

bool
Cw1Solution::pick(geometry_msgs::Point position)
{
  /* This function picks up an object using a pose. The given point is where the
  centre of the gripper fingers will converge */

  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose.position = position;
  grasp_pose.orientation = grasp_orientation;
  grasp_pose.position.z += z_offset_;

  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose;
  approach_pose = grasp_pose;
  approach_pose.position.z += approach_distance_;

  /* Now perform the pick */

  bool success = true;

  ROS_INFO("Begining pick operation");

  // move the arm above the object
  success *= moveArm(approach_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to pick approach pose failed");
    return false;
  }

  // open the gripper
  success *= moveGripper(gripper_open_);

  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to pick failed");
    return false;
  }

  // approach to grasping pose
  success *= moveArm(grasp_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
    return false;
  }

  // grasp!
  success *= moveGripper(gripper_closed_);

  if (not success) 
  {
    ROS_ERROR("Closing gripper to grasp failed");
    return false;
  }

  // retreat with object
  success *= moveArm(approach_pose);

  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed");
    return false;
  }

  ROS_INFO("Pick operation successful");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
Cw1Solution::place(geometry_msgs::Point position)
{
  /* This function places an object using a pose. The given point is where the
  centre of the gripper fingers will converge */

  // define placing as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the placing orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion place_orientation = tf2::toMsg(q_result);

  // set the desired placing pose
  geometry_msgs::Pose place_pose;
  place_pose.position = position;
  place_pose.orientation = place_orientation;
  place_pose.position.z += z_offset_;

  // set the desired pre-placing pose
  geometry_msgs::Pose approach_pose;
  approach_pose = place_pose;
  approach_pose.position.z += approach_distance_;

  /* Now perform the place */

  bool success = true;

  ROS_INFO("Begining place operation");

  // move the arm above the place location
  success *= moveArm(approach_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to place approach pose failed");
    return false;
  }

  // approach to placing pose
  success *= moveArm(place_pose);

  if (not success) 
  {
    ROS_ERROR("Moving arm to placing pose failed");
    return false;
  }

  // open the gripper
  success *= moveGripper(gripper_open_);
  if (not success) 
  {
    ROS_ERROR("Could not open Gripper");
    return false;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////

void
Cw1Solution::cloudCallBackOne
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);


  // Perform the filtering
  applyVX (g_cloud_ptr, g_cloud_filtered);
  applyCF (g_cloud_ptr, g_cloud_filtered);
  
  // Segment plane and cube
  findNormals (g_cloud_filtered);
  segPlane (g_cloud_filtered);
  segClusters (g_cloud_filtered);

  g_centroids.clear();

  for (std::vector<pcl::PointIndices>::const_iterator it = g_cluster_indices.begin (); it != g_cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (const auto& idx : it->indices)
    cloud_cluster->push_back ((*g_cloud_filtered)[idx]); //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    ROS_INFO("Number of data points in the curent PointCloud cluster: ", cloud_cluster->size () );

    // finding centroid pose of current cube cluster found
    g_current_centroid = findCubePose (cloud_cluster);

    g_centroids.push_back(g_current_centroid);

  }
  // Finding centroid pose of the entire filtered cloud to publish
  findCubePose (g_cloud_filtered);
    
  // Publish the data
  ROS_INFO ("Publishing Filtered Cloud");
  pubFilteredPCMsg (g_pub_cloud, *g_cloud_filtered);
  //pubFilteredPCMsg (g_pub_cloud, *g_cloud_cylinder);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw1Solution::applyVX (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  /*this is used to downsample a point cloud using a voxel grid filter*/
  g_vx.setInputCloud (in_cloud_ptr);
  g_vx.setLeafSize (g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter (*out_cloud_ptr);
  
  return;
}


////////////////////////////////////////////////////////////////////////////////
void
Cw1Solution::applyCF (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{


  /* This function is used to apply a colour filter to a point cloud*/


  //determines if a point meets this condition
  pcl::ConditionAnd<PointT>::Ptr condition (new pcl::ConditionAnd<PointT> ());


   // set the Lower bound for red conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr lb_red(new pcl::PackedRGBComparison<PointT>("r", pcl::ComparisonOps::GT, g_cf_red-25.5));
  condition->addComparison (lb_red);

  // set the Lower bound for green conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr lb_green(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::GT, g_cf_green-25.5));
  condition->addComparison (lb_green);

  // set the Lower bound for blue conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr lb_blue(new pcl::PackedRGBComparison<PointT>("b", pcl::ComparisonOps::GT, g_cf_blue-25.5));
  condition->addComparison (lb_blue);

 
 // set the upper bound for red conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr ub_red(new pcl::PackedRGBComparison<PointT>("r", pcl::ComparisonOps::LT, g_cf_red+25.5));
  condition->addComparison (ub_red);

  // set the upper bound for green conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr ub_green(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::LT, g_cf_green+25.5));
  condition->addComparison (ub_green);

  // set the upper bound for green conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr ub_blue(new pcl::PackedRGBComparison<PointT>("b", pcl::ComparisonOps::LT, g_cf_blue+25.5));
  condition->addComparison (ub_blue);

  g_cf.setInputCloud (in_cloud_ptr);
  g_cf.setCondition (condition);
  g_cf.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw1Solution::findNormals (PointCPtr &in_cloud_ptr)
{
  // Estimate point normals
  g_ne.setInputCloud (in_cloud_ptr);
  g_ne.setSearchMethod (g_tree_ptr);
  g_ne.setKSearch (g_k_nn);
  g_ne.compute (*g_cloud_normals);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw1Solution::segPlane (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight (0.1);
  g_seg.setMethodType (pcl::SAC_RANSAC);
  g_seg.setMaxIterations (100); 
  g_seg.setDistanceThreshold (0.03); 
  g_seg.setInputCloud (in_cloud_ptr);
  g_seg.setInputNormals (g_cloud_normals);

  // Obtain the plane inliers and coefficients
  g_seg.segment (*g_inliers_plane, *g_coeff_plane);


  Cw1Solution::extractInlier (in_cloud_ptr);

}

////////////////////////////////////////////////////////////////////////////////
void
Cw1Solution::segClusters (PointCPtr &in_cloud_ptr)
{

  /*this function is used to extract euclidean cluster*/

  std::cout << "Number of data points in the unclustered PointCloud: " << in_cloud_ptr->size () << std::endl; 

  //To clear previous cluster indices
  g_cluster_indices.clear();

  g_ec.setClusterTolerance (0.02); // 2cm
  
  //Minimum set so that half cut cubes are not classified as clusters
  g_ec.setMinClusterSize (500); 
  g_ec.setMaxClusterSize (3000);
  g_ec.setSearchMethod (g_tree_ptr);
  g_ec.setInputCloud (in_cloud_ptr);
  g_ec.extract (g_cluster_indices);

}

////////////////////////////////////////////////////////////////////////////////
void
Cw1Solution::extractInlier (PointCPtr &in_cloud_ptr)
{
   /* A function to extract the inliers from the input cloud */


  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud (in_cloud_ptr);
  g_extract_pc.setIndices (g_inliers_plane);
  g_extract_pc.setNegative (false);
  
  // Write the planar inliers to disk
  g_extract_pc.filter (*g_cloud_plane);
  
  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative (true);
  g_extract_pc.filter (*g_cloud_filtered2);
  g_extract_normals.setNegative (true);
  g_extract_normals.setInputCloud (g_cloud_normals);
  g_extract_normals.setIndices (g_inliers_plane);
  g_extract_normals.filter (*g_cloud_normals2);

}

////////////////////////////////////////////////////////////////////////////////
geometry_msgs::PointStamped
Cw1Solution::findCubePose (PointCPtr &in_cloud_ptr)
{

  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*in_cloud_ptr, centroid_in);
  
  g_cube_pt_msg.header.frame_id = g_input_pc_frame_id_;
  g_cube_pt_msg.header.stamp = ros::Time (0);
  g_cube_pt_msg.point.x = centroid_in[0];
  g_cube_pt_msg.point.y = centroid_in[1];
  g_cube_pt_msg.point.z = centroid_in[2];
  
  // Transform the point to new frame
  geometry_msgs::PointStamped g_cube_pt_msg_out;
  try
  {
    g_listener_.transformPoint ("panda_link0", 
                                g_cube_pt_msg,
                                g_cube_pt_msg_out);
                                
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }
  
  publishPose (g_cube_pt_msg_out);

  g_current_centroid = g_cube_pt_msg_out;


  
  return g_cube_pt_msg_out;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw1Solution::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw1Solution::publishPose (geometry_msgs::PointStamped &cube_pt_msg)
{
  // Create and publish the cube pose (ignore orientation)

  g_pub_pose.publish (cube_pt_msg);
  
  return;
}
