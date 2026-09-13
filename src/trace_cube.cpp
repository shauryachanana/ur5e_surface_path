#include "trace_cube.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//global node
#include <pcl/kdtree/kdtree_flann.h>
std::shared_ptr<rclcpp::Node> node;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface;

//load a file
std::string mesh_path = "file://" + ament_index_cpp::get_package_share_directory("ur5e_surface_path") + "/meshes/50cmCube.stl";
//now we have info on all the triangles
shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);

extern std::stack<Waypoint> pathHistory;
std::vector<bool> traced;

int main(int argc, char** argv){
    std::atomic<bool> measuring{false};
    double total_path_length = 0.0;
    std::vector<geometry_msgs::msg::Point> trace_points;
    rclcpp::init(argc, argv);

    // initialize them here after rclcpp::init()
    node = std::make_shared<rclcpp::Node>(
        "trace_cube",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto spinner = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    spinner->add_node(node);

    std::thread spinner_thread([spinner]() {
        spinner->spin();
    });

    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        "tcp_trace_marker", 10
    );

    auto tcp_marker_timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [marker_pub, &measuring, &total_path_length, &trace_points]() {

        if (!measuring.load()) {
            return;
        }
        
        double tcp[3] = {0.0, 0.0, 0.0};
        getTCPpose(tcp);

        geometry_msgs::msg::Point point;
        point.x = tcp[0];
        point.y = tcp[1];
        point.z = tcp[2];

        static geometry_msgs::msg::Point previous_sample;
        static bool have_previous_sample = false;
        

        if (have_previous_sample) {
            double step_distance = std::sqrt(
                std::pow(point.x - previous_sample.x, 2) +
                std::pow(point.y - previous_sample.y, 2) +
                std::pow(point.z - previous_sample.z, 2)
            );

            total_path_length += step_distance;
        }

        previous_sample = point;
        have_previous_sample = true;

        static geometry_msgs::msg::Point last_point;
        static bool first_point = true;

        double distance = std::sqrt(
            std::pow(point.x - last_point.x, 2) +
            std::pow(point.y - last_point.y, 2) +
            std::pow(point.z - last_point.z, 2)
        );

        if (first_point || distance >= 0.002) {
            trace_points.push_back(point);
            last_point = point;
            first_point = false;
        }

        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "base_link";
        marker.header.stamp = node->now();

        marker.ns = "tcp_live";
        marker.id = 0;

        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.points = trace_points;

        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.005;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        static int print_counter = 0;

        print_counter++;

        if (print_counter >= 20) {
            RCLCPP_INFO(
                node->get_logger(),
                "Actual TCP path length: %.3f m",
                total_path_length
            );

            print_counter = 0;
        }

        marker_pub->publish(marker);
    });

    gripper_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        node, "ur_manipulator"
    );

    init();

    goHome();

    auto logger = rclcpp::get_logger("main");
    
    /*===================FIND THE POINT TO START===================*/

    std::vector<Triangle> vectorOfTriangles;

    triangleExtraction(vectorOfTriangles);

    pcl::PointCloud<pcl::PointXYZ>::Ptr performance_cloud(
    new pcl::PointCloud<pcl::PointXYZ>
    );

    performance_cloud->points.reserve(vectorOfTriangles.size());

    for (const auto &triangle : vectorOfTriangles) {

    pcl::PointXYZ p;

    p.x = -(triangle.centreOfTriangle[0] * 0.001f)
          - (triangle.normal_x * 0.05f);

    p.y = -(triangle.centreOfTriangle[1] * 0.001f)
          + 0.65f
          - (triangle.normal_y * 0.05f);

    p.z = (triangle.centreOfTriangle[2] * 0.001f)
          + (triangle.normal_z * 0.05f);

    performance_cloud->push_back(p);
    
    }

    RCLCPP_INFO(
    logger,
    "Performance cloud contains %zu points",
    performance_cloud->size()
    );

    pcl::KdTreeFLANN<pcl::PointXYZ> performance_kdtree;
    performance_kdtree.setInputCloud(performance_cloud);
    
    traced = std::vector<bool>(vectorOfTriangles.size(), false);

    #ifndef DEBUGGER
    for(int i = 0; i < (int)vectorOfTriangles.size(); i++){
        RCLCPP_WARN(logger, "neighbours of %d : %d, %d, %d", i, vectorOfTriangles[i].myNeighbours[0],vectorOfTriangles[i].myNeighbours[1],vectorOfTriangles[i].myNeighbours[2]);
    }
    #endif
    #ifdef POINTCLOUDS
    //create a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
w   s   
    //now we have all the triangle centres and theor normals stored
    //centers go to the point cloud obj for easy nearest neighbour calculation
    //vectorOfTriangles is used to store the normal vectors of the triangles to orient the TCP
    triangleExtraction(vectorOfTriangles, cloud);
    std::vector<bool> traced = std::vector<int>(vectorOfTriangles.size(), false);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;

    //create kdTree
    kdTree.setInputCloud(cloud);

    //in this vector we will get an index of the nearest neighbour
    std::vector<int> pointIdxKNNSearch(1);
    //in this vector we will get a distance to the nearest neighbour
    std::vector<float> pointKNNSquaredDistance(1);
    #endif

    /*=============================================================*/

    moveit_msgs::msg::RobotTrajectory trajectory;

    //create a vector that contains the set of "waypoints"
    std::vector<geometry_msgs::msg::Pose> target_poses;

    //msg type to set a pose
    geometry_msgs::msg::Pose target_pose;

    /*========FIND TRIANGLE WITH THE LEAST AMOUNT OF EDGES=========*/

    //two arrays to store min neighbour triangles
    std::vector<Triangle> singleNeighbpurTrangles;
    std::vector<Triangle> doubleNeighbourTriangles;
    int chosenVector = 0;
    // double number_of_outward_triangles = 0;
    for(std::size_t i = 0; i < vectorOfTriangles.size(); i++){
        // double sign = 0;

        // sign = vectorOfTriangles[i].centreOfTriangle[0] * vectorOfTriangles[i].normal_x + vectorOfTriangles[i].centreOfTriangle[1] * vectorOfTriangles[i].normal_y + vectorOfTriangles[i].centreOfTriangle[2] * vectorOfTriangles[i].normal_z;
        // if (sign > 0){
        //     RCLCPP_WARN(logger, "Normal is outward");
        //     number_of_outward_triangles++;
        // }
        // if (sign < 0){
        //     RCLCPP_WARN(logger, "Normal is inward");
        // }
        if(vectorOfTriangles[i].getValidNeighbours(traced, vectorOfTriangles) == 1){
            //use one if a truiangle only has one neighbour (prefferable)
            singleNeighbpurTrangles.push_back(vectorOfTriangles[i]);
        }else if(vectorOfTriangles[i].getValidNeighbours(traced, vectorOfTriangles) == 2){
            //for 2 neighbours
            doubleNeighbourTriangles.push_back(vectorOfTriangles[i]);
        }
        //if there are no < 2 neighbours triangles we use vectorOfTriangles

    }
    
    //depending on how full the arrays are use them to determine the initial one
    if(singleNeighbpurTrangles.size() != 0){
        //if there are triangles with a single neighbour
        chosenVector = 0;
    }else if(doubleNeighbourTriangles.size() != 0){
        //if there are no triangles with a single neighbour but with two neighbours
        chosenVector = 1;
    }else{
        //if there are only triangles with 3 neighbpurs:
        chosenVector = 2;
    }

    /*=============================================================*/

    /*=================GET TO THE CLOSEST TRIANGLE=================*/
    
    AttemptToReach initialTriangle = AttemptToReach::EMPTY_VECTOR;
    int closestTriangleIndex = 0;
    while((rclcpp::ok()) && (initialTriangle != AttemptToReach::TRIANGLE_REACHED) && (chosenVector < 3)){
        switch(chosenVector){
            case 0:
            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "attempt in singleNeighbpurTrangles");
            #endif
            initialTriangle = attemptToReachNextClosest(singleNeighbpurTrangles, closestTriangleIndex);
            chosenVector++;
            break;

            case 1:
            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "attempt in doubleNeighbourTriangles");
            #endif
            initialTriangle = attemptToReachNextClosest(doubleNeighbourTriangles, closestTriangleIndex);
            chosenVector++;
            break;

            case 2:
            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "attempt in vectorOfTriangles");
            #endif
            initialTriangle = attemptToReachNextClosest(vectorOfTriangles, closestTriangleIndex);
            chosenVector++;
            break;
        }
    }
    if((initialTriangle != AttemptToReach::TRIANGLE_REACHED) && (chosenVector > 3)){
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "there are no reachable triangles!");
        #endif
    }else if(initialTriangle == AttemptToReach::TRIANGLE_REACHED){
        vectorOfTriangles[closestTriangleIndex].traced = true;
        traced[closestTriangleIndex] = true;
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "reached initial triangle, %d!", closestTriangleIndex);
        #endif
    }

    /*=============================================================*/


    /*=====================START THE OPERATION=====================*/

    measuring = true;

    auto tracing_start = std::chrono::steady_clock::now();

    int nextOne = startOperation(vectorOfTriangles, traced, vectorOfTriangles[closestTriangleIndex]);
    while(rclcpp::ok() && nextOne != -1){
        nextOne = startOperation(vectorOfTriangles, traced, vectorOfTriangles[nextOne]);
    }
    measuring = false;

    auto tracing_end = std::chrono::steady_clock::now();

    double tracing_time =
    std::chrono::duration<double>(tracing_end - tracing_start).count();
    double average_tcp_speed = total_path_length / tracing_time;


    


    /*=============================================================*/

    if (rclcpp::ok()) {
        goHome();
    }

    RCLCPP_INFO(
        logger,
        "Performance: distance = %.3f m | time = %.2f s | average speed = %.3f m/s",
        total_path_length,
        tracing_time,
        average_tcp_speed
    );

    spinner->cancel();

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    if (spinner_thread.joinable()) {
        spinner_thread.join();
    }

    gripper_group_interface.reset();
    node.reset();

    return 0;
    
}