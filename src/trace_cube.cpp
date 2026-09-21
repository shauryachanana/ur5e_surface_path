#include "trace_cube.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <thread>


// ================================================================
// GLOBAL ROS / MOVEIT OBJECTS
// ================================================================

std::shared_ptr<rclcpp::Node> node;

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

std::unique_ptr<moveit::planning_interface::MoveGroupInterface>
    gripper_group_interface;


// ================================================================
// MESH
// ================================================================

std::string mesh_path =
    "file://" +
    ament_index_cpp::get_package_share_directory("ur5e_surface_path") +
    "/meshes/50cmCube.stl";

shapes::Mesh* mesh =
    shapes::createMeshFromResource(mesh_path);


// ================================================================
// GLOBAL PATH DATA
// ================================================================

extern std::stack<Waypoint> pathHistory;
extern std::stack<Waypoint> stackOfReachableWaypoints;

std::vector<bool> traced;


// ================================================================
// PERFORMANCE ANALYSIS RESULT
// ================================================================

struct CoverageResult
{
    std::vector<bool> covered;

    std::size_t covered_triangles = 0;

    double triangle_coverage_percent = 0.0;

    double covered_area_m2 = 0.0;
    double total_area_m2 = 0.0;
    double area_coverage_percent = 0.0;

    double average_nearest_distance_mm = 0.0;
    double maximum_nearest_distance_mm = 0.0;
};


// ================================================================
// TRIANGLE AREA
// ================================================================

double triangleAreaM2(const Triangle& t)
{
    // STL coordinates are in millimetres.
    // Convert vectors to metres.

    double ax = (t.x[1] - t.x[0]) * 0.001;
    double ay = (t.y[1] - t.y[0]) * 0.001;
    double az = (t.z[1] - t.z[0]) * 0.001;

    double bx = (t.x[2] - t.x[0]) * 0.001;
    double by = (t.y[2] - t.y[0]) * 0.001;
    double bz = (t.z[2] - t.z[0]) * 0.001;

    double cx = ay * bz - az * by;
    double cy = az * bx - ax * bz;
    double cz = ax * by - ay * bx;

    return 0.5 * std::sqrt(
        cx * cx +
        cy * cy +
        cz * cz
    );
}


// ================================================================
// COVERAGE ANALYSIS
// ================================================================

CoverageResult analyseCoverage(
    const std::vector<geometry_msgs::msg::Point>& trace_points,
    const std::vector<Triangle>& triangles,
    pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
    double coverage_radius_m,
    double interpolation_step_m)
{
    CoverageResult result;

    result.covered.resize(
        triangles.size(),
        false
    );

    if(trace_points.empty() || triangles.empty())
    {
        return result;
    }


    // ============================================================
    // TCP DISTANCE FROM STL
    // ============================================================

    double nearest_sum_mm = 0.0;
    double nearest_max_mm = 0.0;
    std::size_t nearest_count = 0;

    for(const auto& p : trace_points)
    {
        pcl::PointXYZ search_point;

        search_point.x = p.x;
        search_point.y = p.y;
        search_point.z = p.z;

        std::vector<int> index(1);
        std::vector<float> squared_distance(1);

        if(kdtree.nearestKSearch(
                search_point,
                1,
                index,
                squared_distance) > 0)
        {
            double distance_mm =
                std::sqrt(
                    squared_distance[0]
                ) * 1000.0;

            nearest_sum_mm += distance_mm;

            nearest_max_mm =
                std::max(
                    nearest_max_mm,
                    distance_mm
                );

            nearest_count++;
        }
    }

    if(nearest_count > 0)
    {
        result.average_nearest_distance_mm =
            nearest_sum_mm /
            static_cast<double>(nearest_count);

        result.maximum_nearest_distance_mm =
            nearest_max_mm;
    }


    // ============================================================
    // HELPER TO MARK TRIANGLES NEAR TCP
    // ============================================================

    auto markNearbyTriangles =
        [&](double x, double y, double z)
    {
        pcl::PointXYZ search_point;

        search_point.x = x;
        search_point.y = y;
        search_point.z = z;

        std::vector<int> indices;
        std::vector<float> squared_distances;

        if(kdtree.radiusSearch(
                search_point,
                coverage_radius_m,
                indices,
                squared_distances) > 0)
        {
            for(int index : indices)
            {
                if(index >= 0 &&
                   static_cast<std::size_t>(index) <
                       result.covered.size())
                {
                    result.covered[index] = true;
                }
            }
        }
    };


    // ============================================================
    // FOLLOW COMPLETE TCP LINE
    //
    // We interpolate between recorded TCP samples so coverage does
    // not depend only on the discrete 100 ms samples.
    // ============================================================

    for(std::size_t i = 1;
        i < trace_points.size();
        ++i)
    {
        const auto& a =
            trace_points[i - 1];

        const auto& b =
            trace_points[i];

        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double dz = b.z - a.z;

        double segment_length =
            std::sqrt(
                dx * dx +
                dy * dy +
                dz * dz
            );

        int steps =
            std::max(
                1,
                static_cast<int>(
                    std::ceil(
                        segment_length /
                        interpolation_step_m
                    )
                )
            );

        for(int s = 0; s < steps; ++s)
        {
            double t =
                static_cast<double>(s) /
                static_cast<double>(steps);

            double x =
                a.x + t * dx;

            double y =
                a.y + t * dy;

            double z =
                a.z + t * dz;

            markNearbyTriangles(
                x,
                y,
                z
            );
        }
    }


    // Include final TCP position.

    const auto& last =
        trace_points.back();

    markNearbyTriangles(
        last.x,
        last.y,
        last.z
    );


    // ============================================================
    // CALCULATE COVERAGE
    // ============================================================

    for(std::size_t i = 0;
        i < triangles.size();
        ++i)
    {
        double area =
            triangleAreaM2(
                triangles[i]
            );

        result.total_area_m2 += area;

        if(result.covered[i])
        {
            result.covered_triangles++;
            result.covered_area_m2 += area;
        }
    }


    result.triangle_coverage_percent =
        100.0 *
        static_cast<double>(
            result.covered_triangles
        ) /
        static_cast<double>(
            triangles.size()
        );


    if(result.total_area_m2 > 0.0)
    {
        result.area_coverage_percent =
            100.0 *
            result.covered_area_m2 /
            result.total_area_m2;
    }


    return result;
}


// ================================================================
// MAIN
// ================================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);


    // ============================================================
    // PERFORMANCE DATA
    // ============================================================

    std::atomic<bool> measuring{false};

    double total_path_length = 0.0;

    std::vector<geometry_msgs::msg::Point>
        trace_points;


    // ============================================================
    // NODE
    // ============================================================

    node =
        std::make_shared<rclcpp::Node>(
            "trace_cube",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(
                    true
                )
        );


    // ============================================================
    // TF
    // ============================================================

    tf_buffer =
        std::make_shared<tf2_ros::Buffer>(
            node->get_clock()
        );

    tf_listener =
        std::make_shared<
            tf2_ros::TransformListener
        >(
            *tf_buffer,
            node,
            false
        );

    // TF callbacks are serviced by our executor thread.
    tf_buffer->setUsingDedicatedThread(true);


    // ============================================================
    // EXECUTOR
    // ============================================================

    auto spinner =
        std::make_shared<
            rclcpp::executors::MultiThreadedExecutor
        >();

    spinner->add_node(node);

    std::thread spinner_thread(
        [spinner]()
        {
            spinner->spin();
        }
    );


    // ============================================================
    // RVIZ TCP TRACE PUBLISHER
    // ============================================================

    auto marker_pub =
        node->create_publisher<
            visualization_msgs::msg::Marker
        >(
            "tcp_trace_marker",
            10
        );


    // ============================================================
    // TCP SAMPLING TIMER
    //
    // Runs every 100 ms but records points ONLY while
    // measuring == true.
    // ============================================================

    auto tcp_marker_timer =
        node->create_wall_timer(
            std::chrono::milliseconds(100),

            [
                marker_pub,
                &measuring,
                &total_path_length,
                &trace_points
            ]()
            {
                if(!measuring.load())
                {
                    return;
                }


                // -----------------------------------------------
                // Read TCP from TF.
                // -----------------------------------------------

                double tcp[3] =
                {
                    0.0,
                    0.0,
                    0.0
                };

                // Do not record an invalid zero point if TF fails.
                if(!getTCPpose(tcp))
                {
                    return;
                }


                geometry_msgs::msg::Point point;

                point.x = tcp[0];
                point.y = tcp[1];
                point.z = tcp[2];


                // -----------------------------------------------
                // Calculate total physical TCP path length.
                // -----------------------------------------------

                static geometry_msgs::msg::Point
                    previous_sample;

                static bool
                    have_previous_sample = false;


                if(have_previous_sample)
                {
                    double step_distance =
                        std::sqrt(
                            std::pow(
                                point.x -
                                previous_sample.x,
                                2
                            )
                            +
                            std::pow(
                                point.y -
                                previous_sample.y,
                                2
                            )
                            +
                            std::pow(
                                point.z -
                                previous_sample.z,
                                2
                            )
                        );

                    total_path_length +=
                        step_distance;
                }


                previous_sample = point;
                have_previous_sample = true;


                // -----------------------------------------------
                // Store trace points only when the TCP has moved
                // at least 2 mm from the last stored point.
                // -----------------------------------------------

                static geometry_msgs::msg::Point
                    last_point;

                static bool first_point = true;


                double distance =
                    std::sqrt(
                        std::pow(
                            point.x -
                            last_point.x,
                            2
                        )
                        +
                        std::pow(
                            point.y -
                            last_point.y,
                            2
                        )
                        +
                        std::pow(
                            point.z -
                            last_point.z,
                            2
                        )
                    );


                if(first_point ||
                   distance >= 0.002)
                {
                    trace_points.push_back(
                        point
                    );

                    last_point = point;
                    first_point = false;
                }


                // -----------------------------------------------
                // Publish RViz line marker.
                // -----------------------------------------------

                visualization_msgs::msg::Marker
                    marker;

                marker.header.frame_id =
                    "base_link";

                marker.header.stamp =
                    node->now();

                marker.ns =
                    "tcp_live";

                marker.id = 0;

                marker.type =
                    visualization_msgs::msg::Marker::
                        LINE_STRIP;

                marker.action =
                    visualization_msgs::msg::Marker::
                        ADD;

                marker.points =
                    trace_points;

                marker.pose.orientation.w =
                    1.0;

                marker.scale.x =
                    0.005;

                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;


                static int print_counter = 0;

                print_counter++;


                if(print_counter >= 20)
                {
                    RCLCPP_INFO(
                        node->get_logger(),
                        "Actual TCP path length: %.3f m",
                        total_path_length
                    );

                    print_counter = 0;
                }


                marker_pub->publish(
                    marker
                );
            }
        );


    // ============================================================
    // WAIT FOR TF / CLOCK / JOINT STATES
    // ============================================================

    rclcpp::sleep_for(
        std::chrono::seconds(2)
    );


    // ============================================================
    // MOVEIT
    // ============================================================

    gripper_group_interface =
        std::make_unique<
            moveit::planning_interface::
                MoveGroupInterface
        >(
            node,
            "ur_manipulator"
        );


    init();

    goHome();


    auto logger =
        rclcpp::get_logger(
            "main"
        );


    // ============================================================
    // EXTRACT TRIANGLES
    // ============================================================

    std::vector<Triangle>
        vectorOfTriangles;

    triangleExtraction(
        vectorOfTriangles
    );


    if(vectorOfTriangles.empty())
    {
        RCLCPP_ERROR(
            logger,
            "No triangles extracted from mesh"
        );

        gripper_group_interface.reset();

        spinner->cancel();

        if(spinner_thread.joinable())
        {
            spinner_thread.join();
        }

        tcp_marker_timer.reset();
        marker_pub.reset();

        tf_listener.reset();
        tf_buffer.reset();

        spinner->remove_node(node);
        node.reset();

        spinner.reset();

        if(rclcpp::ok())
        {
            rclcpp::shutdown();
        }

        return 1;
    }


    // ============================================================
    // PERFORMANCE CLOUD
    //
    // One point corresponding to each triangle centre / TCP target.
    // ============================================================

    pcl::PointCloud<
        pcl::PointXYZ
    >::Ptr performance_cloud(
        new pcl::PointCloud<
            pcl::PointXYZ
        >
    );


    performance_cloud->points.reserve(
        vectorOfTriangles.size()
    );


    for(const auto& triangle :
        vectorOfTriangles)
    {
        pcl::PointXYZ p;

        p.x =
            -(
                triangle.centreOfTriangle[0] *
                0.001f
            )
            -
            (
                triangle.normal_x *
                0.05f
            );

        p.y =
            -(
                triangle.centreOfTriangle[1] *
                0.001f
            )
            +
            0.65f
            -
            (
                triangle.normal_y *
                0.05f
            );

        p.z =
            (
                triangle.centreOfTriangle[2] *
                0.001f
            )
            +
            (
                triangle.normal_z *
                0.05f
            );


        performance_cloud->push_back(
            p
        );
    }


    RCLCPP_INFO(
        logger,
        "Performance cloud contains %zu points",
        performance_cloud->size()
    );


    pcl::KdTreeFLANN<
        pcl::PointXYZ
    > performance_kdtree;

    performance_kdtree.setInputCloud(
        performance_cloud
    );


    traced =
        std::vector<bool>(
            vectorOfTriangles.size(),
            false
        );


    // ============================================================
    // FIND TRIANGLES WITH FEWEST NEIGHBOURS
    // ============================================================

    std::vector<Triangle>
        singleNeighbpurTrangles;

    std::vector<Triangle>
        doubleNeighbourTriangles;

    int chosenVector = 0;


    for(std::size_t i = 0;
        i < vectorOfTriangles.size();
        ++i)
    {
        int validNeighbours =
            vectorOfTriangles[i]
                .getValidNeighbours(
                    traced,
                    vectorOfTriangles
                );


        if(validNeighbours == 1)
        {
            singleNeighbpurTrangles.push_back(
                vectorOfTriangles[i]
            );
        }
        else if(validNeighbours == 2)
        {
            doubleNeighbourTriangles.push_back(
                vectorOfTriangles[i]
            );
        }
    }


    if(!singleNeighbpurTrangles.empty())
    {
        chosenVector = 0;
    }
    else if(!doubleNeighbourTriangles.empty())
    {
        chosenVector = 1;
    }
    else
    {
        chosenVector = 2;
    }


    // ============================================================
    // FIND INITIAL REACHABLE TRIANGLE
    // ============================================================

    AttemptToReach initialTriangle =
        AttemptToReach::EMPTY_VECTOR;

    int closestTriangleIndex = 0;


    while(
        initialTriangle !=
            AttemptToReach::TRIANGLE_REACHED
        &&
        chosenVector < 3
    )
    {
        switch(chosenVector)
        {
            case 0:
            {
                #ifdef DEBUGGER
                RCLCPP_WARN(
                    logger,
                    "attempt in singleNeighbpurTrangles"
                );
                #endif

                initialTriangle =
                    attemptToReachNextClosest(
                        singleNeighbpurTrangles,
                        closestTriangleIndex
                    );

                chosenVector++;

                break;
            }


            case 1:
            {
                #ifdef DEBUGGER
                RCLCPP_WARN(
                    logger,
                    "attempt in doubleNeighbourTriangles"
                );
                #endif

                initialTriangle =
                    attemptToReachNextClosest(
                        doubleNeighbourTriangles,
                        closestTriangleIndex
                    );

                chosenVector++;

                break;
            }


            case 2:
            {
                #ifdef DEBUGGER
                RCLCPP_WARN(
                    logger,
                    "attempt in vectorOfTriangles"
                );
                #endif

                initialTriangle =
                    attemptToReachNextClosest(
                        vectorOfTriangles,
                        closestTriangleIndex
                    );

                chosenVector++;

                break;
            }
        }
    }


    // ============================================================
    // ABORT IF NO INITIAL TRIANGLE WAS REACHABLE
    // ============================================================

    if(
        initialTriangle !=
            AttemptToReach::TRIANGLE_REACHED
    )
    {
        RCLCPP_ERROR(
            logger,
            "There are no reachable initial triangles. Aborting."
        );


        gripper_group_interface.reset();

        spinner->cancel();

        if(spinner_thread.joinable())
        {
            spinner_thread.join();
        }

        tcp_marker_timer.reset();
        marker_pub.reset();

        tf_listener.reset();
        tf_buffer.reset();

        spinner->remove_node(node);
        node.reset();

        spinner.reset();

        if(rclcpp::ok())
        {
            rclcpp::shutdown();
        }

        return 1;
    }


    vectorOfTriangles[
        closestTriangleIndex
    ].traced = true;

    traced[
        closestTriangleIndex
    ] = true;


    #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "reached initial triangle, %d!",
        closestTriangleIndex
    );
    #endif


    // ============================================================
    // PLANNING PHASE
    //
    // IMPORTANT:
    // measuring remains FALSE here.
    //
    // startOperation() plans the complete virtual trajectory.
    // We do NOT want to measure TCP performance here because the
    // real robot is not executing the surface path yet.
    //
    // For large meshes, planning can spend a long time backtracking.
    // The monitor below distinguishes genuine progress from repeated
    // retry/backtracking behaviour.
    // ============================================================

    RCLCPP_WARN(
        logger,
        "START PLANNING OPERATION"
    );


    std::size_t planningIterations = 0;

    std::size_t previousTracedCount =
        std::count(
            traced.begin(),
            traced.end(),
            true
        );

    std::size_t iterationsWithoutProgress = 0;

    /*
     * Backtracking is expected, especially on a dense curved surface.
     * Therefore this limit is deliberately generous.
     *
     * Planning is stopped only if thousands of consecutive calls to
     * startOperation() fail to add even one new traced triangle.
     */
    const std::size_t MAX_NO_PROGRESS =
        std::max<std::size_t>(
            5000,
            vectorOfTriangles.size() * 4
        );

    constexpr std::size_t PROGRESS_LOG_INTERVAL = 100;

    const auto planningStart =
        std::chrono::steady_clock::now();


    int nextOne =
        startOperation(
            vectorOfTriangles,
            traced,
            vectorOfTriangles[
                closestTriangleIndex
            ]
        );


    while(
        rclcpp::ok() &&
        nextOne != -1
    )
    {
        planningIterations++;


        // --------------------------------------------------------
        // Measure real planning progress.
        // --------------------------------------------------------

        std::size_t currentTracedCount =
            std::count(
                traced.begin(),
                traced.end(),
                true
            );


        if(currentTracedCount > previousTracedCount)
        {
            // At least one new surface triangle was reached.
            iterationsWithoutProgress = 0;
            previousTracedCount = currentTracedCount;
        }
        else
        {
            // This iteration only retried or backtracked.
            iterationsWithoutProgress++;
        }


        // --------------------------------------------------------
        // Print one useful progress message every 100 iterations.
        // --------------------------------------------------------

        if(
            planningIterations %
                PROGRESS_LOG_INTERVAL ==
            0
        )
        {
            const double percent =
                100.0 *
                static_cast<double>(
                    currentTracedCount
                ) /
                static_cast<double>(
                    vectorOfTriangles.size()
                );

            const double elapsedSeconds =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() -
                    planningStart
                ).count();


            RCLCPP_WARN(
                logger,
                "PLANNING: iteration=%zu | "
                "traced=%zu/%zu (%.2f%%) | "
                "current triangle=%d | "
                "pathHistory=%zu | "
                "execution waypoints=%zu | "
                "no-progress=%zu | "
                "elapsed=%.1f s",
                planningIterations,
                currentTracedCount,
                vectorOfTriangles.size(),
                percent,
                nextOne,
                pathHistory.size(),
                stackOfReachableWaypoints.size(),
                iterationsWithoutProgress,
                elapsedSeconds
            );
        }


        // --------------------------------------------------------
        // Detect probable endless retry/backtracking behaviour.
        // --------------------------------------------------------

        if(
            iterationsWithoutProgress >
            MAX_NO_PROGRESS
        )
        {
            RCLCPP_ERROR(
                logger,
                "Planning stopped: no new triangle was traced "
                "for %zu consecutive iterations. "
                "This strongly suggests repeated backtracking "
                "or retry behaviour.",
                iterationsWithoutProgress
            );

            break;
        }


        // --------------------------------------------------------
        // Protect against a bad index returned by startOperation().
        // --------------------------------------------------------

        if(
            nextOne < 0 ||
            static_cast<std::size_t>(nextOne) >=
                vectorOfTriangles.size()
        )
        {
            RCLCPP_ERROR(
                logger,
                "Planning stopped because startOperation() "
                "returned invalid triangle index %d",
                nextOne
            );

            break;
        }


        // --------------------------------------------------------
        // Continue planning from the selected next triangle.
        // --------------------------------------------------------

        nextOne =
            startOperation(
                vectorOfTriangles,
                traced,
                vectorOfTriangles[
                    nextOne
                ]
            );
    }


    const std::size_t finalTracedCount =
        std::count(
            traced.begin(),
            traced.end(),
            true
        );

    const double finalPlanningPercent =
        100.0 *
        static_cast<double>(
            finalTracedCount
        ) /
        static_cast<double>(
            vectorOfTriangles.size()
        );

    const double totalPlanningSeconds =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() -
            planningStart
        ).count();


    RCLCPP_WARN(
        logger,
        "PLANNING FINISHED: iterations=%zu | "
        "traced=%zu/%zu (%.2f%%) | "
        "stored execution waypoints=%zu | "
        "time=%.1f s",
        planningIterations,
        finalTracedCount,
        vectorOfTriangles.size(),
        finalPlanningPercent,
        stackOfReachableWaypoints.size(),
        totalPlanningSeconds
    );


    // ============================================================
    // PLAN COMPLETE
    // ============================================================

    gripper_group_interface
        ->setStartStateToCurrentState();


    #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "setStartStateToCurrentState passed"
    );
    #endif


    // Convert stack into chronological execution order.

    std::vector<Waypoint>
        orderedWaypoints =
            extractOrderedPath(
                stackOfReachableWaypoints
            );


    #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "extractOrderedPath passed"
    );
    #endif


    plannedPath.clear();


    for(const auto& wp :
        orderedWaypoints)
    {
        // EDGE waypoints may use triangleIndex = 0.
        // Keep existing project behaviour.
        if(
            wp.triangleIndex >= 0 &&
            static_cast<std::size_t>(
                wp.triangleIndex
            ) < vectorOfTriangles.size()
        )
        {
            plannedPath.push_back(
                vectorOfTriangles[
                    wp.triangleIndex
                ]
            );
        }


        #ifdef DEBUGGER
        RCLCPP_WARN(
            logger,
            "added waypoint"
        );
        #endif
    }


    // ============================================================
    // PLANNED COVERAGE
    // ============================================================

    float tracedf = 0.0f;


    for(std::size_t i = 0;
        i < vectorOfTriangles.size();
        ++i)
    {
        if(vectorOfTriangles[i].traced)
        {
            tracedf++;
        }
    }


    #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "traced: %f",
        tracedf
    );

    RCLCPP_WARN(
        logger,
        "vectorOfTriangles: %zu",
        vectorOfTriangles.size()
    );
    #endif


    float coveragePercent =
        100.0f *
        tracedf /
        static_cast<float>(
            vectorOfTriangles.size()
        );


    // ============================================================
    // PHYSICAL EXECUTION + PERFORMANCE MEASUREMENT
    // ============================================================

    bool executionSuccessful = false;
    bool executionAttempted = false;

    std::chrono::steady_clock::time_point
        tracing_start;

    std::chrono::steady_clock::time_point
        tracing_end;


    if(confirmPathExecution(
            coveragePercent
        ))
    {
        executionAttempted = true;


        RCLCPP_WARN(
            logger,
            "execution start"
        );


        // --------------------------------------------------------
        // Reset performance values immediately before actual robot
        // motion begins.
        // --------------------------------------------------------

        trace_points.clear();

        total_path_length = 0.0;


        // Start actual physical performance measurement.

        tracing_start =
            std::chrono::steady_clock::now();

        measuring.store(true);


        // --------------------------------------------------------
        // This executes the complete combined continuous trajectory.
        // --------------------------------------------------------

        executionSuccessful =
            executePlannedPath(
                orderedWaypoints
            );


        // Stop recording immediately after execution.

        measuring.store(false);

        tracing_end =
            std::chrono::steady_clock::now();


        if(executionSuccessful)
        {
            RCLCPP_WARN(
                logger,
                "Complete planned path executed successfully"
            );
        }
        else
        {
            RCLCPP_ERROR(
                logger,
                "Planned path execution FAILED"
            );
        }
    }
    else
    {
        RCLCPP_WARN(
            logger,
            "path execution cancelled by user"
        );
    }


    // ============================================================
    // STOP PERFORMANCE SAMPLING
    // ============================================================

    measuring.store(false);

    tcp_marker_timer->cancel();


    // Allow an already-running timer callback to finish.
    std::this_thread::sleep_for(
        std::chrono::milliseconds(200)
    );


    // ============================================================
    // PERFORMANCE TIMING
    // ============================================================

    double tracing_time = 0.0;


    if(executionAttempted)
    {
        tracing_time =
            std::chrono::duration<double>(
                tracing_end -
                tracing_start
            ).count();
    }


    double average_tcp_speed = 0.0;


    if(tracing_time > 0.0)
    {
        average_tcp_speed =
            total_path_length /
            tracing_time;
    }


    // ============================================================
    // GEOMETRIC COVERAGE ANALYSIS
    // ============================================================

    constexpr double COVERAGE_RADIUS_M =
        0.005;   // 5 mm

    constexpr double INTERPOLATION_STEP_M =
        0.002;   // 2 mm


    RCLCPP_INFO(
        logger,
        "Starting geometric coverage analysis..."
    );


    CoverageResult coverage =
        analyseCoverage(
            trace_points,
            vectorOfTriangles,
            performance_kdtree,
            COVERAGE_RADIUS_M,
            INTERPOLATION_STEP_M
        );


    // ============================================================
    // RETURN HOME
    // ============================================================

    if(rclcpp::ok())
    {
        goHome();
    }


    // ============================================================
    // SAVE TCP TRACE CSV
    // ============================================================

    std::ofstream trace_file(
        "tcp_trace.csv"
    );


    if(trace_file.is_open())
    {
        trace_file <<
            "x_m,y_m,z_m\n";


        for(const auto& p :
            trace_points)
        {
            trace_file
                << p.x << ","
                << p.y << ","
                << p.z << "\n";
        }


        trace_file.close();
    }
    else
    {
        RCLCPP_ERROR(
            logger,
            "Could not open tcp_trace.csv"
        );
    }


    // ============================================================
    // SAVE PERFORMANCE SUMMARY
    // ============================================================

    std::ofstream summary_file(
        "performance_summary.csv"
    );


    if(summary_file.is_open())
    {
        summary_file <<
            "metric,value\n";


        summary_file
            << "tcp_samples,"
            << trace_points.size()
            << "\n";


        summary_file
            << "path_length_m,"
            << total_path_length
            << "\n";


        summary_file
            << "tracing_time_s,"
            << tracing_time
            << "\n";


        summary_file
            << "average_speed_m_s,"
            << average_tcp_speed
            << "\n";


        summary_file
            << "covered_triangles,"
            << coverage.covered_triangles
            << "\n";


        summary_file
            << "total_triangles,"
            << vectorOfTriangles.size()
            << "\n";


        summary_file
            << "triangle_coverage_percent,"
            << coverage.triangle_coverage_percent
            << "\n";


        summary_file
            << "surface_coverage_percent,"
            << coverage.area_coverage_percent
            << "\n";


        summary_file
            << "covered_area_m2,"
            << coverage.covered_area_m2
            << "\n";


        summary_file
            << "total_area_m2,"
            << coverage.total_area_m2
            << "\n";


        summary_file
            << "average_stl_distance_mm,"
            << coverage.average_nearest_distance_mm
            << "\n";


        summary_file
            << "maximum_stl_distance_mm,"
            << coverage.maximum_nearest_distance_mm
            << "\n";


        summary_file.close();
    }
    else
    {
        RCLCPP_ERROR(
            logger,
            "Could not open performance_summary.csv"
        );
    }


    // ============================================================
    // PRINT RESULTS
    // ============================================================

    RCLCPP_INFO(
        logger,
        "========== PERFORMANCE RESULTS =========="
    );


    RCLCPP_INFO(
        logger,
        "Execution successful: %s",
        executionSuccessful ?
            "YES" :
            "NO"
    );


    RCLCPP_INFO(
        logger,
        "TCP samples: %zu",
        trace_points.size()
    );


    RCLCPP_INFO(
        logger,
        "Actual TCP distance: %.3f m",
        total_path_length
    );


    RCLCPP_INFO(
        logger,
        "Tracing time: %.2f s",
        tracing_time
    );


    RCLCPP_INFO(
        logger,
        "Average TCP speed: %.3f m/s",
        average_tcp_speed
    );


    RCLCPP_INFO(
        logger,
        "Covered triangles: %zu / %zu",
        coverage.covered_triangles,
        vectorOfTriangles.size()
    );


    RCLCPP_INFO(
        logger,
        "Triangle coverage: %.2f %%",
        coverage.triangle_coverage_percent
    );


    RCLCPP_INFO(
        logger,
        "Surface area coverage: %.2f %%",
        coverage.area_coverage_percent
    );


    RCLCPP_INFO(
        logger,
        "Covered surface area: %.4f m^2 / %.4f m^2",
        coverage.covered_area_m2,
        coverage.total_area_m2
    );


    RCLCPP_INFO(
        logger,
        "Average TCP-to-STL distance: %.2f mm",
        coverage.average_nearest_distance_mm
    );


    RCLCPP_INFO(
        logger,
        "Maximum TCP-to-STL distance: %.2f mm",
        coverage.maximum_nearest_distance_mm
    );


    // ============================================================
    // CLEAN SHUTDOWN
    // ============================================================

    gripper_group_interface.reset();


    spinner->cancel();


    if(spinner_thread.joinable())
    {
        spinner_thread.join();
    }


    // Destroy ROS entities before rclcpp::shutdown().

    tcp_marker_timer.reset();
    marker_pub.reset();

    tf_listener.reset();
    tf_buffer.reset();


    spinner->remove_node(node);

    node.reset();

    spinner.reset();


    if(rclcpp::ok())
    {
        rclcpp::shutdown();
    }


    return 0;
}