#pragma once

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
//#include <ros/ros.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <nav_msgs/Path.h>
//#include <nav_msgs/OccupancyGrid.h>

//#include <moveit/ompl_interface/ompl_interface.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

// Boost
#include <boost/thread.hpp>

// standard
#include <mutex>
#include <iostream>
#include <thread>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

namespace ompl_example_2d {

/*!
 * 2D planner example class
 */
class Planner2D
{
public:

    /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
    Planner2D();

    /*!
   * Destructor.
   */
    virtual ~Planner2D();

    /*!
   * plan path
   */
    std::vector<std::vector<float>> planPath(const cv::Mat& globalMap);

    /// configure node
    void configure(int start_x,int start_y,int end_x,int end_y);

private:

    /// problem dim
    int dim;

    /// max step length
    double maxStepLength;

    /// bounds for the x axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordXBound;

    /// bounds for the y axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordYBound;

    /// start position
    std::shared_ptr<ompl::base::ScopedState<>> start;

    /// goal position
    std::shared_ptr<ompl::base::ScopedState<>> goal;

    /// search space
    std::shared_ptr<ompl::base::StateSpace> space;

    /// extract path
    std::vector<std::vector<float>> extractPath(ompl::base::ProblemDefinition* pdef);
};

} /* namespace */
