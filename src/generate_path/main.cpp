#include <Classes/Planer_2D.hpp>

// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>
#include <thread>

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace cv;

struct Coordinates
{
    std::string map_name;
    Point start;
    Point end;
};

namespace ompl_example_2d {

/// occupancy map used for planning
Mat occupancyMap;

Planner2D::Planner2D()
{
}

Planner2D::~Planner2D()
{
}

/// check if the current state is valid
bool isStateValid(const ob::State *state){
    // get x coord of the robot
    const auto *coordX =
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    // get y coord of the robot
    const auto *coordY =
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    //! Comment this part of the code if you'd like to use occupancy grid
    // define the obstacle
//    if (coordX->values[0]<5.1&&coordX->values[0]>5.0){
//        if (coordY->values[0]<4.0&&coordY->values[0]>-5.0){
//            return false;
//        }
//    }
    //! Comment this part of the code if you'd like to use occupancy grid

    //! Your code goes below
    // Hint: uncoment the code below:
    uint8_t *myData = occupancyMap.data;
    if((int)myData[(3*((int)coordY->values[0] * 100 + (int)coordX->values[0]))] == 255)
    {
        return false;
    }

    //! Your code goes above
    return true;
}

/// extract path
std::vector<std::vector<float>> Planner2D::extractPath(ob::ProblemDefinition* pdef){
    std::vector<std::vector<float>> plannedPath;
    // get the obtained path
    ob::PathPtr path = pdef->getSolutionPath();
    // print the path to screen
//    path->print(std::cout);
    // convert to geometric path
    const auto *path_ = path.get()->as<og::PathGeometric>();
    // iterate over each position
    for(unsigned int i=0; i<path_->getStateCount(); ++i){
        // get state
        const ob::State* state = path_->getState(i);
        // get x coord of the robot
        const auto *coordX =
                state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        // get y coord of the robot
        const auto *coordY =
                state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
        // fill in the ROS PoseStamped structure...
        std::vector<float> position;
        position.emplace_back(coordX->values[0]);
        position.emplace_back(coordY->values[0]);
        // ... and add the pose to the path
        plannedPath.emplace_back(position);
    }
    return plannedPath;
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

std::vector<std::vector<float>> Planner2D::planPath(const Mat& globalMap){
    occupancyMap = globalMap;

    // search space information
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    // define state checking callback
    si->setStateValidityChecker(isStateValid);
    // set State Validity Checking Resolution (avoid going through the walls)
    si->setStateValidityCheckingResolution(0.001);

    // problem definition
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(*start.get(), *goal.get());
    pdef->setOptimizationObjective(getPathLengthObjective(si));

    // create planner
    auto planner(std::make_shared<og::RRTstar>(si));
    // configure the planner
    planner->setRange(maxStepLength);// max step length
    planner->setProblemDefinition(pdef);
//    planner->setNumSamplingAttempts(1000000);
//    planner->setGoalBias(0.5);
//    planner->setInformedSampling(true);
//    planner->setTreePruning(true);
//    planner->setNewStateRejection(true);
//    planner->setFocusSearch(true);
    planner->setup();

    // solve motion planning problem
    ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

    std::vector<std::vector<float>> plannedPath;
    if (solved) {// if cussess
        // get the planned path
        plannedPath=extractPath(pdef.get());
    }
    std::cout << "Sumples Num:" << planner->getNumSamplingAttempts() << std::endl;
    return plannedPath;
}

/// configure planner
void Planner2D::configure(int start_x,int start_y,int end_x,int end_y){
    dim = 2;//2D problem
    maxStepLength = 1;// max step length

    // create bounds for the x axis
    coordXBound.reset(new ob::RealVectorBounds(dim-1));
    coordXBound->setLow(0);
    coordXBound->setHigh(99);

    // create bounds for the y axis
    coordYBound.reset(new ob::RealVectorBounds(dim-1));
    coordYBound->setLow(0);
    coordYBound->setHigh(99);

    // construct the state space we are planning in
    auto coordX(std::make_shared<ob::RealVectorStateSpace>(dim-1));
    auto coordY(std::make_shared<ob::RealVectorStateSpace>(dim-1));
    space = coordX +coordY;

    // create bounds for the x axis
    coordX->setBounds(*coordXBound.get());

    // create bounds for the y axis
    coordY->setBounds(*coordYBound.get());

    // define the start position
    start.reset(new ob::ScopedState<>(space));
    (*start.get())[0]=start_x;
    (*start.get())[1]=start_y;
//    start.get()->random();

    // define the goal position
    goal.reset(new ob::ScopedState<>(space));
    (*goal.get())[0]=end_x;
    (*goal.get())[1]=end_y;
//    goal.get()->random();
}

} /* namespace */

void plan_path(std::string image_name, Point start, Point end)
{
    std::string dir_path = "/home/mikolaj/Desktop/";
    std::string image_path = dir_path + "maps2/" + image_name;

    Mat map = imread(image_path, IMREAD_COLOR);
    ompl_example_2d::Planner2D planner;
    planner.configure(start.x, start.y, end.x, end.y);
    std::vector<std::vector<float>> planned_path = planner.planPath(map);
    cv::Mat blank_1(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat blank_2(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));

    for(unsigned long i = 0; i < planned_path.size()-1; i++)
    {
        float x = planned_path[i][0];
        float y = planned_path[i][1];
        float x_n = planned_path[i+1][0];
        float y_n = planned_path[i+1][1];

        line(blank_1, Point(x,y),Point(x_n,y_n),Scalar(0,255,0),1);
        line(blank_2, Point(x,y),Point(x_n,y_n),Scalar(0,255,0),2);
        //line(map, Point(x,y),Point(x_n,y_n),Scalar(0,255,0),2);
    }

    std::string save_path_1 = dir_path + "planned_paths/planned_" + image_name;
    std::string save_path_2 = dir_path + "planned_paths_2/planned_" + image_name;

    imwrite(save_path_1, blank_1);
    imwrite(save_path_2, blank_2);
    //imwrite(save_path, map);
    std::cout << "Map Saved!" << std::endl;
}

Coordinates read_record(int line_number)
{
    std::fstream file("/home/mikolaj/Desktop/maps2/maps_coordinates.csv", std::fstream::in);
    Coordinates cords;

    if (file.is_open()) {
        std::string line;
        std::getline(file, line);
        int current_line = 0;

        while (std::getline(file, line)) {

            if(current_line == line_number)
            {
                std::stringstream str(line);

                std::string double_str;
                std::getline(str, double_str, ',');
                cords.map_name = double_str;
                std::getline(str, double_str, ',');
                cords.start.x = std::stoi(double_str);
                std::getline(str, double_str, ',');
                cords.start.y = std::stoi(double_str);
                std::getline(str, double_str, ',');
                cords.end.x = std::stoi(double_str);
                std::getline(str, double_str, ',');
                cords.end.y = std::stoi(double_str);
            }
            current_line++;
        }
    }
    return cords;
}

int main()
{
    for(int i = 0; i < 1000; i++)
    {
        Coordinates cords = read_record(i);
        plan_path(cords.map_name, cords.start, cords.end);
        std::cout << i << std::endl;
    }

    return 0;
}
