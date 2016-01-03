#include <ros/ros.h>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>

std::vector<Eigen::Vector3d> camera_observations;
std::vector<Eigen::Vector3d> robot_observations;
int numCameraObservations = 0;
int numRobotObservations = 0;
const int numObservations = 40;
double rotation[3];
double translation[3];
double rotationArray[3];
double translationArray[3];
double margin = .00001;
int numPassed = 0;
int reps = 10000;
std::ofstream myfile;
std::string filename = "pointOutput.txt";

/**
 * @brief Calculates a transformed point
 *
 * @param point The point to be transformed
 * @param rotation[3] The rotation component of the transform, specified as an x,y,z rotation
 * @param translation[3] The translation component of the transform, specified as an x,y,z translation
 *
 * @return The transformed point 
 */
Eigen::Vector3d calculateTransformedPoint(Eigen::Vector3d point, const double rotation[3], const double translation[3])
{
    // Creating Euler angle rotation matrices
    Eigen::Matrix3d Rx;
    Rx <<   1, 0, 0, 
            0, cos(rotation[0]), -sin(rotation[0]),
            0, sin(rotation[0]), cos(rotation[0]);

    Eigen::Matrix3d Ry;
    Ry <<   cos(rotation[1]), 0, sin(rotation[1]),
            0, 1, 0,
            -sin(rotation[1]), 0, cos(rotation[1]);

    Eigen::Matrix3d Rz;
    Rz <<   cos(rotation[2]), -sin(rotation[2]), 0,
            sin(rotation[2]), cos(rotation[2]), 0,
            0, 0, 1;

    Eigen::Matrix3d R = Rx * Ry * Rz;

    // Create homogenous representation rigid body motion g matrix
    Eigen::Matrix4d g;
    g <<    R(0,0), R(0,1), R(0,2), translation[0],
            R(1,0), R(1,1), R(1,2), translation[1],
            R(2,0), R(2,1), R(2,2), translation[2],
            0, 0, 0, 1;

    // Convert point to homogeneous representation
    Eigen::Vector4d pointHomogeneous;
    pointHomogeneous << point(0),
                        point(1),
                        point(2),
                        1;

    // Calculate transformed point in homogenous representation
    Eigen::Vector4d robotPointHomogenous = g * pointHomogeneous;
    
    // Convert out of homogeneous representation
    Eigen::Vector3d robotPoint;
    robotPoint <<   robotPointHomogenous(0),
                    robotPointHomogenous(1),
                    robotPointHomogenous(2);
    
    return robotPoint;
}

/**
 * @brief CostFunctor needed for Ceres
 */
struct CostFunctor
{
    /**
     * @brief Constructor for CostFunctor, takes a camera and robot observation as it's data set
     *
     * @param camera_x  X coordinate of the camera observation
     * @param camera_y  Y coordinate of the camera observation
     * @param camera_z  Z coordinate of the camera observation
     * @param robot_x   X coordinate of the robot observation
     * @param robot_y   Y coordinate of the robot observation
     * @param robot_z   Z coordinate of the robot observation
     */
    CostFunctor(Eigen::Vector3d kinectPoint, Eigen::Vector3d robotPoint) :
        kinectPoint(kinectPoint),
        robotPoint(robotPoint) 
    {}


    /**
     * @brief Operator for calculating residuals on our CostFunctor for optimizing the camera transform
     *
     * @tparam T Template type, Ceres can automatically use doubles or Jacobian types as needed, this is designed for doubles
     * @param rotation  The rotation mutable that will be changed as Ceres optimizes the transform
     * @param translation   The translation mutable that will be changed as Ceres optimizes the transform
     * @param residuals The residuals of the transform that are being optimized
     *
     * @return Returns true when residuals have been calculated
     */
/*    template <typename T>
   bool operator()(const T* const rotation, 
                    const T* const translation,
                    T* residuals) const*/
    bool operator()(const double* const rotation,
                    const double* const translation,
                    double* residuals) const 
    {
        Eigen::Vector3d transformedPoint = calculateTransformedPoint(kinectPoint, rotation, translation);
        residuals[0] = robotPoint(0) - transformedPoint(0);
        residuals[1] = robotPoint(1) - transformedPoint(1);
        residuals[2] = robotPoint(2) - transformedPoint(2);

        /*residuals[0] = T(robotPoint(0)) - translation[0] - T(kinectPoint(2))*sin(rotation[2]) - T(kinectPoint(0))*cos(rotation[1])*cos(rotation[2]) + T(kinectPoint(1))*cos(rotation[2])*sin(rotation[1]);
        residuals[1] = T(robotPoint(1)) - translation[1] - T(kinectPoint(0))*(cos(rotation[0])*sin(rotation[1]) + cos(rotation[1])*sin(rotation[0])*sin(rotation[2])) - T(kinectPoint(1))*(cos(rotation[0])*cos(rotation[1]) - sin(rotation[0])*sin(rotation[1])*sin(rotation[2])) + T(kinectPoint(2))*cos(rotation[2])*sin(rotation[0]);
        residuals[2] = T(robotPoint(2)) - translation[2] - T(kinectPoint(0))*(sin(rotation[0])*sin(rotation[1]) - cos(rotation[0])*cos(rotation[1])*sin(rotation[2])) - T(kinectPoint(1))*(cos(rotation[1])*sin(rotation[0]) + cos(rotation[0])*sin(rotation[1])*sin(rotation[2])) - T(kinectPoint(2))*cos(rotation[0])*cos(rotation[2]);
*/
        return true;
    }

    /**
     * @brief Factory for hiding creating of Cost Function from user 
     *
     * @@param camera_x  X coordinate of the camera observation
     * @param camera_y  Y coordinate of the camera observation
     * @param camera_z  Z coordinate of the camera observation
     * @param robot_x   X coordinate of the robot observation
     * @param robot_y   Y coordinate of the robot observation
     * @param robot_z   Z coordinate of the robot observation
     *
     * @return Returns the new ceres::AutoDiffCostFunction result
     */
    static ceres::CostFunction* Create(const Eigen::Vector3d kinectPoint, const Eigen::Vector3d robotPoint)
    {
        //return (new ceres::AutoDiffCostFunction<CostFunctor, 3, 3, 3>( new CostFunctor(kinectPoint, robotPoint)));
        return (new ceres::NumericDiffCostFunction<CostFunctor, ceres::CENTRAL, 3, 3, 3>( new CostFunctor(kinectPoint, robotPoint)));
    }

    Eigen::Vector3d kinectPoint;
    Eigen::Vector3d robotPoint;
};

/**
 * @brief Adds a new camera observation that will be input to the optimizer
 *
 * @param point The input point
 */
void add_camera_observation(Eigen::Vector3d point)
{
    camera_observations.push_back(point);
//    camera_observations.push_back(point(1));
//    camera_observations.push_back(point(2));
    numCameraObservations++;
}

/**
 * @brief Adds a new robot observation that will be input to the optimizer
 *
 * @param point The input point
 */
void add_robot_observation(Eigen::Vector3d point)
{
    robot_observations.push_back(point);
//    robot_observations.push_back(point(1));
//    robot_observations.push_back(point(2));
    numRobotObservations++;
}



/**
 * @brief Generates a random collection of camera and robot observations for testing purposes 
 */
void generate_points()
{
    // Get rid of any old observations
    camera_observations.clear();
    robot_observations.clear();

    numCameraObservations = 0;
    numRobotObservations = 0;

    for (int i = 0; i < numObservations; i++)
    {
        Eigen::Vector3d kinectPoint = Eigen::Vector3d::Random();  // Create random column vector
        Eigen::Vector3d robotPoint = calculateTransformedPoint(kinectPoint, rotation, translation); // Finds robot frame equivalent of generated camera observation

        // Add points into the observations vectors
        add_camera_observation(kinectPoint);
        add_robot_observation(robotPoint);
    }
}

/**
 * @brief Creates a random rotation and translation matrix
 */
void initalizeRandomRotationAndTranslationMatrices()
{
    // Rotations from 0 to pi
    rotation[0] = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 3.14;
    rotation[1] = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 3.14;
    rotation[2] = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 3.14;

    // Translations from 0 to 1
    translation[0] = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    translation[1] = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    translation[2] = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
}

/**
 * @brief Converts a Eigen::Vector3d to an array
 *
 * @param vector Input vector for conversion
 * @param &array Output array
 */
void vector3dToArray(Eigen::Vector3d vector, double (&array)[3])
{
    array[0] = vector(0);
    array[1] = vector(1);
    array[2] = vector(2);
}

/**
 * @brief Converts an array to an Eigen::Vector3d
 *
 * @param array[3] Input array for conversion
 *
 * @return Output vector
 */
Eigen::Vector3d arrayToVector3D(double array[3])
{
    Eigen::Vector3d point;
    point(0) = array[0];
    point(1) = array[1];
    point(2) = array[2];
    return point;
}

/**
 * @brief Prints a comparison between the original points and the transformed points
 */
void printTransformComparisons()
{
    ROS_INFO_STREAM("Printing point comparisons");
    for (int i = 0; i < numObservations; i++)
    {
        //Eigen::Vector3d point;
        //point <<    camera_observations[i];
                    //camera_observations[3*i + 1],
                    //camera_observations[3*i + 2]; 
        ROS_INFO_STREAM("Difference in correct and calculated point is\n" << calculateTransformedPoint(camera_observations[i], rotation, translation) - calculateTransformedPoint(camera_observations[i], rotationArray, translationArray)); 
     }
}

/**
 * @brief Calculates the Euclidean distance between two points
 *
 * @param point1 The first point
 * @param point2 The second point
 *
 * @return The distance between the two points
 */
double distanceBetweenPoints(Eigen::Vector3d point1, Eigen::Vector3d point2)
{
    return sqrt(pow(point1(0) - point2(0),2) + pow(point1(1) - point2(1),2) + pow(point1(2) - point2(2),2));
}

void printTransforms()
{
        myfile.open(filename.c_str(), std::ios::app);
        myfile << "Original rotation was [" << rotation[0] << "," << rotation[1] << "," << rotation[2] <<"]" << "\n";
        myfile << "Ceres-generated rotation is [" << rotationArray[0] << "," << rotationArray[1] << "," << rotationArray[2] << "]" << "\n";
        myfile << "Original translation was [" << translation[0] << "," << translation[1] << "," << translation[2] << "," << "\n";
        myfile << "Ceres-generated translation is [" << translationArray[0] << "," << translationArray[1] << "," << translationArray[2] << "]" << std::endl;
        myfile.close();
}

/**
 * @brief Checks if two transforms are equivalent
 *
 * @param rotation1[3] The rotation component of the first transform
 * @param translation1[3] The translation component of the first transform
 * @param rotation2[3] The rotation component of the second transform
 * @param translation2[3] The translation component of the second transform
 */
void equivalentTransforms()// double rotation1[3], double translation1[3], double rotation2[3], double translation2[3])
{
    double distance;

    for (int i = 0; i < numObservations; i++)
    {
        //Eigen::Vector3d point;
        //point <<    camera_observations[3*i + 0],
        //            camera_observations[3*i + 1],
        //            camera_observations[3*i + 2];

        Eigen::Vector3d point1 = calculateTransformedPoint(camera_observations[i], rotation, translation); //rotation1, translation1);
        Eigen::Vector3d point2 = calculateTransformedPoint(camera_observations[i], rotationArray, translationArray);//rotation2, translation2);

        distance =  distanceBetweenPoints(point1, point2);
        if (distance < margin)  // If the distance falls below the margin then count this as a successful calculation 
        {
            numPassed++;
        }
        myfile.open(filename.c_str(), std::ios::app);
        myfile << "Point in camera frame is\n" << camera_observations[i] << "\n";
        myfile << "Point transformed with original transform is\n" << point1 << "\n";
        myfile << "Point transformed with Ceres-generated transform is\n" <<  point2 << "\n\n";
        myfile.close();
    }
}

void solve()
{
    ceres::Problem problem;
    for (int j = 0 ; j < numObservations; j++)
    {
        ceres::CostFunction* cost_function = CostFunctor::Create(   camera_observations[j],
                                                                // camera_observations[3* j + 1],
                                                                // camera_observations[3*j + 2],
                                                                // robot_observations[3*j + 0],
                                                                // robot_observations[3*j + 1],
                                                                 robot_observations[j]);
        problem.AddResidualBlock(cost_function, NULL, rotationArray, translationArray) ;
    }
    
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
}



void runRandomTests()
{
    for (int i = 0; i < reps; i++)
    {
        initalizeRandomRotationAndTranslationMatrices();
        generate_points();

    //    ROS_INFO("Solving");
        solve();
     //   ROS_INFO("Solve complete");

      //  ROS_INFO("Printing transforms"); 
        printTransforms();
       // ROS_INFO("Print complete");

        //ROS_INFO("Printing equivalence");
        // Check if the calculated transfor is equivalent 
        equivalentTransforms();//rotation, translation, rotationArray, translationArray);
       // ROS_INFO("Print complete"); 

        // Print occasional output
        if (i % (reps / 10) == 0)
        {
            ROS_INFO_STREAM("Completed " << i << "/" << reps);
        }
    }
    
    // Output what percentage of points passed the calibration margin
    ROS_INFO_STREAM("Passed " << static_cast<double>(numPassed) / (numObservations * reps) * 100.0 << " \% within margin.");
}

void getKinectPoints()
{
    // Add logic here to save the coordinates of the kinect platform center in the frame of the kinect camera as an Eigen::Vector3d
}

void getRobotPoints()
{
    // Add logic here to save the coordinates of the kinect platform center in the robot torso frame as an Eigen::Vector3d
}

/**
 * @brief Calculates a transform after getting kinect and robot points
 */
void calculateTransform()
{
    getKinectPoints();
    getRobotPoints();
    solve();

    ROS_INFO_STREAM("Calculated rotation component of the transform is " << rotationArray);  
    ROS_INFO_STREAM("Calculated translation component of the transform is " << translationArray); 
    
    printTransforms();
}

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);  // Start loggin
    srand(time(0));  // Set seed for random number generation
    
    myfile.open(filename.c_str()); // Overwrite file if it already exists
    runRandomTests(); 

    return 0;
}
