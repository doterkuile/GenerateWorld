#ifndef BLOCK_H
#define BLOCK_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <tinyxml2.h>
#include <stdlib.h>
#include <random>




namespace GenerateWorld
{

struct BlockProperties
{
    Eigen::Vector3d orientationMin;
    Eigen::Vector3d orientationMax;

    Eigen::Vector3d positionMin;
    Eigen::Vector3d positionMax;

    Eigen::Vector3d sizeMin;
    Eigen::Vector3d sizeMax;

};




class Block
{
public:
        // Block constructor
        Block(const XmlRpc::XmlRpcValue &config, ros::NodeHandle &nh);
        ~Block();

        void setupParams();

        // Convert Euler angles to a rotation matrix
        Eigen::Matrix3d euler2Rotation(const Eigen::Vector3d &eulerAngles);
        void setPosition(const Eigen::Vector3d &position);
        void setOrientation(const Eigen::Vector3d &orientation);
        void setPose(const Eigen::Vector3d &position,const Eigen::Vector3d &orientation);
        void setSize(const Eigen::Vector3d &size);


        // Generate a random height (length, width, height) for an object
        void setRandomSize();

        // Generate a random position (x,y,z) for an object
        void setRandomPosition();

        // Generate a random orientation (roll, pitch) for an object
        void setRandomOrientation();

        // Generate a random vector within a certain range (min, max)
        Eigen::Vector3d generateRandomVector(const Eigen::Vector3d &minVector, const Eigen::Vector3d &maxVector);

        // Generate a random pose for a block
        void setRandomPose();

        Eigen::Vector3d getSize() const;
        Eigen::Vector3d getPosition() const;
        Eigen::Vector3d getOrientation() const;

private:


    XmlRpc::XmlRpcValue config_;
    std::string loadBlockFile_;

    ros::NodeHandle nh_;

    // Pose of the block
    Eigen::Isometry3d   blockPose_;

    // Orientation and position of the block
    Eigen::Vector3d     blockPosition_;
    Eigen::Vector3d     blockOrientation_;

    // Minimum and maximum values for the orientation of the block
    Eigen::Vector3d     blockOrientationMin_;
    Eigen::Vector3d     blockOrientationMax_;

    // Minimum and maximum values for the position of the block
    Eigen::Vector3d     blockPositionMin_;
    Eigen::Vector3d     blockPositionMax_;

    // Size of the block
    Eigen::Vector3d     blockSize_;

    // Minimum and maximum values of the size of the block
    Eigen::Vector3d     blockSizeMin_;
    Eigen::Vector3d     blockSizeMax_;

    // xmlDocument of a standard block
    tinyxml2::XMLDocument xmlBlock_;

    // Required xmlElements
    tinyxml2::XMLElement* nodeCollision_;
    tinyxml2::XMLElement* nodeVisual_;
    tinyxml2::XMLElement* nodePose_;
    tinyxml2::XMLElement* nodeName_;
    tinyxml2::XMLElement* nodeLinkName_;



    friend class World;

};

} // End namespace GenerateWorld
#endif // BLOCK_H
