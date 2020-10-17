#ifndef WORLD_H
#define WORLD_H
#include "Block.h"
#include <random> // for std::mt19937


namespace GenerateWorld
{



void XMLCheckresult(tinyxml2::XMLError &eResult);




class World
{

public:

    // GenerateWorld constructor
    World(ros::NodeHandle &nh);

    void setupParams();

    void setName(const std::string &name);


    void createRandomWorld();

    void createStairs();

    void createSteppingStones();

    // Generate blocks with random size, position and orientation
    void generateBlocks(Block &block, const int &ii);

    // Insert the generated block in an xml file xmlBlock_
    void insertBlocks(Block &block);

    // Add the xmlBlock_ file to xmlWorld_
    bool addBlock();

    // Print the xmlWorld_ file
    void print();

    bool checkCollision(const Block &block);

    // Save xmlWorld_ to filename
    void saveFile();

    void addBlocktoVector(const Block &block);

    double getEuclidean(const Eigen::Vector3d &positionBlockA, const Eigen::Vector3d &positionBlockB);

private:

    ros::NodeHandle nh_;

    XmlRpc::XmlRpcValue config_;

    // xmlDocument of the created world
    tinyxml2::XMLDocument xmlWorld_;

    // Initialize meresse twister with random seed based on the clock:
    static std::mt19937 Mersenne_;

    // Vectors of sizes and positions of generated blocks
    std::vector<Eigen::Vector3d>  sizes_;
    std::vector<Eigen::Vector3d>  positions_;

    // Parameters
    int nrOfBlocks_;
    std::string loadWorldFile_;
    std::string saveWorldPath_;
    std::string backupSavePath_;
    std::string worldName_;


    // stair parameters
    Eigen::Vector3d origin_;
    Eigen::Vector3d stepSize_;
    int nrOfSteps_;

    // stepping stones parameters
    int nrOfSteppingStones_;
    Eigen::Vector3d steppingStoneSize_;
    double heightDifference_;
    double stepHeight_;

    friend class Block;
};



} // End namespace

#endif // WORLD_H
