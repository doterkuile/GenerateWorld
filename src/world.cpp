#include "world.h"

namespace GenerateWorld
{


// Constructor: set up default world
World::World(ros::NodeHandle &nh):
  nh_(nh)
{
    this->setupParams();
    const char* filePtr = loadWorldFile_.c_str();
    // Open standard world xml-file
    tinyxml2::XMLError eOpenWorld = xmlWorld_.LoadFile(filePtr);
    XMLCheckresult(eOpenWorld);


}

void World::setupParams()
{
  nh_.getParam("/generate_world_config", config_);
  nh_.getParam("/generate_world/filenameWorld", loadWorldFile_);
  nh_.getParam("/generate_world/savefileWorld",saveWorldPath_);
  nh_.getParam("/generate_world/backupSaveFile",backupSavePath_);

  XmlRpc::XmlRpcValue randomWorld = config_["random_world"];
  XmlRpc::XmlRpcValue stairs = config_["stairs"];
  XmlRpc::XmlRpcValue steppingStones = config_["stepping_stones"];



//  std::cout << "config_nrofblocks" << config_["nrOfBlocks"] << '\n';
  nrOfBlocks_ = randomWorld["nrOfBlocks"];


  nrOfSteps_ = stairs["nr_of_steps"];
  origin_ = Eigen::Vector3d(stairs["origin"]["x"],stairs["origin"]["y"], 0.0);
  stepSize_ = Eigen::Vector3d(stairs["size"]["x"],stairs["size"]["y"],stairs["size"]["z"]);

  // Stepping Stones
  nrOfSteppingStones_ = steppingStones["nr_of_steps"];
  steppingStoneSize_ = Eigen::Vector3d(steppingStones["size"]["x"],steppingStones["size"]["y"],steppingStones["size"]["z"]);
  heightDifference_ = steppingStones["height_difference"];
  stepHeight_ = steppingStones["step_height"];

}

void World::setName(const std::string &name)
{
  worldName_ = name;
}

void World::createRandomWorld()
{
    for (int ii{0}; ii < nrOfBlocks_; ++ii)
    {
        // Construct block
        Block block(config_, nh_);



        // Create random values for the size and pose of the block
        block.setRandomPose();
        block.setRandomSize();

        // Check if the new created block collides with previous blocks
        while (this->checkCollision(block))
        {

            block.setRandomPose();
            block.setRandomSize();

        }

        // Generate random pose and size for each block and insert them in xmlBlock
        this->generateBlocks(block,ii);


        // Insert size and position of new block in vector
        positions_.push_back(block.getPosition());
        sizes_.push_back(block.getSize());

        // Insert xmlBlock in xmlWorld
        this->insertBlocks(block);
    }
    std::cout << "Finished generating blocks\n";

}

void World::createStairs()
{

    Eigen::Vector3d position = Eigen::Vector3d(origin_.x(), origin_.y(), stepSize_.z()/2.0);
    Eigen::Vector3d size = stepSize_;
    for (int ii{0}; ii < nrOfSteps_; ++ii)
    {
        // Construct block
        Block block(config_, nh_);



        // Generate random pose and size for each block and insert them in xmlBlock
        block.setSize(size);
        block.setPosition(position);
        block.setOrientation(Eigen::Vector3d(0.0,0.0,0.0));

        this->generateBlocks(block,ii);


        // Insert xmlBlock in xmlWorld
        this->insertBlocks(block);

        size += Eigen::Vector3d(0.0,0.0, stepSize_.z()/2.0);
        position += Eigen::Vector3d(stepSize_.x(), 0.0, stepSize_.z()/2.0);
    }
    std::cout << "Finished generating blocks\n";
}


void World::createSteppingStones()
{
    Eigen::Vector3d size = steppingStoneSize_;
    Eigen::Vector3d position = Eigen::Vector3d(size.x()/2.0, size.y() / 2.0, stepHeight_ - 3.0/4.0 * heightDifference_ - size.z()/2.0);

    // Set starting block
    Block startBlock(config_, nh_);
    startBlock.setSize(Eigen::Vector3d(size.x(), 2.0 * size.y(), 0.05));
    startBlock.setPosition(Eigen::Vector3d(position.x() - size.x(), 0 , stepHeight_ - size.z()/2.0));
    startBlock.setOrientation(Eigen::Vector3d(0.0,0.0,0.0));
//    generate block with unique id
    this->generateBlocks(startBlock,nrOfSteppingStones_ * 2);
    // Insert xmlBlock in xmlWorld
    this->insertBlocks(startBlock);


    startBlock.setSize(Eigen::Vector3d(size.x(), 2.0 * size.y(), 0.05));
    startBlock.setPosition(Eigen::Vector3d(position.x() - 2.0 * size.x(), 0 , stepHeight_/2.0 -size.z()/2.0));
    startBlock.setOrientation(Eigen::Vector3d(0.0,0.0,0.0));
//    generate block with unique id
    this->generateBlocks(startBlock,nrOfSteppingStones_ * 2 + 2);
    // Insert xmlBlock in xmlWorld
    this->insertBlocks(startBlock);



    for (int ii{0}; ii < nrOfSteppingStones_ * 2; ++ii)
    {
        // Construct block
        Block block(config_, nh_);

        // Generate pose and size for each block and insert them in xmlBlock
        block.setSize(size);
        block.setPosition(position);

        block.setOrientation(Eigen::Vector3d(0.0,0.0,0.0));

        this->generateBlocks(block,ii);


        // Insert xmlBlock in xmlWorld
        this->insertBlocks(block);


        position.y() = -position.y();
        if(ii % 2 == 0)
        {
            if(position.z() > stepHeight_)
            {
                position.z() -=  heightDifference_;
            } else
            {
                position.z() +=  heightDifference_;

            }
        }

        if(ii % 2 == 1)
        {
            position.x() += size.x();
        }
    }
    // Set finishing block block
        startBlock.setPosition(Eigen::Vector3d(position.x(), 0 , stepHeight_ - size.z()/2.0));
        startBlock.setOrientation(Eigen::Vector3d(0.0,0.0,0.0));
    //    generate block with unique id
        this->generateBlocks(startBlock,nrOfSteppingStones_ * 2 + 1);
        // Insert xmlBlock in xmlWorld
        this->insertBlocks(startBlock);

    std::cout << "Finished generating blocks\n";
}


bool World::checkCollision(const Block &block)
{


    if(sizes_.size() == 0)
    {
      return 0;
    }
    
    // Loop through existing blocks
    for (size_t ii{0}; ii <sizes_.size(); ++ii)
    {
        // Calculate euclidean distance of new block with respect to existing blocks
        double euclideandistance{this->getEuclidean(positions_[ii], block.getPosition())};

        // Calculate minimum distance new block required between new block and existing blocks
        double minDist{sizes_[ii].maxCoeff()/2 + block.getSize().maxCoeff()/2-0.05};

        // Check if distance is large enough
        if (euclideandistance < minDist)
            return 1;
    }

    return 0;
}


double World::getEuclidean(const Eigen::Vector3d &positionBlockA, const Eigen::Vector3d &positionBlockB)
{
    return std::sqrt(std::pow((positionBlockA[0]-positionBlockB[0]),2) + std::pow((positionBlockA[1]-positionBlockB[1]),2));
}


void World::print()
{
    xmlWorld_.Print();
}



void World::insertBlocks(Block &block)
{
     //Select right element in xmlWorld
    tinyxml2::XMLElement *nodeWorld{xmlWorld_.RootElement()->FirstChildElement()};

    // Copy block node from xmlBlock to xmlWorld
    tinyxml2::XMLNode *nodeModelClone{block.xmlBlock_.RootElement()->DeepClone(&xmlWorld_)};
    nodeWorld->InsertEndChild(nodeModelClone);

}

void World::saveFile()
{
    std::string filename = saveWorldPath_ + worldName_;
    std::string backupSave = backupSavePath_ + worldName_;


    tinyxml2::XMLError eSave = xmlWorld_.SaveFile(filename.c_str());
    std::cout << "Saving saved_world.world: \n";
    std::cout << "saving file to " << filename << '\n';
    XMLCheckresult(eSave);
    tinyxml2::XMLError eSave2 = xmlWorld_.SaveFile(backupSave.c_str());
    if(eSave != 0)
    {

      ROS_WARN_STREAM("Could not save inside pal_gazebo_world package because user has no write permission. Saving in this package instead. "
                      "Manually copy the file saved_world.world from generate_world/worlds/ to pal_gazebo_world/worlds/ ");
    }

}


void World::generateBlocks(Block &block, const int &ii)
{

    // Insert the random size values in the right place in xml_block
    std::string size{std::to_string(block.getSize().x()) + " "
                   + std::to_string(block.getSize().y()) + " "
                   + std::to_string(block.getSize().z())};

    block.nodeCollision_->SetText(size.c_str());
    block.nodeVisual_->SetText(size.c_str());

    // Insert the random pose values in the right place in xml_block
    std::string pose{std::to_string(block.getPosition().x()) + " "
                   + std::to_string(block.getPosition().y()) + " "
                   + std::to_string(block.getPosition().z()) + " "
                   + std::to_string(block.getOrientation().x()) + " "
                   + std::to_string(block.getOrientation().y()) + " "
                   + std::to_string(block.getOrientation().z()) };

    block.nodePose_->SetText(pose.c_str());

    // Give the right names to the attributes in xmlBlock
    std::string nameLink{"rectangle_" + std::to_string(ii) + "_body"};
    std::string name{"rectangle_" + std::to_string(ii)};

    block.nodeLinkName_->SetAttribute("name", nameLink.c_str());
    block.nodeName_->SetAttribute("name",name.c_str());

}

}
