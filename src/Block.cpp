#include "Block.h"
#include "world.h"

namespace GenerateWorld
{
void XMLCheckresult(tinyxml2::XMLError& eResult)
{
  if (eResult != tinyxml2::XML_SUCCESS)
  {
    printf("Error is: %i\n", eResult);
  }
}


Block::Block(const XmlRpc::XmlRpcValue &config, ros::NodeHandle &nh) :
config_(config),
nh_(nh)
{
   this->setupParams();

    tinyxml2::XMLError eOpenBlock = xmlBlock_.LoadFile(loadBlockFile_.c_str());
    XMLCheckresult(eOpenBlock);

    nodeName_ = this->xmlBlock_.RootElement();
    nodeCollision_ = this->xmlBlock_.RootElement()->FirstChildElement("link")
                       ->FirstChildElement("collision")->FirstChildElement()
                       ->FirstChildElement()->FirstChildElement();
    nodeVisual_ = this->xmlBlock_.RootElement()->FirstChildElement("link")
                    ->FirstChildElement("visual")->FirstChildElement()
                    ->FirstChildElement()->FirstChildElement();
    nodePose_ = this->xmlBlock_.RootElement()->FirstChildElement();
    nodeLinkName_ = this->xmlBlock_.RootElement()->FirstChildElement("link");
}

void Block::setupParams()
{
  nh_.getParam("/generate_world/filenameBlock",loadBlockFile_);


  XmlRpc::XmlRpcValue randomWorld = config_["random_world"];


  blockSizeMin_ =  Eigen::Vector3d(randomWorld["block_size"]["min"]["x"],
                                   randomWorld["block_size"]["min"]["y"],
                                   randomWorld["block_size"]["min"]["z"]);

  blockSizeMax_ =  Eigen::Vector3d(randomWorld["block_size"]["max"]["x"],
                                   randomWorld["block_size"]["max"]["y"],
                                   randomWorld["block_size"]["max"]["z"]);

  blockPositionMin_ =  Eigen::Vector3d(randomWorld["block_position"]["min"]["x"],
                                       randomWorld["block_position"]["min"]["y"],
                                      randomWorld["block_position"]["min"]["z"]);

  blockPositionMax_ =  Eigen::Vector3d(randomWorld["block_position"]["max"]["x"],
                                       randomWorld["block_position"]["max"]["y"],
                                       randomWorld["block_position"]["max"]["z"]);

  blockOrientationMin_ =  -Eigen::Vector3d(randomWorld["angleParam"]["x"],
                                           randomWorld["angleParam"]["y"],
                                           randomWorld["angleParam"]["z"]);

  blockOrientationMax_ =  Eigen::Vector3d(randomWorld["angleParam"]["x"],
                                          randomWorld["angleParam"]["y"],
                                          randomWorld["angleParam"]["z"]);




}




void Block::setPosition(const Eigen::Vector3d &position)
{
    blockPosition_ = position;
}

void Block::setOrientation(const Eigen::Vector3d &orientation)
{
    blockOrientation_ = orientation;
}

void Block::setPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation)
{
    this->setPosition(position);
    this->setOrientation(orientation);
    blockPose_.linear() = this->euler2Rotation(blockOrientation_);
    blockPose_.translation() = blockPosition_;
}

void Block::setSize(const Eigen::Vector3d &size)
{
    blockSize_ = size;
}


void Block::setRandomPose()
{
    this->setRandomPosition();
    this->setRandomOrientation();
    blockPose_.linear() = this->euler2Rotation(blockOrientation_);
    blockPose_.translation() = blockPosition_;

}

void  Block::setRandomOrientation()
{
    blockOrientation_ = this->generateRandomVector(blockOrientationMin_, blockOrientationMax_);
}


void  Block::setRandomPosition()
{
    blockPosition_ = this->generateRandomVector(blockPositionMin_, blockPositionMax_);
}

void Block::setRandomSize()
{
    blockSize_ = this->generateRandomVector(blockSizeMin_, blockSizeMax_);
}


// Generate random number
Eigen::Vector3d Block::generateRandomVector(const Eigen::Vector3d &minVector, const Eigen::Vector3d &maxVector)
{    // construct a trivial random generator engine from a time-based seed:
      std::uniform_real_distribution<double> distribution1 (minVector[0],maxVector[0]);
      std::uniform_real_distribution<double> distribution2 (minVector[1],maxVector[1]);
      std::uniform_real_distribution<double> distribution3 (minVector[2],maxVector[2]);

      Eigen::Vector3d randomVector{distribution1(World::Mersenne_),distribution2(World::Mersenne_),distribution3(World::Mersenne_)};
      return randomVector;

}


Eigen::Vector3d Block::getSize() const
{
  return blockSize_;
}

Eigen::Vector3d Block::getPosition() const
{
 return blockPosition_;
}

Eigen::Vector3d Block::getOrientation() const
{
 return blockOrientation_;
}


Block::~Block()
{
    nodeCollision_ = nullptr;
    nodeVisual_ = nullptr;
    nodePose_ = nullptr;
    nodeName_ = nullptr;
    nodeLinkName_ = nullptr;
}


Eigen::Matrix3d Block::euler2Rotation(const Eigen::Vector3d &eulerAngles)
{

    Eigen::Matrix3d m {Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitX())
                     * Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitZ())};

    return m;
}


} //End namespace
