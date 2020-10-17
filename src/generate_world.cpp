#include "world.h"

// Generate random seed
std::mt19937 GenerateWorld::World::Mersenne_{
    static_cast<std::mt19937::result_type>(std::time(nullptr))};

int main(int argc, char** argv)
{

  // Set up node
  ros::init(argc, argv, "generate_world");
  ros::NodeHandle nh("~");

  // Create world
  GenerateWorld::World world(nh);
  std::string worldName = "random_world.world";
  world.setName(worldName);
  world.createRandomWorld();

  // Print world
  //    world.print();

  // Save world to xmlfile
  world.saveFile();

  std::cout << "File has executed\n";
  return (0);
}
