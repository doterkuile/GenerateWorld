# Generate world package

This package generates different simple simulated worlds in gazebo. The worlds are used for performance check of Talos at uneven terrain. The worlds can be altered by changing [parameters](./config/world_parameters.yaml). The package will attempt to save the created gazebo file of the world in the package *pal\_gazebo\_worlds*. If the user does not have write permission for this package the file will be saved in the [worlds/](./worlds/) directory. This file then has to be copied manually to the *pal\_gazebo\_worlds* package.

## Random world

To create this world:
```
roslaunch generate_world random_world.launch
```

This launch file creates a world ([random_world.world](./worlds/random_world.world)) with obstacles with random size, position and orientation. The minimum and maximum values for size, position and orientation can be altered to create new worlds. Furtermore the number of obstacles can also be changed in [world_parameters.yaml](./config/world_parameters.yaml).

![random_world](./images/random_world.png#center)

## Stairs world

To create this world:
```
roslaunch generate_world stairs.launch
```

This launch file creates a simple stairs ([custom_stairs.world](./worlds/custom_stairs.world)). The number of steps and the size of the steps can be specified in [world_parameters.yaml](./config/world_parameters.yaml).

![custom_stairs](./images/custom_stairs.png#center)

## Stepping stones world

To create this world:
```
roslaunch generate_world stepping_stones.launch
```

This launch file creates a stepping stones world ([stepping_stones.world](./worlds/stepping_stones.world)). The number of steps and the size of the steps, in combination with the height difference can can be specified in [world_parameters.yaml](./config/world_parameters.yaml).

![stepping_stones](./images/stepping_stones.png#center)

