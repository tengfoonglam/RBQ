### This project is provided by the [Rainbow-Robotics](https://rainbow-robotics.com/)

### Checkout [developer's guide](https://rainbowrobotics.github.io/RBQ/software/developers-guide.html)

## Docker Setup

#### Building the Container
In the git root directory, run the following
```bash
docker build --no-cache --file ./rbq_examples/scripts/docker/Dockerfile --network host -t rbq-examples .
```

#### Running the Container

Add the following lines to your host `.bashrc`
```bash
xhost si:localuser:root
sudo ip link set lo multicast on
```

To run the container
```bash
docker run \
-it \
--rm \
--cap-add SYS_ADMIN \
--device /dev/fuse \
--security-opt apparmor:unconfined \
--privileged \
-e DISPLAY=$DISPLAY \
-e NVIDIA_VISIBLE_DEVICES=all \
-e NVIDIA_DRIVER_CAPABILITIES=all \
--network=host \
--gpus=all \
--runtime nvidia \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ${PWD}:/workspace \
--name=rbq-examples rbq-examples-main 
```

Once the container is running, attach to it in a separate terminal using
```bash
docker exec -it rbq-examples bash
```

#### Running the Examples

To test that the container is working, launch the full simulation stack
```bash
./scripts/sim.bash --vision
```

In a separate terminal

Build the example
```bash
cd rbq_examples/
./scripts/build.bash 
```

Run the example
```bash
./rbq_examples/bin/rbq_example_level_0 -p /workspace/rbq_lab/policy/rbq10
```

Example notes:
 - Need to sit (press z) before standing for the first time
 - If robot crashes, just close Mujoco to respawn
 - Use the joystick in the GUI to control the robot when locomotion policy is loaded