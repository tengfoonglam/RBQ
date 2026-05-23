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
--name=rbq-examples rbq-examples
```

Once the container is running, attach to it in a separate terminal using
```bash
docker exec -it rbq-examples bash
```
#### Remote VS Code Server

In order to use devcontainer offline, it is required to download the VS Code Server and set up a bind mount for that folder

Note the down the commit hash of the VS Code currently being used. Example:
```bash
export commit_id=cfbea10c5ffb233ea9177d34726e6056e89913dc
```

Download the VS Code server
```bash
curl -sSL "https://update.code.visualstudio.com/commit:${commit_id}/server-linux-x64/stable" -o vscode-server-linux-x64.tar.gz
```

Create the VS Code server folder and unzip contents into the correct location
```bash
mkdir -p ~/vscode-server-devcontainer/bin/${commit_id}
tar zxvf vscode-server-linux-x64.tar.gz -C ~/vscode-server-devcontainer/bin/${commit_id} --strip 1
touch ~/vscode-server-devcontainer/bin/${commit_id}/0`
```

If successful, the folder should look like the following
```
vscode-server-devcontainer
└── bin
    └── cfbea10c5ffb233ea9177d34726e6056e89913dc
        ├── 0
        ├── bin
        ...
```

`devcontainer.json` already creates a bind mount to the correct location and the devcontainer should initialise successfully even when there is no online connection.

#### Activating Pre-Commit Hooks

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

#### Other notes
- Some applications with have the following warning: `selected interface "lo" is not multicast-capable: disabling multicast`
  - To solve this, on a host terminal, run `sudo ip link set lo multicast on`
