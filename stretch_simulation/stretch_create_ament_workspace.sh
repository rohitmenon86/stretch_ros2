#!/bin/bash

# Resolve workspace root: go 2 levels up from where script is run
WS_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SRC_DIR="$WS_ROOT/src"

set -e

if [[ $EUID = 0 ]]; then
   echo "Please run this script without sudo."
   exit 1
fi

# stretch_install repo
if [ ! -d "$HOME/stretch_install" ]; then
    git clone https://github.com/hello-robot/stretch_install.git --depth 1 $HOME/stretch_install
fi

LOCALROBOT_NAME="stretch-se3-local"
sudo mkdir -p /etc/hello-robot
echo "HELLO_FLEET_ID=$LOCALROBOT_NAME" | sudo tee /etc/hello-robot/hello-robot.conf

. /etc/hello-robot/hello-robot.conf

echo "###########################################"
echo "NEW INSTALLATION OF USER SOFTWARE"
echo "###########################################"
echo "Updating $HOME/.bashrc dotfile..."

# Helper: append only if not already present
append_if_missing() {
    local LINE="$1"
    local FILE="$HOME/.bashrc"
    grep -qxF "$LINE" "$FILE" || echo "$LINE" >> "$FILE"
}

append_if_missing ""
append_if_missing "######################"
append_if_missing "# STRETCH BASHRC SETUP"
append_if_missing "######################"
append_if_missing "export HELLO_FLEET_PATH=${HOME}/stretch_user"
append_if_missing "export HELLO_FLEET_ID=${HELLO_FLEET_ID}"
append_if_missing "export PATH=\${PATH}:$HOME/.local/bin"
append_if_missing "export LRS_LOG_LEVEL=None #Debug"
append_if_missing "export PYTHONWARNINGS='ignore:setup.py install is deprecated,ignore:Invalid dash-separated options,ignore:pkg_resources is deprecated as an API,ignore:Usage of dash-separated'"

# Use workspace root instead of hardcoded ~/ament_ws
append_if_missing "export _colcon_cd_root=$WS_ROOT"
append_if_missing "source /opt/ros/humble/setup.bash"


echo "Creating repos and stretch_user directories..."
mkdir -p $HOME/.local/bin
mkdir -p $HOME/repos
mkdir -p $HOME/stretch_user/log
mkdir -p $HOME/stretch_user/debug
mkdir -p $HOME/stretch_user/maps
mkdir -p $HOME/stretch_user/models
mkdir -p $HOME/stretch_user/$LOCALROBOT_NAME

# Ensure config files exist
touch $HOME/stretch_user/$LOCALROBOT_NAME/stretch_configuration_params.yaml
touch $HOME/stretch_user/$LOCALROBOT_NAME/stretch_user_params.yaml

# Write default config if empty
if ! grep -q "model_name" "$HOME/stretch_user/$LOCALROBOT_NAME/stretch_configuration_params.yaml"; then
cat <<EOF > $HOME/stretch_user/$LOCALROBOT_NAME/stretch_configuration_params.yaml
robot:
  model_name: SE3
EOF
fi

echo "Setting up user copy of robot factory data (if not already there) at $HOME/stretch_user/$HELLO_FLEET_ID"

sudo chown -R : $HOME/stretch_user
sudo chmod -R a-x,o-w,+X $HOME/stretch_user

export PATH=${PATH}:$HOME/.local/bin
export HELLO_FLEET_ID=$HELLO_FLEET_ID
export HELLO_FLEET_PATH=$HOME/stretch_user

echo "Ensuring correct version of params present..."
params_dir_path=$HELLO_FLEET_PATH/$HELLO_FLEET_ID
echo "Checking params directory path: $params_dir_path"

# Define file paths based on the provided directory
user_params="$params_dir_path/stretch_re1_user_params.yaml"
factory_params="$params_dir_path/stretch_re1_factory_params.yaml"
new_config_params="$params_dir_path/stretch_configuration_params.yaml"
new_user_params="$params_dir_path/stretch_user_params.yaml"

echo "###########################################"
echo "USING EXISTING ROS2 WORKSPACE at $WS_ROOT"
echo "###########################################"

echo "Ensuring correct version of ROS is sourced..."
if [[ $ROS_DISTRO && ! $ROS_DISTRO = "humble" ]]; then
    echo "Cannot setup workspace while a conflicting ROS version is sourced. Exiting."
    exit 1
fi
source /opt/ros/humble/setup.bash

# make sure src exists
mkdir -p "$SRC_DIR"

echo "Downgrade to numpy 1.26.4..."
pip3 install numpy==1.26.4

export PATH=${PATH}:~/.local/bin
echo "Updating rosdep indices..."
rosdep update --include-eol-distros

echo "Cloning the workspace's packages..."
cd "$SRC_DIR"
vcs import --input ~/stretch_install/factory/22.04/stretch_ros2_humble.repos

echo "Fetch ROS packages' dependencies (this might take a while)..."
cd "$WS_ROOT"
rosdep install --rosdistro=humble -iy --skip-keys="librealsense2 realsense2_camera" --from-paths src
sudo apt remove -y ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs
pip3 cache purge

echo "Install web interface dependencies..."
cd "$SRC_DIR/stretch_web_teleop"
pip3 install -r requirements.txt
npm install --force
npx playwright install

echo "Generating web interface certs..."
cd "$SRC_DIR/stretch_web_teleop/certificates"
curl -JLO "https://dl.filippo.io/mkcert/latest?for=linux/amd64"
chmod +x mkcert-v*-linux-amd64
sudo cp mkcert-v*-linux-amd64 /usr/local/bin/mkcert
CAROOT=`pwd` mkcert --install
mkdir -p ~/.local/share/mkcert
rm -rf ~/.local/share/mkcert/root*
cp root* ~/.local/share/mkcert
mkcert ${HELLO_FLEET_ID} ${HELLO_FLEET_ID}.local ${HELLO_FLEET_ID}.dev localhost 127.0.0.1 0.0.0.0 ::1
rm mkcert-v*-linux-amd64
cd "$SRC_DIR/stretch_web_teleop"
touch .env
echo certfile=${HELLO_FLEET_ID}+6.pem >> .env
echo keyfile=${HELLO_FLEET_ID}+6-key.pem >> .env
cd "$WS_ROOT"

set +e

echo "###########################################"
echo "INSTALLATION OF USER LEVEL PIP3 PACKAGES"
echo "###########################################"
echo "Clear pip cache"
python3 -m pip cache purge
echo "Upgrade pip3"
python3 -m pip -q install --no-warn-script-location --user --upgrade pip
echo "Install Stretch Body"
python3 -m pip -q install --no-warn-script-location --upgrade hello-robot-stretch-body
echo "Install Stretch Body Tools"
python3 -m pip -q install --no-warn-script-location --upgrade hello-robot-stretch-body-tools
echo "Install Stretch Factory"
python3 -m pip -q install --no-warn-script-location --upgrade hello-robot-stretch-factory
echo "Install Stretch Tool Share"
python3 -m pip -q install --no-warn-script-location --upgrade hello-robot-stretch-tool-share
echo "Install Stretch Diagnostics"
python3 -m pip -q install --no-warn-script-location --upgrade hello-robot-stretch-diagnostics
echo "Install Stretch URDF"
python3 -m pip -q install --no-warn-script-location --upgrade hello-robot-stretch-urdf
echo "Upgrade prompt_toolkit"
python3 -m pip -q install --no-warn-script-location -U prompt_toolkit
pip3 install setuptools==59.6.0
pip3 install numpy==1.26.4
echo "Remove setuptools-scm"
python3 -m pip -q uninstall -y setuptools-scm
echo ""

set -e

sudo add-apt-repository ppa:sweptlaser/python3-pcl
sudo apt update
sudo apt install python3-pcl

cd "$WS_ROOT"
sudo rosdep init
rosdep update
rosdep install --rosdistro=humble -iy --skip-keys="librealsense2 realsense2_camera" --from-paths src
colcon build
source ./install/setup.bash
