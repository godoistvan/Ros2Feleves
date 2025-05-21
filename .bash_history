locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py talker
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev python3-osrf-pycommon libasound2-dev libgl1-mesa-dev xorg-dev python3-vcstool python3-colcon-common-extensions python3-pykdl python3-pyudev libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev ros-humble-joint-state-publisher* ros-humble-xacro gfortran-9
sudo apt update
sudo apt install terminator
code-install-extension ms-vscode-remote.remote-wsl
'code-install-extension ms-vscode-remote.remote-wsl'
code -install-extension ms-vscode-remote.remote-wsl
code -install-extension ms-vscode-remote.remote-wsl
code-install-extension ms-vscode-remote.remote-wsl
sudo snap install code
code -install-extension ms-vscode-remote.remote-wsl
code .
.code
. code
code
. code
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py talker
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev python3-osrf-pycommon libasound2-dev libgl1-mesa-dev xorg-dev python3-vcstool python3-colcon-common-extensions python3-pykdl python3-pyudev libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev ros-humble-joint-state-publisher* ros-humble-xacro gfortran-9
sudo apt update
sudo apt install terminator
source ~/ros2_ws/install/setup.bash
code .
cd ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
git config --global credential.helper store
cd ~/ros2_ws/src/ros2_course
git init
git add .
git commit -m "Initial commit"
git branch -M main
git remote add origin <REPO_GITHUB_ADDRESS>.git
git push -u origin main
git config --global user.email godopisti@gmail.com
git config --global user.name Godo Istvan
cd ~/ros2_ws/src/ros2_course
git init
git add .
git commit -m "Initial commit"
git branch -M main
git remote add origin https://github.com/godoistvan/Ros2Feleves.git
git push -u origin main 
git add .
git commit -m "Add README"
git push
echo ".git-credentials" >> .gitignore
git add .gitignore
git rm --cached .git-credentials
git commit -m "Stop tracking .git-credentials and ignore it"
git push
git rebase -i --root 
git rm --cached .git-credentials   # remove it from that commit
git commit --amend --no-edit       # rewrite the commit without the secret
git rebase --continue              # let Git re-play the rest of the commits
bash git restore --staged 
git rebase -i --root
bash git stash push -u -m "temp-save" 
git restore --staged
git status
git stash push -u -m "temp save before rebase"
git stash pop
git push
git add .gitignore
git commit -m "Ignore VSCode log files"
pip install --user git-filter-repo
git filter-repo --path .git-credentials --invert-paths --force
rm -rf .git
git init
echo ".git-credentials" >> .gitignore
# add any other patterns you need (build/, install/, .vscode-server/, etc.)
git add .
git commit -m "Initial commit"
git branch -M main
git remote add origin https://github.com/godoistvan/Ros2Feleves.git
git push -u origin main --force
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/ros2/examples src/examples -b humble
colcon build --symlink-install
colcon test
source install/setup.bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
pip install --user git-filter-repo
sudo apt install python3-pip
git filter-repo --path .git-credentials --invert-paths --force
colcon build --symlink-install
cd feleves
cd src
colcon build --symlink-install
cd ros2_ws
ros2 run ros2_course koch_snowflake
cd ro2_course
cd ros2_ws
ros2 run ros2_course koch_snowflake
ros2 pkg list | grep ros2_course
ros2 run turtlesim turtlesim_node
cd ~/ros2_ws
colcon build --symlink-install --packages-select ros2_course
# *then*, in the same terminal:
source install/setup.bash
ros2 pkg list | grep ros2_course
ros2 run ros2_course koch_snowflake
cd ~/ros2_ws
colcon build --symlink-install --packages-select ros2_course
source install/setup.bash
ros2 pkg executables ros2_course
ros2 run ros2_course koch_snowflake
ros2 run turtlesim turtlesim_node
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --node-name hello ros2_course
git add .
git commit -m "Start"
git push
cd ~/ros2_ws
colcon build --symlink-install
chmod +x src/ros2_course/ros2_course/feleves.py
colcon build --symlink-install --packages-select ros2_course turtlesim
source install/setup.bash
ros2 run ros2_course koch_snowflake
cd ros2_ws
ros2 run ros2_course koch_snowflake
cd ros2_course
ros2 run ros2_course koch_snowflake
ros2 run turtlesim turtlesim_node
colcon build --symlink-install --packages-select ros2_course
source install/setup.bash
ros2 pkg list | grep ros2_course
ros2 pkg executables ros2_course
ros2 run ros2_course koch_snowflake
colcon build --symlink-install --packages-select ros2_course
source install/setup.bash
ros2 run ros2_course koch_snowflake # term 2
colcon build --symlink-install --packages-select ros2_course
ros2 run ros2_course koch_snowflake # term 2
colcon build --symlink-install --packages-select ros2_course
ros2 run ros2_course koch_snowflake # term 2
colcon build --symlink-install --packages-select ros2_course
ros2 run ros2_course koch_snowflake # term 2rclpy.init(args=args)
    node = KochSnowflake()
    node.get_logger().info('Starting in 1 second…')
    time.sleep(1.0)

    # depth=2, side_length=5 spans ~8.9 units
    node.run(side_length=5.0, depth=2)

    node.get_logger().info('Done drawing!')
    node.destroy_node()
    rclpy.shutdown()
colcon build --symlink-install --packages-select ros2_course
ros2 run ros2_course koch_snowflake    # term 2
colcon build --symlink-install --packages-select ros2_course
ros2 run ros2_course koch_snowflake    # term 2
colcon build --symlink-install --packages-select ros2_course
ros2 run ros2_course koch_snowflake    # term 2
colcon build --symlink-install --packages-select ros2_course
ros2 run ros2_course koch_snowflake    # term 2
colcon build --symlink-install --packages-select ros2_course
ros2 run ros2_course koch_snowflake    # term 2
cd ~/ros2_ws
colcon build --symlink-install --packages-select ros2_course
source install/setup.bash
ros2 run ros2_course koch_snowflake
colcon build --symlink-install --packages-select ros2_course
ros2 run ros2_course koch_snowflake
colcon build --symlink-install --packages-select ros2_course
ros2 run ros2_course koch_snowflake
colcon build --symlink-install --packages-select ros2_course
ros2 run ros2_course koch_snowflake
cd ~/ros2_ws
colcon build --symlink-install --packages-select ros2_course
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc # Apply to current terminal
ros2 run ros2_course koch_snowflake
ros2 service list
ros2 service type /reset
ros2 service call /reset std_srvs/srv/Empty {}
source ~/ros2_ws/install/setup.bash
ros2 run turtlesim turtlesim_node
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
code .
pip install rclpy
cd ~/ros2_ws # Or your workspace root
rm -rf build/ros2_course install/ros2_course log/ros2_course
colcon build --symlink-install --packages-select ros2_course
source ~/ros2_ws/install/setup.bash
ros2 run ros2_course koch_snowflake
colcon build --symlink-install --packages-select ros2_course
source ~/ros2_ws/install/setup.bash
ros2 run ros2_course koch_snowflake
colcon build --symlink-install --packages-select ros2_course
source ~/ros2_ws/install/setup.bash
ros2 run ros2_course koch_snowflake
colcon build --symlink-install --packages-select ros2_course
source ~/ros2_ws/install/setup.bash
ros2 run ros2_course koch_snowflake
source ~/ros2_ws/install/setup.bash
ros2 run turtlesim turtlesim_node
source ~/ros2_ws/install/setup.bash
ros2 run turtlesim turtlesim_node
cd ~/ros2_ws
colcon build --packages-select ros2_course
source ~/ros2_ws/install/setup.bash
ros2 run turtlesim turtlesim_node
source ~/ros2_ws/install/setup.bash # If it's a new terminal
ros2 run ros2_course koch_snowflake
ros2 node list
ros2 param list /koch_snowflake
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
colcon build --packages-select ros2_course
source install/setup.bash
ros2 run turtlesim turtlesim_node
pkill turtlesim_node   # or just close the window / Ctrl-C
ros2 run turtlesim turtlesim_node
# Terminal 1
pkill turtlesim_node   # or just close the window / Ctrl-C
ros2 run turtlesim turtlesim_node
# Terminal 1  –  keep it open
ros2 run turtlesim turtlesim_node
ros2 run ros2_course koch_snowflake --ros-args -p order:=4 -p side_length:=5.0
ros2 run ros2_course koch_snowflake -p order:=4 -p side_length:=6.0
ros2 service list | grep teleport
# Terminal 2  (same workspace sourced!)
ros2 run ros2_course koch_snowflake -p order:=4 -p side_length:=6.0
ros2 run ros2_course koch_snowflake         --ros-args         -p order:=4         -p side_length:=6.0 
# In *any* terminal
pkill -f koch_snowflake        # if it is still running
pkill -f turtlesim_node        # optional, just in case
ros2 daemon stop
ros2 daemon start              # clear the ROS 2 discovery cache
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 service list | grep teleport
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_course koch_snowflake         --ros-args         -p order:=4         -p side_length:=6.0     # ~6 m fills most of the window
godoistvan@DESKTOP-4F4225M:~$ source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
godoistvan@DESKTOP-4F4225M:~$ ros2 run ros2_course koch_snowflake         --ros-args         -p order:=4         -p side_length:=6.0
cd ~/ros2_ws
colcon build --packages-select ros2_course
source install/setup.bash          # VERY important: refresh PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_course koch_snowflake --ros-args --log-level debug -p order:=4 -p side_length:=6.0
cd ~/ros2_ws
colcon build --packages-select ros2_course
source install/setup.bash          # VERY important: refresh PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run turtlesim turtlesim_node
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_course koch_snowflake --ros-args         --log-level debug         -p order:=4         -p side_length:=6.0
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_course koch_snowflake --ros-args --log-level debug -p order:=4 -p side_length:=6.0
# ── Terminal, anywhere ────────────────────────────────────────────────────
cd ~/ros2_ws                       # top of your ROS 2 workspace
colcon build --packages-select ros2_course
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
# launch the canvas
ros2 run turtlesim turtlesim_node
cd ~/ros2_ws
colcon build --packages-select ros2_course
# every terminal must first source the ROS 2 distro and your workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
# launch the canvas
ros2 run turtlesim turtlesim_node
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_course koch_snowflake --ros-args         -p order:=4 -p side_length:=6.0
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_course koch_snowflake --ros-args --log-level debug -p order:=4 -p side_length:=6.0
ros2 run ros2_course koch_snowflake --ros-args -p order:=4 -p side_length:=6.0
ros2 run ros2_course koch_snowflake --ros-args -p order:=3 -p side_length:=8.0
ros2 node list
# → /koch_snowflake  /turtlesim
ros2 param get /koch_snowflake side_length
ros2 param get /koch_snowflake side_length
ros2 node list
ros2 param get /koch_snowflake side_length
ros2 run turtlesim turtlesim_node
colcon build --symlink-install
git add .
git commit -m "First version"
git push
ros2 run turtlesim turtlesim_node
colcon build --symlink-install
source /otp/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash # Adjust if your workspace name is different
source ~/ros2_ws/install/setup.bash
ros2 run ros2_course koch_snowflake --ros-args         -p order:=3 -p side_length:=5.0
ros2 run ros2_course koch_snowflake --ros-args -p order:=3 -p side_length:=5.0
colcon build --symlink-install
ros2 run ros2_course koch_snowflake --ros-args -p order:=3 -p side_length:=5.0
ros2 run 
ros2 run ros2_course koch_snowflake --ros-args -p order:=3 -p side_length=5.0
ros2 run ros2_course koch_snowflake --ros-args -p order:=3 -p side_length:=5.0
ros2 run ros2_course koch_snowflake 
source /opt/ros/humble/setup.bash  
python3 koch_snowflake.py --ros-args -p order:=3 -p side_length:=5.0
ros2 run ros2_course koch_snowflake --ros-args -p order:=3 -p side_length=5.0
ros2 run ros2_course koch_snowflake --ros-args -p order:=3 side_length=5.0
ros2 run ros2_course koch_snowflake --ros-args -p order:=3 side_length:=5.0
ros2 run ros2_course koch_snowflake order:=3 side_length:=5.0
ros2 run ros2_course koch_snowflake 
colcon build --symlink-install
ros2 run ros2_course koch_snowflake 
