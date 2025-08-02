#!/bin/bash

# Colors and formatting
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# ASCII Art
echo -e "${BLUE}"
echo '    ____  ____  ____     __           __'
echo '   / __ \/ __ \/ __/    / /   ____ _/ /_'
echo '  / /_/ / /_/ /\ \     / /   / __ `/ __ \\'
echo ' / _, _/\____/___/    / /___/ /_/ / /_/ /'
echo '/_/ |_|              /_____/\__,_/_.___/'
echo -e "${NC}"
echo -e "${BOLD}Workspace Setup Script${NC}"
echo -e "${YELLOW}════════════════════════════════════════${NC}\n"

# Check if running from the correct directory
if [ ! -d "src" ] || [ ! -f "src/CMakeLists.txt" ]; then
    echo -e "${RED}Error: This script must be run from the root of the catkin workspace${NC}"
    echo -e "${YELLOW}Please ensure you are in the correct directory with the following structure:${NC}"
    echo -e "${BLUE}  ├── src/"
    echo "  │   └── CMakeLists.txt"
    echo "  └── setup.sh (this script)${NC}"
    echo ""
    echo -e "${YELLOW}Current directory: ${BLUE}$(pwd)${NC}"
    exit 1
else
    echo -e "${GREEN}✓ Running from correct workspace directory${NC}"
fi

echo -e "\n${BOLD}Checking system compatibility...${NC}"

# Check if running on Ubuntu 20.04
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "20.04" ] || [ "$ID" != "ubuntu" ]; then
        echo -e "${RED}Error: This setup requires Ubuntu 20.04${NC}"
        echo -e "${YELLOW}Current OS: ${BLUE}$PRETTY_NAME${NC}"
        echo -e "${YELLOW}Please use Ubuntu 20.04 to continue${NC}"
        exit 1
    else
        echo -e "${GREEN}✓ Ubuntu 20.04 detected${NC}"
    fi
else
    echo -e "${RED}Error: Cannot determine OS version${NC}"
    exit 1
fi

# Check if ROS is installed and check version
if command -v rosversion &> /dev/null; then
    ROS_VERSION=$(rosversion -d)
    if [ "$ROS_VERSION" != "noetic" ]; then
        echo -e "${RED}Error: Incompatible ROS version detected: ${BLUE}$ROS_VERSION${NC}"
        echo -e "${YELLOW}This workspace requires ROS1 Noetic. Please install the correct version.${NC}"
        exit 1
    else
        echo -e "${GREEN}✓ Compatible ROS1 Noetic detected${NC}"
    fi
else
    echo -e "${YELLOW}ROS is not installed${NC}"
    echo -ne "${BOLD}Do you want to install ROS1 Noetic? ${GREEN}(y/n)${NC}: "
    read choice
    if [[ $choice =~ ^[Yy]$ ]]; then
        echo -e "\n${BOLD}Starting ROS installation...${NC}\n"
        
        echo -e "${YELLOW}Adding ROS repositories...${NC}"
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        echo -e "${YELLOW}Installing curl...${NC}"
        sudo apt install curl -y
        echo -e "${YELLOW}Adding ROS keys...${NC}"
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
        echo -e "${GREEN}✓ ROS repositories and keys added${NC}"
        
        echo -e "\n${YELLOW}Updating package lists...${NC}"
        sudo apt update
        echo -e "${GREEN}✓ Package lists updated${NC}"
        
        echo -e "\n${YELLOW}Installing ROS and dependencies...${NC}"
        sudo apt install -y ros-noetic-desktop-full
        sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
        echo -e "${GREEN}✓ ROS and dependencies installed${NC}"
        
        echo -e "\n${YELLOW}Initializing rosdep...${NC}"
        sudo rosdep init
        rosdep update
        echo -e "${GREEN}✓ rosdep initialized${NC}"
    else
        echo -e "${RED}ROS installation cancelled. Cannot continue without ROS.${NC}"
        exit 1
    fi
fi

echo -e "\n${YELLOW}Sourcing ROS environment...${NC}"
source /opt/ros/noetic/setup.bash
echo -e "${GREEN}✓ ROS environment sourced${NC}"

echo -e "\n${YELLOW}Building the workspace...${NC}"
if catkin_make; then
    echo -e "${GREEN}✓ Workspace built successfully${NC}"
else
    echo -e "${RED}Error: Workspace build failed${NC}"
    exit 1
fi

echo -e "\n${YELLOW}Sourcing the workspace...${NC}"
source devel/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"
echo -e "$ROS_PACKAGE_PATH"
echo -e "\n${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}✓ Setup complete! ${NC}"
echo -e "${BLUE}Your ROS workspace is now ready to use!${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}"
