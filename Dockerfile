FROM ros:kinetic

# Bring up to date https://automodality.atlassian.net/wiki/spaces/AUTOMOD/pages/491579/Ubuntu+Setup#UbuntuSetup-BringUpToDate
RUN apt-get -y update
RUN apt-get -y dist-upgrade

RUN apt-get -y install exfat-fuse exfat-utils
RUN apt-get -y install libusb-dev libusb-1.0-0-dev libusb-1.0-0
RUN usermod -aG dialout root 

RUN apt-get -y install apt-utils apt-transport-https
RUN apt-get -y install xsdcxx
RUN apt-get -y install python-catkin-tools 
# needed for mk-build-deps
RUN apt-get -y install devscripts equivs javahelper # required for debhelper

SHELL ["/bin/bash", "-c"]

RUN echo "export AM_PLATFORM=AM_VM" >> ~/.bashrc

# Create Catkin Workspace Manually

# run this now so catkin make gets set up
RUN mkdir -p ~/catkin_ws/src 
RUN cd ~/catkin_ws/src;. /opt/ros/kinetic/setup.bash; catkin_init_workspace; cd ..; catkin_make
# ensure ros commands are available on startup
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


# Copies your code file from your action repository to the filesystem path `/` of the container
COPY entrypoint.sh /entrypoint.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/entrypoint.sh"]