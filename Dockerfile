FROM ros:melodic

ENV DEBIAN_FRONTEND noninteractive

# Bring up to date https://automodality.atlassian.net/wiki/spaces/AUTOMOD/pages/491579/Ubuntu+Setup#UbuntuSetup-BringUpToDate
RUN apt-get -y update
RUN apt-get -y install apt-utils 
RUN apt-get -y dist-upgrade

RUN apt-get -y install \
        apt-utils=1.6.12 \
        apt-transport-https=1.6.12 \
        python-catkin-tools=0.4.5-1 \
        xsdcxx=4.0.0-7build1 \ 
        devscripts=2.17.12ubuntu1.1 \
        equivs=2.1.0 \ 
        debhelper=12.1.1ubuntu1~ubuntu18.04.1 \ 
        javahelper=0.72.1~18.04.1
        
RUN usermod -aG dialout root 

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Copies your code file from your action repository to the filesystem path `/` of the container
COPY entrypoint.sh /entrypoint.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/entrypoint.sh"]