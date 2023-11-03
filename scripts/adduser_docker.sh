#!/usr/bin/env bash
addgroup --gid "$DOCKER_GRP_ID" "$DOCKER_GRP"
adduser --home "${DOCKER_HOME}" --shell "/bin/bash" --no-create-home \
    --disabled-password --force-badname --gecos '' "$DOCKER_USER" \
    --uid "$DOCKER_USER_ID" --gid "$DOCKER_GRP_ID" 2>/dev/null
usermod -aG sudo "$DOCKER_USER"
echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
cp -r /etc/skel/. ${DOCKER_HOME}/
sed -i 's/u@\\h/u@in_docker/' ${DOCKER_HOME}/.bashrc
echo 'if [ -e "/jmcauto/scripts/jmc_auto_base.sh" ]; then source /jmcauto/scripts/jmc_auto_base.sh; fi' >> "${DOCKER_HOME}/.bashrc"
echo "ulimit -c unlimited" >> ${DOCKER_HOME}/.bashrc
echo "ok \"Hooorrrrray!\"" >> ${DOCKER_HOME}/.bashrc
echo "ok \"You are in docker now!\"" >> ${DOCKER_HOME}/.bashrc
echo "ok \"Your Dreamview_PORT is ${DOCKER_DV_PORTS}\"" >> ${DOCKER_HOME}/.bashrc

chown ${DOCKER_USER}:${DOCKER_GRP} "${DOCKER_HOME}"

# setup GPS device
if [ -e /dev/novatel0 ]; then
  chmod a+rw /dev/novatel0
fi
if [ -e /dev/novatel1 ]; then
  chmod a+rw /dev/novatel1
fi
if [ -e /dev/novatel2 ]; then
  chmod a+rw /dev/novatel2
fi
if [ -e /dev/ttyACM0 ]; then
  chmod a+rw /dev/ttyACM0
fi
if [ -e /dev/ttyUSB0 ]; then
  chmod a+rw /dev/ttyUSB0
fi
if [ -e /dev/ttyUSB1 ]; then
  chmod a+rw /dev/ttyUSB1
fi
if [ -e /dev/ttyUSB2 ]; then
  chmod a+rw /dev/ttyUSB2
fi

# setup camera device
if [ -e /dev/camera/obstacle ]; then
  chmod a+rw /dev/camera/obstacle
fi
if [ -e /dev/camera/trafficlights ]; then
  chmod a+rw /dev/camera/trafficlights
fi


if [ "$RELEASE_DOCKER" != "1" ];then
  # setup map data
  if [ -e /home/tmp/modules_data ]; then
    cp -r /home/tmp/modules_data/* /jmcauto/modules/
    chown -R ${DOCKER_USER}:${DOCKER_GRP} "/jmcauto/modules"
  fi

  # setup ros package
  # this is a temporary solution to avoid ros package downloading.
  ROS="/home/tmp/ros"
  chmod a+w "${ROS}/share/velodyne/launch/start_velodyne.launch"
  chmod a+w -R "${ROS}/share/velodyne_pointcloud/params"
fi
