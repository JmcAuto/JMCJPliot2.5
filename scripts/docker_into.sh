JMC_AUTO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

#if [ ! -e /jmcauto ]; then
#    sudo ln -sf ${JMC_AUTO_ROOT_DIR} /jmcauto
#fi

if [[ -n `groups |grep sudo` ]] && [[ -e /proc/sys/kernel ]]; then
    echo "/jmcauto/data/core/core_%e.%p" | sudo tee /proc/sys/kernel/core_pattern > /dev/null
fi


source ${JMC_AUTO_ROOT_DIR}/scripts/jmc_auto_base.sh

function local_volumes() {
    volumes="-v $JMC_AUTO_ROOT_DIR:/jmcauto \
             -v $HOME/.cache:${DOCKER_HOME}/.cache \
             -v $HOME/.ros:${DOCKER_HOME}/.ros \
             -v /home/tmp:/home/tmp \
             -v /dev:/dev \
             -v /media:/media \
             -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
             -v /etc/localtime:/etc/localtime:ro \
             -v /usr/src:/usr/src \
             -v /lib/modules:/lib/modules"
    #if [ "${JMC_AUTO_VOLUME}" == "release" ]; then
    #    volumes="-v $HOME/.cache/jmcauto_release/jmcauto:/jmcauto ${volumes}"
    #else
    #    volumes="-v $JMC_AUTO_ROOT_DIR:/jmcauto ${volumes}"
    #fi
    if [ -e /home/gpu_jmc ]; then
        volumes="${volumes} -v /home/gpu_jmc:/home/gpu_jmc"
    fi
    if [ -e $HOME/$JMC_AUTO_VOLUME ]; then
        volumes="${volumes} -v $HOME/$JMC_AUTO_VOLUME:${DOCKER_HOME}/$JMC_AUTO_VOLUME"
    fi
    if [ -e ${HOME}/public/04-personal_folders/$JMC_AUTO_VOLUME ]; then
        volumes="${volumes} -v ${HOME}/public/04-personal_folders/$JMC_AUTO_VOLUME:${DOCKER_HOME}/public/04-personal_folders/$JMC_AUTO_VOLUME"
    fi
    if [ -e $HOME/.Xauthority ]; then
        volumes="${volumes} -v $HOME/.Xauthority:${DOCKER_HOME}/.Xauthority \
                            -v $HOME/.Xauthority:/root/.Xauthority"
    fi
    echo "${volumes}"
}

function docker_start(){
    info "Start docker container \"${JMC_AUTO_VOLUME}\""
    local display=""
    if [[ -z ${DISPLAY} ]];then
        display=":0"
    else
        display="${DISPLAY}"
    fi

    if [[ -n `groups |grep sudo` ]]; then
        setup_device
    else
        warning "${USER} is not in the sudoers file, can't setup devices"
    fi

    USER_ID=$(id -u)
    GRP=$(id -g -n)
    GRP_ID=$(id -g)
    LOCAL_HOST=`hostname`
    DOCKER_HOME="$( cd "${HOME}/.." && pwd )/$JMC_AUTO_VOLUME"
    if [ "$USER" == "root" ]; then
        DOCKER_HOME="/root"
    fi
    if [ ! -d "$HOME/.cache" ];then
        mkdir "$HOME/.cache"
    fi

    #LOCALIZATION_VOLUME=localization_volume
    #docker stop ${LOCALIZATION_VOLUME} > /dev/null 2>&1

    #LOCALIZATION_VOLUME_IMAGE=${DOCKER_REPO}:localization_volume-${ARCH}
    #docker run -it -d --rm --name ${LOCALIZATION_VOLUME} ${LOCALIZATION_VOLUME_IMAGE}

    #YOLO3D_VOLUME=yolo3d_volume
    #docker stop ${YOLO3D_VOLUME} > /dev/null 2>&1

    #YOLO3D_VOLUME_IMAGE=${DOCKER_REPO}:yolo3d_volume-${ARCH}
    #docker run -it -d --rm --name ${YOLO3D_VOLUME} ${YOLO3D_VOLUME_IMAGE}

    docker run -it \
        -d \
        --privileged \
        --name ${JMC_AUTO_VOLUME} \
        -e DISPLAY=$display \
        -e DOCKER_USER=$JMC_AUTO_VOLUME \
        -e USER=$JMC_AUTO_VOLUME \
        -e DOCKER_USER_ID=$USER_ID \
        -e DOCKER_GRP=$JMC_AUTO_VOLUME \
        -e DOCKER_GRP_ID=$GRP_ID \
        -e DOCKER_IMG=${DOCKER_REPO}:latest \
        -e DOCKER_HOME=${DOCKER_HOME} \
        -e DOCKER_DV_PORTS=${DV_PORTS} \
        -e ROS_DOMAIN_ID=${ROS_PORTS} \
        $(local_volumes) \
        --net host \
        -w /jmcauto \
        --add-host in_${JMC_AUTO_VOLUME}_docker:127.0.0.1 \
        --add-host ${LOCAL_HOST}:127.0.0.1 \
        --shm-size 2G \
        --pid=host \
        ${DOCKER_REPO}:${REPO_VERSION} \
        /bin/bash

    if [ $? -ne 0 ];then
        error "Failed to start docker container \"${JMC_AUTO_VOLUME}\""
        exit 1
    elif [ "${USER}" != "root" ]; then
        docker exec -u root ${JMC_AUTO_VOLUME} bash -c '/jmcauto/scripts/adduser_docker.sh'
    else
        ok "Successfully start"
    fi

    docker_into

}

function docker_into(){

    if [[ -z ${DISPLAY} ]];then
        display=":0"
    else
        display="${DISPLAY}"
    fi
    info "Into..."
    xhost +local:root 1>/dev/null 2>&1
    docker exec \
    	-u $JMC_AUTO_VOLUME \
	    -e DISPLAY=$display \
    	-it ${JMC_AUTO_VOLUME} \
    	/bin/bash
#    if [ $? -ne 0 ];then
#	error "Failed to start docker container \"${JMC_AUTO_VOLUME}\""
#	info "Try to restart ${JMC_AUTO_VOLUME}..."
#	sleep 3
#	stop
#	start
#    fi

    xhost -local:root 1>/dev/null 2>&1

}

function docker_stop(){

    docker ps -a --format "{{.Names}}" | grep "${JMC_AUTO_VOLUME}" 1>/dev/null
    if [ $? == 0 ]; then
        printf %-*s 50 "Stop docker container \"${JMC_AUTO_VOLUME}\" ..."
        docker stop ${JMC_AUTO_VOLUME} 1>/dev/null
	if [ $? -eq 0 ];then
	    printf "\033[32m[DONE]\033[0m\n"
	    docker rm -vf ${JMC_AUTO_VOLUME} 1>/dev/null
	else
	    printf "\033[31m[FAILED]\033[0m\n"
	fi
    fi

}

if [ ! -d "$HOME/.ros/log"  ]; then
    mkdir -p $HOME/.ros/log
fi

USER_FILE="${JMC_AUTO_ROOT_DIR}/.USER_NAME"
DV_FILE="${JMC_AUTO_ROOT_DIR}/.DREAMVIEW.txt"
if [ ! -e ${USER_FILE} ]; then
    info "请输入用户名,用以区分docker镜像:"
    read user_name
    echo "$user_name" > $USER_FILE
    echo "--dreamview_module_name=dreamview_$user_name" > $DV_FILE
    dv_ports=`expr $(id -u) + 7889`
    info "为您分配的DV端口号为:$dv_ports"
    echo "--server_ports=$dv_ports" >> $DV_FILE
    JMC_AUTO_VOLUME=$(cat $USER_FILE)
    docker_stop
fi
JMC_AUTO_VOLUME=$(cat $USER_FILE)
DV_PORTS=$(sed -e '1d' -e 's/--server_ports\=//g' $DV_FILE)
if [ -z ${DV_PORTS} ]; then
    error "找不到分配的DV端口号,请删除<.USER_NAME>文件后重试"
    exit
fi
ROS_PORTS=`expr ${DV_PORTS} + 5153288`

DOCKER_REPO=jmc/jmcauto
ARCH=$(uname -m)

docker images |grep $JMC_AUTO_VOLUME
if [ $? == 0 ]; then
    REPO_VERSION=$JMC_AUTO_VOLUME
else
    REPO_VERSION=latest
fi

case $1 in
    start)
	docker_start
	;;
    into)
	docker_into
	;;
    stop)
	docker_stop
	;;
    *)
	docker ps | grep -w "${JMC_AUTO_VOLUME}" 1>/dev/null
	if [ $? == 0 ]; then
    		docker_into
	else
		docker_stop
    		docker_start
	fi
esac
