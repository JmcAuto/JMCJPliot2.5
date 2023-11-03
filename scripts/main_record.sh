#####################################################
# File Name: test.sh
# Author: Leo
# mail: yli97@jmc.com.cn
# Created Time: 一  8/10 14:28:36 2020
#####################################################

#!/bin/bash

JMC_AUTO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
source ${JMC_AUTO_ROOT_DIR}/scripts/jmc_auto_base.sh

#硬盘大小
disk_size=31
disk_type=exfat
disk_dir=/media/${USER}/mnt

#需要启动的模块
MODULES="canbus usb_cam localization control gnss" #...
#硬盘最大存放容量
MAX_PERCENT=95

function check_ssd() {
    disk=`sudo parted -l |grep "Disk" |grep "${disk_size}" | awk '{print $2}' | tr -d :`
    flag=""
    flag=$(sudo df -h |grep "${disk}")
    if [ -n "$disk" ]; then
        if [ -z "$flag" ]; then
             sudo mount -t ${disk_type} ${disk} ${disk_dir}
             sudo chown ${USER}:${USER} ${disk_dir}
             sudo chmod -R 755 ${disk_dir}
             check_space
        else
             check_space
        fi
    elif(whiptail --yesno "未检测到硬盘!请插入硬盘后重试." --yes-button "重试" --no-button "退出" 20 50); then
        check_ssd
    else
        exit
    fi
}

function check_space() {
    avail_space=$(df |grep "${disk}" |awk '{print $4 }')
    if [ ${avail_space} -le "1000000" ]; then
        whiptail --msgbox "硬盘空间不足!" 20 50 --ok-button "返回" 3>&1 1>&2 2>&3
        sudo umount ${disk_dir}
        #查错
        check_ssd
    else
        if (whiptail --yesno "硬盘挂载成功，是否录制点云数据?" --yes-button "是" --no-button "否" 20 50 3>&1 1>&2 2>&3); then
            export full_or_simple=FULL
        else
            export full_or_simple=SIMPLE
        fi
        decide_task_dir >/dev/null 2>&1
        main
    fi
}

function start_modules() {
    check_modules
    if [ ${modules_status} == 0 ]; then
        whiptail --msgbox "所有模块启动成功." 20 50 --ok-button "确定" 3>&1 1>&2 2>&3
        main
    else
        DISTROS=$(whiptail --title "启动模块" --checklist \
        "空格键选择模块,回车键启动选中模块,TAB键切换光标;" 20 50 8 \
        ${checklist_modules} \
        --ok-button "启动" \
        --cancel-button "返回" \
        3>&1 1>&2 2>&3)
        exitstatus=$?
        if [ $exitstatus = 0 ]; then
            for s in ${DISTROS}
            do
                MODULE=$(echo ${s} |sed -e 's/\"//g')
                bash  ${JMC_AUTO_ROOT_DIR}/scripts/${MODULE}.sh 1>&2
            done
            sleep 3
            main
        else
            main
        fi
    fi
}

function check_modules() {
    list_modules=""
    checklist_modules=""
    export modules_status=0
    for MODULE in ${MODULES}
    do
        NUM_PROCESSES="$(pgrep -c -f "${MODULE}")"
        if [ "${NUM_PROCESSES}" -eq 0 ]; then
            export ${MODULE}_status=OFF
            export modules_status=1
            if [ "${usb_cam_status}" = "OFF" ]; then
            bash ${JMC_AUTO_ROOT_DIR}/scripts/usb_cam.sh
            fi
        else
            export ${MODULE}_status=ON
            if [ -e ${JMC_AUTO_ROOT_DIR}/data/log/${MODULE}*.out ]; then
                tail -n 7 ${JMC_AUTO_ROOT_DIR}/data/log/${MODULE}*.out |grep ^W >/dev/null 2>&1
                if [ $? -ne 1 ]; then
                    export ${MODULE}_status=WARNING
                fi
                tail -n 7 ${JMC_AUTO_ROOT_DIR}/data/log/${MODULE}*.out |grep ^E >/dev/null 2>&1
                if [ $? -ne 1 ]; then
                    export ${MODULE}_status=ERROR
                fi
            fi
            if [ -e ${JMC_AUTO_ROOT_DIR}/data/log/usb_cam.out ]; then
                cat ${JMC_AUTO_ROOT_DIR}/data/log/usb_cam.out |grep "died" >/dev/null 2>&1
                if [ $? -ne 1 ]; then
                    export usb_cam_status=ERROR
                    bash ${JMC_AUTO_ROOT_DIR}/scripts/usb_cam.sh stop
                    bash ${JMC_AUTO_ROOT_DIR}/scripts/usb_cam.sh
                fi
                cat ${JMC_AUTO_ROOT_DIR}/data/log/usb_cam.out |grep "ERROR" >/dev/null 2>&1
                if [ $? -ne 1 ]; then
                    export usb_cam_status=ERROR
                    bash ${JMC_AUTO_ROOT_DIR}/scripts/usb_cam.sh stop
                    bash ${JMC_AUTO_ROOT_DIR}/scripts/usb_cam.sh
                fi
            fi
        fi
        export list_modules="${list_modules}\n    ${MODULE}:    $(eval echo '$'"${MODULE}_status")"
        export checklist_modules="${checklist_modules} ${MODULE} $(eval echo '$'"${MODULE}_status") $(eval echo '$'"${MODULE}_status")"
    done
}

function check_log() {
    cd ${JMC_AUTO_ROOT_DIR}/data/log
    ls -1 *.ERROR *.out >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        whiptail --msgbox "文件夹空!" --ok-button "返回" --clear 20 50 3>&1 1>&2 2>&3
        main
    else
        selec_log=$(whiptail --title "选择文件" --menu "上下键选择文件查看,TAB键切换光标." 20 50 10 `for x in $(ls -1 *.ERROR *.out); do echo  $x "-"; done` --ok-button "确定" --cancel-button "返回" --clear 3>&1 1>&2 2>&3)
        exitstatus=$?
        if [ ${exitstatus} = 0 ]; then
            whiptail --title ${selec_log} --msgbox "$(tail ${selec_log})" --ok-button "返回" 20 110 3>&1 1>&2 2>&3
            check_log
        else
            main
        fi
    fi
}

function help_box() {
    whiptail --title "帮助" --msgbox " \
        h键         此帮助界面\n \
        l键         错误日志查看\n \
        r键         重新录制\n \
        s键         模块启动\n \
        t键         查看topic\n \
        q键         退出程序" \
        --ok-button "返回" \
        20 80 3>&1 1>&2 2>&3
    main
}

function main() {
    (
    PUSH_KEY=""
    echo "模块启动中..."
    echo "XXX"
    until [ "${PUSH_KEY}" == "q" ]; do
        case $PUSH_KEY in
            h)
                help_box
                break
                ;;
            l)
                check_log
                break
                ;;
            r)
                bash ${JMC_AUTO_ROOT_DIR}/scripts/record_bag.sh stop
                bash ${JMC_AUTO_ROOT_DIR}/scripts/record_img.sh stop
                check_ssd
                break
                ;;
            s)
                start_modules
                break
                ;;
            t)
                bash ${JMC_AUTO_ROOT_DIR}/scripts/diagnostics.sh 3>&1 1>&2 2>&3
                main
                break
                ;;
        esac
        USED_PERCENT=$(df -P ${TASK_DIR} | tail -1 | awk '{print $5 }' | cut -d'%' -f1)
        if [ "${USED_PERCENT}" -ge "${MAX_PERCENT}" ]; then
            bash ${JMC_AUTO_ROOT_DIR}/scripts/record_bag.sh stop
            bash ${JMC_AUTO_ROOT_DIR}/scripts/record_img.sh stop
            if (whiptail --yesno "硬盘使用超过${MAX_PERCENT}%,请更换硬盘." --yes-button "重试" --no-button "退出" 20 50 3>&1 1>&2 2>&3); then
                check_ssd
                break
            else
                break
            fi
        fi
        check_modules
        if [ "${modules_status}" == "0" ]; then
            NUM_PROCESSES="$(pgrep -c -f "rosbag record")"
            if [ "${NUM_PROCESSES}" -eq 0 ]; then
                bash ${JMC_AUTO_ROOT_DIR}/scripts/record_bag.sh ${full_or_simple}
                bash ${JMC_AUTO_ROOT_DIR}/scripts/record_img.sh
            fi
            used_imgs="$(expr $(du ${TASK_DIR}/image_short | awk '{print $1 }') - 4)"
            used_imgl="$(expr $(du ${TASK_DIR}/image_long | awk '{print $1 }') - 4)"
            used_imgr="$(expr $(du ${TASK_DIR}/image_right | awk '{print $1 }') - 4)"
            used_data="$(du ${TASK_DIR} | tail -1 | awk '{print $1 }')"
            used_bags="$(expr ${used_data} - ${used_imgs} - ${used_imgl} - ${used_imgr})"
            echo "XXX"
            echo "模块启动成功,正在录制...  (按h键查看操作说明)\n \
                  ${list_modules}\n\n已录制大小:\n    image_short:    ${used_imgs}\n    image_long:     ${used_imgl}\n    image_right:    ${used_imgr}\n    rosbags:        ${used_bags}\n\n硬盘容量:"
            echo "XXX"
            echo "${USED_PERCENT}"
            echo "XXX"
        else
            echo "XXX"
            echo "模块未全启动	(按h键查看操作说明)\n${list_modules}\n\n硬盘容量:"
            echo "XXX"
            echo "${USED_PERCENT}"
            echo "XXX"
        fi
        read -s -t 2 -n1 PUSH_KEY
    done
    ) |whiptail --gauge "模块启动中...\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n" 20 80 0 3>&1 1>&2 2>&3
}

if [ ! -d ${disk_dir} ]; then
    sudo mkdir ${disk_dir}
    sudo chown ${USER}:${USER} ${disk_dir}
fi
setup_device
check_ssd
bash ${JMC_AUTO_ROOT_DIR}/scripts/record_bag.sh stop
bash ${JMC_AUTO_ROOT_DIR}/scripts/record_img.sh stop
if (whiptail --yesno "是否拷贝日志并关机?" --yes-button "是" --no-button "否" 20 50 3>&1 1>&2 2>&3); then
    echo "正在拷贝日志,请稍后"
    #添加拷贝进度条
    cp -rd ${JMC_AUTO_ROOT_DIR}/data/log ${TASK_DIR}
    cp -rd ${JMC_AUTO_ROOT_DIR}/data/gpsbin ${TASK_DIR}
    sudo rm -rf ${JMC_AUTO_ROOT_DIR}/data/log
    sudo rm -rf ${JMC_AUTO_ROOT_DIR}/data/gpsbin
    sleep 1
    sudo umount ${DISK}
    whiptail --msgbox "拷贝完毕,确认关机,请拔出硬盘." --ok-button "确定" 20 50 3>&1 1>&2 2>&3
    sudo poweroff
else
    sleep 1
    #sudo umount ${DISK}
    whiptail --msgbox "退出程序,请拔出硬盘." --ok-button "确定" 20 50 3>&1 1>&2 2>&3
fi
