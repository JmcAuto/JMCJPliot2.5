FROM jmc/jmcauto:latest
ENV DISPLAY :0
ENV DOCKER_USER caros
ENV USER caros
ENV DOCKER_USER_ID 1000
ENV DOCKER_GRP caros
ENV DOCKER_GRP_ID 1000
ENV DOCKER_IMG jmc/jmcauto:dev
VOLUME ["/jmcauto/modules/perception/model/yolo_camera_detector/lane13d_0716","/jmcauto/modules/perception/model/yolo_camera_detector/lane2d_0627","/jmcauto/modules/perception/model/yolo_camera_detector/yolo3d_1128"]
COPY yolo_camera_detector/lane13d_0716 /jmcauto/modules/perception/model/yolo_camera_detector/lane13d_0716
COPY yolo_camera_detector/lane2d_0627 /jmcauto/modules/perception/model/yolo_camera_detector/lane2d_0627
COPY yolo_camera_detector/yolo3d_1128 /jmcauto/modules/perception/model/yolo_camera_detector/yolo3d_1128
RUN echo 20180809 > /jmcauto/modules/perception/model/yolo_camera_detector/yolo3d_1128//VERSION.txt
WORKDIR /jmcauto
USER caros
ENV LANG C.UTF-8
