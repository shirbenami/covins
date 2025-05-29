docker build -f sparx_docker/Dockerfile.base -t covins_base --build-arg NR_JOBS_BASE=14 .

docker build -f sparx_docker/Dockerfile.dev -t covins_dev --build-arg NR_JOBS=$(nproc) .

install ros2 dashing
install colcon


colcon build \
  --packages-select covins_comm_ros2 \
  --build-base ros2_build \
  --install-base ros2_install



change run.sh
        docker run \
        -it \
        --rm \
        --net=host \
        --name covins_terminal \
        --env="DISPLAY=:1" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="XAUTHORITY=/run/user/1017/gdm/Xauthority" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="/run/user/1017/gdm/Xauthority:/run/user/1017/gdm/Xauthority" \
        --volume "/home/user1/GIT/TheAgency/covins_ws/src/covins:${CATKIN_WS}/src/covins" \
        covins_dev\
        /bin/bash

        from covins_dev to covins_terminal:with_ros2\