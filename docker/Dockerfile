FROM docker.io/arm64v8/ros:humble-ros-base

RUN apt-get update && \
    apt-get install -y \
    git \
    can-utils \
    bash \
    vim \
    ros-humble-pcl-ros \
    freeglut3-dev \
    libeigen3-dev \
    libglu1-mesa-dev \
    nmap \
    inetutils-ping \
    iproute2 \
    net-tools \
    ifmetric \
    && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN git clone --recursive --depth 1 https://github.com/aentinger/Livox-SDK2 --branch elrob24 && \
    mkdir -p Livox-SDK2/build && cd Livox-SDK2/build && \
    cmake .. && make install

RUN git clone --recursive --depth 1 https://github.com/gsl-lite/gsl-lite --branch v0.40.0 && \
    mkdir -p gsl-lite/build && cd gsl-lite/build && \
    cmake .. && make -j8 && \
    sudo make install

RUN git clone --recursive --depth 1 https://github.com/catchorg/Catch2 && \
    mkdir -p Catch2/build && cd Catch2/build && \
    cmake .. && make -j8 && \
    sudo make install

RUN git clone --recursive --depth 1 https://github.com/fmtlib/fmt --branch 10.1.0 && \
    mkdir -p fmt/build && cd fmt/build && \
    cmake -DFMT_TEST=OFF .. && \
    make -j8 && \
    sudo make install

RUN git clone --recursive --depth 1 https://github.com/nlohmann/json --branch v3.11.3 && \
    mkdir -p json/build && cd json/build && \
    cmake -DJSON_BuildTests=OFF .. && make -j8 && \
    sudo make install

RUN mkdir -p /opt/colcon_ws/src
WORKDIR /opt/colcon_ws/src

RUN git clone --recursive --depth 1 https://github.com/aentinger/msas_enrich_2023 --branch elrob24-tracked && cd msas_enrich_2023 && git pull origin elrob24-tracked
RUN git clone --recursive --depth 1 https://github.com/107-systems/t07_robot --branch elrob24 && cd t07_robot && git pull origin elrob24
RUN git clone --recursive --depth 1 https://github.com/pika-spark/pika-spark-bno085-driver --branch elrob24 && cd pika-spark-bno085-driver && git pull origin elrob24
RUN git clone --recursive --depth 1 https://github.com/107-systems/robotem_rovne --branch elrob24-tracked && cd robotem_rovne && git pull origin elrob24-tracked

WORKDIR /opt/colcon_ws

COPY fastrtps-whitelist-wlan.sh /opt/colcon_ws
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/colcon_ws/fastrtps-whitelist-wlan.xml
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select mandeye_unicorn t07_robot pika_spark_bno085_driver robotem_rovne --cmake-args -DCMAKE_BUILD_TYPE=Release
