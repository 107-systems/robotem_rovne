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
    && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    mkdir -p Livox-SDK2/build && cd Livox-SDK2/build && \
    cmake .. && make install

RUN git clone https://github.com/gsl-lite/gsl-lite && cd gsl-lite && \
    git checkout v0.40.0 && \
    mkdir build && cd build && \
    cmake .. && make -j8 && \
    sudo make install

RUN git clone https://github.com/catchorg/Catch2 && cd Catch2 && \
    mkdir build && cd build && \
    cmake .. && make -j8 && \
    sudo make install

RUN git clone https://github.com/fmtlib/fmt && cd fmt && \
    git checkout 10.1.0 && \
    mkdir build && cd build && \
    cmake -DFMT_TEST=OFF .. && \
    make -j8 && \
    sudo make install

RUN git clone https://github.com/nlohmann/json && cd json && \
    git checkout v3.11.3 && \
    mkdir build && cd build && \
    cmake .. && make -j8 && \
    sudo make install

RUN mkdir -p /tmp/colcon_ws/src
WORKDIR /tmp/colcon_ws/src

RUN git clone --recursive --depth 1 https://github.com/aentinger/msas_enrich_2023 --branch elrob24
RUN git clone --recursive --depth 1 https://github.com/107-systems/t07_robot --branch robotem-rovne
RUN git clone --recursive --depth 1 https://github.com/pika-spark/pika-spark-bno085-driver --branch robotem-rovne
RUN git clone --recursive --depth 1 https://github.com/107-systems/robotem_rovne

WORKDIR /tmp/colcon_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select mandeye_unicorn t07_robot pika_spark_bno085_driver robotem_rovne
