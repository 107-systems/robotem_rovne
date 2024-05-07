FROM docker.io/arm64v8/ros:humble-ros-base

RUN apt-get update && \
    apt-get install -y git can-utils && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

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

RUN git clone https://github.com/mpusz/mp-units && cd mp-units && \
    git checkout v2.1.0 && \
    mkdir build && cd build && \
    cmake -DMP_UNITS_USE_FMTLIB=ON -DMP_UNITS_BUILD_LA=OFF .. && \
    make -j8 && \
    sudo make install

RUN mkdir -p /tmp/colcon_ws/src
WORKDIR /tmp/colcon_ws/src
RUN git clone --recursive https://github.com/107-systems/t07_robot
RUN git clone --recursive https://github.com/pika-spark/pika-spark-bno085-driver

WORKDIR /tmp/colcon_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select t07_robot pika_spark_bno085_driver

COPY ./run.sh /
RUN chmod +x /run.sh

ENTRYPOINT ["/run.sh"]
CMD ["build"]
