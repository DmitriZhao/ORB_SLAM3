FROM nvidia/cudagl:11.4.2-devel-ubuntu20.04
ARG EIGEN_VERSION=3.4.0
ARG OPENCV_VERSION=4.5.5
ARG CERES_VERSION=2.1.0
ARG FMT_VERSION=8.1.1
ARG VTK_VERSION=9.1
ARG VTK_SUB_VERSION=9.1.0
ARG OPENEXR_VERSION=3.1.5
ARG ABSEIL_VERSION=20211102.0
ARG CLANG_VER=12
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai LANG=C.UTF-8 LC_ALL=C.UTF-8 PIP_NO_CACHE_DIR=1

RUN sed -i  "s/archive.ubuntu.com/mirrors.ustc.edu.cn/g" /etc/apt/sources.list &&\
    sed -i  "s/security.ubuntu.com/mirrors.ustc.edu.cn/g" /etc/apt/sources.list &&\
    rm -f /etc/apt/sources.list.d/* &&\
    apt-get update && apt-get upgrade -y &&\
    # Install build tools, build dependencies and python
    apt-get install -y --no-install-recommends \
        autoconf automake autotools-dev build-essential ca-certificates \
        make cmake yasm pkg-config gdb gcovr \
        gcc-10 gcc-10-multilib g++-10 g++-10-multilib cpp-10 \
        ninja-build ccache doxygen graphviz plantuml \
        libatlas-base-dev libboost-all-dev libcereal-dev \
        libcgal-dev \
        libcxsparse3 libeigen3-dev liblapack-dev liblapacke-dev \
        metis libmetis-dev libopenblas-dev libsuitesparse-dev \
        libgtk2.0-dev libglew-dev libgl1-mesa-dev mesa-utils \
        libgflags-dev libgoogle-glog-dev libgtest-dev \
        libjpeg-dev libpng-dev libtiff-dev libtiff5-dev \
        libavcodec-dev libavformat-dev libavutil-dev libavdevice-dev \ 
        libpostproc-dev libpq-dev libswscale-dev ffmpeg \
        libtbb2 libtbb-dev libxine2-dev zlib1g-dev \
        libkrb5-dev libssl-dev libtool libyaml-dev libyaml-cpp-dev \
        ## Tools
        git curl wget unzip nano net-tools sudo htop iotop \
        cloc daemontools krb5-user nfs-common rsync \
        openssh-server openssh-client unattended-upgrades \
        ## Python
        python3-dev python3-pip python3-numpy \
        ## dev
        qt5-default \
        libllvm-${CLANG_VER}-ocaml-dev libllvm${CLANG_VER} llvm-${CLANG_VER} \
        llvm-${CLANG_VER}-dev llvm-${CLANG_VER}-doc llvm-${CLANG_VER}-examples \
        llvm-${CLANG_VER}-runtime \
        clang-${CLANG_VER} clang-tools-${CLANG_VER} clang-${CLANG_VER}-doc \
        libclang-common-${CLANG_VER}-dev libclang-${CLANG_VER}-dev \
        libclang1-${CLANG_VER} \
        clang-format-${CLANG_VER} python3-clang-${CLANG_VER} \
        clangd-${CLANG_VER} clang-tidy-${CLANG_VER} \
        libfuzzer-${CLANG_VER}-dev lldb-${CLANG_VER} lld-${CLANG_VER} \
        libc++-${CLANG_VER}-dev libc++abi-${CLANG_VER}-dev \
        libomp-${CLANG_VER}-dev libclc-${CLANG_VER}-dev \
    # Configure clang, gcc, ssh
    && update-alternatives \
            --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 \
            --slave /usr/bin/g++ g++ /usr/bin/g++-10 \
            --slave /usr/bin/gcov gcov /usr/bin/gcov-10 \
    && /tmp/det_dockerfile_scripts/update_alternatives_clang.sh ${CLANG_VER} 100 \
    # && update-alternatives --install /usr/bin/cc cc /usr/bin/clang 100 \
    # && update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++ 100 \
    && unattended-upgrade \
    && rm /etc/ssh/ssh_host_ecdsa_key \
    && rm /etc/ssh/ssh_host_ed25519_key \
    && rm /etc/ssh/ssh_host_rsa_key \
    && cp /etc/ssh/sshd_config /etc/ssh/sshd_config_bak \
    && sed -i "s/^.*X11Forwarding.*$/X11Forwarding yes/" /etc/ssh/sshd_config \
    && sed -i "s/^.*X11UseLocalhost.*$/X11UseLocalhost no/" /etc/ssh/sshd_config \
    && grep "^X11UseLocalhost" /etc/ssh/sshd_config || echo "X11UseLocalhost no" >> /etc/ssh/sshd_config \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Determined AI stuff
#! ---EDIT notebook-requirements.txt TO ADD PYPI PACKAGES----
WORKDIR /tmp
ENV PYTHONUNBUFFERED=1 PYTHONFAULTHANDLER=1 PYTHONHASHSEED=0
ENV JUPYTER_CONFIG_DIR=/run/determined/jupyter/config
ENV JUPYTER_DATA_DIR=/run/determined/jupyter/data
ENV JUPYTER_RUNTIME_DIR=/run/determined/jupyter/runtime
RUN git clone https://github.com/LingzheZhao/determinedai-container-scripts &&\
    cd determinedai-container-scripts &&\
    git checkout v0.1 &&\
    pip config set global.index-url https://mirrors.bfsu.edu.cn/pypi/web/simple &&\
    pip install determined && pip uninstall -y determined &&\
    pip install -r notebook-requirements.txt &&\
    pip install -r additional-requirements.txt &&\
    ./add_det_nobody_user.sh &&\
    ./install_libnss_determined.sh &&\
    rm -rf /tmp/*

# Install newest Eigen
RUN cd /tmp &&\
    wget https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz &&\
    tar -xvzf eigen-${EIGEN_VERSION}.tar.gz &&\
    cd eigen-${EIGEN_VERSION} &&\
    cmake -B build -GNinja -DCMAKE_BUILD_TYPE=RELEASE &&\
    cmake --build build -t install &&\
    ldconfig && rm -rf /tmp/*

# Install libfmt>7.0 (or Sophus will use a crazy fmt macro)
RUN cd /tmp &&\
    wget https://github.com/fmtlib/fmt/releases/download/${FMT_VERSION}/fmt-${FMT_VERSION}.zip &&\
    unzip fmt-${FMT_VERSION}.zip &&\
    cd fmt-${FMT_VERSION} &&\
    cmake -B build -GNinja -DCMAKE_BUILD_TYPE=RELEASE &&\
    cmake --build build -t install &&\
    ldconfig && rm -rf /tmp/*

# Install Sophus
RUN cd /tmp &&\
    git clone https://github.com/strasdat/Sophus &&\
    cd Sophus &&\
    cmake -B build -GNinja -DCMAKE_BUILD_TYPE=RELEASE &&\
    cmake --build build -t install &&\
    ldconfig && rm -rf /tmp/*

# Install Ceres
RUN cd /tmp &&\
    wget http://ceres-solver.org/ceres-solver-${CERES_VERSION}.tar.gz &&\
    tar -xvzf ceres-solver-${CERES_VERSION}.tar.gz &&\
    cd ceres-solver-${CERES_VERSION} &&\
    cmake -B build -GNinja -DCMAKE_BUILD_TYPE=RELEASE &&\
    cmake --build build -t install &&\
    ldconfig && rm -rf /tmp/*

# Install vtk (that shipped with ubuntu seems broken)
RUN cd /tmp &&\
    wget https://www.vtk.org/files/release/${VTK_VERSION}/VTK-${VTK_SUB_VERSION}.tar.gz &&\
    tar -xvzf VTK-${VTK_SUB_VERSION}.tar.gz &&\
    cd VTK-${VTK_SUB_VERSION} &&\
    cmake -B build -GNinja \
        -DVTK_USE_CUDA=ON \
        -DVTK_OPENGL_HAS_EGL:BOOL=ON \
        -DVTK_USE_X:BOOL=OFF \
        -DVTK_DEFAULT_RENDER_WINDOW_HEADLESS:BOOL=ON &&\
    cmake --build build -t install &&\
    ldconfig && rm -rf /tmp/*

# Install OpenEXR
RUN cd /tmp &&\
    wget https://github.com/AcademySoftwareFoundation/openexr/archive/refs/tags/v${OPENEXR_VERSION}.tar.gz &&\
    tar -xvzf v${OPENEXR_VERSION}.tar.gz &&\
    cd openexr-${OPENEXR_VERSION} &&\
    cmake -B build -GNinja &&\
    cmake --build build -t install &&\
    ldconfig && rm -rf /tmp/*

# Install OpenCV
RUN cd /tmp/ &&\
    wget https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip &&\
    unzip $OPENCV_VERSION.zip &&\ 
    rm $OPENCV_VERSION.zip &&\
    wget https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.zip &&\
    unzip ${OPENCV_VERSION}.zip &&\
    cd opencv-${OPENCV_VERSION} &&\
    cmake \
        -B /tmp/opencv-${OPENCV_VERSION}/build \
        -GNinja \
        -DCMAKE_CXX_STANDARD=17 \
        -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib-${OPENCV_VERSION}/modules \
        -DOPENCV_ENABLE_NONFREE=ON \
        -DWITH_CUDA=ON \
        -DCUDA_ARCH_BIN=8.6 \
        -DCMAKE_BUILD_TYPE=RELEASE \
        -DCMAKE_INSTALL_PREFIX=/usr/local &&\
    cmake --build build -t install &&\
    ldconfig && rm -rf /tmp/*

# Install Pangolin
RUN cd /tmp &&\
    wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.tar.gz -O pangolin.tgz &&\
    tar -xzf pangolin.tgz &&\
    cd Pangolin-0.6 &&\
    cmake -B build -GNinja -DCMAKE_BUILD_TYPE=RELEASE &&\
    cmake --build build -t install &&\
    ldconfig && rm -rf /tmp/*

# Install abseil-cpp
RUN cd /tmp &&\
    wget https://github.com/abseil/abseil-cpp/archive/refs/tags/${ABSEIL_VERSION}.tar.gz &&\
    tar -xzf ${ABSEIL_VERSION}.tar.gz &&\
    cd abseil-cpp-${ABSEIL_VERSION} &&\
    cmake -B build -GNinja \
        -DCMAKE_CXX_STANDARD=17 \
        -DABSL_ENABLE_INSTALL=ON &&\
    cmake --build build -t install &&\
    ldconfig && rm -rf /tmp/*

# Install g2o
RUN cd /tmp &&\
    git clone https://github.com/RainerKuemmerle/g2o &&\
    cd g2o &&\
    cmake -B build -GNinja \
        -DCMAKE_BUILD_TYPE=RELEASE \
        -DG2O_BUILD_APPS:BOOL=OFF \
        -DG2O_BUILD_EXAMPLES:BOOL=OFF \
        -DG2O_USE_CSPARSE:BOOL=ON &&\
    cmake --build build -t install &&\
    ldconfig && rm -rf /tmp/*

# Install cve
RUN cd /tmp &&\
    git clone https://github.com/Rookfighter/cv-eigen &&\
    cd cv-eigen &&\
    cmake -B build -GNinja && \
    cmake --build build -t install && \
    ldconfig && rm -rf /tmp/*

ADD . /opt/ORB_SLAM3
WORKDIR /opt/ORB_SLAM3
RUN ./build.sh