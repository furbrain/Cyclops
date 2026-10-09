FROM nvidia/cuda:12.6.3-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive

# ---- 1. System packages (apt provides most of vcpkg.json) ---------------------
# boost, eigen3, libpng, tiff, opencv, nanoflann, tinyxml2, zlib, plus the
# libraries Ceres needs (suitesparse, glog, gflags, lapack). Ceres itself is
# built from source below so it can use CUDA.
RUN apt-get update -yq && apt-get install -yq \
        build-essential git cmake pkg-config ca-certificates \
        libboost-all-dev libeigen3-dev libsuitesparse-dev \
        libgoogle-glog-dev libgflags-dev liblapack-dev libblas-dev \
        libpng-dev libjpeg-dev libtiff-dev zlib1g-dev \
        libopencv-dev libnanoflann-dev libtinyxml2-dev \
        libgmp-dev libmpfr-dev \
        libglu1-mesa-dev libglew-dev libglfw3-dev \
        python3-dev python-is-python3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build

# ---- 2. OpenMVS source (cloned early so its ports/ info shows in the log) ----
ARG OPENMVS_BRANCH=develop
RUN git clone --depth 1 --branch ${OPENMVS_BRANCH} \
        https://github.com/cdcseacave/openMVS.git /opt/openMVS \
    && git clone --depth 1 https://github.com/cdcseacave/VCG.git /opt/vcglib \
    && echo "=== OpenMVS custom vcpkg ports ===" \
    && ls /opt/openMVS/ports \
    && (grep -rhE "REPO|REF|URL" /opt/openMVS/ports || true)

# ---- 3. CGAL ------------------------------------------------------------------
RUN git clone --depth 1 --branch v6.0.1 https://github.com/cgal/cgal \
    && cmake -S cgal -B cgal_build \
    && cmake --build cgal_build -j4 && cmake --install cgal_build \
    && rm -rf cgal cgal_build

# ---- 4. Small CMake libraries not in Ubuntu 24.04 (one layer each) ------------
RUN git clone --depth 1 https://github.com/cdcseacave/TinyEXIF.git \
    && cmake -S TinyEXIF -B TinyEXIF_build \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && cmake --build TinyEXIF_build -j4 && cmake --install TinyEXIF_build \
    && rm -rf TinyEXIF TinyEXIF_build

RUN git clone --depth 1 https://github.com/cdcseacave/TinyNPY.git \
    && cmake -S TinyNPY -B TinyNPY_build \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && cmake --build TinyNPY_build -j4 && cmake --install TinyNPY_build \
    && rm -rf TinyNPY TinyNPY_build

RUN git clone --depth 1 https://github.com/syoyo/tinygltf.git \
    && cmake -S tinygltf -B tinygltf_build \
        -DCMAKE_BUILD_TYPE=Release -DTINYGLTF_BUILD_LOADER_EXAMPLE=OFF \
    && cmake --build tinygltf_build -j4 && cmake --install tinygltf_build \
    && rm -rf tinygltf tinygltf_build

RUN git clone --depth 1 https://github.com/PoseLib/PoseLib.git \
    && cmake -S PoseLib -B PoseLib_build \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && cmake --build PoseLib_build -j4 && cmake --install PoseLib_build \
    && rm -rf PoseLib PoseLib_build

# Ceres Solver built from source WITH CUDA (Ubuntu's libceres-dev has no CUDA).
# I believe Ceres CUDA needs compute capability >= 7.0, so this list is separate
# from CUDA_ARCHS below: 75=T4, 80=A100, 86=A10G, 89=L4/L40S, 90=H100.
# CUDA 12.6 can't target Blackwell (B200); that needs CUDA 12.8+.
ARG CERES_CUDA_ARCHS="70;75;80;86;89;90"
RUN git clone --depth 1 --branch 2.2.0 https://github.com/ceres-solver/ceres-solver.git \
    && cmake -S ceres-solver -B ceres_build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DUSE_CUDA=ON \
        -DCMAKE_CUDA_ARCHITECTURES="${CERES_CUDA_ARCHS}" \
        -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
    && cmake --build ceres_build -j4 && cmake --install ceres_build \
    && rm -rf ceres-solver ceres_build

# halfmesh and its dependencies (halfmesh also needs eigen3, opencv, tinygltf,
# which are already installed above).
# bshoshany-thread-pool is header-only: copy the headers into place.
RUN git clone --depth 1 https://github.com/bshoshany/thread-pool.git \
    && cp thread-pool/include/BS_*.hpp /usr/local/include/ \
    && rm -rf thread-pool

# tinyply: this is the upstream repo vcpkg's port uses. If the "custom vcpkg
# ports" output in step 2 shows a cdcseacave fork, use that URL instead.
RUN git clone --depth 1 https://github.com/ddiakopoulos/tinyply.git \
    && cmake -S tinyply -B tinyply_build \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && cmake --build tinyply_build -j4 && cmake --install tinyply_build \
    && rm -rf tinyply tinyply_build

# tinygltf v2: the layer above installed v3 (the repo's default branch), but
# halfmesh needs the v2 API (tiny_gltf.h). Placed here, after Ceres, so the
# earlier layers stay cached; v3's headers are removed first. Once things work
# you can delete the earlier tinygltf layer, pin v2.9.7 there, and drop this
# one (that rebuilds everything after it).
RUN rm -f /usr/local/include/tiny_gltf_v3.h /usr/local/include/tiny_gltf_v3.c \
          /usr/local/include/tinygltf_json_c.h \
    && git clone --depth 1 --branch v2.9.7 https://github.com/syoyo/tinygltf.git tinygltf_v2 \
    && cmake -S tinygltf_v2 -B tinygltf_v2_build \
        -DCMAKE_BUILD_TYPE=Release \
        -DTINYGLTF_HEADER_ONLY=ON -DTINYGLTF_INSTALL=ON \
        -DTINYGLTF_BUILD_LOADER_EXAMPLE=OFF \
    && cmake --install tinygltf_v2_build \
    && rm -rf tinygltf_v2 tinygltf_v2_build


# OpenCV from source: OpenMVS develop uses cv::IMWRITE_JPEGXL_QUALITY, which
# Ubuntu 24.04's OpenCV 4.6 lacks (it appears in the 4.11 docs). This must come
# BEFORE halfmesh and OpenMVS so both build against the same OpenCV. Ubuntu's
# libopencv packages are purged so only one version is on the system. JPEG XL
# codec support (libjxl) is left off; only the API constant is needed to compile.
RUN apt-get purge -y 'libopencv*' \
    && git clone --depth 1 --branch 4.12.0 https://github.com/opencv/opencv.git \
    && cmake -S opencv -B opencv_build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
        -DBUILD_DOCS=OFF -DBUILD_opencv_python3=OFF -DBUILD_opencv_java=OFF \
        -DWITH_EIGEN=ON -DWITH_CUDA=OFF \
    && cmake --build opencv_build -j4 && cmake --install opencv_build \
    && ldconfig && rm -rf opencv opencv_build



RUN git clone --depth 1 https://github.com/cdcseacave/halfmesh.git \
    && cmake -S halfmesh -B halfmesh_build \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && cmake --build halfmesh_build -j4 && cmake --install halfmesh_build \
    && rm -rf halfmesh halfmesh_build

# ---- 4b. Choose the OpenMVS version (tag or branch) ---------------------------
# Re-clones here, after all dependency layers, so changing the ref only
# invalidates the layers below. Build with e.g. --build-arg OPENMVS_REF=v4.2.0
ARG OPENMVS_REF=develop
RUN apt-get update -yq && apt-get install -yq libjxl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN rm -rf /opt/openMVS \
    && git clone --depth 1 --branch v2.4.0 \
        https://github.com/cdcseacave/openMVS.git /opt/openMVS


# ---- 5. Configure OpenMVS (fails fast if a dependency is still missing) -------
# "all" is slow for nvcc; once working, try e.g. "86;89" for A10G + L4.
ARG CUDA_ARCHS=all
RUN mkdir -p /opt/openMVS_build && cd /opt/openMVS_build \
    && cmake /opt/openMVS \
        -DCMAKE_BUILD_TYPE=Release \
        -DVCG_ROOT=/opt/vcglib \
        -DOpenMVS_USE_CUDA=ON \
        -DOpenMVS_MAX_CUDA_COMPATIBILITY=ON \
        -DOpenMVS_USE_PYTHON=OFF \
        -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs/ \
        -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda/ \
        -DCUDA_INCLUDE_DIRS=/usr/local/cuda/include/ \
        -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64 \
        -DCUDA_NVCC_EXECUTABLE=/usr/local/cuda/bin/ \
        -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCHS}"

# ---- 6. Compile and install ---------------------------------------------------
ARG JOBS=4
RUN cmake --build /opt/openMVS_build -j${JOBS}
RUN cmake --install /opt/openMVS_build && ldconfig

ENV PATH=/usr/local/bin/OpenMVS:$PATH