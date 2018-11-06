FROM project8/p8compute_dependencies:v0.1.0 as locust_common

ENV LOCUST_BUILD_PREFIX=/usr/local/p8/locust-v1.8.0

RUN mkdir -p $LOCUST_BUILD_PREFIX &&\
    cd $LOCUST_BUILD_PREFIX &&\
    echo "source ${COMMON_BUILD_PREFIX}/setup.sh" > setup.sh &&\
    echo "export LOCUST_BUILD_PREFIX=${LOCUST_BUILD_PREFIX}" >> setup.sh &&\
    echo 'ln -sf $LOCUST_BUILD_PREFIX $LOCUST_BUILD_PREFIX/../locust' >> setup.sh &&\
    echo 'export PATH=$LOCUST_BUILD_PREFIX/bin:$PATH' >> setup.sh &&\
    echo 'export LD_LIBRARY_PATH=$LOCUST_BUILD_PREFIX/lib:$LD_LIBRARY_PATH' >> setup.sh &&\
    /bin/true

########################
FROM locust_common as locust_done

# repeat the cmake command to get the change of install prefix to set correctly (a package_builder known issue)
RUN source $LOCUST_BUILD_PREFIX/setup.sh &&\
    mkdir /tmp_install &&\
    cd /tmp_install &&\
    git clone https://github.com/project8/locust_mc &&\
    cd locust_mc &&\
    git checkout develop &&\
    git submodule update --init --recursive &&\
    mkdir build &&\
    cd build &&\
    cmake -D CMAKE_INSTALL_PREFIX:PATH=$LOCUST_BUILD_PREFIX .. &&\
    cmake -D CMAKE_INSTALL_PREFIX:PATH=$LOCUST_BUILD_PREFIX .. &&\
    make -j3 install &&\
    /bin/true

########################
FROM locust_common

COPY --from=locust_done $LOCUST_BUILD_PREFIX $LOCUST_BUILD_PREFIX
