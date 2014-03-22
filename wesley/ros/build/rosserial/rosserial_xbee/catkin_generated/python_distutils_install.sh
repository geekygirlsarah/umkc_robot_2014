#!/bin/sh -x

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

cd "/home/umkc/umkc_robot_2014_arduino/wesley/ros/src/rosserial/rosserial_xbee"

# todo --install-layout=deb per platform
# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/umkc/umkc_robot_2014_arduino/wesley/ros/install/lib/python2.7/site-packages:/home/umkc/umkc_robot_2014_arduino/wesley/ros/build/lib/python2.7/site-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/umkc/umkc_robot_2014_arduino/wesley/ros/build" \
    "/usr/bin/python" \
    "/home/umkc/umkc_robot_2014_arduino/wesley/ros/src/rosserial/rosserial_xbee/setup.py" \
    build --build-base "/home/umkc/umkc_robot_2014_arduino/wesley/ros/build/rosserial/rosserial_xbee" \
    install \
    $DESTDIR_ARG \
     --prefix="/home/umkc/umkc_robot_2014_arduino/wesley/ros/install" --install-scripts="/home/umkc/umkc_robot_2014_arduino/wesley/ros/install/bin"
