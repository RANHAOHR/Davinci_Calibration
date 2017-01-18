#!/bin/sh

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

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/usr/local/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/usr/local/lib/python2.7/dist-packages:/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/cmake-build-debug/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/cmake-build-debug" \
    "/usr/bin/python" \
    "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/setup.py" \
    build --build-base "/home/rxh349/ros_ws/src/Davinci_Calibration/camera_calibration/cmake-build-debug" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/usr/local" --install-scripts="/usr/local/bin"
