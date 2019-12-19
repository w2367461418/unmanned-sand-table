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

echo_and_run cd "/home/sz/unpeople/src/vision_opencv-melodic/image_geometry"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sz/unpeople/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sz/unpeople/install/lib/python2.7/dist-packages:/home/sz/unpeople/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sz/unpeople/build" \
    "/usr/bin/python" \
    "/home/sz/unpeople/src/vision_opencv-melodic/image_geometry/setup.py" \
    build --build-base "/home/sz/unpeople/build/vision_opencv-melodic/image_geometry" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/sz/unpeople/install" --install-scripts="/home/sz/unpeople/install/bin"
