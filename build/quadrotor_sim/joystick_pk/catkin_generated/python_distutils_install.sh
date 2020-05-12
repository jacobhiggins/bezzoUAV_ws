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

echo_and_run cd "/home/bezzo/bezzoUAV_ws/src/quadrotor_sim/joystick_pk"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/bezzo/bezzoUAV_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/bezzo/bezzoUAV_ws/install/lib/python2.7/dist-packages:/home/bezzo/bezzoUAV_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/bezzo/bezzoUAV_ws/build" \
    "/usr/bin/python" \
    "/home/bezzo/bezzoUAV_ws/src/quadrotor_sim/joystick_pk/setup.py" \
    build --build-base "/home/bezzo/bezzoUAV_ws/build/quadrotor_sim/joystick_pk" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/bezzo/bezzoUAV_ws/install" --install-scripts="/home/bezzo/bezzoUAV_ws/install/bin"
