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

echo_and_run cd "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_visualizer"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/install/lib/python2.7/dist-packages:/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build" \
    "/usr/bin/python" \
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_visualizer/setup.py" \
    build --build-base "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build/team_nust_visualizer" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/install" --install-scripts="/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/install/bin"
