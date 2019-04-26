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

echo_and_run cd "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_network_handler"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/install/lib/python2.7/dist-packages:/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build" \
    "/usr/bin/python" \
    "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_network_handler/setup.py" \
    build --build-base "/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build/team_nust_network_handler" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/install" --install-scripts="/home/umaidzz/Documents/team-nust/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/install/bin"
