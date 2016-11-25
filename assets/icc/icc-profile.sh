#! /bin/sh
### BEGIN INIT INFO
# Provides:          icc-profile
# Required-Start:
# Required-Stop:
# Should-Start:
# Default-Start:
# Default-Stop:
# Short-Description: Loads ICC profile
# Description: Loads ICC profile
### END INIT INFO

PATH=/sbin:/bin:/usr/bin

. /lib/init/vars.sh
. /lib/lsb/init-functions

do_start () {
        # FIXME: root does not have access to current X11
        xcalib -d $DISPLAY "/usr/share/color/icc/N156BGE-L41 #1 2016-11-24 04-15 2.2 M-S XYZLUT+MTX.icc"

        [ "$VERBOSE" != no ] && log_action_begin_msg "Loading ICC profile"
        ES=$?
        [ "$VERBOSE" != no ] && log_action_end_msg $ES
        exit $ES
}

case "$1" in
  start|"")
        do_start
        ;;
  restart|reload|force-reload)
        echo "Error: argument '$1' not supported" >&2
        exit 3
        ;;
  stop)
        # No-op
        ;;
  *)
        echo "Usage: icc-profile.sh [start|stop]" >&2
        exit 3
        ;;
esac

:
