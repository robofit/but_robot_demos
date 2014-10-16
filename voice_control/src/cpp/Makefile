#CC=gcc
#CC=clang
#CP=g++
CP=clang++
CFLAGS=-Wall -Wextra -std=c99 -pedantic -g
CPFLAGS=-Wall -Wextra -std=c++11 -g #-pedantic
LIBS=-lasound -lpocketsphinx -lsphinxbase -lm -lfreenect
LIBS_ROS=-I/opt/ros/hydro/include -L/opt/ros/hydro/lib -lroscpp -lrosconsole -lrostime
LINK_ROS=-rdynamic /opt/ros/hydro/lib/libtf.so /opt/ros/hydro/lib/libtf2_ros.so /opt/ros/hydro/lib/libactionlib.so /opt/ros/hydro/lib/libmessage_filters.so /opt/ros/hydro/lib/libtf2.so /opt/ros/hydro/lib/libconsole_bridge.so /opt/ros/hydro/lib/libtopic_tools.so /opt/ros/hydro/lib/libroscpp.so -lboost_signals -lboost_filesystem /opt/ros/hydro/lib/librosconsole.so /opt/ros/hydro/lib/librosconsole_log4cxx.so /opt/ros/hydro/lib/librosconsole_backend_interface.so -llog4cxx -lboost_regex /opt/ros/hydro/lib/libroscpp_serialization.so /opt/ros/hydro/lib/librostime.so -lboost_date_time -lboost_system -lboost_thread -lpthread /opt/ros/hydro/lib/libxmlrpcpp.so /opt/ros/hydro/lib/libcpp_common.so /opt/ros/hydro/lib/libconsole_bridge.so -Wl,-rpath,/opt/ros/hydro/lib
PROGNAME=kinect

all: $(PROGNAME)

.PHONY: clean

kinect: kinect.cpp
	$(CP) $(CPFLAGS) $(DMODEL_CFG) $(LIBS_SPHINX)  $(LIBS) $< -o $@

clean:
	-rm -f *.o

cleanall: clean
	-rm -f $(PROGNAME)

play:
	-play -e signed-integer -r 16k -b 16 tmp.raw

play32:
	-play -e signed-integer -r 16k -b 32 tmp.raw
