REPO_ROOT := $(shell git -C $(dir $(abspath $(lastword $(MAKEFILE_LIST)))) rev-parse --show-toplevel)
OWL_INC   := $(REPO_ROOT)/phasespace-mocap-ros/phasespace_bringup/include/phasespace_bringup
OWL_LIB   := $(REPO_ROOT)/phasespace-mocap-ros/phasespace_bringup/bin

owl_bridge.so: owl_bridge.cpp
	g++ -shared -fPIC -std=c++17 -O2 \
	    -I$(OWL_INC) \
	    -L$(OWL_LIB) -lowlsock \
	    -Wl,-rpath,$(OWL_LIB) \
	    -o $@ $<

clean:
	rm -f owl_bridge.so
