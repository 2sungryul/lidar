CX = g++    
CXFLAGS = -Wall  
CXFLAGS += -I/home/ncslab/rplidar_sdk/sdk/include -I/home/ncslab/rplidar_sdk/sdk/src
LD_LIBS = -lstdc++ -lpthread -lrt  
LD_LIBS += -L/home/ncslab/rplidar_sdk/output/Linux/Release/ -lsl_lidar_sdk
OPENCV = `pkg-config opencv4 --cflags --libs`  
CXSRC = main.cpp
TARGET = lidar
OBJS = 
$(TARGET) :  $(CXSRC)
	$(CX) $(CXFLAGS) -o $(TARGET) $(CXSRC) $(LD_LIBS) $(OPENCV)

.PHONY: all clean

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(OBJS)

