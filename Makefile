UNAME := $(shell uname -s)
SIM_SRC = alliance.cpp  displayPlatform.cpp  platform.cpp  robot.cpp  simulator.cpp
SIM_OBJ = alliance.o  displayPlatform.o  platform.o  robot.o  simulator.o

SIM_EXE = simulator
ifeq ($(UNAME),Darwin)
	SIM_EXE = simulator.mac
endif

CPPFLAGS = -c -Wall -O3 -g
INCLUDE  = -I.
LDFLAGS  = -g -lopencv_core -lopencv_imgproc -lopencv_photo -lopencv_highgui -lopencv_calib3d -lpthread

all: $(SIM_SRC) $(SIM_EXE)

$(SIM_EXE): $(SIM_OBJ)
	$(CXX) -o $@ $(SIM_OBJ) $(LDFLAGS) 

.cpp.o: $(SIM_SRC)
	$(CXX) $(CPPFLAGS) $(INCLUDE) $< -o $@

.PHONY: clean
clean:
	@$(RM) *.o $(SIM_EXE)
