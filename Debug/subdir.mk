################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../alliance.cpp \
../platform.cpp \
../robot.cpp \
../simulator.cpp 

OBJS += \
./alliance.o \
./platform.o \
./robot.o \
./simulator.o 

CPP_DEPS += \
./alliance.d \
./platform.d \
./robot.d \
./simulator.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


