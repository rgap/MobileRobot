################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../2D_Processing/CanRecognition.cpp \
../2D_Processing/PreProcess.cpp \
../2D_Processing/Retinex.cpp \
../2D_Processing/WaterRecognition.cpp 

OBJS += \
./2D_Processing/CanRecognition.o \
./2D_Processing/PreProcess.o \
./2D_Processing/Retinex.o \
./2D_Processing/WaterRecognition.o 

CPP_DEPS += \
./2D_Processing/CanRecognition.d \
./2D_Processing/PreProcess.d \
./2D_Processing/Retinex.d \
./2D_Processing/WaterRecognition.d 


# Each subdirectory must supply rules for building sources it contributes
2D_Processing/%.o: ../2D_Processing/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include -I/usr/include -I/usr/local/include/opencv -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 -I/usr/include/ni -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


