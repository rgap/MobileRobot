################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../3D_Processing/CloudDownsampling/VoxelDownsampling.cpp 

OBJS += \
./3D_Processing/CloudDownsampling/VoxelDownsampling.o 

CPP_DEPS += \
./3D_Processing/CloudDownsampling/VoxelDownsampling.d 


# Each subdirectory must supply rules for building sources it contributes
3D_Processing/CloudDownsampling/%.o: ../3D_Processing/CloudDownsampling/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include -I/usr/include -I/usr/local/include/opencv -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 -I/usr/include/ni -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


