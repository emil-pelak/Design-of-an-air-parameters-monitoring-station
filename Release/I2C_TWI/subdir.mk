################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../I2C_TWI/i2c_twi.c 

OBJS += \
./I2C_TWI/i2c_twi.o 

C_DEPS += \
./I2C_TWI/i2c_twi.d 


# Each subdirectory must supply rules for building sources it contributes
I2C_TWI/%.o: ../I2C_TWI/%.c I2C_TWI/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega32 -DF_CPU=11059200UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


