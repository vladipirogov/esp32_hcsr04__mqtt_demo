/*
 * Display.c
 *
 *  Created on: 14.08.2017
 *      Author: darek
 */
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "freertos/queue.h"
#include "include/MqttService.h"
#include "include/ServoService.h"
#include "include/PidService.h"
#include "include/QueueService.h"

#define PIN_SDA 22
#define PIN_CLK 21

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

void task_initI2C(void *ignore) {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	servo_init();
	pid_init();

	vTaskDelete(NULL);
}

void setup_pid_parameters(char *data) {
	setup_parameters(data);
}

void task_display(void* xQueue){
	printf("Initialilization MPU6050\n");
	MPU6050 mpu = MPU6050();
	mpu.initialize();
	mpu.dmpInitialize();

	// This need to be setup individually
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);

	mpu.setDMPEnabled(true);

	char x_data[sizeof(int32_t)];
	portBASE_TYPE xStatus;
	Tdata data;

	while(1){
	    mpuIntStatus = mpu.getIntStatus();
		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

	    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        // reset so we can continue cleanly
	        mpu.resetFIFO();

	    // otherwise, check for DMP data ready interrupt frequently)
	    } else if (mpuIntStatus & 0x02) {
	        // wait for correct available data length, should be a VERY short wait
	        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

	        // read a packet from FIFO

	        mpu.getFIFOBytes(fifoBuffer, packetSize);
	 		mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			float pitch = ypr[1] * 180/M_PI;
			float roll = ypr[2] * 180/M_PI;
			float yaw = ypr[0] * 180/M_PI;
			printf("YAW: %3.1f, ", yaw);
			printf("PITCH: %3.1f, ", pitch);
			printf("ROLL: %3.1f \n", roll);

			float angle =  pid_control(pitch + 180);
			servo_control((uint32_t)angle);

			itoa((int32_t)pitch+180, x_data, 10);
			data.type = GYROSCOPE;
			data.value = x_data;

			xStatus = xQueueSend( (QueueHandle_t)xQueue, (void *)&data, 0);
			if( xStatus != pdPASS ) {
				printf("Could not send to the queue from gyroscope.\r\n");
			}
	    }

	    //Best result is to match with DMP refresh rate
	    // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
	    // Now its 0x13, which means DMP is refreshed with 10Hz rate
		vTaskDelay(100/portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}
