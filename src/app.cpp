#include <Arduino.h>

#include <I2Cdev.h>
#include <MPU6050.h>

#define ACCEL_INT_GPIO_SEL GPIO_SEL_34
#define ACCEL_INT_GPIO_NUM GPIO_NUM_34

MPU6050 accelerometer;

uint8_t range = MPU6050_ACCEL_FS_16; // scale 16g
float scale = 32768.0 / 16.0; // scale of 1g, 16.0 matches range (FS_16)
//uint8_t range = MPU6050_ACCEL_FS_8; // scale 8g
//float scale = 32768.0 / 8.0; // scale of 1g, 8 matches range (FS_8)
//uint8_t range = MPU6050_ACCEL_FS_4; // scale 4g
//float scale = 32768.0 / 4.0; // scale of 1g, 4.0 matches range (FS_4)
//uint8_t range = MPU6050_ACCEL_FS_2; // scale 2g
//float scale = 32768.0 / 2.0; // scale of 1g, 2 matches range (FS_2)
int count = 3;

void setup() {
  Serial.begin(115200);

  pinMode(ACCEL_INT_GPIO_NUM, INPUT);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  uint64_t mask = cause == ESP_SLEEP_WAKEUP_EXT1 ? esp_sleep_get_ext1_wakeup_status() : 0;
  bool wake_up_accel = (mask & ACCEL_INT_GPIO_SEL) == ACCEL_INT_GPIO_SEL;
  Serial.printf("wakeup cause %i, mask %llu, wu accel %i\n", cause, mask, wake_up_accel);

  Serial.println("Initializing I2C devices...");
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  accelerometer.initialize();
  delay(500);

  bool motionDetected = accelerometer.getIntMotionStatus() == 1;
  bool zeroMotionDetected = accelerometer.getIntZeroMotionStatus() == 1;
  Serial.printf("Accelerometer::getWakeUpReason motionDetected %d, zeroMotionDetected %d\n", motionDetected, zeroMotionDetected);

  // verify connection
  Serial.println("Testing device connections...");
  if(accelerometer.testConnection())
    Serial.println("MPU6050 connection successful");
  else
    Serial.println("MPU6050 connection failed");

  accelerometer.setFullScaleAccelRange(range);
  accelerometer.setIntZeroMotionEnabled(true);
  accelerometer.setZeroMotionDetectionThreshold(100);
  accelerometer.setZeroMotionDetectionDuration(3);
  accelerometer.setIntMotionEnabled(true);
  accelerometer.setMotionDetectionThreshold(255);
  accelerometer.setMotionDetectionDuration(8);

  Serial.printf("Range %d, scale %f\n", range, scale);
}

void loop() {
  int16_t ax, ay, az;
  //accelerometer.getAcceleration(&ax, &ay, &az);
  //Serial.printf("motion ax %fg, ay %fg, az %fg\n", ax / scale, ay / scale, az / scale);
  //delay(300);
  int mpuIntStatus = accelerometer.getIntStatus();
  bool motionDetected = mpuIntStatus & _BV(MPU6050_INTERRUPT_MOT_BIT);
  bool zeroMotionDetected = mpuIntStatus & _BV(MPU6050_INTERRUPT_ZMOT_BIT);
  //bool motionDetected = accelerometer.getIntMotionStatus() == 1;
  //bool zeroMotionDetected = accelerometer.getIntZeroMotionStatus() == 1;
  if(motionDetected || zeroMotionDetected)
    Serial.printf("motion detected %d, zero motion detected %d\n", motionDetected, zeroMotionDetected);
  if(motionDetected) {
    accelerometer.getAcceleration(&ax, &ay, &az);
    Serial.printf("#%i motion detected ax %fg (%d), ay %fg (%d), az %fg (%d)\n", count, ax / scale, ax, ay / scale, ay, az / scale, az);
    count--;
  }

  if(count < 0) {
    // sleep
    Serial.println("sleep");
    delay(500);
    accelerometer.setMotionDetectionThreshold(15);
    accelerometer.setMotionDetectionDuration(7);
    accelerometer.setIntMotionEnabled(true);
    accelerometer.setZeroMotionDetectionThreshold(0);
    accelerometer.setZeroMotionDetectionDuration(0);
    accelerometer.setIntZeroMotionEnabled(false);
    delay(100);
    esp_sleep_enable_ext1_wakeup(ACCEL_INT_GPIO_SEL, ESP_EXT1_WAKEUP_ANY_HIGH);
    delay(100);

    esp_deep_sleep_start();
  }

}
