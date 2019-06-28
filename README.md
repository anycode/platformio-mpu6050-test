# platformio-mpu6050-test
Test of MPU6050 accelerometer/gyroscope

## IDE, Libraries
- Platformio (Atom IDE)
- I2Cdevlib-Core_ID11
- I2Cdevlib-MPU6050_ID107

## Hardware
- ESP32 chip
- MPU6050 accelerometer

## Setup
I have MPU6050 interrupt pin connected to ESP32 (Wemos D1 mini) GPIO34 pin to
detect motion and zero motion interrupts. I don't need (and I don't want) to
read accelerometer data in each loop() call.

I observe some strange behaviour.
1. no matter what threshold I set, it's still too sensitive and gives interrupt
   on little movement (even max value 255).
2. when the duration is 9 (or less) the interrupt is fired even on little
   movement. When it's 10 (or more) the interrupt is not fired no matter how
   hard I shake the device.
3. same applies to wakeup. I set the ESP32 to wake up on interrupt on PIN 34
   and put it to sleep. It wakes up on little movement. I tried various
   thresholds and durations.

```
    accelerometer.setIntMotionEnabled(true);
    accelerometer.setMotionDetectionThreshold(255);
    accelerometer.setMotionDetectionDuration(9);
```
