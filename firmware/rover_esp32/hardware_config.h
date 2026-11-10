#pragma once
// Original ESP32 Dev Module, Arduino ESP32 core 2.0.17. Not an S2/S3/C3 pin map.
constexpr int PWM_L=25, DIR_L=27, PWM_R=26, DIR_R=14;
constexpr int ENC_L_A=34, ENC_L_B=35, ENC_R_A=32, ENC_R_B=33;
constexpr int SDA_PIN=21, SCL_PIN=22;
// Set each independently after a lifted-wheel polarity check.
constexpr int MOTOR_L_SIGN=1, MOTOR_R_SIGN=1;
constexpr int ENCODER_L_SIGN=1, ENCODER_R_SIGN=1;
constexpr int MPU_ADDR=0x68;
