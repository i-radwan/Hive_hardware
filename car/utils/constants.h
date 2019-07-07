#pragma once

#define CAR1
// #define CAR2
// #define CAR3


// ====================
// Communication

const char* NET_NAME                        = "Ibrahim's iPhone";
const char* NET_PASS                        = "20061996";
const char* SERVER                          = "169.254.97.178";
const int PORT                              = 12344;
const int RECONNECT_INTERVAL                = 5000; // ms
const int PING_INTERVAL                     = 100; // ms
const int PONG_TIMEOUT                      = 3000; // ms
const int RETRIES_COUNT                     = 2;
const int DONE_DELAY                        = 1000; // ms

// Serial
const int BAUD_RATE                         = 115200;


// ====================
// Motors

// Servo
const int SERVO_PIN                         = D3;
const int SERVO_DOWN_ANGLE                  = 0;
const int SERVO_UP_ANGLE                    = 50;

// Motors pins
const int LEFT_DIR1                         = 0;
const int LEFT_DIR2                         = 1;
const int LEFT_SPED                         = D4;

const int RIGHT_DIR1                        = 2;
const int RIGHT_DIR2                        = 3;
const int RIGHT_SPED                        = D0;

// Motors encoders
const int LEFT_ENC                          = D1;
const int RGHT_ENC                          = D2;

// Motor directions
enum class DIRECTION {
    FORWARD                                 = 0,
    BACKWARD                                = 1
};

// Configs
const double STEP                           = 300; // mm
const double EXCESS_DISTANCE_LIMIT          = STEP; // mm, the distance the robot can go beyond the black line before reporting to the server.
const double EXCESS_ANGLES_LIMIT            = 10; // degrees, the angles the robot can rotate beyond the 90 degrees.
const double HOLD_STATE_PERIOD              = 1000; // ms, time to wait before changing states

// PID
#ifdef CAR1
const double MOTORS_ADJUST_DELTA            = 100; // ms
const double MOTORS_SPEED                   = 50; // rpm
const double MOTORS_ROTATION_SPEED          = 70; // rpm
const double MOTORS_MOVE_SPEED_INCREMENT    = 7; // rpm
const double MOTORS_ROTATE_SPEED_INCREMENT  = 10; // rpm
const double LEFT_INIT_THROTTLE             = PWMRANGE / 6; // pwm
const double RIGHT_INIT_THROTTLE            = PWMRANGE / 6; // pwm

const double LEFT_KP                        = 1.8;
const double LEFT_KI                        = 0.07;
const double LEFT_KD                        = 0;
const double RIGHT_KP                       = 1.8;
const double RIGHT_KI                       = 0.07;
const double RIGHT_KD                       = 0;

const double C1                             = 0.8;
const double C2                             = 0.7;
const double C3                             = 0.6;
const double C4                             = 0.4;
#endif

#ifdef CAR2
const double MOTORS_ADJUST_DELTA            = 100; // ms
const double MOTORS_SPEED                   = 50; // rpm
const double MOTORS_ROTATION_SPEED          = 80; // rpm
const double MOTORS_ROTATE_SPEED_INCREMENT  = 40; // rpm
const double MOTORS_MOVE_SPEED_INCREMENT    = 70; // rpm
const double LEFT_INIT_THROTTLE             = PWMRANGE / 3.8; // pwm
const double RIGHT_INIT_THROTTLE            = PWMRANGE / 3.8; // pwm

const double LEFT_KP                        = 2.7;
const double LEFT_KI                        = 0.07;
const double LEFT_KD                        = 0;
const double RIGHT_KP                       = 2.7;
const double RIGHT_KI                       = 0.07;
const double RIGHT_KD                       = 0;

const double C1                             = 0.9;
const double C2                             = 0.9;
const double C3                             = 0.6;
const double C4                             = 0.7;
#endif

#ifdef CAR3
const double MOTORS_ADJUST_DELTA            = 150; // ms
const double MOTORS_SPEED                   = 50; // rpm
const double MOTORS_ROTATION_SPEED          = 70; // rpm
const double MOTORS_MOVE_SPEED_INCREMENT    = 6; // rpm
const double MOTORS_ROTATE_SPEED_INCREMENT  = 10; // rpm
const double LEFT_INIT_THROTTLE             = PWMRANGE / 3.5; // pwm
const double RIGHT_INIT_THROTTLE            = PWMRANGE / 3.2; // pwm

const double LEFT_KP                        = 1.8;
const double LEFT_KI                        = 0.07;
const double LEFT_KD                        = 0;
const double RIGHT_KP                       = 1.8;
const double RIGHT_KI                       = 0.07;
const double RIGHT_KD                       = 0;

const double C1                             = 0.6;
const double C2                             = 0.5;
const double C3                             = 0.4;
const double C4                             = 0.2;
#endif

// Wheel
const double DISK_SLOTS                     = 20.0;
const int WHEEL_DIAMETER                    = 65; // mm
const int DEBOUNCE_DELTA                    = 1000; // us


// ====================
// Sensors

// MPU
const int MPU_ADDRESS                       = 0x68;
const int I2C_SCL                           = D6;
const int I2C_SDA                           = D7;
const int MPU_INTR                          = D8;
const double MPU_REFRESH_RATE               = 0; // ms

#ifdef CAR1
const double MPU_ACCEL_X_OFF                = -24280 / 8.8;
const double MPU_ACCEL_Y_OFF                = -1100 / 6.3;
const double MPU_ACCEL_Z_OFF                = 7000 / 6.9;
const double MPU_GYRO_X_OFF                 = -78 / 3.5;
const double MPU_GYRO_Y_OFF                 = -391 / 4.05;
const double MPU_GYRO_Z_OFF                 = -185 / 3.5;
#endif

#ifdef CAR2
const double MPU_ACCEL_X_OFF                = 3810 / 8.8;
const double MPU_ACCEL_Y_OFF                = 4499.03 / 6.3;
const double MPU_ACCEL_Z_OFF                = 7130 / 6.9;
const double MPU_GYRO_X_OFF                 = 145.85 / 3.5;
const double MPU_GYRO_Y_OFF                 = -175.27 / 4.05;
const double MPU_GYRO_Z_OFF                 = -32 / 3.5;
#endif

#ifdef CAR3
const double MPU_ACCEL_X_OFF                = -17530 / 8.8;
const double MPU_ACCEL_Y_OFF                = 200 / 6.3;
const double MPU_ACCEL_Z_OFF                = 9806 / 6.9;
const double MPU_GYRO_X_OFF                 = 342.55 / 3.5;
const double MPU_GYRO_Y_OFF                 = 144.27 / 4.05;
const double MPU_GYRO_Z_OFF                 = -141 / 3.5;
#endif

// Optical sensor
const int OPTICAL_CLCK                      = 1;
const int OPTICAL_DATA                      = 3;
const double OPTICAL_DPI_TO_MM              = 1 / 39.0;

// Ultrasonic
const int ULTRA_SONIC_TRIGGER_PIN           = D9;
const int ULTRA_SONIC_ECHO_PIN              = D5;
const int ULTRA_SONIC_REFRESH_TIME          = 50; // ms
const int ULTRA_SONIC_TIMEOUT               = 2000; // us ... 2cm * 2     = 40cm     = ~0.5m takes the sound 300 m/s -> 0.001666666667 seconds to travel     = 1666.667us
const int MIN_DISTANCE                      = 15; // cm. Min distance ahead of the robot

// Battery
const int BATTERY_SENSOR_PIN                = A0;
const int BATTERY_LEVEL_SIZE                = 10;  // Each level represents 10% of the battery life.
const double BATTERY_REF_VOLT               = 12.18;
const double BATTERY_EMPTY_REF_VOLT         = 10.2;
const double BATTERY_DIVIDED_REF_VOLT       = 3.1; // Divided by 15kΩ & [5kΩ]
const double BATTERY_WARNING_LEVEL          = 3;
const double BATTERY_RESISTORS_RATIO        = 20 / 5; // (15 + 5) / 5
const double BATTERY_SENSOR_REFRESH_RATE    = 5000; // ms

// Black sensors
const int FRT_LFT_BLACK_SENSOR_PIN          = 6;
const int FRT_CNT_BLACK_SENSOR_PIN          = D8;
const int FRT_RGT_BLACK_SENSOR_PIN          = 7;
const int BAK_LFT_BLACK_SENSOR_PIN          = 4;
const int BAK_RGT_BLACK_SENSOR_PIN          = 5;

// Car 1 --> IP : .3
#ifdef CAR1
const bool FRT_LFT_BLACK_SENSOR_INV         = false;
const bool FRT_CNT_BLACK_SENSOR_INV         = false;
const bool FRT_RGT_BLACK_SENSOR_INV         = false;
const bool BAK_LFT_BLACK_SENSOR_INV         = true;
const bool BAK_RGT_BLACK_SENSOR_INV         = true;
#endif

// Car 2 --> IP : .2
#ifdef CAR2
const bool FRT_LFT_BLACK_SENSOR_INV         = true;
const bool FRT_CNT_BLACK_SENSOR_INV         = false;
const bool FRT_RGT_BLACK_SENSOR_INV         = true;
const bool BAK_LFT_BLACK_SENSOR_INV         = false;
const bool BAK_RGT_BLACK_SENSOR_INV         = false;
#endif

// Car 3 --> IP : .4
#ifdef CAR3
const bool FRT_LFT_BLACK_SENSOR_INV         = true;
const bool FRT_CNT_BLACK_SENSOR_INV         = false;
const bool FRT_RGT_BLACK_SENSOR_INV         = true;
const bool BAK_LFT_BLACK_SENSOR_INV         = false;
const bool BAK_RGT_BLACK_SENSOR_INV         = false;
#endif

const bool FRT_LFT_BLACK_SENSOR_PCF         = true;
const bool FRT_CNT_BLACK_SENSOR_PCF         = false;
const bool FRT_RGT_BLACK_SENSOR_PCF         = true;
const bool BAK_LFT_BLACK_SENSOR_PCF         = true;
const bool BAK_RGT_BLACK_SENSOR_PCF         = true;


// ====================
// General

// PWM
const int PWM_FREQUENCY                     = 1; // In range 1-1000 KHz

// PCF8574
const int PCF1_ADDRESS                      = 0x20;
const int PCF1_CONFIGS                      = 0xF0;

// LEDs
const int BLUE_LED_PIN                      = D3;
const int RED_LED_PIN                       = D10;
const int FLASH_PERIOD                      = 750; // ms

const double EPS                            = 1e-6;


// ====================
// FSM
enum class EXECUTION_STATE {
    IDLE                                    = 0, // Stopped and waiting for a new action
    ONGOING                                 = 1, // Exeuting action
    FINISHED                                = 2, // Finished the action
    PAUSE                                   = 3, // Paused due to server STOP command or blockage
    ERROR                                   = 4  // Unexpected error occured
};

enum class EXECUTION_ERROR {
    NONE                                    = -1,
    EXCEEDED_ALLOWED_DISTANCE               = 0,
    UNKNOWN_ROTATION                        = 1,
    UNKNOWN                                 = 2
};

enum class MOVE_STATE {
    NONE                                    = -1,
    HOLD                                    = 0,
    STRAIGHT                                = 1,
    STRAIGHT_LEFT                           = 2,
    STRAIGHT_RIGHT                          = 3,
    DRIFTING_LEFT                           = 4,
    DRIFTING_RIGHT                          = 5,
    OFFLINE_LEFT                            = 6,
    OFFLINE_RIGHT                           = 7,
    ALIGNMENT                               = 8  // Move using the angle signal only
};

enum class ROTATE_STATE {
    NONE                                    = -1,
    PREPARE_ROTATE_RIGHT                    = 0,
    PREPARE_ROTATE_LEFT                     = 1,
    PRE_ROTATE_RIGHT                        = 2,
    PRE_ROTATE_LEFT                         = 3,
    ROTATE_RIGHT                            = 4,
    ROTATE_LEFT                             = 5,
    POST_ROTATE_RIGHT                       = 6,
    POST_ROTATE_LEFT                        = 7,
    FINISH_ROTATE_RIGHT                     = 8,
    FINISH_ROTATE_LEFT                      = 9,
    FAILED_ROTATE_RIGHT_OFFLINE_RIGHT       = 10,
    FAILED_ROTATE_LEFT_OFFLINE_RIGHT        = 11,
    FAILED_ROTATE_RIGHT_OFFLINE_LEFT        = 12,
    FAILED_ROTATE_LEFT_OFFLINE_LEFT         = 13
};

enum class RETREAT_STATE {
    NONE                                    = -1,
    HOLD                                    = 0,
    RETREAT                                 = 1,
    POST_RETREAT_MOVE                       = 2,
    POST_RETREAT_ALIGNMENT                  = 3
};

struct ExecutionState {
    EXECUTION_STATE state                   = EXECUTION_STATE::IDLE;
    EXECUTION_ERROR error                   = EXECUTION_ERROR::NONE;
};

// ====================
// Communication Protocol

// From server
enum class MSG_FROM_SERVER {
    CONFIG                                  = 0,
    ACTION                                  = 1,
    LIGHT                                   = 2
};

enum class ACTION {
    NONE                                    = -1,
    STOP                                    = 0,
    MOVE                                    = 1,
    ROTATE_RIGHT                            = 2,
    ROTATE_LEFT                             = 3,
    RETREAT                                 = 4,
    LOAD                                    = 5,
    OFFLOAD                                 = 6
};

enum class LIGHT {
    RED                                     = 0,
    BLUE                                    = 1
};

enum class LIGHT_MODE {
    OFF                                     = 0,
    ON                                      = 1,
    FLASH                                   = 2
};

enum class ACTION_TYPE {
    NORMAL                                  = 0,
    RECOVER                                 = 1
};

// To server
enum class MSG_TO_SERVER {
    DONE                                    = 0,
    BATTERY                                 = 1,
    BLOCKING                                = 2,
    ERROR                                   = 3
};

enum class BLOCKING_MODE {
    UNBLOCKED                               = 0,
    BLOCKED                                 = 1
};

enum class SERVER_TASKS {
    OTHER                                   = -1,
    CONFIG                                  = 0,
    STOP                                    = 1,
    MOVE                                    = 2,
    ROTATE_RIGHT                            = 3,
    ROTATE_LEFT                             = 4,
    RETREAT                                 = 5,
    RECOVER_MOVE                            = 6,
    RECOVER_ROTATE_RIGHT                    = 7,
    RECOVER_ROTATE_LEFT                     = 8,
    RECOVER_RETREAT                         = 9,
    LOAD                                    = 10,
    OFFLOAD                                 = 11,
    RED_LED_OFF                             = 12,
    RED_LED_ON                              = 13,
    RED_LED_FLASH                           = 14,
    BLUE_LED_OFF                            = 15,
    BLUE_LED_ON                             = 16,
    BLUE_LED_FLASH                          = 17
};