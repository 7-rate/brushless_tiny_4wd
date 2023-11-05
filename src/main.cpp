#include <Arduino.h>
#include <Servo.h>
#include <tinyNeoPixel.h>
#include <MPU6050.h>

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Calibration parameter           */
/***********************************/
// ESC 
#define MAX_SIGNAL 2000 // power 100%
#define MIN_SIGNAL 1000 // power 0%

// 横転ギリギリ角速度
#define MAX_ROLLING_GZ (29475) //450deg/s * 65.5(LSB)

// スロープ検出角速度
#define SLOOP_GX (3275) //50deg/s * 65.5(LSB)

/***********************************/
/* Local definitions               */
/***********************************/
// ESC
#define ESC_LEFT_PIN (PIN_PA4)
#define ESC_RIGHT_PIN (PIN_PA5)

// LED
#define LED_RIGHT (0)
#define LED_LEFT (1)

#define COLOR_SLOOP_PRE   (pixels.ColorHSV(43200, 255, 255)) //BLUE
#define COLOR_SLOOP_DET   (pixels.ColorHSV(0, 255, 255)) //RED
#define COLOR_ROLLING_PRE (pixels.ColorHSV(21600, 255, 255)) //GREEN
#define COLOR_ROLLING_DET (pixels.ColorHSV(10800, 255, 255)) //YELLOW

// MPU6050
#define MPU6050_XA_OFFSET (-130 )
#define MPU6050_YA_OFFSET ( 1878 )
#define MPU6050_ZA_OFFSET ( 1540 )
#define MPU6050_XG_OFFSET ( 115 )
#define MPU6050_YG_OFFSET ( -49 )
#define MPU6050_ZG_OFFSET ( -29 )

// Run mode/event
enum {
    RUN_MODE_STABLE,
    RUN_MODE_ROLLOVER,
    RUN_MODE_SLOOP
};
enum {
    RUN_EVENT_ROLLING_PRE,  //ローリング検出中
    RUN_EVENT_ROLLING,      //ローリング制御開始
    RUN_EVENT_SLOOPING_PRE, //スロープ検出中
    RUN_EVENT_SLOOPING,     //スロープ制御開始
    RUN_EVENT_NONE
};
#define ROLLING_DETECT_TIME (50) //50ms
#define SLOOP_DETECT_TIME (50) //50ms

/***********************************/
/* Local Variables                 */
/***********************************/
// ESC
Servo esc_left;
Servo esc_right;

// LED
tinyNeoPixel pixels = tinyNeoPixel(2, PIN_PA1, NEO_GRB + NEO_KHZ800);

// MPU6050
MPU6050 mpu(0x68);
int16_t ax, ay, az;
int16_t gx, gy, gz; // LSB 0.1deg/s gz:右回りが正 range=500deg/s 65.5 = 1deg/s

// Run mode/event
int run_mode = RUN_MODE_STABLE;
int run_event = RUN_EVENT_NONE;
bool rolling_pre = false;
bool slooping_pre = false;
unsigned long rolling_pre_time;
unsigned long slooping_pre_time;

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/
#define limit(x, min, max) do { if (x < min) x = min; else if (x > max) x = max; } while(0)

//0~100をMIN_SIGNAL~MAX_SIGNALで線形補完してwriteMicrosecondsに指定する
void motor_drive(int left, int right) {
    limit(left, 0, 100);
    limit(right, 0, 100);

    int left_signal = (MAX_SIGNAL - MIN_SIGNAL) * left / 100 + MIN_SIGNAL;
    int right_signal = (MAX_SIGNAL - MIN_SIGNAL) * right / 100 + MIN_SIGNAL;

    esc_left.writeMicroseconds(left_signal);
    esc_right.writeMicroseconds(right_signal);
}


//横転ギリギリ検出
bool rolling_judge() {
    uint16_t roll = abs(gz);
    return (roll > MAX_ROLLING_GZ);
}

//上りスロープ検出
bool sloop_judge() {
    uint16_t sloop = abs(gx);
    return (sloop > SLOOP_GX);
}

// gzに応じて左右モーターを制御する
//gzから旋回中の左右駆動配分を決める。
//ロールによる速度限界に近づくにつれ、内側のモーターを減速させる。
//0deg/sを100%とし、450deg/sを0%とする。
static void run_stable() {
    long inside;

    inside = 100l - ((abs(gz) * 100l) / MAX_ROLLING_GZ);
    limit(inside, 0, 100);

    if (gz > 0) {
        motor_drive(100, inside);

    } else {
        motor_drive(inside, 100);
    }
}

// Run event判定
static void run_event_process() {
    run_event = RUN_EVENT_NONE;

    //rolling_judgeがROLLING_DETECT_TIME(msec)継続したとき、run_event=RUN_EVENT_ROLLING
    //rolling_judgeがROLLING_DETECT_TIME(msec)未満のとき、run_event=RUN_EVENT_ROLLING_PRE
    if (rolling_judge()) {
        if (rolling_pre) {
            if (millis() - rolling_pre_time > ROLLING_DETECT_TIME) {
                run_event = RUN_EVENT_ROLLING;
            }
        } else {
            rolling_pre = true;
            rolling_pre_time = millis();
            run_event = RUN_EVENT_ROLLING_PRE;
        }
    } else {
        rolling_pre = false;
    }

    //slope_judgeがSLOOP_DETECT_TIME(msec)継続したとき、run_event=RUN_EVENT_SLOOPING_PRE
    //slope_judgeがSLOOP_DETECT_TIME(msec)未満のとき、run_event=RUN_EVENT_SLOOPING
    if (sloop_judge()) {
        if (slooping_pre) {
            if (millis() - slooping_pre_time > SLOOP_DETECT_TIME) {
                run_event = RUN_EVENT_SLOOPING;
            }
        } else {
            slooping_pre = true;
            slooping_pre_time = millis();
            run_event = RUN_EVENT_SLOOPING_PRE;
        }
    } else {
        slooping_pre = false;
    }
}

// Run mode/eventに応じて以下LED処理を行う
// Slope検出中：青色
// Slope制御中：赤色
// Rolling検出中：緑色
// Rolling制御中：黄色
static void led_process() {
    uint32_t left_c, right_c;
    uint8_t val_left, val_right;

    pixels.clear();
    switch (run_mode) {
        case RUN_EVENT_ROLLING_PRE:
            left_c = COLOR_ROLLING_PRE;
            right_c = COLOR_ROLLING_PRE;
            break;
        case RUN_EVENT_ROLLING:
            left_c = COLOR_ROLLING_DET;
            right_c = COLOR_ROLLING_DET;
            break;
        case RUN_EVENT_SLOOPING_PRE:
            left_c = COLOR_SLOOP_PRE;
            right_c = COLOR_SLOOP_PRE;
            break;
        case RUN_EVENT_SLOOPING:
            left_c = COLOR_SLOOP_DET;
            right_c = COLOR_SLOOP_DET;
            break;
        case RUN_EVENT_NONE:
        default:
            if (gz > 0) {
                val_left = 127;
                val_right = 127 - abs((gz >> 8));
            } else {
                val_left = 127 - abs((gz >> 8));
                val_right = 127;
            }
            left_c = pixels.ColorHSV(16384, 255, val_left);
            right_c = pixels.ColorHSV(16384, 255, val_right);
            break;
    }
    pixels.setPixelColor(LED_LEFT, left_c);
    pixels.setPixelColor(LED_RIGHT, right_c);
    pixels.show();
}

/***********************************/
/* Global functions                */
/***********************************/
void setup() {
    pinMode(ESC_LEFT_PIN, OUTPUT);
    pinMode(ESC_RIGHT_PIN, OUTPUT);
    
    // esc_left.attach(ESC_LEFT_PIN);
    esc_right.attach(ESC_RIGHT_PIN);

    delay(2000); //ESC 位置同定待ち

    // esc_left.writeMicroseconds(MIN_SIGNAL);
    esc_right.writeMicroseconds(MIN_SIGNAL);
    delay(2000);

    // esc_left.writeMicroseconds(1200);
    esc_right.writeMicroseconds(1200);

    //mpu
    Wire.begin();
    mpu.initialize();
    mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_500); // R=400のコースを10m/sで通過することを想定したレンジ
    mpu.setXAccelOffset(MPU6050_XA_OFFSET);
    mpu.setYAccelOffset(MPU6050_YA_OFFSET);
    mpu.setZAccelOffset(MPU6050_ZA_OFFSET);
    mpu.setXGyroOffset(MPU6050_XG_OFFSET);
    mpu.setYGyroOffset(MPU6050_YG_OFFSET);
    mpu.setZGyroOffset(MPU6050_ZG_OFFSET);

    //neopixel
    pixels.begin();
}

void loop() {
    // //mpu
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // //run mode
    run_event_process();
    switch (run_mode) {
        case RUN_MODE_STABLE:
            run_stable();
            if (run_event == RUN_EVENT_ROLLING) {
                run_mode = RUN_MODE_ROLLOVER;
            } else if (run_event == RUN_EVENT_SLOOPING) {
                run_mode = RUN_MODE_SLOOP;
            }
            break;
        case RUN_MODE_ROLLOVER:
            //run_rollover(gx);
            if (run_event == RUN_EVENT_NONE) {
                run_mode = RUN_MODE_STABLE;
            }
            break;
        case RUN_MODE_SLOOP:
            //run_sloop(gx);
            if (run_event == RUN_EVENT_NONE) {
                run_mode = RUN_MODE_STABLE;
            }
            break;
    }

    //neopixel
    led_process();
}
