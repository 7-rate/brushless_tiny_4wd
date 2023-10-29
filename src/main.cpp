#include <Arduino.h>
#include <Servo.h>
#include <tinyNeoPixel.h>
#include <MPU6050.h>

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
// ESC
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define ESC_LEFT_PIN (PIN_PB2)
#define ESC_RIGHT_PIN (PIN_PB1)

// LED
#define LED_RIGHT (0)
#define LED_LEFT (1)

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
int16_t gx, gy, gz; // gz:右回りが正

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


//横G = v^2/r
//最小r = 0.4m 最大r = 0.7m
//7m/s以上はカーブ走行不可(と仮定)
//最大rのときの横G閾値 = 70m/s^2
bool rolling_judge() {
    return (gz > 700);
}

//1.2G以上
bool sloop_judge() {
    return (gz > 12);
}


// gzに応じて左右のモーターの駆動力分配を行う
// 外輪を100とし、横G=5のときは内輪を50とする
static void run_stable(int gx) {

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

}

/***********************************/
/* Global functions                */
/***********************************/
void setup() {
    // pinMode(ESC_LEFT_PIN, OUTPUT);
    // pinMode(ESC_RIGHT_PIN, OUTPUT);
    
    // esc_left.attach(ESC_LEFT_PIN);
    // esc_right.attach(ESC_RIGHT_PIN);

    // delay(2000); //ESC 位置同定待ち

    // esc_left.writeMicroseconds(MIN_SIGNAL);
    // esc_right.writeMicroseconds(MIN_SIGNAL);
    // delay(2000);

    // esc_left.writeMicroseconds(MAX_SIGNAL);
    // esc_right.writeMicroseconds(MAX_SIGNAL);

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
    // switch (run_mode) {
    //     case RUN_MODE_STABLE:
    //         run_stable(gx);
    //         if (run_event == RUN_EVENT_ROLLING) {
    //             run_mode = RUN_MODE_ROLLOVER;
    //         } else if (run_event == RUN_EVENT_SLOOPING) {
    //             run_mode = RUN_MODE_SLOOP;
    //         }
    //         break;
    //     case RUN_MODE_ROLLOVER:
    //         //run_rollover(gx);
    //         if (run_event == RUN_EVENT_NONE) {
    //             run_mode = RUN_MODE_STABLE;
    //         }
    //         break;
    //     case RUN_MODE_SLOOP:
    //         //run_sloop(gx);
    //         if (run_event == RUN_EVENT_NONE) {
    //             run_mode = RUN_MODE_STABLE;
    //         }
    //         break;
    // }

    //neopixel
    uint8_t val_left, val_right;
    
    pixels.clear();
    if (gx > 6000) { //傾斜突入 4m/sで10度 50cm→80deg/s→2500以上
        pixels.setPixelColor(LED_LEFT, pixels.ColorHSV(0, 255, 255));
        pixels.setPixelColor(LED_RIGHT, pixels.ColorHSV(0, 255, 255));
    } else { //カーブ走行
        if (gz > 0) {
            val_left = 127;
            val_right = 127 - abs((gz >> 8));
        } else {
            val_left = 127 - abs((gz >> 8));
            val_right = 127;
        }
        pixels.setPixelColor(LED_LEFT, pixels.ColorHSV(16384, 255, val_left));
        pixels.setPixelColor(LED_RIGHT, pixels.ColorHSV(16384, 255, val_right));
    }
    pixels.show();
    

    delay(100);
}
