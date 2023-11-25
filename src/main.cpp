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
#define MAX_SIGNAL 1600 // power 100%
#define MIN_SIGNAL 900 // power 0%

// 横転ギリギリ角速度
#define MAX_ROLLING_GZ (20000) //450deg/s * 65.5(LSB)
#define MAX_CURVE_GZ (10000) //450deg/s * 65.5(LSB)

// スロープ検出角速度
#define SLOPE_GX (1000) //50deg/s * 65.5(LSB)

// 横転ギリギリ検出時間
#define ROLLING_DETECT_TIME (200) //ms

// スロープ検出時間
#define SLOPE_DETECT_TIME (50) //ms

// スロープ確定からの減速時間
#define SLOPE_SLOW_DOWN_TIME (200) //ms

// スロープ検出マスク時間
#define SLOPE_DETECT_MASK_TIME (500) //ms

/***********************************/
/* Local definitions               */
/***********************************/
// ESC
#define ESC_LEFT_PIN (PIN_PA4)
#define ESC_RIGHT_PIN (PIN_PA5)

// LED
#define LED_RIGHT (0)
#define LED_LEFT (1)

#define COLOR_SLOPE_PRE   (pixels.ColorHSV(43200, 255, 255)) //BLUE
#define COLOR_SLOPE_DET   (pixels.ColorHSV(0, 255, 255)) //RED
#define COLOR_ROLLING_PRE (pixels.ColorHSV(49151, 255, 255)) //PURPLE
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
    RUN_MODE_SLOPE
};
enum {
    RUN_EVENT_ROLLING_PRE,  //ローリング検出中
    RUN_EVENT_ROLLING,      //ローリング制御開始
    RUN_EVENT_ROLLLING_END, //ローリング制御終了
    RUN_EVENT_SLOPING_PRE, //スロープ検出中
    RUN_EVENT_SLOPING,     //スロープ制御開始
    RUN_EVENT_NONE
};

#define RUN_LIMIT_TIME (60000) //msec

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
bool sloping_pre = false;
unsigned long tmr_rolling_pre;
unsigned long tmr_sloping_pre;

// slope用
bool slope_detected_mask = false;
unsigned long tmr_slope_detected_mask;

unsigned long tmr_slope_slow_down;

// 走行時間制限
unsigned long tmr_run_limit;

// motor command平滑用
unsigned long tmr_motor_command;
int left_signal_old = 0;
int right_signal_old = 0;

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/
#define limit(x, min, max) do { if (x < min) x = min; else if (x > max) x = max; } while(0)

void motor_command(int left_signal, int right_signal) {
    int left = (left_signal * 8 + left_signal_old * 2)/10;
    int right = (right_signal * 8 + right_signal_old * 2)/10;
    esc_left.writeMicroseconds(left);
    esc_right.writeMicroseconds(right);
    left_signal_old = left_signal;
    right_signal_old = right_signal;
    tmr_motor_command = millis();
}

//0~100をMIN_SIGNAL~MAX_SIGNALで線形補完してwriteMicrosecondsに指定する
void motor_drive(int left, int right) {
    if (tmr_motor_command != millis()) {
        limit(left, 0, 100);
        limit(right, 0, 100);

        int left_signal = (MAX_SIGNAL - MIN_SIGNAL) * left / 100 + MIN_SIGNAL;
        int right_signal = (MAX_SIGNAL - MIN_SIGNAL) * right / 100 + MIN_SIGNAL;

        motor_command(left_signal, right_signal);
    }
}


//横転ギリギリ検出
bool rolling_judge() {
    uint16_t roll = abs(gz);
    return (roll > MAX_ROLLING_GZ);
}

//上りスロープ検出
bool slope_judge() {
    uint16_t slope = abs(gx);
    return (slope > SLOPE_GX);
}

// gzに応じて左右モーターを制御する
// gzから旋回中の左右駆動配分を決める。
// ロールによる速度限界に近づくにつれ、内側のモーターを減速させる。
// 0deg/sを100%とし、450deg/sを0%とする。
static void run_stable() {
    long inside;

    inside = 100l - ((abs(gz) * 100l) / MAX_CURVE_GZ);
    limit(inside, 0, 100);

    if (gz > 0) {
        motor_drive(100, inside);

    } else {
        motor_drive(inside, 100);
    }
}

//横転回避制御
//外側モーターを30%とし、内側モーターを0%とする。
//400deg/s以下になったところで通常制御に移行する。
static void run_rollover() {
    if (gz > 0) {
        motor_drive(10, 0);

    } else {
        motor_drive(0, 10);
    }
}

//スロープ制御
//スロープを検出した場合は両モーターを50ms間0%にし減速する。
static void run_slope() {
    motor_drive(20, 20);
}

// Run event判定
static void run_event_process() {
    int run_event_old = run_event;
    run_event = RUN_EVENT_NONE;

    //rolling_judgeがROLLING_DETECT_TIME(msec)継続したとき、run_event=RUN_EVENT_ROLLING
    //rolling_judgeがROLLING_DETECT_TIME(msec)未満のとき、run_event=RUN_EVENT_ROLLING_PRE
    if (rolling_judge()) {
        run_event = RUN_EVENT_ROLLING_PRE;
        if (rolling_pre) {
            if (millis() - tmr_rolling_pre > ROLLING_DETECT_TIME) {
                run_event = RUN_EVENT_ROLLING;
            }
        } else {
            rolling_pre = true;
            tmr_rolling_pre = millis();
        }
    } else {
        rolling_pre = false;
    }

    //slope_judgeがSLOPE_DETECT_TIME(msec)継続したとき、run_event=RUN_EVENT_SLOPING
    //slope_judgeがSLOPE_DETECT_TIME(msec)未満のとき、run_event=RUN_EVENT_SLOPING_PRE
    //2秒以内に再度スロープを検出した場合は下りスロープであるため、減速処理は行わない。→slope_detected_maskにてマスクを行う
    if (slope_judge() && !slope_detected_mask) {
        run_event = RUN_EVENT_SLOPING_PRE;
        if (sloping_pre) {
            if (millis() - tmr_sloping_pre > SLOPE_DETECT_TIME) {
                run_event = RUN_EVENT_SLOPING;
            }
        } else {
            sloping_pre = true;
            tmr_sloping_pre = millis();
        }
    } else {
        sloping_pre = false;
    }
    if (run_event_old == RUN_EVENT_SLOPING && run_event != RUN_EVENT_SLOPING) {
        slope_detected_mask = true;
        tmr_slope_detected_mask = millis();
    }
    if (slope_detected_mask && millis() - tmr_slope_detected_mask > SLOPE_DETECT_MASK_TIME) {
        slope_detected_mask = false;
    }
}

// Run mode/eventに応じて以下LED処理を行う
// Slope検出中：青色
// Slope制御中：赤色
// Rolling検出中：紫色
// Rolling制御中：黄色
static void led_process() {
    uint32_t left_c, right_c;
    uint8_t val_left, val_right;

    pixels.clear();
    switch (run_event) {
        // case RUN_EVENT_ROLLING_PRE:
            // left_c = COLOR_ROLLING_PRE;
            // right_c = COLOR_ROLLING_PRE;
            // break;
        // case RUN_EVENT_ROLLING:
            // left_c = COLOR_ROLLING_DET;
            // right_c = COLOR_ROLLING_DET;
            // break;
        case RUN_EVENT_SLOPING_PRE:
            left_c = COLOR_SLOPE_PRE;
            right_c = COLOR_SLOPE_PRE;
            break;
        case RUN_EVENT_SLOPING:
            left_c = COLOR_SLOPE_DET;
            right_c = COLOR_SLOPE_DET;
            break;
        case RUN_EVENT_ROLLING_PRE:
        case RUN_EVENT_ROLLING:
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
    digitalWrite(ESC_LEFT_PIN, HIGH);
    digitalWrite(ESC_RIGHT_PIN, HIGH);
    delay(100);
    digitalWrite(ESC_LEFT_PIN, LOW);
    digitalWrite(ESC_RIGHT_PIN, LOW);
    
    esc_left.attach(ESC_LEFT_PIN, MIN_SIGNAL, MAX_SIGNAL);//22ms中1500us
    esc_right.attach(ESC_RIGHT_PIN, MIN_SIGNAL, MAX_SIGNAL);
    delay(100); //ESC 位置同定待ち

    esc_left.writeMicroseconds(1200); //20ms中1200us
    esc_right.writeMicroseconds(1200);

    delay(100); //ESC 位置同定待ち

    // esc_left.writeMicroseconds(1200); //20ms中1200us
    // esc_right.writeMicroseconds(1200);
    // delay(2000);
    esc_left.writeMicroseconds(MIN_SIGNAL);
    esc_right.writeMicroseconds(MIN_SIGNAL);
    delay(4000); //ESC 位置同定待ち

    //mpu
    Wire.begin();
    mpu.initialize();
    mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_500); // R=400のコースを10m/sで通過することを想定したレンジ
    mpu.setDLPFMode(3); // 42Hz Delay:4.8ms
    mpu.setXAccelOffset(MPU6050_XA_OFFSET);
    mpu.setYAccelOffset(MPU6050_YA_OFFSET);
    mpu.setZAccelOffset(MPU6050_ZA_OFFSET);
    mpu.setXGyroOffset(MPU6050_XG_OFFSET);
    mpu.setYGyroOffset(MPU6050_YG_OFFSET);
    mpu.setZGyroOffset(MPU6050_ZG_OFFSET);

    //neopixel
    pixels.begin();

    //opening
    bool left = false;
    for ( int i = 0 ; i < 10; i++ ) {
        pixels.clear();
        if ( left ) {
            pixels.setPixelColor(LED_LEFT, pixels.ColorHSV(0, 255, 255));
            pixels.setPixelColor(LED_RIGHT, pixels.ColorHSV(0, 255, 0));
        } else {
            pixels.setPixelColor(LED_LEFT, pixels.ColorHSV(0, 255, 0));
            pixels.setPixelColor(LED_RIGHT, pixels.ColorHSV(0, 255, 255));
        }
        left = !left;
        pixels.show();
        delay(100);
    }
    pixels.clear();
    pixels.show();

    // 走行時間制限スタート
    tmr_run_limit = millis();
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
            } else if (run_event == RUN_EVENT_SLOPING) {
                run_mode = RUN_MODE_SLOPE;
                tmr_slope_slow_down = millis();
            }
            break;
        case RUN_MODE_ROLLOVER:
            run_rollover();
            if (run_event == RUN_EVENT_NONE) {
                run_mode = RUN_MODE_STABLE;
            }
            break;
        case RUN_MODE_SLOPE:
            run_slope();
            if (run_event == RUN_EVENT_NONE || tmr_slope_slow_down + SLOPE_SLOW_DOWN_TIME < millis()) {
                run_mode = RUN_MODE_STABLE;
            }
            break;
        default:
            run_mode = RUN_MODE_STABLE;
            break;
    }

    //neopixel
    led_process();

    // 走行時間制限
    if (millis() - tmr_run_limit > RUN_LIMIT_TIME) {
        uint16_t hue = 0;
        while(1) {
            motor_drive(0, 0);
            pixels.clear();
            pixels.setPixelColor(LED_LEFT, pixels.ColorHSV(hue, 255, 255));
            pixels.setPixelColor(LED_RIGHT, pixels.ColorHSV(hue, 255, 255));
            hue += 100;
            pixels.show();
            delay(5);
        }
    }
}
