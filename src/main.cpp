#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_task_wdt.h>
#include <math.h>

// 新增模块头文�?
#include "i2c_mux.h"
#include "ads122c04.h"
#include "protocol_packets.h"
#include "control_watchdog.h"

ControlWatchdog g_controlWatchdog;
void stopAllOutputs();
void checkControlWatchdog();

// ============== PID 参数定义（可由上位机配置�?==============
float g_pidKp = 0.14f;            // 比例系数
float g_pidKi = 0.015f;           // 积分系数
float g_pidKd = 0.06f;            // 微分系数
float g_pidOutputMin = 1.0f;      // 最小输�?
float g_pidOutputMax = 6.0f;      // 最大输�?
#define PID_DEADBAND 0.1f
#define PID_INTEGRAL_LIMIT 25.0f  // 积分限幅

// ============== 平滑度优先参�?==============
#define PID_STARTUP_SPEED_RATIO 0.5f  // 启动速度 = 最大速度 * 0.5 (中位速度启动)
#define PID_SMOOTH_RAMP_TIME 200      // 平滑过渡时间 (ms)
#define PID_JERK_LIMIT 50.0f          // 加速度变化率限�?(RPM/s²)
#define PID_RAMP_RATE 5.0f            // RPM/周期
#define PID_INTEGRAL_ZONE 10.0f       // 积分区间
#define PID_DAMPING_ZONE 2.0f         // 阻尼区间

#define CAL_TIMEOUT 15000
#define CAL_STABLE_COUNT 15
#define CAL_INTERVAL 20

#define PID_MOVE_TIMEOUT 30000
#define PID_MOVE_STABLE_COUNT 10

#define PID_PACKET_INTERVAL 20  // 50Hz 数据包发送间�?

#define DET_FIRMWARE_ID "USV_DETECTOR"
#define DET_FIRMWARE_VERSION "2026.09.17-safety1"
#define COMMS_TASK_DELAY_MS 5
#define SENSOR_TASK_DELAY_ACTIVE_MS 5
#define SENSOR_TASK_DELAY_IDLE_MS 20
#define SENSOR_TASK_DELAY_SPECTRO_MS 15
#define I2C_SPECTRO_TAKE_TIMEOUT_MS 15
#define MAX_COMMAND_LENGTH 160
#define TASK_WDT_TIMEOUT_SEC 5
#define STRESS_DEFAULT_DURATION_S 300UL
#define STRESS_MAX_DURATION_S 1800UL
#define STRESS_BUSY_SLICE_MS 40UL
#define I2C_TRANSACTION_TIMEOUT_MS 15
#define I2C_RECOVERY_FAILURE_THRESHOLD 8
#define I2C_RECOVERY_MIN_INTERVAL_MS 5000UL

// 抑制 ADC 单点瞬态：大于阈值的跳变需要下一点在容差内重复确认。
#define SPECTRO_TRANSIENT_THRESHOLD_V 0.020f
#define SPECTRO_TRANSIENT_CONFIRM_TOLERANCE_V 0.010f

#define MAX_OPEN_LOOP_RPM 20.0f
#define MAX_COMMAND_DEGREES 3600.0f

// ============== 传感器缓存 ==============
volatile float g_cachedAngles[4] = {0, 0, 0, 0};
volatile bool  g_angleValid[4]   = {false, false, false, false};
volatile uint32_t g_angleTimestampMs[4] = {0, 0, 0, 0};
SemaphoreHandle_t i2cMutex;

bool g_angleStreamActive = false;
unsigned long g_lastAngleSendTime = 0;
const unsigned long ANGLE_SEND_INTERVAL = 20;  // 50Hz
TaskHandle_t g_loopTaskHandle = NULL;
TaskHandle_t g_commsTaskHandle = NULL;
TaskHandle_t g_sensorsTaskHandle = NULL;
TaskHandle_t g_stressCore0TaskHandle = NULL;
TaskHandle_t g_stressCore1TaskHandle = NULL;
unsigned long g_lastHealthSendTime = 0;
const unsigned long HEALTH_SEND_INTERVAL = 1000;  // 1Hz
volatile unsigned long g_maxActiveLoopGapUs = 0;
volatile uint32_t g_i2cReadFailCount = 0;
volatile uint32_t g_i2cRecoverCount = 0;
volatile uint32_t g_spectroSuccessCount = 0;
volatile uint32_t g_spectroMutexTimeoutCount = 0;
volatile uint32_t g_spectroI2CErrorCount = 0;
volatile uint32_t g_spectroCrcErrorCount = 0;
volatile uint32_t g_spectroDuplicateCount = 0;
volatile uint32_t g_spectroTransientDropCount = 0;
uint8_t g_i2cConsecutiveFailureCount = 0;
unsigned long g_lastI2CRecoverMs = 0;

enum StressMode {
    STRESS_MODE_CPU,
    STRESS_MODE_FULL
};

volatile bool g_stressActive = false;
volatile bool g_stressDonePending = false;
volatile StressMode g_stressMode = STRESS_MODE_CPU;
volatile unsigned long g_stressStartMs = 0;
volatile unsigned long g_stressDurationS = 0;
volatile uint32_t g_stressSinkU32 = 0;
volatile float g_stressSinkF = 0.0f;
unsigned long g_stressLastVirtualAnglePacketMs = 0;
unsigned long g_stressLastVirtualSpectroPacketMs = 0;
unsigned long g_stressLastVirtualPIDPacketMs = 0;
uint8_t g_stressVirtualPidMotor = 0;
uint32_t g_stressVirtualSampleCounter = 0;

// ============== PID 测试模式相关定义 ==============
#define PID_TEST_MAX_SAMPLES 200    // 最大采样点�?
#define PID_TEST_SAMPLE_INTERVAL 20 // 采样间隔 (ms)

// 测试数据采集结构
struct PIDTestData {
    bool active;                    // 测试是否进行�?
    uint8_t motorIndex;             // 测试电机
    float targetAngle;              // 转动角度增量（配置值）
    float actualTargetAngle;        // 实际目标角度（计算后�?
    bool direction;                 // 转动方向: true=正转(F), false=反转(B)
    uint8_t currentRun;             // 当前轮次
    uint8_t totalRuns;              // 总轮�?

    // 单轮数据采集
    unsigned long runStartTime;     // 本轮开始时�?
    float initialAngle;             // 初始角度
    float samples[PID_TEST_MAX_SAMPLES];       // 角度采样
    float outputSamples[PID_TEST_MAX_SAMPLES]; // 输出采样
    uint16_t sampleCount;           // 采样计数
    unsigned long lastSampleTime;   // 上次采样时间

    // 边缘计算中间变量
    float maxAngle;                 // 过程中最大角�?
    float minAngle;                 // 过程中最小角�?
    int8_t lastErrorSign;           // 上次误差符号
    uint8_t zeroCrossCount;         // 过零次数
    bool hasConverged;              // 是否已收�?
    unsigned long convergenceTime;  // 收敛时刻
    bool waitingForNextRun;         // Non-blocking settling interval between runs
    uint32_t waitStartedMs;
};

PIDTestData pidTest = {};

// 评分权重配置
struct ScoreWeights {
    uint8_t convergence;   // 收敛时间权重 (默认30)
    uint8_t overshoot;     // 过冲权重 (默认25)
    uint8_t steadyError;   // 稳态误差权�?(默认15)
    uint8_t smoothness;    // 平滑度权�?(默认20)
    uint8_t oscillation;   // 振荡权重 (默认10)
};

// 权重: 收敛35, 过冲20, 稳态误�?0, 平滑�?5, 振荡10
ScoreWeights scoreWeights = {35, 20, 20, 15, 10};

// 校准状态枚�?
enum CalibrationState {
    CAL_IDLE, CAL_RUNNING, CAL_SUCCESS, CAL_TIMEOUT_ERR, CAL_SENSOR_ERR
};

// PID 控制器结�?
struct PIDController {
    float Kp, Ki, Kd;
    float integral;
    float lastError;
    float outputMin, outputMax;
    float deadband;
    unsigned long lastTime;
    float lastAcceleration;  // 用于jerk限制

    void reset() {
        integral = 0;
        lastError = 0;
        lastTime = 0;
        lastAcceleration = 0;
    }
};

// 校准上下文结�?
struct CalibrationContext {
    CalibrationState state;
    uint8_t motorMask;
    float targetAngle[4];
    unsigned long startTime;
    uint8_t stableCount[4];
    uint8_t sensorErrorCount[4];
    bool motorDone[4];
    PIDController pid[4];
};

CalibrationContext calCtx;

// ============== 可配�?I2C 通道 (i2c_mux.h 中声明为 extern) ==============
uint8_t g_angleChannels[4] = {0, 3, 4, 7};
uint8_t g_spectroChannel = 2;

// ============== ADS122C04 全局状�?==============
ADSConfig g_adsConfig;
volatile bool g_spectroRunning = false;
uint32_t g_nextSpectroDueMs = 0;
int32_t g_lastSpectroRawCode = 0;
float g_lastSpectroVoltage = 0.0f;
uint8_t g_lastSpectroStatus = SPECTRO_STATUS_NOT_CONFIG;
bool g_spectroCounterValid = false;
uint8_t g_lastSpectroConversionCounter = 0;
bool g_spectroAcceptedSampleValid = false;
float g_spectroAcceptedVoltage = 0.0f;
bool g_spectroTransientPending = false;
float g_spectroPendingVoltage = 0.0f;

// --- 引脚定义 ---
#define X_STP 13
#define X_DIR 12
#define Y_STP 14
#define Y_DIR 27
#define Z_STP 26
#define Z_DIR 25
#define A_STP 33
#define A_DIR 32
#define BOOT 0
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_CLOCK_HZ 100000

// ============== 进样泵控制定�?==============
#define PUMP_PIN 2              // 进样泵控制引�?(GPIO2)
#define PUMP_PWM_CHANNEL 0      // LEDC PWM 通道
#define PUMP_PWM_FREQ 1000      // PWM 频率 (Hz)
#define PUMP_PWM_RESOLUTION 8   // PWM 分辨�?(8�? 0-255)

// 进样泵状�?
struct PumpState {
    bool enabled;               // 进样泵启停状�?
    uint8_t speedPercent;       // 转速百分比 (0-100)
    uint8_t pwmDuty;            // 实际PWM占空�?(0-255)
};

PumpState pumpState = {false, 0, 0};

// --- 电机参数 (TCA/MT6701 常量已移�?i2c_mux.h) ---
const float STEP_PER_DEGREE = 1.0 / (2.25/379.16);
const float STEPS_PER_REV = 360.0 * STEP_PER_DEGREE;
const float MIN_INTERVAL = 50;
const float GLOBAL_DUTY_CYCLE = 0.45;

SemaphoreHandle_t motorMutex;

struct MotorState {
    bool enabled;
    bool direction;
    bool isContinuous;
    float rpm;
    unsigned long stepInterval;
    unsigned long lastStepTime;
    bool stepState;
    long targetSteps;
    long executedSteps;
    long stepsRemaining;
    long signedSteps;
    unsigned long highTime;
    unsigned long lowTime;
    bool currentDirection;
    bool justFinished;
    unsigned long finishTime;
    bool waitingToSend;

    // PID 定位模式
    bool isPIDMode;
    float pidTargetAngle;       // 存储环形目标 (0~360)，用于显�?
    float pidPrecision;
    uint8_t pidStableCount;
    uint8_t pidSensorErrCount;
    unsigned long pidStartTime;
    PIDController pidCtrl;

    // 解环角度追踪（多圈PID�?
    float lastRawAngle;         // 上一次传感器读数 (0~360)
    float absAngle;             // 连续累积角度
    float absTargetAngle;       // 绝对目标角度
    bool absAngleValid;         // 解环是否已初始化

    // 二进制数据包相关
    float pidInitialAngle;
    long pidStartSteps;
    long pidStartSignedSteps;
    unsigned long lastPacketTime;

    // 速度斜坡控制
    float lastOutputRPM;

    // 重置解环角度（防溢出�?
    void resetAbsAngle(float currentRaw) {
        absAngle = 0.0f;
        lastRawAngle = currentRaw;
        absAngleValid = true;
    }
};

MotorState motors[4] = {
    {false, true, false, 0, 0, 0, false, 0, 0, 0, 0, 0, 0, true, false, 0, false, false, 0, 0.1f, 0, 0, 0, {}, 0, 0, 0, false, 0, 0, 0, 0, 0},
    {false, true, false, 0, 0, 0, false, 0, 0, 0, 0, 0, 0, true, false, 0, false, false, 0, 0.1f, 0, 0, 0, {}, 0, 0, 0, false, 0, 0, 0, 0, 0},
    {false, true, false, 0, 0, 0, false, 0, 0, 0, 0, 0, 0, true, false, 0, false, false, 0, 0.1f, 0, 0, 0, {}, 0, 0, 0, false, 0, 0, 0, 0, 0},
    {false, true, false, 0, 0, 0, false, 0, 0, 0, 0, 0, 0, true, false, 0, false, false, 0, 0.1f, 0, 0, 0, {}, 0, 0, 0, false, 0, 0, 0, 0, 0}
};

const byte motorPins[4][2] = {
    {X_STP, X_DIR}, {Y_STP, Y_DIR}, {Z_STP, Z_DIR}, {A_STP, A_DIR}
};

// --- 函数声明 ---
void TaskComms(void *pvParameters);
void TaskSensors(void *pvParameters);
void TaskStressCore0(void *pvParameters);
void TaskStressCore1(void *pvParameters);
float readAngleWithRetry(uint8_t channel);
void sendAngles();
float rpmToInterval(float rpm);
void updateDutyCycleTiming(MotorState &m);
void setMotorDirection(byte dirPin, bool direction);
void parseCommand(String cmd);
void stepMotor(MotorState &m, byte stepPin, byte dirPin, int idx);
void initCalibration(uint8_t motorMask);
void stopCalibration();
void runCalibrationPID();
float computePID(PIDController &pid, float currentAngle, float targetAngle);
float computePIDWithSmoothing(PIDController &pid, float currentAngle, float targetAngle, MotorState &motor);
float normalizeAngleError(float target, float current);
void sendCalibrationStatus();
void initPIDMove(int motorIndex, float targetAngle, float precision);
void stopPIDMove(int motorIndex);
void runMotorPID();
void stopAllPIDMoves();
void sendPIDDataPacket(int motorIndex, float currentAngle, float error, float pidOutput);

// 解环角度函数
float unwrapAngle(float newRaw, float lastRaw, float currentAbs);

// 直接误差PID计算（用于多圈控制）
float computePIDDirect(PIDController &pid, float error, MotorState &motor);

// PID测试模式函数声明
void parsePIDConfig(String cmd);
void parsePIDTest(String cmd);
void initPIDTest(uint8_t motorIndex, float targetAngle, bool direction, uint8_t runs);
void startNextTestRun();
void runPIDTestSampling();
void finishTestRun();
uint8_t calculateSmoothnessScore();
uint16_t calculateStartupJerk();
uint8_t calculateTotalScore(uint16_t convTime, int16_t overshoot, int16_t finalErr, uint8_t oscCount, uint8_t smoothness);
void stopPIDTest();

// 角度流函数声�?
void sendAnglePacket();
void sendHealthPacket();
void sendIdentity();
void initTaskWatchdog();
void registerCurrentTaskWatchdog();
void feedTaskWatchdog();
bool isDetectorMotionActive();
void initI2CBus();
void recoverI2CBus();
void recordI2CBusFault(bool fault);
void parseStressCommand(String cmd);
void startStressTest(unsigned long durationS, StressMode mode);
void stopStressTest(bool completed);
void updateStressTest();
void sendStressStatus();
void runStressWorkerSlice(uint8_t workerId);
const char* stressModeName();
bool isStressFullActive();
void updateVirtualStressAngles();
void runStressFullCommsLoad();
void sendVirtualStressAnglePacket();
void sendVirtualStressSpectroPacket();
void sendVirtualStressPIDPacket();

float getCachedAngle(int motorIndex);

// --- 进样泵控制函�?---
void setPumpSpeed(uint8_t speedPercent) {
    if (speedPercent > 100) speedPercent = 100;
    pumpState.speedPercent = speedPercent;
    pumpState.pwmDuty = (uint8_t)((float)speedPercent / 100.0f * 255.0f);
    if (pumpState.enabled) {
        ledcWrite(PUMP_PWM_CHANNEL, pumpState.pwmDuty);
    }
}

void setPumpEnabled(bool enabled) {
    pumpState.enabled = enabled;
    if (enabled) {
        ledcWrite(PUMP_PWM_CHANNEL, pumpState.pwmDuty);
    } else {
        ledcWrite(PUMP_PWM_CHANNEL, 0);
    }
}

void initTaskWatchdog() {
    esp_err_t result = esp_task_wdt_init(TASK_WDT_TIMEOUT_SEC, true);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        Serial.printf("WDT_ERR:INIT=%d\n", (int)result);
    }
}

void registerCurrentTaskWatchdog() {
    esp_err_t result = esp_task_wdt_add(NULL);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        Serial.printf("WDT_ERR:ADD=%d\n", (int)result);
    }
}

void feedTaskWatchdog() {
    esp_task_wdt_reset();
}

void initI2CBus() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS);
    invalidateTcaCache();
}

void recoverI2CBus() {
    g_i2cRecoverCount++;
    feedTaskWatchdog();

    Wire.end();
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(50);

    for (int i = 0; i < 9; i++) {
        digitalWrite(I2C_SCL_PIN, LOW);
        delayMicroseconds(5);
        digitalWrite(I2C_SCL_PIN, HIGH);
        delayMicroseconds(5);
    }

    pinMode(I2C_SDA_PIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(I2C_SDA_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(I2C_SDA_PIN, HIGH);
    delayMicroseconds(5);

    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    initI2CBus();
    Serial.printf("I2C_RECOVER:%lu\n", (unsigned long)g_i2cRecoverCount);
    feedTaskWatchdog();
}

void recordI2CBusFault(bool fault) {
    if (!fault) {
        g_i2cConsecutiveFailureCount = 0;
        return;
    }

    g_i2cReadFailCount++;
    if (g_i2cConsecutiveFailureCount < 255) {
        g_i2cConsecutiveFailureCount++;
    }
    if (g_i2cConsecutiveFailureCount >= I2C_RECOVERY_FAILURE_THRESHOLD) {
        unsigned long now = millis();
        if (g_lastI2CRecoverMs == 0 || now - g_lastI2CRecoverMs >= I2C_RECOVERY_MIN_INTERVAL_MS) {
            g_i2cConsecutiveFailureCount = 0;
            g_lastI2CRecoverMs = now;
            recoverI2CBus();
        }
    }
}

void resetSpectroSamplingIntegrity() {
    g_spectroCounterValid = false;
    g_lastSpectroConversionCounter = 0;
    g_spectroAcceptedSampleValid = false;
    g_spectroAcceptedVoltage = 0.0f;
    g_spectroTransientPending = false;
    g_spectroPendingVoltage = 0.0f;
}

bool acceptSpectroSample(float voltage, bool *spectroTransient) {
    *spectroTransient = false;
    if (!g_spectroAcceptedSampleValid) {
        g_spectroAcceptedSampleValid = true;
        g_spectroAcceptedVoltage = voltage;
        g_spectroTransientPending = false;
        return true;
    }

    if (fabsf(voltage - g_spectroAcceptedVoltage) <= SPECTRO_TRANSIENT_THRESHOLD_V) {
        g_spectroAcceptedVoltage = voltage;
        g_spectroTransientPending = false;
        return true;
    }

    if (g_spectroTransientPending
        && fabsf(voltage - g_spectroPendingVoltage) <= SPECTRO_TRANSIENT_CONFIRM_TOLERANCE_V) {
        g_spectroAcceptedVoltage = voltage;
        g_spectroTransientPending = false;
        return true;
    }

    g_spectroPendingVoltage = voltage;
    g_spectroTransientPending = true;
    *spectroTransient = true;
    return false;
}

void advanceSpectroDeadline(uint32_t now, uint32_t intervalMs) {
    if (intervalMs == 0) intervalMs = 1;
    do {
        g_nextSpectroDueMs += intervalMs;
    } while ((int32_t)(now - g_nextSpectroDueMs) >= 0);
}

bool isDetectorMotionActive() {
    if (calCtx.state == CAL_RUNNING || pidTest.active) {
        return true;
    }
    if (motorMutex == NULL) {
        return true;
    }
    if (xSemaphoreTake(motorMutex, 0) != pdTRUE) {
        return true;
    }

    bool active = false;
    for (int i = 0; i < 4; i++) {
        if ((motors[i].enabled && motors[i].stepInterval > 0) || motors[i].isPIDMode) {
            active = true;
            break;
        }
    }
    xSemaphoreGive(motorMutex);
    return active;
}


// --- Setup ---
void setup() {
    setCpuFrequencyMhz(160);

    pinMode(BOOT, OUTPUT);
    digitalWrite(BOOT, LOW);

    for(auto &p : motorPins) {
        pinMode(p[0], OUTPUT);
        pinMode(p[1], OUTPUT);
        digitalWrite(p[1], LOW);
    }

    // 初始化进样泵 PWM
    ledcSetup(PUMP_PWM_CHANNEL, PUMP_PWM_FREQ, PUMP_PWM_RESOLUTION);
    ledcAttachPin(PUMP_PIN, PUMP_PWM_CHANNEL);
    ledcWrite(PUMP_PWM_CHANNEL, 0);  // 初始停止

    initI2CBus();
    Serial.begin(115200);
    delay(100);

    // 初始化 ADS122C04 默认配置
    adsConfigDefault(g_adsConfig);

    motorMutex = xSemaphoreCreateMutex();
    i2cMutex   = xSemaphoreCreateMutex();

    Serial.printf("Global duty cycle set to: %.1f%%\n", GLOBAL_DUTY_CYCLE * 100);
    Serial.println("System Initialized. Core 1: Motors, Core 0: Comms+Sensors(I2C).");
    Serial.printf("PID Params: Kp=%.4f, Ki=%.5f, Kd=%.4f\n", g_pidKp, g_pidKi, g_pidKd);
    Serial.println("Injection pump initialized on GPIO2.");
    sendIdentity();
    initTaskWatchdog();
    g_loopTaskHandle = xTaskGetCurrentTaskHandle();
    registerCurrentTaskWatchdog();

    xTaskCreatePinnedToCore(TaskComms, "Comms", 8192, NULL, 2, &g_commsTaskHandle, 0);
    xTaskCreatePinnedToCore(TaskSensors, "Sensors", 4096, NULL, 1, &g_sensorsTaskHandle, 0);
    xTaskCreatePinnedToCore(TaskStressCore0, "Stress0", 2048, NULL, 0, &g_stressCore0TaskHandle, 0);
    xTaskCreatePinnedToCore(TaskStressCore1, "Stress1", 2048, NULL, 0, &g_stressCore1TaskHandle, 1);
}

// --- Loop (Core 1 - Motor Task) ---
void loop() {
    static unsigned long lastLoopUs = 0;
    static bool wasMotorRunning = false;
    bool motorRunning = true;

    if (xSemaphoreTake(motorMutex, 0) == pdTRUE) {
        motorRunning = false;
        for(int i=0; i<4; i++) {
            if (motors[i].enabled && motors[i].stepInterval > 0) {
                motorRunning = true;
            }
            stepMotor(motors[i], motorPins[i][0], motorPins[i][1], i);
        }
        xSemaphoreGive(motorMutex);
    }

    unsigned long nowUs = micros();
    if (lastLoopUs != 0 && motorRunning && wasMotorRunning) {
        unsigned long gap = nowUs - lastLoopUs;
        if (gap > g_maxActiveLoopGapUs) {
            g_maxActiveLoopGapUs = gap;
        }
    }
    lastLoopUs = nowUs;
    wasMotorRunning = motorRunning;

    feedTaskWatchdog();
    if (motorRunning) {
        delayMicroseconds(5);
    } else {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}


// --- Task Comms (Core 0) ---
void TaskComms(void *pvParameters) {
    registerCurrentTaskWatchdog();
    String inputBuffer = "";
    inputBuffer.reserve(128);
    bool commandOverflow = false;

    unsigned long lastCalTime = 0;

    calCtx.state = CAL_IDLE;
    calCtx.motorMask = 0;

    while(true) {
        checkControlWatchdog();
        while(Serial.available()) {
            char c = Serial.read();
            if(c == '\n') {
                inputBuffer.trim();
                if(commandOverflow) {
                    commandOverflow = false;
                } else if(inputBuffer.length() > 0) {
                    checkControlWatchdog();
                    if (inputBuffer == "DET?" || inputBuffer == "HELLO?") {
                        sendIdentity();
                    }
                    else if (inputBuffer == "WATCHDOG:ARM") {
                        stopAllOutputs();
                        g_controlWatchdog.arm(millis());
                        Serial.println("WATCHDOG_OK:ARM");
                    }
                    else if (inputBuffer == "WATCHDOG:KEEPALIVE") {
                        g_controlWatchdog.keepalive(millis());
                    }
                    else if (inputBuffer == "STOPALL") {
                        stopAllOutputs();
                        Serial.println("STOPALL_OK");
                    }
                    else if (!g_controlWatchdog.allows_control() &&
                             inputBuffer != "I2CMAP?" && inputBuffer != "ADSSTATUS?" &&
                             inputBuffer != "STRESS:STATUS?" && inputBuffer != "PIDQUERY" &&
                             inputBuffer != "PUMP:STATUS" && inputBuffer != "CALSTATUS" &&
                             inputBuffer != "PUMP:OFF" && inputBuffer != "PIDSTOP" &&
                             inputBuffer != "CALSTOP" && inputBuffer != "PIDTESTSTOP" &&
                             inputBuffer != "ADSSTOP" && inputBuffer != "ANGLESTREAM_STOP") {
                        Serial.println("WATCHDOG_ERR:REARM_REQUIRED");
                    }
                    // ===== 新增：PID参数配置指令 =====
                    else if (inputBuffer.startsWith("PIDCFG:")) {
                        parsePIDConfig(inputBuffer);
                    }
                    // ===== 新增：PID测试模式指令 =====
                    else if (inputBuffer.startsWith("PIDTEST:")) {
                        parsePIDTest(inputBuffer);
                    }
                    // ===== CPU 压力测试命令 =====
                    else if (inputBuffer.startsWith("STRESS:")) {
                        parseStressCommand(inputBuffer);
                    }
                    // ===== 新增：查询当前PID参数 =====
                    else if (inputBuffer == "PIDQUERY") {
                        Serial.printf("PIDPARAM:%.4f,%.5f,%.4f,%.1f,%.1f\n",
                            g_pidKp, g_pidKi, g_pidKd, g_pidOutputMin, g_pidOutputMax);
                    }
                    // ===== 新增：停止测�?=====
                    else if (inputBuffer == "PIDTESTSTOP") {
                        stopPIDTest();
                    }
                    // ===== 新增：配置评分权�?=====
                    else if (inputBuffer.startsWith("PIDWEIGHTS:")) {
                        // 格式: PIDWEIGHTS:conv,ovs,err,smooth,osc
                        int colonIdx = inputBuffer.indexOf(':');
                        String params = inputBuffer.substring(colonIdx + 1);

                        int c1 = params.indexOf(',');
                        int c2 = params.indexOf(',', c1+1);
                        int c3 = params.indexOf(',', c2+1);
                        int c4 = params.indexOf(',', c3+1);

                        if (c1 > 0 && c2 > c1 && c3 > c2 && c4 > c3) {
                            scoreWeights.convergence = params.substring(0, c1).toInt();
                            scoreWeights.overshoot = params.substring(c1+1, c2).toInt();
                            scoreWeights.steadyError = params.substring(c2+1, c3).toInt();
                            scoreWeights.smoothness = params.substring(c3+1, c4).toInt();
                            scoreWeights.oscillation = params.substring(c4+1).toInt();

                            Serial.printf("PIDWEIGHTS_OK:%d,%d,%d,%d,%d\n",
                                scoreWeights.convergence, scoreWeights.overshoot,
                                scoreWeights.steadyError, scoreWeights.smoothness,
                                scoreWeights.oscillation);
                        } else {
                            Serial.println("PIDWEIGHTS_ERR:PARSE");
                        }
                    }
                    // ===== 进样泵控制指�?=====
                    // PUMP:ON - 启动进样�?
                    // PUMP:OFF - 停止进样�?
                    // PUMP:SPD:xx - 设置转速百分比 (0-100)
                    // PUMP:SET:xx - 设置转速并启动 (0=停止)
                    else if (inputBuffer.startsWith("PUMP:")) {
                        String pumpCmd = inputBuffer.substring(5);
                        if (pumpCmd == "ON") {
                            setPumpEnabled(true);
                            Serial.printf("PUMP_OK:ON,SPD=%d\n", pumpState.speedPercent);
                        } else if (pumpCmd == "OFF") {
                            setPumpEnabled(false);
                            Serial.printf("PUMP_OK:OFF\n");
                        } else if (pumpCmd.startsWith("SPD:")) {
                            int speed = pumpCmd.substring(4).toInt();
                            if (speed >= 0 && speed <= 100) {
                                setPumpSpeed((uint8_t)speed);
                                Serial.printf("PUMP_OK:SPD=%d\n", pumpState.speedPercent);
                            } else {
                                Serial.println("PUMP_ERR:SPD_RANGE");
                            }
                        } else if (pumpCmd.startsWith("SET:")) {
                            int speed = pumpCmd.substring(4).toInt();
                            if (speed >= 0 && speed <= 100) {
                                setPumpSpeed((uint8_t)speed);
                                setPumpEnabled(speed > 0);
                                Serial.printf("PUMP_OK:SET=%d,%s\n", speed, speed > 0 ? "ON" : "OFF");
                            } else {
                                Serial.println("PUMP_ERR:SET_RANGE");
                            }
                        } else if (pumpCmd == "STATUS") {
                            Serial.printf("PUMP_STATUS:%s,SPD=%d\n",
                                pumpState.enabled ? "ON" : "OFF", pumpState.speedPercent);
                        } else {
                            Serial.println("PUMP_ERR:UNKNOWN_CMD");
                        }
                    }
                    // ===== I2C 通道映射指令 =====
                    else if (inputBuffer.startsWith("I2CMAP:")) {
                        parseI2CMapCommand(inputBuffer);
                    }
                    else if (inputBuffer == "I2CMAP?") {
                        Serial.printf("I2CMAP_OK:X=%d,Y=%d,Z=%d,A=%d,SPEC=%d\n",
                            g_angleChannels[0], g_angleChannels[1], g_angleChannels[2], g_angleChannels[3], g_spectroChannel);
                    }
                    // ===== ADS122C04 配置与控制指令 =====
                    else if (inputBuffer.startsWith("ADSCFG:")) {
                        parseADSConfigCommand(inputBuffer, g_adsConfig);
                    }
                    else if (inputBuffer == "ADSSTART") {
                        if (!g_adsConfig.enabled) {
                            Serial.println("ADS_ERR:CONFIG");
                        } else {
                            invalidateTcaCache();
                            if (selectTcaChannel(g_adsConfig.tcaChannel)) {
                                if (adsInitAndStart(g_adsConfig)) {
                                    g_spectroRunning = true;
                                    g_nextSpectroDueMs = millis() + (1000UL / g_adsConfig.publishRate);
                                    g_lastSpectroStatus = 0;
                                    resetSpectroSamplingIntegrity();
                                    Serial.println("ADS_OK:START");
                                } else {
                                    g_adsConfig.running = false;
                                    g_spectroRunning = false;
                                    Serial.println("ADS_ERR:I2C");
                                }
                            } else {
                                g_spectroRunning = false;
                                Serial.println("ADS_ERR:I2C");
                            }
                            invalidateTcaCache();
                        }
                    }
                    else if (inputBuffer == "ADSSTOP") {
                        adsStop(g_adsConfig);
                        g_spectroRunning = false;
                        Serial.println("ADS_OK:STOP");
                    }
                    else if (inputBuffer == "ADSSTATUS?") {
                        Serial.printf("ADS_STATUS:%s,CH=%d,ADDR=0x%02X,DR=%d,GAIN=%d,REF=%s\n",
                            g_spectroRunning ? "RUNNING" : "STOPPED",
                            g_adsConfig.tcaChannel, g_adsConfig.address, g_adsConfig.adcRate,
                            g_adsConfig.gain, g_adsConfig.vrefMode == ADS_VREF_AVDD ? "AVDD" : "INT");
                    }
                    // ===== 角度流控制指令 =====
                    else if (inputBuffer == "ANGLESTREAM_START") {
                        g_angleStreamActive = true;
                        g_lastAngleSendTime = millis();
                        Serial.println("ANGLESTREAM_OK");
                    }
                    else if (inputBuffer == "ANGLESTREAM_STOP") {
                        g_angleStreamActive = false;
                        Serial.println("ANGLESTREAM_STOPPED");
                    }
                    // 单次获取角度（兼容性保留，改为二进制包�?
                    else if(inputBuffer == "GETANGLE") {
                        sendAnglePacket();
                    }
                    // 旧版流控制指令（映射到新指令�?
                    else if (inputBuffer == "STREAM1") {
                        g_angleStreamActive = true;
                        g_lastAngleSendTime = millis();
                        Serial.println("ANGLESTREAM_OK");
                    } else if (inputBuffer == "STREAM0") {
                        g_angleStreamActive = false;
                        Serial.println("ANGLESTREAM_STOPPED");
                    } else if (inputBuffer == "PIDSTOP") {
                        if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
                            stopAllPIDMoves();
                            xSemaphoreGive(motorMutex);
                        }
                    } else if (inputBuffer.startsWith("CAL")) {
                        if (inputBuffer == "CALSTOP") {
                            stopCalibration();
                        } else if (inputBuffer == "CALSTATUS") {
                            sendCalibrationStatus();
                        } else {
                            uint8_t mask = 0;
                            for (int i = 3; i < inputBuffer.length(); i++) {
                                char m = inputBuffer[i];
                                if (m == 'X') mask |= 0x01;
                                else if (m == 'Y') mask |= 0x02;
                                else if (m == 'Z') mask |= 0x04;
                                else if (m == 'A') mask |= 0x08;
                            }
                            if (mask > 0) {
                                if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
                                    initCalibration(mask);
                                    xSemaphoreGive(motorMutex);
                                }
                            } else {
                                Serial.println("CAL_ERR:NO_MOTOR");
                            }
                        }
                    } else {
                        if (calCtx.state == CAL_RUNNING) {
                            Serial.println("BUSY:CALIBRATING");
                        } else if (pidTest.active) {
                            Serial.println("BUSY:TESTING");
                        } else {
                            if(xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
                                parseCommand(inputBuffer);
                                xSemaphoreGive(motorMutex);
                            }
                        }
                    }
                }
                inputBuffer = "";
            } else if(c != '\r') {
                if(inputBuffer.length() >= MAX_COMMAND_LENGTH) {
                    inputBuffer = "";
                    commandOverflow = true;
                    Serial.println("CMD_ERR:TOO_LONG");
                } else {
                    inputBuffer += c;
                }
            }
        }

        // 校准PID循环
        if (calCtx.state == CAL_RUNNING) {
            if (millis() - lastCalTime >= CAL_INTERVAL) {
                runCalibrationPID();
                lastCalTime = millis();
            }
        }

        if (millis() - g_lastHealthSendTime >= HEALTH_SEND_INTERVAL) {
            sendHealthPacket();
            updateStressTest();
            if (g_stressDonePending) {
                Serial.printf("STRESS_DONE:mode=%s,duration_s=%lu\n", stressModeName(), g_stressDurationS);
                g_stressDonePending = false;
            } else if (g_stressActive) {
                sendStressStatus();
            }
            g_lastHealthSendTime = millis();
        }

        // 普通PID定位循环（包括测试模式，因为测试模式也使用PID定位�?
        if (calCtx.state != CAL_RUNNING) {
            if (millis() - lastCalTime >= CAL_INTERVAL) {
                runMotorPID();
                lastCalTime = millis();
            }
        }

        // ===== PID测试模式采样（在PID控制之后执行�?=====
        if (pidTest.active) {
            runPIDTestSampling();
        }

        // 完成信号处理
        bool needSend = false;
        if (calCtx.state != CAL_RUNNING && !pidTest.active) {
            if(xSemaphoreTake(motorMutex, 10) == pdTRUE) {
                for(int i=0; i<4; i++) {
                    if(motors[i].justFinished) {
                        motors[i].justFinished = false;
                        motors[i].waitingToSend = true;
                        motors[i].finishTime = millis();
                    }
                    if(motors[i].waitingToSend && millis() - motors[i].finishTime >= 500) {
                        motors[i].waitingToSend = false;
                        needSend = true;
                    }
                }
                xSemaphoreGive(motorMutex);
            }
        }

        // 角度流发送（50Hz�?
        if (g_angleStreamActive) {
            unsigned long now = millis();
            if (now - g_lastAngleSendTime >= ANGLE_SEND_INTERVAL) {
                g_lastAngleSendTime = now;
                sendAnglePacket();
            }
        }

        // 完成信号发送角�?
        if(needSend) {
            sendAnglePacket();
        }

        runStressFullCommsLoad();

        // ===== ADS122C04 分光数据轮询与发送（需要 I2C 互斥）=====
        if (!isStressFullActive() && g_spectroRunning && g_adsConfig.publishRate > 0) {
            uint32_t now = millis();
            uint32_t specInterval = 1000UL / g_adsConfig.publishRate;
            if ((int32_t)(now - g_nextSpectroDueMs) >= 0) {
                if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_SPECTRO_TAKE_TIMEOUT_MS)) == pdTRUE) {
                    bool spectroMuxOk = false;
                    bool spectroPublishOk = false;
                    ADSReadStatus spectroReadStatus = ADS_READ_I2C_ERROR;
                    int32_t spectroRawCode = 0;
                    float spectroVoltage = 0.0f;
                    uint8_t spectroStatus = 0;
                    invalidateTcaCache();
                    if (selectTcaChannel(g_adsConfig.tcaChannel)) {
                        spectroMuxOk = true;
                        int32_t rawCode = 0;
                        uint8_t spectroConversionCounter = 0;
                        spectroReadStatus = adsReadData(
                            g_adsConfig.address, &rawCode, &spectroConversionCounter);
                        if (spectroReadStatus == ADS_READ_CRC_ERROR) {
                            g_spectroCrcErrorCount++;
                            spectroReadStatus = adsReadData(
                                g_adsConfig.address, &rawCode, &spectroConversionCounter);
                            if (spectroReadStatus == ADS_READ_CRC_ERROR) {
                                g_spectroCrcErrorCount++;
                            }
                        }

                        if (spectroReadStatus == ADS_READ_OK) {
                            bool duplicate = g_spectroCounterValid
                                && spectroConversionCounter == g_lastSpectroConversionCounter;
                            if (duplicate) {
                                g_spectroDuplicateCount++;
                            } else {
                                g_spectroCounterValid = true;
                                g_lastSpectroConversionCounter = spectroConversionCounter;

                                float voltage = codeToVoltage(
                                    rawCode, g_adsConfig.vrefValue, (float)g_adsConfig.gain);
                                bool spectroTransient = false;
                                if (acceptSpectroSample(voltage, &spectroTransient)) {
                                    uint8_t status = SPECTRO_STATUS_VALID;
                                    if (rawCode >= 8388607 || rawCode <= -8388608) {
                                        status |= SPECTRO_STATUS_SATURATED;
                                    }

                                    g_lastSpectroRawCode = rawCode;
                                    g_lastSpectroVoltage = voltage;
                                    g_lastSpectroStatus = status;
                                    spectroRawCode = rawCode;
                                    spectroVoltage = voltage;
                                    spectroStatus = status;
                                    spectroPublishOk = true;
                                } else if (spectroTransient) {
                                    g_spectroTransientDropCount++;
                                }
                            }
                        } else {
                            g_lastSpectroStatus = SPECTRO_STATUS_I2C_ERROR;
                            if (spectroReadStatus == ADS_READ_I2C_ERROR) {
                                g_spectroI2CErrorCount++;
                            }
                        }
                    } else {
                        g_lastSpectroStatus = SPECTRO_STATUS_I2C_ERROR;
                        g_spectroI2CErrorCount++;
                    }
                    invalidateTcaCache();
                    recordI2CBusFault(!spectroMuxOk || spectroReadStatus != ADS_READ_OK);
                    xSemaphoreGive(i2cMutex);
                    advanceSpectroDeadline(now, specInterval);
                    if (spectroPublishOk) {
                        SpectroDataPacket pkt;
                        pkt.head1 = PACKET_HEADER1;
                        pkt.head2 = HEADER2_SPECTRO;
                        pkt.timestamp_ms = now;
                        pkt.tca_channel = g_adsConfig.tcaChannel;
                        pkt.status = spectroStatus;
                        pkt.raw_code = spectroRawCode;
                        pkt.voltage = spectroVoltage;
                        uint8_t* pktData = (uint8_t*)&pkt.head2;
                        pkt.checksum = 0;
                        for (int ci = 0; ci < (int)sizeof(SpectroDataPacket) - 3; ci++) {
                            pkt.checksum ^= pktData[ci];
                        }
                        pkt.tail = PACKET_TAIL;
                        Serial.write((uint8_t*)&pkt, sizeof(pkt));
                        g_spectroSuccessCount++;
                    }
                } else {
                    g_spectroMutexTimeoutCount++;
                }
            }
        }

        feedTaskWatchdog();
        vTaskDelay(pdMS_TO_TICKS(COMMS_TASK_DELAY_MS));
    }
}

// ============== TaskSensors：独立 I2C 传感器读取任务 ==============
// 在 Core 0 以低优先级运行，持续刷新 g_cachedAngles[]。
// 与 TaskComms 通过 i2cMutex 共享 I2C 总线，互不阻塞串口命令。
void TaskSensors(void *pvParameters) {
    registerCurrentTaskWatchdog();
    uint8_t nextAngleIndex = 0;
    while (true) {
        if (isStressFullActive()) {
            updateVirtualStressAngles();
            feedTaskWatchdog();
            vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_DELAY_ACTIVE_MS));
            continue;
        }

        if (xSemaphoreTake(i2cMutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
            uint8_t anglesPerCycle = g_spectroRunning ? 1 : 4;
            uint8_t startIndex = g_spectroRunning ? nextAngleIndex : 0;
            for (uint8_t offset = 0; offset < anglesPerCycle; offset++) {
                uint8_t i = (startIndex + offset) % 4;
                I2CReadStatus angleStatus = I2C_READ_DEVICE_ERROR;
                float a = readMt6701AngleWithStatus(g_angleChannels[i], &angleStatus);
                bool angleOk = (a >= 0 && a <= 360);
                if (angleOk) {
                    g_cachedAngles[i] = a;
                    g_angleValid[i] = true;
                    g_angleTimestampMs[i] = millis();
                } else {
                    g_angleValid[i] = false;
                }
                recordI2CBusFault(angleStatus == I2C_READ_MUX_ERROR);
                feedTaskWatchdog();
            }
            if (g_spectroRunning) {
                nextAngleIndex = (nextAngleIndex + anglesPerCycle) % 4;
            } else {
                nextAngleIndex = 0;
            }
            xSemaphoreGive(i2cMutex);
        }
        feedTaskWatchdog();
        uint32_t delayMs = isDetectorMotionActive()
            ? SENSOR_TASK_DELAY_ACTIVE_MS
            : SENSOR_TASK_DELAY_IDLE_MS;
        if (g_spectroRunning && delayMs < SENSOR_TASK_DELAY_SPECTRO_MS) {
            delayMs = SENSOR_TASK_DELAY_SPECTRO_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

void TaskStressCore0(void *pvParameters) {
    while (true) {
        if (g_stressActive) {
            runStressWorkerSlice(0);
            vTaskDelay(1);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void TaskStressCore1(void *pvParameters) {
    while (true) {
        if (g_stressActive) {
            runStressWorkerSlice(1);
            vTaskDelay(1);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void runStressWorkerSlice(uint8_t workerId) {
    uint32_t acc = 0xA5A50000UL ^ workerId ^ millis();
    float mix = 1.0f + (float)workerId;
    unsigned long startMs = millis();

    while (g_stressActive && millis() - startMs < STRESS_BUSY_SLICE_MS) {
        for (int i = 0; i < 128; i++) {
            acc = acc * 1664525UL + 1013904223UL + workerId;
            mix += (float)(acc & 0xFF) * 0.00013f;
            mix *= 1.00001f;
            if (mix > 1000.0f) {
                mix *= 0.25f;
            }
        }
    }

    g_stressSinkU32 ^= acc;
    g_stressSinkF += mix * 0.000001f;
    if (g_stressSinkF > 1000.0f) {
        g_stressSinkF = 0.0f;
    }
}

const char* stressModeName() {
    return g_stressMode == STRESS_MODE_FULL ? "FULL" : "CPU";
}

bool isStressFullActive() {
    return g_stressActive && g_stressMode == STRESS_MODE_FULL;
}

void updateVirtualStressAngles() {
    unsigned long now = millis();
    float base = (float)(now % 360000UL) * 0.18f;
    while (base >= 360.0f) {
        base -= 360.0f;
    }

    for (int i = 0; i < 4; i++) {
        float angle = base + (float)i * 73.0f + (float)((g_stressVirtualSampleCounter + i * 17UL) % 100UL) * 0.01f;
        while (angle >= 360.0f) {
            angle -= 360.0f;
        }
        g_cachedAngles[i] = angle;
        g_angleValid[i] = true;
        g_angleTimestampMs[i] = now;
    }
    g_stressVirtualSampleCounter++;
}

void sendVirtualStressAnglePacket() {
    static float lastValid[4] = {0};
    AngleDataPacket packet;

    packet.head1 = PACKET_HEADER1;
    packet.head2 = HEADER2_ANGLE;
    for (int i = 0; i < 4; i++) {
        if (g_angleValid[i]) {
            packet.angles[i] = g_cachedAngles[i];
            lastValid[i] = packet.angles[i];
        } else {
            packet.angles[i] = lastValid[i];
        }
    }

    uint8_t* data = (uint8_t*)&packet.head2;
    packet.checksum = 0;
    for (int i = 0; i < 17; i++) {
        packet.checksum ^= data[i];
    }
    packet.tail = PACKET_TAIL;

    Serial.write((uint8_t*)&packet, sizeof(packet));
}

void sendVirtualStressSpectroPacket() {
    unsigned long now = millis();
    int32_t rawCode = (int32_t)((g_stressVirtualSampleCounter * 7919UL + now) % 200000UL) - 100000;
    uint8_t status = SPECTRO_STATUS_TEST;

    g_lastSpectroRawCode = rawCode;
    g_lastSpectroVoltage = codeToVoltage(rawCode, 3.3f, 1.0f);
    g_lastSpectroStatus = status;

    SpectroDataPacket packet;
    packet.head1 = PACKET_HEADER1;
    packet.head2 = HEADER2_SPECTRO;
    packet.timestamp_ms = now;
    packet.tca_channel = g_spectroChannel;
    packet.status = status;
    packet.raw_code = rawCode;
    packet.voltage = g_lastSpectroVoltage;

    uint8_t* data = (uint8_t*)&packet.head2;
    packet.checksum = 0;
    int checksumLen = sizeof(SpectroDataPacket) - 3;
    for (int i = 0; i < checksumLen; i++) {
        packet.checksum ^= data[i];
    }
    packet.tail = PACKET_TAIL;

    Serial.write((uint8_t*)&packet, sizeof(packet));
}

void sendVirtualStressPIDPacket() {
    uint8_t motorIndex = g_stressVirtualPidMotor & 0x03;
    g_stressVirtualPidMotor = (g_stressVirtualPidMotor + 1) & 0x03;

    float actualAngle = g_angleValid[motorIndex] ? g_cachedAngles[motorIndex] : 0.0f;
    float targetAngle = actualAngle + 15.0f + (float)motorIndex * 5.0f;
    while (targetAngle >= 360.0f) {
        targetAngle -= 360.0f;
    }
    float error = normalizeAngleError(targetAngle, actualAngle);
    float pidOutput = fabs(error) * 0.2f + 1.0f;

    PIDDataPacket packet;
    packet.head1 = PACKET_HEADER1;
    packet.head2 = HEADER2_PID;
    packet.motor_id = motorIndex;
    packet.timestamp = micros();
    packet.target_angle = targetAngle;
    packet.actual_angle = actualAngle;
    packet.theo_angle = targetAngle;
    packet.pid_out = pidOutput;
    packet.error = error;

    uint8_t* data = (uint8_t*)&packet.motor_id;
    packet.checksum = 0;
    for (int i = 0; i < 25; i++) {
        packet.checksum ^= data[i];
    }
    packet.tail = PACKET_TAIL;

    Serial.write((uint8_t*)&packet, sizeof(packet));
}

void runStressFullCommsLoad() {
    if (!isStressFullActive()) {
        return;
    }

    unsigned long now = millis();
    if (now - g_stressLastVirtualAnglePacketMs >= ANGLE_SEND_INTERVAL) {
        g_stressLastVirtualAnglePacketMs = now;
        sendVirtualStressAnglePacket();
    }
    if (now - g_stressLastVirtualSpectroPacketMs >= 20) {
        g_stressLastVirtualSpectroPacketMs = now;
        sendVirtualStressSpectroPacket();
    }
    if (now - g_stressLastVirtualPIDPacketMs >= PID_PACKET_INTERVAL) {
        g_stressLastVirtualPIDPacketMs = now;
        sendVirtualStressPIDPacket();
    }
}

void parseStressCommand(String cmd) {
    if (cmd == "STRESS:STATUS?") {
        updateStressTest();
        if (g_stressDonePending) {
            Serial.printf("STRESS_DONE:mode=%s,duration_s=%lu\n", stressModeName(), g_stressDurationS);
            g_stressDonePending = false;
        } else {
            sendStressStatus();
        }
        return;
    }

    if (cmd == "STRESS:STOP") {
        stopStressTest(false);
        Serial.println("STRESS_OK:STOP");
        return;
    }

    // Format: STRESS:START, STRESS:START:<seconds>, STRESS:START:<seconds>,FULL
    if (cmd == "STRESS:START" || cmd.startsWith("STRESS:START:")) {
        unsigned long durationS = STRESS_DEFAULT_DURATION_S;
        StressMode mode = STRESS_MODE_CPU;

        if (cmd.startsWith("STRESS:START:")) {
            String params = cmd.substring(13);
            if (params.length() == 0) {
                Serial.println("STRESS_ERR:FORMAT");
                return;
            }

            if (params.endsWith(",FULL")) {
                mode = STRESS_MODE_FULL;
                params = params.substring(0, params.length() - 5);
            } else if (params == "FULL") {
                mode = STRESS_MODE_FULL;
                params = "";
            }

            if (params.length() > 0) {
                for (int i = 0; i < params.length(); i++) {
                    if (!isDigit(params[i])) {
                        Serial.println("STRESS_ERR:FORMAT");
                        return;
                    }
                }
                durationS = params.toInt();
            } else if (mode != STRESS_MODE_FULL) {
                Serial.println("STRESS_ERR:FORMAT");
                return;
            }
        }
        if (durationS < 1 || durationS > STRESS_MAX_DURATION_S) {
            Serial.println("STRESS_ERR:DURATION_RANGE");
            return;
        }
        if (isDetectorMotionActive()) {
            Serial.println("STRESS_ERR:BUSY_MOTION");
            return;
        }
        startStressTest(durationS, mode);
        Serial.printf("STRESS_OK:START,mode=%s,duration_s=%lu\n", stressModeName(), durationS);
        return;
    }

    Serial.println("STRESS_ERR:FORMAT");
}

void startStressTest(unsigned long durationS, StressMode mode) {
    g_stressMode = mode;
    g_stressDurationS = durationS;
    g_stressStartMs = millis();
    g_stressLastVirtualAnglePacketMs = 0;
    g_stressLastVirtualSpectroPacketMs = 0;
    g_stressLastVirtualPIDPacketMs = 0;
    g_stressVirtualPidMotor = 0;
    g_stressVirtualSampleCounter = 0;
    g_stressDonePending = false;
    g_stressActive = true;
}

void stopStressTest(bool completed) {
    if (g_stressActive) {
        g_stressActive = false;
        g_stressDonePending = completed;
    } else if (!completed) {
        g_stressDonePending = false;
    }
}

void updateStressTest() {
    if (!g_stressActive) {
        return;
    }
    unsigned long elapsedMs = millis() - g_stressStartMs;
    if (elapsedMs >= g_stressDurationS * 1000UL) {
        stopStressTest(true);
    }
}

void sendStressStatus() {
    updateStressTest();
    if (!g_stressActive) {
        Serial.println("STRESS_STATUS:IDLE");
        return;
    }

    unsigned long elapsedS = (millis() - g_stressStartMs) / 1000UL;
    unsigned long remainingS = elapsedS >= g_stressDurationS ? 0 : g_stressDurationS - elapsedS;
    Serial.printf("STRESS_STATUS:RUNNING,mode=%s,duration_s=%lu,elapsed_s=%lu,remaining_s=%lu\n",
        stressModeName(), g_stressDurationS, elapsedS, remainingS);
}



// ============== PID参数配置解析 ==============
void parsePIDConfig(String cmd) {
    // 格式: PIDCFG:Kp,Ki,Kd[,OutputMin,OutputMax]
    int colonIdx = cmd.indexOf(':');
    if (colonIdx == -1) {
        Serial.println("PIDCFG_ERR:FORMAT");
        return;
    }

    String params = cmd.substring(colonIdx + 1);
    float kp, ki, kd;
    float outMin = g_pidOutputMin;
    float outMax = g_pidOutputMax;

    int comma1 = params.indexOf(',');
    int comma2 = params.indexOf(',', comma1 + 1);
    int comma3 = params.indexOf(',', comma2 + 1);
    int comma4 = params.indexOf(',', comma3 + 1);

    if (comma1 == -1 || comma2 == -1) {
        Serial.println("PIDCFG_ERR:PARAMS");
        return;
    }

    kp = params.substring(0, comma1).toFloat();
    ki = params.substring(comma1 + 1, comma2).toFloat();

    if (comma3 != -1) {
        kd = params.substring(comma2 + 1, comma3).toFloat();
        if (comma4 != -1) {
            outMin = params.substring(comma3 + 1, comma4).toFloat();
            outMax = params.substring(comma4 + 1).toFloat();
        } else {
            outMin = params.substring(comma3 + 1).toFloat();
        }
    } else {
        kd = params.substring(comma2 + 1).toFloat();
    }

    // 参数范围检�?
    if (kp < 0.01f || kp > 2.0f) { Serial.println("PIDCFG_ERR:KP_RANGE"); return; }
    if (ki < 0.0f || ki > 1.0f) { Serial.println("PIDCFG_ERR:KI_RANGE"); return; }
    if (kd < 0.0f || kd > 1.0f) { Serial.println("PIDCFG_ERR:KD_RANGE"); return; }
    if (outMin < 0.1f || outMin > 5.0f) { Serial.println("PIDCFG_ERR:OUTMIN_RANGE"); return; }
    if (outMax < 1.0f || outMax > 20.0f) { Serial.println("PIDCFG_ERR:OUTMAX_RANGE"); return; }

    // 更新全局参数
    g_pidKp = kp;
    g_pidKi = ki;
    g_pidKd = kd;
    g_pidOutputMin = outMin;
    g_pidOutputMax = outMax;

    Serial.printf("PIDCFG_OK:%.4f,%.5f,%.4f,%.1f,%.1f\n", g_pidKp, g_pidKi, g_pidKd, g_pidOutputMin, g_pidOutputMax);
}

// ============== PID测试模式解析 ==============
void parsePIDTest(String cmd) {
    // 格式: PIDTEST:<Motor>,<Dir>,<Angle>,<Runs>
    // 示例: PIDTEST:X,F,60.0,5  X电机正转60°测试5�?
    int colonIdx = cmd.indexOf(':');
    if (colonIdx == -1) {
        Serial.println("PIDTEST_ERR:FORMAT");
        return;
    }

    String params = cmd.substring(colonIdx + 1);
    int comma1 = params.indexOf(',');
    int comma2 = params.indexOf(',', comma1 + 1);
    int comma3 = params.indexOf(',', comma2 + 1);

    if (comma1 == -1 || comma2 == -1 || comma3 == -1) {
        Serial.println("PIDTEST_ERR:PARAMS");
        return;
    }

    char motorChar = params.charAt(0);
    uint8_t motorIndex;
    if (motorChar == 'X') motorIndex = 0;
    else if (motorChar == 'Y') motorIndex = 1;
    else if (motorChar == 'Z') motorIndex = 2;
    else if (motorChar == 'A') motorIndex = 3;
    else {
        Serial.println("PIDTEST_ERR:MOTOR");
        return;
    }

    // 解析方向: F=正转, B=反转
    char dirChar = params.charAt(comma1 + 1);
    bool direction = (dirChar == 'F' || dirChar == 'f');

    float targetAngle = params.substring(comma2 + 1, comma3).toFloat();
    uint8_t runs = params.substring(comma3 + 1).toInt();

    if (runs < 1 || runs > 20) {
        Serial.println("PIDTEST_ERR:RUNS_RANGE");
        return;
    }

    // 角度必须为正（方向由 dir 决定�?
    if (targetAngle < 0) targetAngle = fabs(targetAngle);
    targetAngle = constrain(targetAngle, 0.0f, MAX_COMMAND_DEGREES);

    initPIDTest(motorIndex, targetAngle, direction, runs);
}

// ============== PID测试模式实现 ==============
void initPIDTest(uint8_t motorIndex, float targetAngle, bool direction, uint8_t runs) {
    pidTest.active = true;
    pidTest.waitingForNextRun = false;
    pidTest.motorIndex = motorIndex;
    pidTest.targetAngle = targetAngle;
    pidTest.direction = direction;
    pidTest.currentRun = 0;
    pidTest.totalRuns = runs;

    const char motorNames[] = {'X', 'Y', 'Z', 'A'};
    Serial.printf("PIDTEST_START:%c,%c,%d\n", motorNames[motorIndex], direction ? 'F' : 'B', runs);

    startNextTestRun();
}

void startNextTestRun() {
    if (pidTest.currentRun >= pidTest.totalRuns) {
        // 所有测试完�?
        pidTest.active = false;
        const char motorNames[] = {'X', 'Y', 'Z', 'A'};
        Serial.printf("PIDTEST_DONE:%c\n", motorNames[pidTest.motorIndex]);
        return;
    }

    Serial.printf("PIDTEST_RUN:%d\n", pidTest.currentRun);

    // 重置采集数据
    pidTest.sampleCount = 0;
    pidTest.runStartTime = millis();
    pidTest.lastSampleTime = 0;
    pidTest.maxAngle = -999;
    pidTest.minAngle = 999;
    pidTest.lastErrorSign = 0;
    pidTest.zeroCrossCount = 0;
    pidTest.hasConverged = false;
    pidTest.convergenceTime = 0;

    // 读取初始角度
    pidTest.initialAngle = getCachedAngle(pidTest.motorIndex);
    if (pidTest.initialAngle < 0) pidTest.initialAngle = 0;

    // 获取电机状态以初始�?更新解环角度
    MotorState* m = &motors[pidTest.motorIndex];
    if (!m->absAngleValid) {
        m->absAngle = 0.0f;
        m->lastRawAngle = pidTest.initialAngle;
        m->absAngleValid = true;
    } else {
        m->absAngle = unwrapAngle(pidTest.initialAngle, m->lastRawAngle, m->absAngle);
        m->lastRawAngle = pidTest.initialAngle;
    }

    // 根据方向计算绝对目标角度
    int sign = pidTest.direction ? +1 : -1;
    m->absTargetAngle = m->absAngle + sign * pidTest.targetAngle;

    // 计算环形目标（用于显示）
    float displayTarget = fmod(m->absTargetAngle, 360.0f);
    if (displayTarget < 0) displayTarget += 360.0f;
    pidTest.actualTargetAngle = displayTarget;

    // 启动 PID 定位
    if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
        m->isPIDMode = true;
        m->pidTargetAngle = displayTarget;
        m->pidPrecision = 0.1f;
        m->pidStableCount = 0;
        m->pidSensorErrCount = 0;
        m->pidStartTime = millis();
        m->pidInitialAngle = pidTest.initialAngle;
        m->pidStartSteps = m->executedSteps;
        m->pidStartSignedSteps = m->signedSteps;
        m->lastPacketTime = 0;
        m->lastOutputRPM = 0;

        m->pidCtrl.Kp = g_pidKp;
        m->pidCtrl.Ki = g_pidKi;
        m->pidCtrl.Kd = g_pidKd;
        m->pidCtrl.outputMin = g_pidOutputMin;
        m->pidCtrl.outputMax = g_pidOutputMax;
        m->pidCtrl.deadband = 0.1f;
        m->pidCtrl.reset();

        m->enabled = false;
        m->stepInterval = 0;
        m->isContinuous = false;

        xSemaphoreGive(motorMutex);
    }

    const char motorNames[] = {'X', 'Y', 'Z', 'A'};
    Serial.printf("PID_START:%c,delta=%.1f,dir=%c,prec=0.10,absTarget=%.1f\n",
        motorNames[pidTest.motorIndex], pidTest.targetAngle,
        pidTest.direction ? 'F' : 'B', m->absTargetAngle);
}

void stopPIDTest() {
    if (pidTest.active) {
        pidTest.active = false;
        pidTest.waitingForNextRun = false;
        if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
            stopPIDMove(pidTest.motorIndex);
            xSemaphoreGive(motorMutex);
        }
        Serial.println("PIDTEST_STOPPED");
    }
}


void runPIDTestSampling() {
    if (!pidTest.active) return;
    checkControlWatchdog();
    if (!g_controlWatchdog.allows_control() || !pidTest.active) return;

    MotorState* m = &motors[pidTest.motorIndex];
    unsigned long now = millis();

    if (pidTest.waitingForNextRun) {
        if (uint32_t(now - pidTest.waitStartedMs) >= 2000U) {
            pidTest.waitingForNextRun = false;
            pidTest.currentRun++;
            startNextTestRun();
        }
        return;
    }

    // 检查PID是否完成
    if (!m->isPIDMode) {
        // PID完成，结束本轮测�?
        finishTestRun();

        // 等待一段时间后开始下一轮（连续正转，不返回�?
        // Return to TaskComms so keepalives, STOPALL and watchdog expiry remain
        // serviceable throughout the settling interval.
        pidTest.waitingForNextRun = true;
        pidTest.waitStartedMs = uint32_t(now);
        return;
    }

    // 采样
    if (now - pidTest.lastSampleTime >= PID_TEST_SAMPLE_INTERVAL) {
        if (pidTest.sampleCount < PID_TEST_MAX_SAMPLES) {
            float angle = getCachedAngle(pidTest.motorIndex);
            if (angle >= 0 && angle <= 360) {
                pidTest.samples[pidTest.sampleCount] = angle;
                pidTest.outputSamples[pidTest.sampleCount] = m->lastOutputRPM;
                pidTest.sampleCount++;

                // 更新极�?
                if (angle > pidTest.maxAngle) pidTest.maxAngle = angle;
                if (angle < pidTest.minAngle) pidTest.minAngle = angle;

                // 振荡检测（误差过零�? 使用实际目标角度
                float error = normalizeAngleError(pidTest.actualTargetAngle, angle);
                int8_t errorSign = (error > 0.1f) ? 1 : ((error < -0.1f) ? -1 : 0);
                if (pidTest.lastErrorSign != 0 && errorSign != 0 &&
                    errorSign != pidTest.lastErrorSign) {
                    pidTest.zeroCrossCount++;
                }
                if (errorSign != 0) pidTest.lastErrorSign = errorSign;

                // 收敛检�?(死区0.1°)
                if (!pidTest.hasConverged && fabs(error) < 0.1f) {
                    pidTest.hasConverged = true;
                    pidTest.convergenceTime = now;
                }
            }
        }
        pidTest.lastSampleTime = now;
    }
}

// ============== 边缘评分计算函数 ==============
uint8_t calculateSmoothnessScore() {
    if (pidTest.sampleCount < 10) return 50;

    float totalJerk = 0;
    float maxJerk = 0;
    int validCount = 0;

    // 计算输出的二阶差分（近似加速度变化率）
    for (int i = 2; i < pidTest.sampleCount; i++) {
        float accel1 = pidTest.outputSamples[i-1] - pidTest.outputSamples[i-2];
        float accel2 = pidTest.outputSamples[i] - pidTest.outputSamples[i-1];
        float jerk = fabs(accel2 - accel1);

        totalJerk += jerk;
        if (jerk > maxJerk) maxJerk = jerk;
        validCount++;
    }

    if (validCount == 0) return 50;

    float avgJerk = totalJerk / validCount;

    // 评分：jerk越小越好
    float jerkScore = 100.0f * (1.0f - constrain(avgJerk / 3.0f, 0.0f, 1.0f));

    // 考虑最大jerk的惩�?
    float maxJerkPenalty = constrain(maxJerk / 8.0f, 0.0f, 0.3f);
    jerkScore *= (1.0f - maxJerkPenalty);

    return (uint8_t)constrain(jerkScore, 0, 100);
}

uint16_t calculateStartupJerk() {
    if (pidTest.sampleCount < 5) return 0;

    // 分析�?个采样点的加速度变化
    float maxStartupJerk = 0;
    for (int i = 2; i < min(5, (int)pidTest.sampleCount); i++) {
        float accel1 = pidTest.outputSamples[i-1] - pidTest.outputSamples[i-2];
        float accel2 = pidTest.outputSamples[i] - pidTest.outputSamples[i-1];
        float jerk = fabs(accel2 - accel1);
        if (jerk > maxStartupJerk) maxStartupJerk = jerk;
    }

    return (uint16_t)(maxStartupJerk * 100);
}

uint8_t calculateTotalScore(uint16_t convTime, int16_t overshoot,
                            int16_t finalErr, uint8_t oscCount,
                            uint8_t smoothness) {
    float score = 0;

    // 1. 收敛时间评分 (目标: <2000ms 满分, >5000ms 0�?
    float convScore;
    if (convTime < 2000) {
        convScore = 100.0f * (1.0f - (float)convTime / 2000.0f);
    } else if (convTime < 5000) {
        convScore = 50.0f * (1.0f - (float)(convTime - 2000) / 3000.0f);
    } else {
        convScore = 0;
    }
    score += convScore * scoreWeights.convergence / 100.0f;

    // 2. 过冲评分 (目标: 0过冲满分, >2�?�?
    float overshootDeg = fabs(overshoot) / 100.0f;
    float overshootScore;
    if (overshootDeg <= 0.05f) {
        overshootScore = 100;
    } else if (overshootDeg < 0.5f) {
        overshootScore = 100.0f - overshootDeg * 40.0f;
    } else if (overshootDeg < 2.0f) {
        overshootScore = 80.0f - (overshootDeg - 0.5f) * 53.3f;
    } else {
        overshootScore = 0;
    }
    score += overshootScore * scoreWeights.overshoot / 100.0f;

    // 3. 稳态误差评�?(目标: <0.1度满�? >1�?�?
    float errDeg = fabs(finalErr) / 100.0f;
    float errScore = 100.0f * (1.0f - constrain(errDeg / 1.0f, 0.0f, 1.0f));
    score += errScore * scoreWeights.steadyError / 100.0f;

    // 4. 平滑度评�?(直接使用)
    score += smoothness * scoreWeights.smoothness / 100.0f;

    // 5. 振荡评分 (目标: 0-1次满�? >5�?�?
    float oscScore;
    if (oscCount <= 1) {
        oscScore = 100;
    } else if (oscCount <= 3) {
        oscScore = 100.0f - (oscCount - 1) * 25.0f;
    } else {
        oscScore = max(0.0f, 50.0f - (oscCount - 3) * 25.0f);
    }
    score += oscScore * scoreWeights.oscillation / 100.0f;

    return (uint8_t)constrain(score, 0, 100);
}

void finishTestRun() {
    PIDTestResultPacket packet;

    packet.head1 = 0x55;
    packet.head2 = 0xBB;
    packet.motor_id = pidTest.motorIndex;
    packet.run_index = pidTest.currentRun;
    packet.total_runs = pidTest.totalRuns;

    // 收敛时间
    if (pidTest.hasConverged) {
        packet.convergence_time_ms = (uint16_t)(pidTest.convergenceTime - pidTest.runStartTime);
    } else {
        packet.convergence_time_ms = (uint16_t)(millis() - pidTest.runStartTime);
    }

    // 过冲计算 - 使用实际目标角度
    float expectedDirection = normalizeAngleError(pidTest.actualTargetAngle, pidTest.initialAngle);
    float overshoot = 0;

    if (expectedDirection > 0) {
        // 正向运动，检查是否超过目�?
        overshoot = normalizeAngleError(pidTest.maxAngle, pidTest.actualTargetAngle);
        if (overshoot < 0) overshoot = 0;
    } else {
        // 反向运动
        overshoot = normalizeAngleError(pidTest.actualTargetAngle, pidTest.minAngle);
        if (overshoot < 0) overshoot = 0;
    }
    packet.max_overshoot_x100 = (int16_t)(overshoot * 100);

    // 最终误�?- 使用实际目标角度
    float finalAngle = (pidTest.sampleCount > 0) ? pidTest.samples[pidTest.sampleCount - 1] : 0;
    float finalError = normalizeAngleError(pidTest.actualTargetAngle, finalAngle);
    packet.final_error_x100 = (int16_t)(finalError * 100);

    // 振荡次数
    packet.oscillation_count = pidTest.zeroCrossCount / 2;

    // 平滑度评�?
    packet.smoothness_score = calculateSmoothnessScore();

    // 启动冲击�?
    packet.startup_jerk_x100 = calculateStartupJerk();

    // 综合评分
    packet.total_score = calculateTotalScore(
        packet.convergence_time_ms,
        packet.max_overshoot_x100,
        packet.final_error_x100,
        packet.oscillation_count,
        packet.smoothness_score
    );

    // 校验�?
    uint8_t* data = (uint8_t*)&packet.motor_id;
    packet.checksum = 0;
    for (int i = 0; i < 14; i++) {
        packet.checksum ^= data[i];
    }
    packet.tail = 0x0A;

    // 发送二进制数据�?
    Serial.write((uint8_t*)&packet, sizeof(packet));

    // 同时发送文本格式便于调�?
    const char motorNames[] = {'X', 'Y', 'Z', 'A'};
    Serial.printf("PIDTEST_RESULT:%c,run=%d,conv=%d,ovs=%.2f,err=%.2f,osc=%d,smooth=%d,score=%d\n",
        motorNames[pidTest.motorIndex],
        pidTest.currentRun,
        packet.convergence_time_ms,
        packet.max_overshoot_x100 / 100.0f,
        packet.final_error_x100 / 100.0f,
        packet.oscillation_count,
        packet.smoothness_score,
        packet.total_score
    );
}


// --- 辅助函数 ---
// readAngleWithRetry 现在委托�?i2c_mux.h 中的 readMt6701Angle
float readAngleWithRetry(uint8_t channel) {
    return readMt6701Angle(channel);
}

// 从缓存读取角度（不走 I2C，供 TaskComms 使用）
float getCachedAngle(int motorIndex) {
    if (motorIndex < 0 || motorIndex > 3) return -1.0f;
    if (!g_angleValid[motorIndex]) return -1.0f;
    return g_cachedAngles[motorIndex];
}

// ============== 二进制角度数据包发�?==============
void sendAnglePacket() {
    static float last_valid[4] = {0};
    AngleDataPacket packet;

    packet.head1 = PACKET_HEADER1;
    packet.head2 = HEADER2_ANGLE;

    // 从缓存读取角度（不走 I2C）
    for (int i = 0; i < 4; i++) {
        float a = getCachedAngle(i);
        if (a < 0 || a > 360) {
            packet.angles[i] = last_valid[i];
        } else {
            packet.angles[i] = a;
            last_valid[i] = a;
        }
    }

    // 计算校验�?(从head2开始到angles结束)
    uint8_t* data = (uint8_t*)&packet.head2;
    packet.checksum = 0;
    for (int i = 0; i < 17; i++) {  // head2(1) + angles(16) = 17
        packet.checksum ^= data[i];
    }
    packet.tail = 0x0A;

    Serial.write((uint8_t*)&packet, sizeof(packet));
}

// 保留旧函数名兼容性，内部调用新函�?
void sendAngles() {
    sendAnglePacket();
}

static uint16_t clampStackHighWaterMark(TaskHandle_t handle) {
    if (handle == NULL) {
        return 0;
    }
    UBaseType_t value = uxTaskGetStackHighWaterMark(handle);
    return value > 65535 ? 65535 : (uint16_t)value;
}

void sendHealthPacket() {
    SystemHealthPacket packet;
    memset(&packet, 0, sizeof(packet));
    packet.head1 = PACKET_HEADER1;
    packet.head2 = HEADER2_HEALTH;
    packet.version = 1;
    packet.flags = 0x03;
    packet.timestamp_ms = millis();
    packet.uptime_s = packet.timestamp_ms / 1000UL;

    float tempC = temperatureRead();
    packet.temp_c_x10 = (int16_t)(tempC * 10.0f);
    packet.cpu_freq_mhz = (uint16_t)ESP.getCpuFreqMHz();
    packet.heap_free = ESP.getFreeHeap();
    packet.heap_min_free = ESP.getMinFreeHeap();
    packet.heap_total = ESP.getHeapSize();
    packet.task_count = uxTaskGetNumberOfTasks() > 255 ? 255 : (uint8_t)uxTaskGetNumberOfTasks();
    packet.loop_stack_hwm = clampStackHighWaterMark(g_loopTaskHandle);
    packet.comms_stack_hwm = clampStackHighWaterMark(g_commsTaskHandle);
    packet.sensors_stack_hwm = clampStackHighWaterMark(g_sensorsTaskHandle);

    uint8_t* data = (uint8_t*)&packet.head2;
    packet.checksum = 0;
    for (int i = 0; i < (int)sizeof(SystemHealthPacket) - 3; i++) {
        packet.checksum ^= data[i];
    }
    packet.tail = PACKET_TAIL;

    Serial.write((uint8_t*)&packet, sizeof(packet));
    unsigned long angleAgeMs[4] = {0, 0, 0, 0};
    unsigned long oldestAngleAgeMs = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (g_angleValid[i] && g_angleTimestampMs[i] != 0) {
            angleAgeMs[i] = packet.timestamp_ms - g_angleTimestampMs[i];
            if (angleAgeMs[i] > oldestAngleAgeMs) oldestAngleAgeMs = angleAgeMs[i];
        }
    }
    unsigned long activeLoopGapUs = g_maxActiveLoopGapUs;
    g_maxActiveLoopGapUs = 0;
    Serial.printf("LOOP_GAP_ACTIVE_MAX_US:%lu\n", activeLoopGapUs);
    Serial.printf("ANGLE_AGE_MS:%lu\n", oldestAngleAgeMs);
    Serial.printf("ANGLE_AGE_CH_MS:%lu,%lu,%lu,%lu\n",
        angleAgeMs[0], angleAgeMs[1], angleAgeMs[2], angleAgeMs[3]);
    Serial.printf(
        "ADS_HEALTH:SUCCESS=%lu,MUTEX_TIMEOUT=%lu,I2C_ERROR=%lu,"
        "CRC_ERROR=%lu,DUPLICATE=%lu,TRANSIENT_DROP=%lu\n",
        (unsigned long)g_spectroSuccessCount,
        (unsigned long)g_spectroMutexTimeoutCount,
        (unsigned long)g_spectroI2CErrorCount,
        (unsigned long)g_spectroCrcErrorCount,
        (unsigned long)g_spectroDuplicateCount,
        (unsigned long)g_spectroTransientDropCount);
}

void sendIdentity() {
    Serial.printf("DET_ID:%s,FW=%s,BAUD=%lu,CAP=WATCHDOG1\n", DET_FIRMWARE_ID, DET_FIRMWARE_VERSION, (unsigned long)115200);
}

void stopAllOutputs() {
    if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
        pidTest.active = false;
        pidTest.waitingForNextRun = false;
        stopCalibration();
        stopAllPIDMoves();
        for (int i = 0; i < 4; i++) {
            motors[i].waitingToSend = false;
            motors[i].justFinished = false;
        }
        setPumpEnabled(false);
        xSemaphoreGive(motorMutex);
    }
}

void checkControlWatchdog() {
    if (g_controlWatchdog.tick(millis())) {
        stopAllOutputs();
        Serial.println("WATCHDOG_TRIPPED");
    }
}

float rpmToInterval(float rpm) {
    return rpm > 0 ? 60000000.0 / (rpm * STEPS_PER_REV) : 0;
}

void updateDutyCycleTiming(MotorState &m) {
    if (m.stepInterval > 0) {
        m.highTime = m.stepInterval * GLOBAL_DUTY_CYCLE;
        m.lowTime = m.stepInterval - m.highTime;
        if (m.highTime < MIN_INTERVAL) m.highTime = MIN_INTERVAL;
        if (m.lowTime < MIN_INTERVAL) m.lowTime = MIN_INTERVAL;
    }
}

void setMotorDirection(byte dirPin, bool direction) {
    digitalWrite(dirPin, direction ? LOW : HIGH);
    delayMicroseconds(50);
}

void stepMotor(MotorState &m, byte stepPin, byte dirPin, int idx) {
    if(!m.enabled || m.stepInterval == 0) return;

    if(m.currentDirection != m.direction) {
        setMotorDirection(dirPin, m.direction);
        m.currentDirection = m.direction;
    }

    unsigned long now = micros();

    if(m.stepState) {
        if(now - m.lastStepTime >= m.highTime) {
            digitalWrite(stepPin, LOW);
            m.stepState = false;
            m.lastStepTime = now;
        }
    } else {
        if(now - m.lastStepTime >= m.lowTime) {
            digitalWrite(stepPin, HIGH);
            m.stepState = true;
            m.lastStepTime = now;

            m.executedSteps++;
            if (m.direction) {
                m.signedSteps++;
            } else {
                m.signedSteps--;
            }
            m.stepsRemaining--;

            if(!m.isContinuous && m.stepsRemaining <= 0) {
                m.enabled = false;
                m.stepInterval = 0;
                m.justFinished = true;
            }
        }
    }
}

// ============== 二进制数据包发�?==============
void sendPIDDataPacket(int motorIndex, float currentAngle, float error, float pidOutput) {
    MotorState* m = &motors[motorIndex];
    PIDDataPacket packet;

    packet.head1 = 0x55;
    packet.head2 = 0xAA;
    packet.motor_id = motorIndex;
    packet.timestamp = micros();
    packet.target_angle = m->pidTargetAngle;
    packet.actual_angle = currentAngle;

    long signedDelta = m->signedSteps - m->pidStartSignedSteps;
    float degreeDelta = (float)signedDelta / STEP_PER_DEGREE;
    float theoAngle = m->pidInitialAngle + degreeDelta;
    while (theoAngle < 0) theoAngle += 360.0f;
    while (theoAngle >= 360) theoAngle -= 360.0f;
    packet.theo_angle = theoAngle;

    packet.pid_out = fabs(pidOutput);
    packet.error = error;

    uint8_t* data = (uint8_t*)&packet.motor_id;
    packet.checksum = 0;
    for (int i = 0; i < 25; i++) {
        packet.checksum ^= data[i];
    }
    packet.tail = 0x0A;

    Serial.write((uint8_t*)&packet, sizeof(packet));
}

void parseCommand(String cmd) {
    int motorIndex = -1;
    bool handled = false;
    const char motorNames[] = {'X', 'Y', 'Z', 'A'};

    for(int i=0; i<cmd.length(); i++){
        char c = cmd[i];

        if(c >= 'X' && c <= 'Z') motorIndex = c - 'X';
        else if(c == 'A') motorIndex = 3;
        else continue;

        if(i+2 >= cmd.length()) break;

        MotorState* m = &motors[motorIndex];
        bool oldEnabled = m->enabled;
        bool oldDirection = m->direction;

        m->enabled = (cmd[++i] == 'E');
        m->direction = (cmd[++i] == 'F');

        if((oldEnabled != m->enabled) || (oldDirection != m->direction)) {
            if(m->enabled) {
                setMotorDirection(motorPins[motorIndex][1], m->direction);
                m->currentDirection = m->direction;
            }
        }

        int searchStart = i;
        int vIndex = cmd.indexOf('V', searchStart);
        int jIndex = cmd.indexOf('J', searchStart);
        int rIndex = cmd.indexOf('R', searchStart);  // R 指令替代�?T 指令

        float cmdRpm = 5.0f;
        if(vIndex != -1 && vIndex < cmd.length()) {
            int endIndex = cmd.length();
            if(jIndex > vIndex) endIndex = jIndex;
            if(rIndex > vIndex && rIndex < endIndex) endIndex = rIndex;
            cmdRpm = cmd.substring(vIndex+1, endIndex).toFloat();
            if (cmdRpm <= 0) cmdRpm = 5.0f;
            cmdRpm = constrain(cmdRpm, 0.0f, MAX_OPEN_LOOP_RPM);
        }

        // ============== R 指令：相对增量闭�?PID ==============
        // 格式: <Motor>E<Dir>R<Delta>P<Precision>
        // �? XEFR360P0.1  X电机正转360°，精�?.1°
        if(rIndex != -1 && rIndex >= searchStart){
            int pIndex = cmd.indexOf('P', rIndex);
            float delta;  // 相对增量�?=0�?
            float precision = 0.1f;  // 默认精度

            if(pIndex != -1 && pIndex > rIndex) {
                delta = cmd.substring(rIndex+1, pIndex).toFloat();
                int nextMotor = cmd.length();
                for(int k = pIndex+1; k < cmd.length(); k++) {
                    char nc = cmd[k];
                    if(nc == 'X' || nc == 'Y' || nc == 'Z' || nc == 'A') { nextMotor = k; break; }
                }
                precision = cmd.substring(pIndex+1, nextMotor).toFloat();
                if(precision <= 0 || precision > 10) precision = 0.1f;
                i = pIndex;
            } else {
                int nextMotor = cmd.length();
                for(int k = rIndex+1; k < cmd.length(); k++) {
                    char nc = cmd[k];
                    if(nc == 'X' || nc == 'Y' || nc == 'Z' || nc == 'A') { nextMotor = k; break; }
                }
                delta = cmd.substring(rIndex+1, nextMotor).toFloat();
                i = rIndex;
            }

            // delta 必须 >= 0，方向由 F/B 决定
            if (delta < 0) delta = fabs(delta);
            delta = constrain(delta, 0.0f, MAX_COMMAND_DEGREES);

            // 从缓存读取当前角度
            float rawAngle = getCachedAngle(motorIndex);
            if (rawAngle < 0) rawAngle = 0;

            // 初始化或更新解环角度
            if (!m->absAngleValid) {
                m->absAngle = 0.0f;
                m->lastRawAngle = rawAngle;
                m->absAngleValid = true;
            } else {
                m->absAngle = unwrapAngle(rawAngle, m->lastRawAngle, m->absAngle);
                m->lastRawAngle = rawAngle;
            }

            // 溢出保护：当累积角度过大时重�?
            if (fabs(m->absAngle) > 1e6f) {
                m->absAngle = 0.0f;
            }

            // 根据方向计算绝对目标角度
            // direction: true=正转(F), false=反转(B)
            int sign = m->direction ? +1 : -1;
            m->absTargetAngle = m->absAngle + sign * delta;

            // 存储环形目标用于显示
            float displayTarget = fmod(m->absTargetAngle, 360.0f);
            if (displayTarget < 0) displayTarget += 360.0f;
            m->pidTargetAngle = displayTarget;

            m->pidInitialAngle = rawAngle;
            m->pidStartSteps = m->executedSteps;
            m->pidStartSignedSteps = m->signedSteps;
            m->lastPacketTime = 0;
            m->lastOutputRPM = 0;

            m->isPIDMode = true;
            handled = true;
            m->pidPrecision = precision;
            m->pidStableCount = 0;
            m->pidSensorErrCount = 0;
            m->pidStartTime = millis();

            // 使用全局可配置的PID参数
            m->pidCtrl.Kp = g_pidKp;
            m->pidCtrl.Ki = g_pidKi;
            m->pidCtrl.Kd = g_pidKd;
            m->pidCtrl.outputMin = g_pidOutputMin;
            m->pidCtrl.outputMax = g_pidOutputMax;
            m->pidCtrl.deadband = precision;
            m->pidCtrl.reset();

            m->enabled = false;
            m->stepInterval = 0;
            m->isContinuous = false;

            Serial.printf("PID_START:%c,delta=%.1f,dir=%c,prec=%.2f,absTarget=%.1f\n",
                motorNames[motorIndex], delta, m->direction ? 'F' : 'B', precision, m->absTargetAngle);
            continue;
        }

        // 传统开环模�? J 指令
        if(jIndex != -1 && jIndex >= searchStart){
            m->isPIDMode = false;

            m->rpm = cmdRpm;
            m->stepInterval = rpmToInterval(m->rpm);
            updateDutyCycleTiming(*m);
            handled = true;

            String jVal = cmd.substring(jIndex+1);
            int nextMotor = jVal.length();
            for(int k = 0; k < jVal.length(); k++) {
                char nc = jVal[k];
                if(nc == 'X' || nc == 'Y' || nc == 'Z' || nc == 'A') { nextMotor = k; break; }
            }
            jVal = jVal.substring(0, nextMotor);

            if(jVal.startsWith("G")) {
                m->isContinuous = true;
                m->executedSteps = 0;
            } else {
                m->isContinuous = false;
                float degrees = jVal.toFloat();
                degrees = constrain(fabs(degrees), 0.0f, MAX_COMMAND_DEGREES);
                m->targetSteps = abs(degrees * STEP_PER_DEGREE);
                m->stepsRemaining = m->targetSteps;
                m->executedSteps = 0;
            }

            i = jIndex;
        }
    }
    if (handled) {
        Serial.println("CMD_OK");
    } else {
        Serial.println("CMD_ERR:UNKNOWN");
    }
}


// ============== PID 核心算法 ==============
float normalizeAngleError(float target, float current) {
    float error = target - current;
    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;
    return error;
}

// 解环算法：将传感�?0~360° 读数转换为连续累积角�?
float unwrapAngle(float newRaw, float lastRaw, float currentAbs) {
    float delta = newRaw - lastRaw;
    // 检�?0/360° 跨越
    if (delta > 180.0f) delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;
    return currentAbs + delta;
}

// 直接误差 PID 计算（用于多圈控制，不使�?normalizeAngleError�?
float computePIDDirect(PIDController &pid, float error, MotorState &motor) {
    if (fabs(error) < pid.deadband) {
        pid.integral = 0;
        motor.lastOutputRPM = 0;
        return 0;
    }

    unsigned long now = millis();
    float dt = (pid.lastTime > 0) ? (now - pid.lastTime) / 1000.0f : 0.02f;
    pid.lastTime = now;

    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.5f) dt = 0.5f;

    // 条件积分
    if (fabs(error) < PID_INTEGRAL_ZONE) {
        pid.integral += error * dt;
        if (pid.integral > PID_INTEGRAL_LIMIT) pid.integral = PID_INTEGRAL_LIMIT;
        if (pid.integral < -PID_INTEGRAL_LIMIT) pid.integral = -PID_INTEGRAL_LIMIT;
    } else {
        pid.integral *= 0.9f;
    }

    float derivative = (dt > 0) ? (error - pid.lastError) / dt : 0;
    pid.lastError = error;

    float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;

    // 限幅
    if (output > pid.outputMax) output = pid.outputMax;
    if (output < -pid.outputMax) output = -pid.outputMax;

    if (output > 0 && output < pid.outputMin) output = pid.outputMin;
    if (output < 0 && output > -pid.outputMin) output = -pid.outputMin;

    return output;
}

float computePID(PIDController &pid, float currentAngle, float targetAngle) {
    float error = normalizeAngleError(targetAngle, currentAngle);

    if (fabs(error) < pid.deadband) {
        pid.integral = 0;
        return 0;
    }

    unsigned long now = millis();
    float dt = (pid.lastTime > 0) ? (now - pid.lastTime) / 1000.0f : 0.02f;
    pid.lastTime = now;

    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.5f) dt = 0.5f;

    // 条件积分
    if (fabs(error) < PID_INTEGRAL_ZONE) {
        pid.integral += error * dt;
        if (pid.integral > PID_INTEGRAL_LIMIT) pid.integral = PID_INTEGRAL_LIMIT;
        if (pid.integral < -PID_INTEGRAL_LIMIT) pid.integral = -PID_INTEGRAL_LIMIT;
    } else {
        pid.integral *= 0.9f;
    }

    float derivative = (dt > 0) ? (error - pid.lastError) / dt : 0;
    pid.lastError = error;

    float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;

    // 阻尼区间
    if (fabs(error) < PID_DAMPING_ZONE) {
        float dampingFactor = 0.5f + 0.5f * (fabs(error) / PID_DAMPING_ZONE);
        output *= dampingFactor;
    }

    if (output > pid.outputMax) output = pid.outputMax;
    if (output < -pid.outputMax) output = -pid.outputMax;

    if (output > 0 && output < pid.outputMin) output = pid.outputMin;
    if (output < 0 && output > -pid.outputMin) output = -pid.outputMin;

    return output;
}

// ============== 平滑度优先的PID控制 ==============
// 基于距离的梯形速度曲线：加�?�?匀�?�?减�?
float computePIDWithSmoothing(PIDController &pid, float currentAngle, float targetAngle, MotorState &motor) {
    float error = normalizeAngleError(targetAngle, currentAngle);
    float absError = fabs(error);

    // 到达目标，停�?
    if (absError < pid.deadband) {
        pid.integral = 0;
        motor.lastOutputRPM = 0;
        pid.lastAcceleration = 0;
        return 0;
    }

    unsigned long now = millis();
    float dt = (pid.lastTime > 0) ? (now - pid.lastTime) / 1000.0f : 0.02f;
    pid.lastTime = now;

    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.5f) dt = 0.5f;

    // ===== 基于距离的速度规划 =====
    float currentRPM = fabs(motor.lastOutputRPM);
    float maxSpeed = pid.outputMax;
    float minSpeed = pid.outputMin;

    // 减速距离计算：假设减速率�?decelRate RPM/�?
    // 当前速度需要多少距离才能减到最小速度
    float decelRate = 1.5f;  // RPM/度，可调参数
    float brakeDistance = (currentRPM - minSpeed) / decelRate;
    if (brakeDistance < 0.5f) brakeDistance = 0.5f;

    // 目标速度计算
    float targetRPM;

    if (absError <= brakeDistance) {
        // 减速区：线性减速到最小速度
        // 速度 = minSpeed + (maxSpeed - minSpeed) * (error / brakeDistance)
        float speedRatio = absError / brakeDistance;
        targetRPM = minSpeed + (currentRPM - minSpeed) * speedRatio;

        // 确保平滑减速，不要突然降�?
        float maxDecel = decelRate * absError * dt * 50;  // 允许的最大减速量
        if (currentRPM - targetRPM > maxDecel) {
            targetRPM = currentRPM - maxDecel;
        }
    } else {
        // 加�?匀速区：使用增强PID计算目标速度

        // ===== 增益调度：根据误差大小动态调整增�?=====
        float effectiveKp = pid.Kp;
        float effectiveKd = pid.Kd;

        if (absError > 180.0f) {
            // 大误�? 激进模�?- 更快响应
            effectiveKp *= 1.2f;
            effectiveKd *= 0.8f;
        } else if (absError < 5.0f) {
            // 小误�? 保守模式 - 更稳�?
            effectiveKp *= 0.7f;
            effectiveKd *= 1.5f;
        }

        // 条件积分
        if (absError < PID_INTEGRAL_ZONE) {
            pid.integral += error * dt;
            pid.integral = constrain(pid.integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
        } else {
            pid.integral *= 0.95f;
        }

        float derivative = (dt > 0) ? (error - pid.lastError) / dt : 0;
        pid.lastError = error;

        // 计算原始PID输出
        float rawOutput = effectiveKp * error + pid.Ki * pid.integral + effectiveKd * derivative;

        // ===== 前馈控制：启动阶段基于目标距离预估速度 =====
        float feedforward = 0;
        unsigned long elapsed = now - motor.pidStartTime;
        if (elapsed < 500 && motor.pidStartTime > 0) {
            float angleToMove = fabs(normalizeAngleError(targetAngle, motor.pidInitialAngle));
            // 预估所需速度：角�?/ 期望完成时间(�? * 比例系数
            float expectedSpeed = angleToMove / 2.0f;  // 假设2秒内完成
            feedforward = constrain(expectedSpeed, 0, maxSpeed * 0.4f);
        }

        float totalOutput = rawOutput + feedforward;

        // ===== Back-calculation 积分抗饱�?=====
        float clampedOutput = constrain(totalOutput, -maxSpeed, maxSpeed);
        if (totalOutput != clampedOutput) {
            // 输出饱和时，回算并减少积分项
            float Kb = 0.5f;  // 回算增益
            pid.integral -= Kb * (totalOutput - clampedOutput) * dt;
        }

        targetRPM = fabs(clampedOutput);
        targetRPM = constrain(targetRPM, minSpeed, maxSpeed);
    }

    // ===== 启动阶段平滑 =====
    unsigned long runTime = now - motor.pidStartTime;
    if (runTime < PID_SMOOTH_RAMP_TIME) {
        float rampProgress = (float)runTime / PID_SMOOTH_RAMP_TIME;
        // S曲线
        float smoothFactor = rampProgress * rampProgress * (3.0f - 2.0f * rampProgress);
        float startSpeed = minSpeed + (maxSpeed - minSpeed) * 0.3f;  // 起始30%速度
        targetRPM = startSpeed + (targetRPM - startSpeed) * smoothFactor;
    }

    // ===== 速度变化率限�?=====
    float maxAccel = PID_RAMP_RATE * dt * 50;  // 最大加�?
    float maxDecel = PID_RAMP_RATE * dt * 80;  // 最大减速（允许更快减速）

    if (targetRPM > currentRPM + maxAccel) {
        targetRPM = currentRPM + maxAccel;
    } else if (targetRPM < currentRPM - maxDecel) {
        targetRPM = currentRPM - maxDecel;
    }

    // 确保在有效范围内
    if (targetRPM > 0 && targetRPM < minSpeed) {
        targetRPM = minSpeed;
    }
    targetRPM = constrain(targetRPM, 0, maxSpeed);

    // 记录加速度
    pid.lastAcceleration = (targetRPM - currentRPM) / dt;

    return (error > 0) ? targetRPM : -targetRPM;
}

// ============== 校准相关函数 ==============
void initCalibration(uint8_t motorMask) {
    calCtx.state = CAL_RUNNING;
    calCtx.motorMask = motorMask;
    calCtx.startTime = millis();

    for (int i = 0; i < 4; i++) {
        calCtx.targetAngle[i] = 0.0f;
        calCtx.stableCount[i] = 0;
        calCtx.sensorErrorCount[i] = 0;
        calCtx.motorDone[i] = !(motorMask & (1 << i));

        calCtx.pid[i].Kp = g_pidKp;
        calCtx.pid[i].Ki = g_pidKi;
        calCtx.pid[i].Kd = g_pidKd;
        calCtx.pid[i].outputMin = g_pidOutputMin;
        calCtx.pid[i].outputMax = g_pidOutputMax;
        calCtx.pid[i].deadband = PID_DEADBAND;
        calCtx.pid[i].reset();

        if (motorMask & (1 << i)) {
            motors[i].enabled = false;
            motors[i].stepInterval = 0;
        }
    }

    Serial.printf("CAL_START:mask=0x%X\n", motorMask);
}

void stopCalibration() {
    for (int i = 0; i < 4; i++) {
        if (calCtx.motorMask & (1 << i)) {
            motors[i].enabled = false;
            motors[i].stepInterval = 0;
            motors[i].isContinuous = false;
        }
    }
    calCtx.state = CAL_IDLE;
    calCtx.motorMask = 0;
    Serial.println("CAL_STOPPED");
}

void sendCalibrationStatus() {
    const char motorNames[] = {'X', 'Y', 'Z', 'A'};
    String status = "CAL_STATUS:";

    for (int i = 0; i < 4; i++) {
        if (calCtx.motorMask & (1 << i)) {
            float angle = getCachedAngle(i);
            float error = normalizeAngleError(calCtx.targetAngle[i], angle);
            status += String(motorNames[i]) + "=" + String(error, 2);
            status += calCtx.motorDone[i] ? "(OK)" : "";
            status += " ";
        }
    }
    Serial.println(status);
}

void runCalibrationPID() {
    if (calCtx.state != CAL_RUNNING) return;

    const char motorNames[] = {'X', 'Y', 'Z', 'A'};
    bool allDone = true;
    unsigned long now = millis();

    if (now - calCtx.startTime > CAL_TIMEOUT) {
        Serial.print("CAL_FAIL:TIMEOUT,done=");
        for (int i = 0; i < 4; i++) {
            if (calCtx.motorMask & (1 << i)) {
                Serial.print(motorNames[i]);
                Serial.print(calCtx.motorDone[i] ? "1" : "0");
            }
        }
        Serial.println();
        stopCalibration();
        calCtx.state = CAL_TIMEOUT_ERR;
        return;
    }

    for (int i = 0; i < 4; i++) {
        if (!(calCtx.motorMask & (1 << i)) || calCtx.motorDone[i]) continue;

        allDone = false;
        float currentAngle = getCachedAngle(i);

        if (currentAngle < 0 || currentAngle > 360) {
            calCtx.sensorErrorCount[i]++;
            if (calCtx.sensorErrorCount[i] > 10) {
                Serial.printf("CAL_FAIL:%c=SENSOR_ERR\n", motorNames[i]);
                calCtx.motorDone[i] = true;
                motors[i].enabled = false;
                motors[i].stepInterval = 0;
            }
            continue;
        }
        calCtx.sensorErrorCount[i] = 0;

        float pidOutput = computePID(calCtx.pid[i], currentAngle, calCtx.targetAngle[i]);
        float error = normalizeAngleError(calCtx.targetAngle[i], currentAngle);

        if (fabs(error) < PID_DEADBAND) {
            calCtx.stableCount[i]++;
            motors[i].enabled = false;
            motors[i].stepInterval = 0;

            if (calCtx.stableCount[i] >= CAL_STABLE_COUNT) {
                calCtx.motorDone[i] = true;
                Serial.printf("CAL_DONE:%c=%.2f\n", motorNames[i], currentAngle);
            }
        } else {
            calCtx.stableCount[i] = 0;

            if (xSemaphoreTake(motorMutex, 10) == pdTRUE) {
                motors[i].direction = (pidOutput > 0);
                float rpm = fabs(pidOutput);
                motors[i].rpm = rpm;
                motors[i].stepInterval = rpmToInterval(rpm);
                updateDutyCycleTiming(motors[i]);
                motors[i].isContinuous = true;
                motors[i].enabled = true;
                xSemaphoreGive(motorMutex);
            }
        }
    }

    if (allDone) {
        calCtx.state = CAL_SUCCESS;
        Serial.println("CAL_COMPLETE:ALL_DONE");
        stopCalibration();
    }
}


// ============== PID 定位模式实现 ==============
void initPIDMove(int motorIndex, float targetAngle, float precision) {
    if (motorIndex < 0 || motorIndex > 3) return;

    MotorState* m = &motors[motorIndex];
    const char motorNames[] = {'X', 'Y', 'Z', 'A'};

    m->enabled = false;
    m->stepInterval = 0;
    m->isContinuous = false;

    float initAngle = getCachedAngle(motorIndex);
    if (initAngle < 0) initAngle = 0;
    m->pidInitialAngle = initAngle;
    m->pidStartSteps = m->executedSteps;
    m->pidStartSignedSteps = m->signedSteps;
    m->lastPacketTime = 0;
    m->lastOutputRPM = 0;

    // 初始化解环角度（用于显示和内部跟踪）
    if (!m->absAngleValid) {
        m->absAngle = 0.0f;
        m->lastRawAngle = initAngle;
        m->absAngleValid = true;
    } else {
        m->absAngle = unwrapAngle(initAngle, m->lastRawAngle, m->absAngle);
        m->lastRawAngle = initAngle;
    }

    // 计算绝对目标：targetAngle 是环形目标，计算最短路径的绝对目标
    float shortestDelta = targetAngle - fmod(fabs(m->absAngle), 360.0f);
    if (m->absAngle < 0) {
        float wrappedCurrent = 360.0f - fmod(fabs(m->absAngle), 360.0f);
        if (wrappedCurrent >= 360.0f) wrappedCurrent = 0.0f;
        shortestDelta = targetAngle - wrappedCurrent;
    }
    if (shortestDelta > 180.0f) shortestDelta -= 360.0f;
    else if (shortestDelta < -180.0f) shortestDelta += 360.0f;
    m->absTargetAngle = m->absAngle + shortestDelta;

    m->isPIDMode = true;
    m->pidTargetAngle = targetAngle;  // 环形目标用于显示
    m->pidPrecision = precision;
    m->pidStableCount = 0;
    m->pidSensorErrCount = 0;
    m->pidStartTime = millis();

    // 使用全局可配置的PID参数
    m->pidCtrl.Kp = g_pidKp;
    m->pidCtrl.Ki = g_pidKi;
    m->pidCtrl.Kd = g_pidKd;
    m->pidCtrl.outputMin = g_pidOutputMin;
    m->pidCtrl.outputMax = g_pidOutputMax;
    m->pidCtrl.deadband = precision;
    m->pidCtrl.reset();

    Serial.printf("PID_START:%c,delta=%.1f,dir=F,prec=%.2f,absTarget=%.1f\n",
        motorNames[motorIndex], fabs(shortestDelta), precision, m->absTargetAngle);
}

void stopPIDMove(int motorIndex) {
    if (motorIndex < 0 || motorIndex > 3) return;
    MotorState* m = &motors[motorIndex];
    m->isPIDMode = false;
    m->enabled = false;
    m->stepInterval = 0;
    m->isContinuous = false;
}

void stopAllPIDMoves() {
    for (int i = 0; i < 4; i++) stopPIDMove(i);
    Serial.println("PID_STOP:ALL");
}

void runMotorPID() {
    const char motorNames[] = {'X', 'Y', 'Z', 'A'};
    unsigned long now = millis();

    bool anyPIDActive = false;
    for (int i = 0; i < 4; i++) {
        if (motors[i].isPIDMode) { anyPIDActive = true; break; }
    }
    if (!anyPIDActive) return;

    for (int i = 0; i < 4; i++) {
        MotorState* m = &motors[i];
        if (!m->isPIDMode) continue;

        // 从缓存读取角度（不阻塞 I2C）
        float rawAngle = getCachedAngle(i);

        // 传感器错误检�?
        if (rawAngle < 0 || rawAngle > 360) {
            m->pidSensorErrCount++;
            if (m->pidSensorErrCount > 10) {
                Serial.printf("PID_FAIL:%c=SENSOR_ERR\n", motorNames[i]);
                if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
                    m->isPIDMode = false;
                    m->enabled = false;
                    m->stepInterval = 0;
                    xSemaphoreGive(motorMutex);
                }
            }
            continue;
        }
        m->pidSensorErrCount = 0;

        // ===== 解环角度更新 =====
        m->absAngle = unwrapAngle(rawAngle, m->lastRawAngle, m->absAngle);
        m->lastRawAngle = rawAngle;

        // ===== 使用绝对角度计算误差（支持多圈）=====
        float error = m->absTargetAngle - m->absAngle;

        // 用于 UI 显示的环形角�?
        float displayAngle = fmod(m->absAngle, 360.0f);
        if (displayAngle < 0) displayAngle += 360.0f;

        // 超时检�?
        if (now - m->pidStartTime > PID_MOVE_TIMEOUT) {
            sendPIDDataPacket(i, displayAngle, error, 0);
            Serial.printf("PID_TIMEOUT:%c,abs=%.2f,err=%.2f\n", motorNames[i], m->absAngle, error);
            if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
                m->isPIDMode = false;
                m->enabled = false;
                m->stepInterval = 0;
                m->isContinuous = false;
                xSemaphoreGive(motorMutex);
            }
            continue;
        }

        // 过冲检�?
        static float lastError[4] = {0, 0, 0, 0};
        bool overshootDetected = false;
        if (lastError[i] != 0) {
            if ((lastError[i] > 0.5f && error < -0.1f) || (lastError[i] < -0.5f && error > 0.1f)) {
                overshootDetected = true;
                m->pidStableCount = 0;
            }
        }
        lastError[i] = error;

        // 完成判定：基于绝对误�?
        if (fabs(error) < m->pidPrecision && !overshootDetected) {
            m->pidStableCount++;

            if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
                m->enabled = false;
                m->stepInterval = 0;
                xSemaphoreGive(motorMutex);
            }

            if (now - m->lastPacketTime >= PID_PACKET_INTERVAL) {
                sendPIDDataPacket(i, displayAngle, error, 0);
                m->lastPacketTime = now;
            }

            if (m->pidStableCount >= PID_MOVE_STABLE_COUNT) {
                sendPIDDataPacket(i, displayAngle, error, 0);
                Serial.printf("PID_DONE:%c,abs=%.2f,err=%.2f\n", motorNames[i], m->absAngle, error);
                m->isPIDMode = false;
                lastError[i] = 0;
            }
        } else {
            m->pidStableCount = 0;

            // ===== 直接误差 PID 计算（不使用 normalize�?====
            float pidOutput = computePIDDirect(m->pidCtrl, error, *m);

            if (now - m->lastPacketTime >= PID_PACKET_INTERVAL) {
                sendPIDDataPacket(i, displayAngle, error, pidOutput);
                m->lastPacketTime = now;
            }

            if (xSemaphoreTake(motorMutex, portMAX_DELAY) == pdTRUE) {
                m->direction = (pidOutput > 0);

                float targetRPM = fabs(pidOutput);
                if (targetRPM < g_pidOutputMin) targetRPM = g_pidOutputMin;
                if (targetRPM > g_pidOutputMax) targetRPM = g_pidOutputMax;

                m->lastOutputRPM = targetRPM;
                m->rpm = targetRPM;
                m->stepInterval = rpmToInterval(targetRPM);
                updateDutyCycleTiming(*m);

                m->isContinuous = true;
                m->enabled = true;

                setMotorDirection(motorPins[i][1], m->direction);
                m->currentDirection = m->direction;

                xSemaphoreGive(motorMutex);
            }
        }
    }
}
