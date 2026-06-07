/**
 * ============================================================
 * 智能船舶控制系统 - Arduino硬件接口固件
 * 兼容: Arduino Uno / Nano / Mega / Leonardo
 * 功能: 传感器采集、电机控制、伺服控制、串口通信
 * ============================================================
 */

#include <Servo.h>
#include <Wire.h>

// ==================== 引脚定义 ====================
// PWM电机驱动 (L298N / TB6612)
#define MOTOR1_PWM   5   // 左电机PWM
#define MOTOR1_IN1   6   // 左电机方向1
#define MOTOR1_IN2   7   // 左电机方向2
#define MOTOR2_PWM   9   // 右电机PWM
#define MOTOR2_IN1   10  // 右电机方向1
#define MOTOR2_IN2   11  // 右电机方向2

// 伺服舵机
#define SERVO_RUDDER 3   // 舵机(转向)
Servo rudderServo;

// 模拟传感器
#define TEMP_SENSOR   A0  // 温度传感器 (LM35 / DS18B20模拟)
#define BATTERY_PIN   A1  // 电池电压检测 (分压电路)
#define CURRENT_PIN   A2  // 电流检测 (ACS712)
#define DEPTH_PIN     A3  // 深度/压力传感器

// 数字传感器  
#define SPEED_SENSOR  2   // 转速/速度传感器 (霍尔/光电)
#define ULTRASONIC_TRIG 8 // 超声波测距 Trig
#define ULTRASONIC_ECHO 12 // 超声波测距 Echo

// LED指示灯
#define STATUS_LED    13  // 状态LED
#define ERROR_LED     A5  // 错误LED

// ==================== 串口通信协议 ====================
// 协议格式: [CMD][DATA...][\n]
// CMD定义:
//   'M' - 电机控制    M<L速度>,<R速度>\n  (-255~255)
//   'S' - 舵机控制    S<角度>\n           (0~180)
//   'R' - 请求传感器  R\n
//   'P' - 设置参数    P<ID>=<值>\n
//   'T' - 测试模式    T<0/1>\n
//   'H' - 心跳        H\n

#define CMD_MOTOR    'M'
#define CMD_SERVO    'S'
#define CMD_SENSOR   'R'
#define CMD_PARAM    'P'
#define CMD_TEST     'T'
#define CMD_HEARTBEAT 'H'

// ==================== 全局变量 ====================
unsigned long lastSensorRead = 0;
unsigned long lastHeartbeat = 0;
const unsigned long SENSOR_INTERVAL = 50;   // 传感器采集间隔 (ms)
const unsigned long HEARTBEAT_INTERVAL = 1000; // 心跳间隔 (ms)

bool testMode = false;
bool emergencyStop = false;

// 传感器数据
float temperature = 25.0;
float batteryVoltage = 12.0;
float currentDraw = 0.0;
float depth = 0.0;
float speed = 0.0;
float distance = 0.0;
int rudderAngle = 90;

// PID控制参数
float kp = 1.0, ki = 0.1, kd = 0.05;
int targetSpeed = 0;
int currentSpeed = 0;

// 速度测量
volatile unsigned int pulseCount = 0;
unsigned long lastPulseTime = 0;
float measuredSpeed = 0.0;

// ==================== 中断服务 ====================
void speedISR() {
    pulseCount++;
}

// ==================== 初始化 ====================
void setup() {
    Serial.begin(115200);  // 高速串口通信
    while (!Serial) delay(10);
    
    // 引脚初始化
    pinMode(MOTOR1_PWM, OUTPUT);
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_PWM, OUTPUT);
    pinMode(MOTOR2_IN1, OUTPUT);
    pinMode(MOTOR2_IN2, OUTPUT);
    
    pinMode(STATUS_LED, OUTPUT);
    pinMode(ERROR_LED, OUTPUT);
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    pinMode(SPEED_SENSOR, INPUT_PULLUP);
    
    // 舵机初始化
    rudderServo.attach(SERVO_RUDDER);
    rudderServo.write(90);
    
    // 速度传感器中断
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR), speedISR, RISING);
    
    // 初始状态
    stopAllMotors();
    digitalWrite(STATUS_LED, HIGH);
    digitalWrite(ERROR_LED, LOW);
    
    // 发送就绪信号
    Serial.println(F("ARD_READY"));
}

// ==================== 主循环 ====================
void loop() {
    unsigned long now = millis();
    
    // 处理串口命令
    handleSerialCommands();
    
    // 定期采集传感器
    if (now - lastSensorRead >= SENSOR_INTERVAL) {
        readAllSensors();
        lastSensorRead = now;
        
        // 自动发送传感器数据（测试模式）
        if (testMode) {
            sendSensorData();
        }
    }
    
    // 心跳信号
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeat = now;
    }
    
    // 紧急停止检查
    if (emergencyStop) {
        stopAllMotors();
        digitalWrite(ERROR_LED, HIGH);
    }
}

// ==================== 串口命令处理 ====================
void handleSerialCommands() {
    if (!Serial.available()) return;
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;
    
    char cmd = input.charAt(0);
    
    switch (cmd) {
        case CMD_MOTOR:
            handleMotorCommand(input);
            break;
        case CMD_SERVO:
            handleServoCommand(input);
            break;
        case CMD_SENSOR:
            sendSensorData();
            break;
        case CMD_PARAM:
            handleParamCommand(input);
            break;
        case CMD_TEST:
            handleTestCommand(input);
            break;
        case CMD_HEARTBEAT:
            Serial.println(F("ALIVE"));
            break;
        default:
            Serial.print(F("ERR:Unknown cmd:"));
            Serial.println(cmd);
    }
}

// ==================== 电机控制 ====================
void handleMotorCommand(String cmd) {
    if (emergencyStop) {
        Serial.println(F("ERR:ESTOP"));
        return;
    }
    
    // 格式: M<L>,<R>\n  如: M100,100\n 或 M-50,200\n
    int commaIdx = cmd.indexOf(',');
    if (commaIdx == -1) {
        Serial.println(F("ERR:Bad format, use M<L>,<R>"));
        return;
    }
    
    int leftSpeed = cmd.substring(1, commaIdx).toInt();
    int rightSpeed = cmd.substring(commaIdx + 1).toInt();
    
    // 限幅
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    setMotor(MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2, leftSpeed);
    setMotor(MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2, rightSpeed);
    
    Serial.print(F("OK:MOTOR L="));
    Serial.print(leftSpeed);
    Serial.print(F(" R="));
    Serial.println(rightSpeed);
}

void setMotor(int pwmPin, int in1Pin, int in2Pin, int speed) {
    if (speed > 0) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
        analogWrite(pwmPin, speed);
    } else if (speed < 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
        analogWrite(pwmPin, -speed);
    } else {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
        analogWrite(pwmPin, 0);
    }
}

void stopAllMotors() {
    setMotor(MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2, 0);
    setMotor(MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2, 0);
    rudderServo.write(90);
}

// ==================== 舵机控制 ====================
void handleServoCommand(String cmd) {
    // 格式: S<角度>\n  如: S90\n
    int angle = cmd.substring(1).toInt();
    angle = constrain(angle, 0, 180);
    rudderServo.write(angle);
    rudderAngle = angle;
    
    Serial.print(F("OK:SERVO "));
    Serial.println(angle);
}

// ==================== 传感器读取 ====================
void readAllSensors() {
    // 温度传感器 (LM35: 10mV/°C, ADC: 5V/1024)
    int tempRaw = analogRead(TEMP_SENSOR);
    temperature = (tempRaw * 5.0 / 1024.0) * 100.0;
    
    // 电池电压 (假设分压比 1:3, 实际电压 = 读数 * 3)
    int battRaw = analogRead(BATTERY_PIN);
    batteryVoltage = (battRaw * 5.0 / 1024.0) * 3.0;
    
    // 电流传感器 (ACS712 30A: 66mV/A, 零点 = 2.5V)
    int currentRaw = analogRead(CURRENT_PIN);
    float currentVoltage = currentRaw * 5.0 / 1024.0;
    currentDraw = abs(currentVoltage - 2.5) / 0.066;
    
    // 深度/压力传感器
    int depthRaw = analogRead(DEPTH_PIN);
    depth = depthRaw * 10.0 / 1024.0;  // 0-10m映射
    
    // 速度计算
    unsigned long now = millis();
    if (now - lastPulseTime >= 500) {
        measuredSpeed = pulseCount * 0.1;  // 假设每个脉冲=0.1m
        pulseCount = 0;
        lastPulseTime = now;
    }
    speed = measuredSpeed;
    
    // 超声波测距
    distance = readUltrasonic();
}

float readUltrasonic() {
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    
    long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // 30ms超时
    if (duration == 0) return -1; // 超时/无回波
    
    return duration * 0.034 / 2.0; // 距离(cm)
}

// ==================== 数据发送 ====================
void sendSensorData() {
    // JSON格式传感器数据包
    Serial.print(F("{\"t\":"));
    Serial.print(millis());
    Serial.print(F(",\"temp\":"));
    Serial.print(temperature, 1);
    Serial.print(F(",\"bat\":"));
    Serial.print(batteryVoltage, 1);
    Serial.print(F(",\"cur\":"));
    Serial.print(currentDraw, 2);
    Serial.print(F(",\"depth\":"));
    Serial.print(depth, 1);
    Serial.print(F(",\"speed\":"));
    Serial.print(speed, 1);
    Serial.print(F(",\"dist\":"));
    Serial.print(distance, 1);
    Serial.print(F(",\"rudder\":"));
    Serial.print(rudderAngle);
    Serial.print(F(",\"estop\":"));
    Serial.print(emergencyStop ? "true" : "false");
    Serial.println(F("}"));
}

void sendHeartbeat() {
    Serial.print(F("{\"hb\":"));
    Serial.print(millis());
    Serial.print(F(",\"temp\":"));
    Serial.print(temperature, 1);
    Serial.print(F(",\"bat\":"));
    Serial.print(batteryVoltage, 1);
    Serial.println(F("}"));
    
    // 心跳时闪烁LED
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
}

// ==================== 参数设置 ====================
void handleParamCommand(String cmd) {
    // 格式: P<ID>=<值>\n  如: Pkp=1.5\n  Pki=0.2\n
    int eqIdx = cmd.indexOf('=');
    if (eqIdx == -1) {
        Serial.println(F("ERR:Bad param format"));
        return;
    }
    
    String paramId = cmd.substring(1, eqIdx);
    float value = cmd.substring(eqIdx + 1).toFloat();
    
    if (paramId == "kp") kp = value;
    else if (paramId == "ki") ki = value;
    else if (paramId == "kd") kd = value;
    else {
        Serial.print(F("ERR:Unknown param:"));
        Serial.println(paramId);
        return;
    }
    
    Serial.print(F("OK:PARAM "));
    Serial.print(paramId);
    Serial.print(F("="));
    Serial.println(value);
}

// ==================== 测试模式 ====================
void handleTestCommand(String cmd) {
    // 格式: T<0/1>\n
    int mode = cmd.substring(1).toInt();
    testMode = (mode != 0);
    
    Serial.print(F("OK:TEST "));
    Serial.println(testMode ? F("ON") : F("OFF"));
    
    if (testMode) {
        digitalWrite(STATUS_LED, HIGH);
    } else {
        digitalWrite(STATUS_LED, HIGH);
    }
}

// ==================== 紧急停止 ====================
void triggerEmergencyStop() {
    emergencyStop = true;
    stopAllMotors();
    digitalWrite(ERROR_LED, HIGH);
    Serial.println(F("ESTOP:Emergency stop triggered!"));
}

void clearEmergencyStop() {
    emergencyStop = false;
    digitalWrite(ERROR_LED, LOW);
    Serial.println(F("ESTOP:Cleared"));
}