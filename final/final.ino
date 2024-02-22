#include <Servo.h>
#define sensorPin A0 //光敏传感器引脚连接模拟口0；检测是否加粮
#define servoPin 12 //舵机引脚连接数字口4
#define buzzerPin 10 //蜂鸣器引脚连接数字口8
int triggerPin = 3; //超声传感器 trigger 引脚为数字端口 4
int echoPin = 6; //超声传感器 echo 引脚为数字端口 7


//Ultration

//获取超声回波时间函数
long getUltrasonicReturnTime (int triggerPin , int echoPin){
//从 trigger 引脚发出 10us 宽度的脉冲，即设置 trigger 引脚为高电平并保持 10us 后再置为低电平
digitalWrite(triggerPin , HIGH);
delayMicroseconds(10);
digitalWrite(triggerPin , LOW);
pinMode(echoPin , INPUT); //设置 echo 引脚为输入模式

//获取从 echo 引脚返回回波脉冲的时间
return pulseIn (echoPin ,HIGH);
}


//wifi
#include <TuyaWifi.h>   //包含涂鸦IoT WiFi通信库头文件
#include <SoftwareSerial.h>  //包含软串口库头文件
#define KEY_PIN 2     //配网按键引脚连接至Arduino数字端口2
#define rLED_PIN 13   //配网指示LED引脚连接至Arduino数字端口13
SoftwareSerial WifiSerial= SoftwareSerial(7, 8);  //实例化软串口对象
TuyaWifi myDevice(&WifiSerial);      //以软串口实例化涂鸦WiFi对象

#define DPID_SWITCH 20  //出粮口开关 DP 20
#define DPID_MODE 21    //LED的工作模式 DP 21
#define light 101        //DHT11温度数值 DP 101 
//定义数据点二维数组
//dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
unsigned char dp_array[][2] = {

{DPID_SWITCH, DP_TYPE_BOOL},   //IoT LED开关状态 BOOL类型
{DPID_MODE, DP_TYPE_ENUM},     //IoT LED工作模式 ENUM类型
{light,DP_TYPE_VALUE}         //光敏值  VALUE类型

}; 
boolean led_state;  //IoT LED开关状态变量
int lightv;

//PID--涂鸦IoT产品ID，应根据具体的产品ID修改！！！
unsigned char pid[] = {"gjwegmaznn5mvmzq"};    //6iu3f2xvqsekp8qn   
unsigned char mcu_ver[] = {"1.0.0"};           //mcu固件版本

unsigned long blinkMillis = 0;    //配网指示LED闪烁定时变量
//unsigned long dhtMillis = 0;      //温湿度采集上传定时变量

//led
int ledPin=9;

// 控制闪烁的时间间隔
const unsigned long interval = 500;  // 闪烁间隔为500毫秒

// 变量用于存储上一次闪烁的时间
unsigned long previousMillis = 0;

// 变量用于追踪 LED 当前状态
bool ledState = HIGH;


Servo myservo; //舵机对象实例化
int pos = 0;

#define PUSH_CODE "UHiY0IUETX2H5mv2" // 替换成实际的推送编号




// Defining frequency of each music note
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_G5 784
#define NOTE_A5 880
#define NOTE_B5 988

// Music notes of the song, 0 is a rest/pulse
int notes[] = {
    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0,

    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0,

    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
    NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
    NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,

    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_D5, NOTE_E5, NOTE_A4, 0,
    NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
    NOTE_C5, NOTE_A4, NOTE_B4, 0,

    NOTE_A4, NOTE_A4
   };

// Durations (in ms) of each music note of the song
// Quarter Note is 250 ms when songSpeed = 1.0
int durations[] = {
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125,

    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125,

    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 125, 250, 125,

    125, 125, 250, 125, 125,
    250, 125, 250, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 375,

    250, 125,
    //Rpeat of First Part
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125,

    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125,

    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 125, 250, 125,

    125, 125, 250, 125, 125,
    250, 125, 250, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 375,
    //End of Repeat

    250, 125, 375, 250, 125, 375,
    125, 125, 125, 125, 125, 125, 125, 125, 375,
    250, 125, 375, 250, 125, 375,
    125, 125, 125, 125, 125, 500,

    250, 125, 375, 250, 125, 375,
    125, 125, 125, 125, 125, 125, 125, 125, 375,
    250, 125, 375, 250, 125, 375,
    125, 125, 125, 125, 125, 500};




void setup() {
Serial.begin(9600);
pinMode(sensorPin, INPUT);
pinMode(buzzerPin, OUTPUT); // 蜂鸣器引脚初始化

pinMode(ledPin , OUTPUT); //设置 led 引脚为输出模式

myservo.attach(servoPin); //设定舵机连接的数字端口


//ultration
pinMode(triggerPin , OUTPUT); //设置 trigger 引脚为输出模式
digitalWrite(triggerPin , LOW); //置 trigger 引脚为低电平
pinMode(echoPin , INPUT); //设置 echo 引脚为输入模式


    //wifi配网
  WifiSerial.begin(9600);  //初始化软串口波特率为9600bps
  pinMode(rLED_PIN, OUTPUT);  //配网指示LED引脚为输出模式
  digitalWrite(rLED_PIN, LOW); //初始熄灭配网指示LED
  pinMode(KEY_PIN, INPUT);       //配网按键引脚为输入模式

  myDevice.init(pid, mcu_ver); //初始化涂鸦WiFi设备
  //传入所有数据点及其类型数组和DP编号
  myDevice.set_dp_cmd_total(dp_array, sizeof(dp_array)/sizeof(dp_array[0]));
  //注册DP下载处理回调函数
  myDevice.dp_process_func_register(dp_process);
  //注册所有DP更新处理回调函数
  myDevice.dp_update_all_func_register(dp_update_all);

  blinkMillis = millis();  


 digitalWrite(ledPin, ledState);



}



void loop() {
  
  


  myDevice.uart_service();  //刷新涂鸦WiFi设备串口接收服务，需在loop函数中反复调用
  //按键进入配网操作
  if (digitalRead(KEY_PIN) == LOW) {  //判别是否按下配网按键
    delay(80);  //延时（可根据需要调整此延时来决定进入配网状态的按键条件）
    if (digitalRead(KEY_PIN) == LOW) {
      myDevice.mcu_set_wifi_mode(SMART_CONFIG); //开始Wifi配网模式
    }
  }
  //配网模式时让配网指示LED闪烁
  if ((myDevice.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (myDevice.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (myDevice.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW)) {
    if (millis()- blinkMillis >= 500) {
      blinkMillis = millis();
      digitalWrite(rLED_PIN, !digitalRead(rLED_PIN));  //闪烁配网指示LED
    }
  }


//如果碰到超声波触发出粮事件

int distance = 0.01723 * getUltrasonicReturnTime(triggerPin , echoPin );

Serial.print("Distance="); //串口输出距离测量结果
Serial.print(distance);
Serial.println("cm");
if (distance < 50) { //根据预设距离阈值点亮 熄灭红色 LED/ 绿色 LED

for(int i=0;i<30;i++){
          pos = 180;  // 设置目标角度为0度
        Serial.println("Turning servo to 0 degrees...");
        myservo.write(pos);  // 写入舵机的角度
        delay(1000);  // 等待舵机运动到位
        
        pos = 0;  // 设置目标角度为900度（5圈）
        Serial.println("Turning servo to 900 degrees...");
        myservo.write(pos);  // 写入舵机的角度
        delay(1000);  // 等待舵机运动到位
          
        }

} 



//如果光敏传感器的值改变（缺粮），手机端提示
int sensorState=analogRead(sensorPin);//读取震动传感器的检测值
Serial.println(sensorState);

if (sensorState > 200) {
    unsigned long currentMillis = millis();
  // 检查是否到达闪烁的时间间隔
  if (currentMillis - previousMillis >= interval) {
    // 更新上一次闪烁的时间
    previousMillis = currentMillis;
    // 切换 LED 状态，实现闪烁
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
  }
    myDevice.mcu_dp_update(light,lightv,1); //DHT11湿度数值 DP 更新至涂鸦IoT
  } 

  delay(1000); // 延迟1秒，避免频繁读取传感器值





/* 数据点处理回调函数
 * 当 App 控制设备的时候，会从云端下发对应的 DP 命令到设备。设备对数据进行解析后，对下发的命令执行相应的动作。
 * @description: DP download callback function.
 * @param {unsigned char} dpid
 * @param {const unsigned char} value
 * @param {unsigned short} length
 * @return {unsigned char}
 */
}
unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length) {
  switch (dpid) {
    case DPID_SWITCH:  // LED开关 DP
      // 获取IoT平台下发的LED开关 DP命令
      led_state = myDevice.mcu_get_dp_download_data(dpid, value, length); 
      
      
      Serial.print("Received DPID_SWITCH command. State: ");
      Serial.println(led_state);

      if (led_state) {  // 控制舵机转，打开粮口
        Serial.print("Received DPID_SWITCH command. State: ");

        for(int i=0;i<10;i++){
          pos = 180;  // 设置目标角度为0度
        Serial.println("Turning servo to 0 degrees...");
        myservo.write(pos);  // 写入舵机的角度
        delay(1000);  // 等待舵机运动到位
        
        pos = 0;  // 设置目标角度为900度（5圈）
        Serial.println("Turning servo to 900 degrees...");
        myservo.write(pos);  // 写入舵机的角度
        delay(1000);  // 等待舵机运动到位
          
        }
        

        
        
        
        //播放音乐
        const int totalNotes = sizeof(notes) / sizeof(int);
  const float songSpeed = 1.0;
  for (int i = 0; i < totalNotes; i++)
  {
    const int currentNote = notes[i];
    float wait = durations[i] / songSpeed;
    // Play tone if currentNote is not 0 frequency, otherwise pause (noTone)
    if (currentNote != 0)
    {
      tone(buzzerPin, notes[i], wait); // tone(pin, frequency, duration)
    }
    else
    {
      noTone(buzzerPin);
    }
    // delay is used to wait for tone to finish playing before moving to next loop
    delay(wait);
  }
  

        } 
      
      myDevice.mcu_dp_update(dpid, value, length);  // 将IoT LED状态更新至涂鸦IoT平台
      break;

    default:
      break;
  }
  return SUCCESS;
}



/*所有DP更新处理回调函数
 * @description: Upload all DP status of the current device.
 * @param {*}
 * @return {*}
 */
 
void dp_update_all(void){
  myDevice.mcu_dp_update(DPID_SWITCH, led_state, 1);  //LED开关状态 DP更新
  myDevice.mcu_dp_update(light,lightv,1); //light数值 DP 更新至涂鸦IoT

}
