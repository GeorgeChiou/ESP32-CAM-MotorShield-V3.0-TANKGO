// 最後編輯 2021-12-25 by ShinWei Chiou
// 初版

// 最後編輯 2022-1-09 by ShinWei Chiou
// 修正 Analog Stick Motor value

// 最後編輯 2022-3-26 by ShinWei Chiou
// 增加 Damage_ValueZ >= 5;
// 被擊中到 5次 後暫停運作並且砲塔會左右搖擺，需按下 PS Button 解除再重新開始。

/*
  Board: ESP32 Wrover Module
  Upload Speed: 115200
  Flash Frequency: 80MHz
  Flash Mode: QIO
  Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)

  ESP32-CAM Motor Shield V3.0
  MotorA = 15;
  MotorA = 14;
  MotorB = 13;
  MotorB = 12;
  Servo = 2;
  IR LED = 4;
  IR Receiver LED = 0;

  PS3 Controller Bluetooth Connect
  Pairing the PS3 Controller : SixaxisPairToolSetup-0.3.1.exe
  <https://github.com/jvpernis/esp32-ps3>

  DFRobot DFPlayer Mini : GND VCC TX
  https://github.com/DFRobot/DFRobotDFPlayerMini
  <https://www.dfrobot.com/product-1121.html>
*/

#include <Ps3Controller.h>
String ESP32_MAC_Address;

#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
HardwareSerial mySerial(1);
DFRobotDFPlayerMini myDFPlayer;

int MP3_Play = 0;
int Volume_Value = 10;

// IR
#define IR_LED_Pin      4
#define IR_Receiver_Pin 0
int IR_Value = 0;
int Damage_Value = 0;

// Motor
#define MotorA14        14
#define MotorA15        15
#define MotorB12        12
#define MotorB13        13
#define MotorServo      2
#define MotorSChannel   1
int Turret_MaxMin_Lock = 0;


// Regarding the Datasheet of SG90 Servo, PWM Period is 20ms and Duty is 1->2ms
void servo_angle(int channel, int angle)
{
  int SERVO_RESOLUTION = 16;
  float range = (pow(2, SERVO_RESOLUTION) - 1) / 10;
  float minDuty = (pow(2, SERVO_RESOLUTION) - 1) / 40;

  uint32_t duty = (range) * (float)angle / 180.0 + minDuty;
  ledcWrite(channel, duty);
  delay(30);
}

// IR Send Code 38kHz
void IR_Send_Code()
{
  for (int i16t = 0; i16t < 16; i16t++)
  {
    for (int i37k = 0; i37k < 125; i37k++)
    {
      digitalWrite(IR_LED_Pin, HIGH);
      delayMicroseconds(12);
      digitalWrite(IR_LED_Pin, LOW);
      delayMicroseconds(12);
    }
    delay(2);
  }
}

void Move_Forward()
{
  digitalWrite(MotorA14, HIGH);
  digitalWrite(MotorA15, LOW);
  digitalWrite(MotorB12, HIGH);
  digitalWrite(MotorB13, LOW);
  delay(50);
}

void Move_Backward()
{
  digitalWrite(MotorA14, LOW);
  digitalWrite(MotorA15, HIGH);
  digitalWrite(MotorB12, LOW);
  digitalWrite(MotorB13, HIGH);
  delay(50);
}

void Move_TurnLeft()
{
  digitalWrite(MotorA14, LOW);
  digitalWrite(MotorA15, HIGH);
  digitalWrite(MotorB12, HIGH);
  digitalWrite(MotorB13, LOW);
  delay(50);
}

void Move_TurnRight()
{
  digitalWrite(MotorA14, HIGH);
  digitalWrite(MotorA15, LOW);
  digitalWrite(MotorB12, LOW);
  digitalWrite(MotorB13, HIGH);
  delay(50);
}

void Motor_Stop()
{
  digitalWrite(MotorA14, LOW);
  digitalWrite(MotorA15, LOW);
  digitalWrite(MotorB12, LOW);
  digitalWrite(MotorB13, LOW);
}

void Move_Fire()
{
  digitalWrite(MotorA14, LOW);
  digitalWrite(MotorA15, HIGH);
  digitalWrite(MotorB12, LOW);
  digitalWrite(MotorB13, HIGH);
  delay(100);
  digitalWrite(MotorA14, HIGH);
  digitalWrite(MotorA15, LOW);
  digitalWrite(MotorB12, HIGH);
  digitalWrite(MotorB13, LOW);
  delay(100);
  digitalWrite(MotorA14, HIGH);
  digitalWrite(MotorA15, HIGH);
  digitalWrite(MotorB12, HIGH);
  digitalWrite(MotorB13, HIGH);
  delay(100);
}

void Move_Damage()
{
  digitalWrite(MotorA14, LOW);
  digitalWrite(MotorA15, HIGH);
  digitalWrite(MotorB12, LOW);
  digitalWrite(MotorB13, HIGH);
  delay(200);
  digitalWrite(MotorA14, HIGH);
  digitalWrite(MotorA15, LOW);
  digitalWrite(MotorB12, HIGH);
  digitalWrite(MotorB13, LOW);
  delay(200);
  digitalWrite(MotorA14, HIGH);
  digitalWrite(MotorA15, HIGH);
  digitalWrite(MotorB12, HIGH);
  digitalWrite(MotorB13, HIGH);

  servo_angle(MotorSChannel, 120);
  delay(300);
  servo_angle(MotorSChannel, 60);
  delay(300);
  servo_angle(MotorSChannel, 120);
  delay(300);
  servo_angle(MotorSChannel, 90);
  delay(300);
  Turret_MaxMin_Lock = 0;
}


void setup()
{
  // 設定 UART 通訊
  Serial.begin(9600);

  // 取得 ESP32-CAM MAC Address
  Ps3.begin();
  ESP32_MAC_Address = Ps3.getAddress();

  // 設定搖桿 LED 顯示
  Ps3.setPlayer(1);

  // 設定 Motor GPIO
  pinMode(MotorA14, OUTPUT); //MotorA
  pinMode(MotorA15, OUTPUT); //MotorA
  pinMode(MotorB12, OUTPUT); //MotorB
  pinMode(MotorB13, OUTPUT); //MotorB

  // 設定 IR GPIO
  pinMode(IR_LED_Pin, OUTPUT);
  pinMode(IR_Receiver_Pin, INPUT);

  // 設定 Servo Channel
  ledcSetup(MotorSChannel, 50, 16);
  ledcAttachPin(MotorServo, MotorSChannel);
  servo_angle(MotorSChannel, 90);

  // 顯示 ESP32-CAM MAC Address 等待連線！
  while (!Ps3.isConnected())
  {
    Serial.print("The ESP32's Bluetooth MAC address is: ");
    Serial.println(ESP32_MAC_Address);
    delay(1000);
  }

  // 將 Serial Port 設定給 DFplayer 使用
  mySerial.begin(9600, SERIAL_8N1, 3, 1); // begin(9600, SERIAL_8N1, RXD, TXD)
  myDFPlayer.begin(mySerial);
  myDFPlayer.volume(Volume_Value);
  myDFPlayer.playMp3Folder(13);
  MP3_Play = 1;
}


void loop()
{
  //--------------- IR Battle System ---------------
  if (digitalRead(IR_Receiver_Pin) == LOW)
  {
    IR_Value = pulseIn(IR_Receiver_Pin, LOW);
    if ( IR_Value >= 2700 && IR_Value <= 3400 )
    {
      Damage_Value++;
      myDFPlayer.playMp3Folder(2);
      Move_Damage();
    }
  }

  //--------------- Reset Damage Value ---------------
  if ( Ps3.data.button.ps && Damage_Value >= 5 )
  {
    Damage_Value = 0;

    myDFPlayer.playMp3Folder(13);
    MP3_Play = 1;
    delay(300);
  }


  //--------------------- Damage ---------------------
  if ( Damage_Value >= 5 )
  {
    servo_angle(MotorSChannel, 110);
    delay(200);
    servo_angle(MotorSChannel, 70);
    delay(200);
    servo_angle(MotorSChannel, 110);
    delay(200);
    servo_angle(MotorSChannel, 90);
    delay(300);
  }
  else
  {


    //--------------- Digital D-pad button --------------
    if ( Ps3.data.button.up )
    {
      Move_Forward();
    }
    else if ( Ps3.data.button.down )
    {
      Move_Backward();
    }
    else if ( Ps3.data.button.left )
    {
      Move_TurnLeft();
    }
    else if ( Ps3.data.button.right )
    {
      Move_TurnRight();
    }

    //--------------- Digital Trigger button ------------
    if ( Ps3.data.button.l1 )
    {
      digitalWrite(MotorA14, HIGH);
      digitalWrite(MotorA15, LOW);
      delay(50);
    }
    else if ( Ps3.data.button.l2 )
    {
      digitalWrite(MotorA14, LOW);
      digitalWrite(MotorA15, HIGH);
      delay(50);
    }

    if ( Ps3.data.button.r1 )
    {
      digitalWrite(MotorB12, HIGH);
      digitalWrite(MotorB13, LOW);
      delay(50);
    }
    else if ( Ps3.data.button.r2 )
    {
      digitalWrite(MotorB12, LOW);
      digitalWrite(MotorB13, HIGH);
      delay(50);
    }

    //--------------- Analog Stick Motor value ------------
    if (abs(Ps3.data.analog.stick.lx) < 30 && abs(Ps3.data.analog.stick.ly) < 30)
    {
      Motor_Stop() ;
    }
    else if (abs(Ps3.data.analog.stick.lx) > abs(Ps3.data.analog.stick.ly))
    {
      if (Ps3.data.analog.stick.lx > 20)
      {
        Move_TurnRight();
      }
      else
      {
        Move_TurnLeft();
      }
    }
    else
    {
      if (Ps3.data.analog.stick.ly < -20)
      {
        Move_Forward();
      }
      else
      {
        Move_Backward();
      }
    }

    //--- Digital cross/square/triangle/circle button ---
    if ( Ps3.data.button.triangle )
    {
      Turret_MaxMin_Lock = 90;
      servo_angle(MotorSChannel, Turret_MaxMin_Lock);
    }
    if ( Ps3.data.button.circle )
    {
      if (Turret_MaxMin_Lock == 0)
      {
        Turret_MaxMin_Lock = 90;
      }
      else if (Turret_MaxMin_Lock >= 20)
      {
        Turret_MaxMin_Lock = Turret_MaxMin_Lock - 2;
      }
      else
      {
        Turret_MaxMin_Lock = 20;
      }
      servo_angle(MotorSChannel, Turret_MaxMin_Lock);
    }
    if ( Ps3.data.button.square )
    {
      if (Turret_MaxMin_Lock == 0)
      {
        Turret_MaxMin_Lock = 90;
      }
      else if (Turret_MaxMin_Lock <= 160)
      {
        Turret_MaxMin_Lock = Turret_MaxMin_Lock + 2;
      }
      else
      {
        Turret_MaxMin_Lock = 160;
      }
      servo_angle(MotorSChannel, Turret_MaxMin_Lock);
    }
    if ( Ps3.data.button.cross )
    {
      myDFPlayer.playMp3Folder(1);
      IR_Send_Code();
      Move_Fire();
      delay (800);
      myDFPlayer.playMp3Folder(3);
    }

    //--------------- Analog Stick Turret value ------------
    if ( abs(Ps3.data.analog.stick.rx) > 2 )
    {
      servo_angle(MotorSChannel, 90 - (Ps3.data.analog.stick.rx) / 2);
      Turret_MaxMin_Lock = 0;
    }
    else
    {
      if (Turret_MaxMin_Lock == 0)
      {
        servo_angle(MotorSChannel, 90);
      }
    }


  }


  //---------- Digital select/start button events ---------
  if ( Ps3.data.button.start )
  {
    if (MP3_Play == 0)
    {
      myDFPlayer.playMp3Folder(13);
      MP3_Play = 1;
      delay(300);
    }
    else
    {
      MP3_Play = 0;
      myDFPlayer.stop();
      delay(300);
    }
  }
  if ( Ps3.data.button.select )
  {
    if (Volume_Value <= 25)
    {
      Volume_Value = Volume_Value + 5;
      myDFPlayer.volume(Volume_Value);
      delay(300);
    }
    else
    {
      Volume_Value = 10;
      myDFPlayer.volume(Volume_Value);
      delay(300);
    }
  }
}
