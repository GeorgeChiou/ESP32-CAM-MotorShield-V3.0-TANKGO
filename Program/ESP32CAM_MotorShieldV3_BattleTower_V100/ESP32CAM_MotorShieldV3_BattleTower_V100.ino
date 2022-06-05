// 最後編輯 2022-3-20 by ShinWei Chiou
// 初版

/*
  Board: ESP32 Wrover Module
  Upload Speed: 115200
  Flash Frequency: 80MHz
  Flash Mode: QIO
  Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)

  ESP32-CAM Motor Shield V3.0
  MAX7219 CLK : P14 , CS : P13 , DIN : P12
  Servo : P2
  IR Receiver : P0

  DFRobot DFPlayer Mini : GND VCC TX
  https://github.com/DFRobot/DFRobotDFPlayerMini
  <https://www.dfrobot.com/product-1121.html>
*/

#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>
HardwareSerial mySerial(1);
DFRobotDFPlayerMini myDFPlayer;

// IR
#define IR_Receiver_Pin 0
int IR_Value = 0;

// Servo
#define MotorServo      2
#define MotorSChannel   1

// MAX7219 8x8 Dot Matrices
#define MAX7219_CLK_Pin  14
#define MAX7219_CS_Pin   13
#define MAX7219_DIN_Pin  12

// MAX7219 address map
#define MAX7219_DECODE_REG      (0x09)
#define MAX7219_INTENSITY_REG   (0x0A)
#define MAX7219_SCANLIMIT_REG   (0x0B)
#define MAX7219_SHUTDOWN_REG    (0X0C)
#define MAX7219_DISPLAYTEST_REG (0x0F)
#define MAX7219_DIGIT_REG(pos)  ((pos) + 1)
#define MAX7219_COLUMN_REG(pos) MAX7219_DIGIT_REG(pos)

// shutdown mode
#define MAX7219_OFF             (0x0)
#define MAX7219_ON              (0x1)

// number of columns of the display matrx
#define NUM_OF_COLUMNS  (8)

// for each character bitmap, it consumes bytes
#define BYTE_PER_MAP    (8)

unsigned int char_index = 5;
unsigned int rsset_index = 0;

// matrix pattern
const byte char_pattern[] =
{
  0x00, 0x3E, 0x67, 0x6F, 0x7B, 0x73, 0x63, 0x3E, // 0
  0x00, 0x3F, 0x0C, 0x0C, 0x0C, 0x0C, 0x0E, 0x0C, // 1
  0x00, 0x3F, 0x33, 0x06, 0x1C, 0x30, 0x33, 0x1E, // 2
  0x00, 0x1E, 0x33, 0x30, 0x1C, 0x30, 0x33, 0x1E, // 3
  0x00, 0x78, 0x30, 0x7F, 0x33, 0x36, 0x3C, 0x38, // 4

  0x00, 0x1E, 0x33, 0x30, 0x30, 0x1F, 0x03, 0x3F, // 5
  0x00, 0x1E, 0x33, 0x33, 0x1F, 0x03, 0x06, 0x1C, // 6
  0x00, 0x0C, 0x0C, 0x0C, 0x18, 0x30, 0x33, 0x3F, // 7
  0x00, 0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E, // 8
  0x00, 0x0E, 0x18, 0x30, 0x3E, 0x33, 0x33, 0x1E, // 9

  //  0x00, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x1E, 0x0C, // A
  //  0x00, 0x3F, 0x66, 0x66, 0x3E, 0x66, 0x66, 0x3F, // B
  //  0x00, 0x3C, 0x66, 0x03, 0x03, 0x03, 0x66, 0x3C, // C
  //  0x00, 0x1F, 0x36, 0x66, 0x66, 0x66, 0x36, 0x1F, // D
  //  0x00, 0x7F, 0x46, 0x16, 0x1E, 0x16, 0x46, 0x7F, // E
  //  0x00, 0x0F, 0x06, 0x16, 0x1E, 0x16, 0x46, 0x7F, // F

  0x00, 0x63, 0x36, 0x1C, 0x1C, 0x36, 0x63, 0x63, // X
  0x00, 0x1C, 0x36, 0x63, 0x63, 0x63, 0x36, 0x1C, // O

  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // Full
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // space
};

#define DISPLAY_STR_LENGTH  (sizeof(char_pattern) / BYTE_PER_MAP)

void Set_MAX7219_register(byte address, byte value)
{
  digitalWrite(MAX7219_CS_Pin, LOW);
  shiftOut(MAX7219_DIN_Pin, MAX7219_CLK_Pin, MSBFIRST, address);
  shiftOut(MAX7219_DIN_Pin, MAX7219_CLK_Pin, MSBFIRST, value);
  digitalWrite(MAX7219_CS_Pin, HIGH);
}

void Clear_MAX7219_matrix()
{
  // clear the dot matrix
  for (int i = 0; i < NUM_OF_COLUMNS; i++)
  {
    Set_MAX7219_register(MAX7219_COLUMN_REG(i), B00000000);
  }
}

void Initialize_MAX2719()
{
  // disable test mode.
  Set_MAX7219_register(MAX7219_DISPLAYTEST_REG, MAX7219_OFF);

  // set medium intensity.
  Set_MAX7219_register(MAX7219_INTENSITY_REG, 0x1);

  // turn off display.
  Set_MAX7219_register(MAX7219_SHUTDOWN_REG, MAX7219_OFF);

  // drive 8 digits.
  Set_MAX7219_register(MAX7219_SCANLIMIT_REG, 7);

  // no decode mode for all positions.
  Set_MAX7219_register(MAX7219_DECODE_REG, B00000000);

  // clear matrix display
  Clear_MAX7219_matrix();
}

void Display_MAX2719()
{
  // turn off display first
  Set_MAX7219_register(MAX7219_SHUTDOWN_REG, MAX7219_OFF);

  // display one bitmap
  for (int i = 0; i < BYTE_PER_MAP; i++)
  {
    // starting from column
    Set_MAX7219_register(MAX7219_COLUMN_REG(i), char_pattern[char_index * BYTE_PER_MAP + i]);
  }

  // turn on display
  Set_MAX7219_register(MAX7219_SHUTDOWN_REG, MAX7219_ON);
}

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

/*------------------------------------------------------------*/
void setup()
{
  // Initialize serial communications at 9600 bps:
  Serial.begin(9600);

  // MAX7219 PIN Set
  pinMode(MAX7219_CLK_Pin, OUTPUT);
  pinMode(MAX7219_CS_Pin, OUTPUT);
  pinMode(MAX7219_DIN_Pin, OUTPUT);

  // IR PIN Set
  pinMode(IR_Receiver_Pin, INPUT);

  // 設定 Servo Channel
  ledcSetup(MotorSChannel, 50, 16);
  ledcAttachPin(MotorServo, MotorSChannel);
  servo_angle(MotorSChannel, 90);

  // Initialize MAX2719 states
  Initialize_MAX2719();
  Display_MAX2719();

  // 將 Serial Port 設定給 DFplayer 使用
  mySerial.begin(9600, SERIAL_8N1, 3, 1); // begin(9600, SERIAL_8N1, RXD, TXD)
  myDFPlayer.begin(mySerial);
  myDFPlayer.volume(30);
  myDFPlayer.playMp3Folder(15);
}


/*------------------------------------------------------------*/
void loop()
{
  //--------------- IR Battle System ---------------
  if (digitalRead(IR_Receiver_Pin) == LOW)
  {
    IR_Value = pulseIn(IR_Receiver_Pin, LOW);

    if ( IR_Value >= 2700 && IR_Value <= 3400 )
    {
      if (char_index >= 10)
      {
        rsset_index++;

        if (rsset_index >= 4)
        {
          char_index = 6;
          rsset_index = 0;
          Display_MAX2719();
        }
      }
      else if (char_index >= 2)
      {
        char_index--;
        Display_MAX2719();

        myDFPlayer.playMp3Folder(2);

        servo_angle(MotorSChannel, 110);
        delay (500);
        servo_angle(MotorSChannel, 70);
        delay (500);
        servo_angle(MotorSChannel, 90);
      }
      else
      {
        char_index = 10;
        Display_MAX2719();

        myDFPlayer.playMp3Folder(1);

        servo_angle(MotorSChannel, 170);
        delay (5000);
      }
    }
  }
}
