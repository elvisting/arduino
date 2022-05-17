/*
  Quadruped robot arduino sketch.
  3/10/2020 by Alexandros Petkos
  Updates available at https://github.com/maestrakos/warp

  This kinematics sketch is placed under CC-BY.

  This file is part of warp_kinematics.

  [source] This is the main file that manages [kinematics] & [hardware]
  all the important parameters are set in this file.

  Comment Description:

  /// comment

  //> used to explain the function of a line
  //: used to summurize the function of multiple lines

  === used for headers
  ::: used for sketch parts

  // ## used to explain the measurement unit of a variable
  // !! used for warnings
*/

#include "datatypes.h"

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*#include <I2Cdev.h>
  #include <MPU6050_6Axis_MotionApps20.h>
  MPU6050 mpu;*/

#include <PS4Controller.h>

/*
  ==============================
  IMPORTANT PARAMETERS
  ==============================
*/
//> stores the frequency of the loop function(循環程式的頻率)
const float frequency = 440.0; // ## Hz

/// Kinematics Parameters //宣告變數48~71、80~93

//: stores the location, rotation and scale of the main [body]
//(儲存移動後主身體的位置(mm)、角度(degree)、大小(mm))
const datatypes::Transform body_transform = {
  {0, 0, 0},  // ## {mm, mm, mm}
  {0, 0, 0},   // ## {deg, deg, deg}
  {300, 40, 180} // ## {mm, mm, mm}
};
//(儲存四肢與身體的相對位置(mm))
//: stores the parent joint location relative to the [body]
const datatypes::Vector p_joint_origin[] = {
  { -50, 0, 0}, // ## {mm, mm, mm}
  { +50, 0, 0}, // ## {mm, mm, mm}
  { +50, 0, 0}, // ## {mm, mm, mm}
  { -50, 0, 0}  // ## {mm, mm, mm}
};
const float bone_length = 105; // ## mm(四肢長度)

//: high level parameters for the step function
const datatypes::Vector step_extent = {40, 40, 26}; // ## {mm, mm}
float vrt_offset = - 16.50; // ## mm
float hrz_offset = - 6.00; // ## mm

float base_offset[] = { 0, -1, 0, -2};
const float precision = 0.001; // ## mm

void setup() {
  Serial.begin(115200);

  init_hardware();
  init_input();
}

//: those local variables control the step direction and period
datatypes::Vector2D _direction = {0, 0};
float turn = 0; //> indicates the direction of rotation
float height = 0; //> indicates the leg extension
//有五種模式(0）停滯（1）小跑（2）轉向（3）翻滾（4）物體檢測
int state = 0; //> indicates the type of gait, (0) idle, (1) trot, (2) yaw, (3) pitch-roll, (4) object-detection
float _period = 10.0; //> indicates the number of steps every second

datatypes::Rotator _sRotation; //> this variable stores the relative rotation of the body

unsigned long duration;
int sample_sum, sample_num = 10,
                sample_index;
float freq;

void loop() {
  duration = millis();//取得週期
//程式執行時，取得handle_hardware(mpu狀態)、handle_kinematics(機器人狀態)並提供handle_input(搖桿輸入)
  handle_hardware();
  handle_kinematics(_direction, turn, height, _period);

  handle_input();
//偵測到available(1 or 0)，使用handle_serial程式(下方有說明1、0的功用)
  if (Serial.available())
    handle_serial();

  // this code gets the frequency of the loop function
  /*sample_sum += 1000.0 / (millis() - duration);
    sample_index++;

    if (sample_index > sample_num) {
    freq = sample_sum / sample_num;
    Serial.println(freq);
    sample_sum = 0;
    sample_index = 0;
    }*/
}
//手把(PS4)藍芽設定
float vo, ho;
void init_input() {
  PS4.begin("F8:C3:9E:3F:F8:10"); // !! replace with your own DualShock4 Controller Bluetooth MAC address
  vo = vrt_offset;
  ho = hrz_offset;
}

bool _tb = false;
float stick_min = 6.f;
float lx, ly, rx, ry;
void handle_input() {
  if (PS4.isConnected()) {
    lx = inter(lx, PS4.data.analog.stick.lx / 4.f, 0.5f); //> gets the interpolated x-position of the left  analog stick
    ly = inter(ly, PS4.data.analog.stick.ly / 4.f, 0.5f); //> gets the interpolated y-position of the left  analog stick
    rx = inter(rx, PS4.data.analog.stick.rx / 4.f, 0.5f); //> gets the interpolated x-position of the right analog stick
    ry = inter(ry, PS4.data.analog.stick.ry / 4.f, 0.5f); //> gets the interpolated y-position of the right analog stick

    if (abs(lx) > stick_min) { //> checks whether the stick position is out of the deadzone
      float x0 = lx - stick_min * sign(lx); //> subtracts the deadzone
      if (state == 1) {
        _direction.y = 0;//x0 / 10.f;
      } else if (state != 4) {
        _direction.y = x0 / 2;
      }
    } else _direction.y = 0;

    if (abs(ly) > stick_min) { //> checks whether the stick position is out of the deadzone
      float y0 = ly - stick_min * sign(ly); //> subtracts the deadzone
      if (state == 1) {
        _direction.x = y0 / 10.f;
        if (y0 > 0)
          vrt_offset = inter(vrt_offset, vo - 6.f, 2.f);
        else
          vrt_offset = inter(vrt_offset, vo + 3.f, 2.f);
      } else if (state != 4) {
        _direction.x = y0 / 2;
        vrt_offset = vo;
      }
    } else {
      _direction.x = 0;
      vrt_offset = vo;
    };

    if (abs(rx) > stick_min) { //> checks whether the stick position is out of the deadzone
      float x1 = rx - stick_min * sign(rx); //> subtracts the deadzone
      if (state == 1)
        turn = x1 / 16.f;
      else if (state != 4)
        turn = x1;
    } else turn = 0;

    if (abs(ry) > stick_min) { //> checks whether the stick position is out of the deadzone
      float y1 = ry - stick_min * sign(ry); //> subtracts the deadzone
      height = y1;
    } else height = 0;
  }
//以上為PS4搖桿偵測(初始狀態(搖桿的座標位置))與死區偵測，並排除死區，結果為搖桿輸出
//控制器「死區」這項設定讓您變更控制器搖桿的死區尺寸。推動搖桿時，需要超過「死區」才會帶動遊戲視角的改變。
  if (PS4.data.button.touchpad) { //> checks the touchpad state//檢查ps4搖桿觸控面板狀態
    if (_tb == true) {
      _tb = false; state++;
      if (state > 4) state = 0;
    }
  } else _tb = true;
}

// !! make sure you have enabled Newline or Carriage return
#define _mode 1 // (0) used for calibration and testing(mode0為校準和測試), (1) uses serial as input(mode1=輸入)
void handle_serial() {
  //: reads and stores the serial data
  int i = 0; float buff[3] = {0, 0, 0};
  String s_buff = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 13 || c == 32 || c == '\n') {
      buff[i] = s_buff.toFloat();
      s_buff = "";
      i++;
    } else
      s_buff += c;
  }

  if (_mode == 0)
    commands_exe(buff[0], buff[1], buff[2]);
  else if (_mode == 1)
    if (state == 4) {
      _direction = {buff[0], buff[1]};
      turn = buff[2];
    }
}

//: this is an interpolation function used to smooth(保持行動順暢)
float inter(float in, float en, float pl) {
  if (in < en - pl) {
    return ((in * 1000.f) + (pl * 1000.f)) / 1000.0;
  } else if (in > en + pl) {
    return ((in * 1000.f) - (pl * 1000.f)) / 1000.0;
  } else return en;
}

#define properties 0
void commands_exe(float val1, float val2, float val3) {
  //當properties=0，可以校準關節
  //當properties=1，可以做細部微調，使機身平衡
  //: properties 0 is used to calibrate the joints
  if (properties == 0) {
    int leg = val1;
    int joint = val2;
    int servo = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" joint ");
    Serial.print(joint);
    Serial.print(" set to ");
    Serial.print(servo);
    Serial.print(".\n");

    set_servo(leg, joint, servo);
  }
  //: properties 1 is used for small adjustments to balance the weight
  else if (properties == 1) {
    int leg = val1;
    int empty = val2;
    int ammount = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" null ");
    Serial.print(empty);
    Serial.print(" set to ");
    Serial.print(ammount);
    Serial.print(".\n");

    base_offset[leg] = ammount;
  }
}
