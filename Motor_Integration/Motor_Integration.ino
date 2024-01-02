#define DIR A0
#define PUL A1
#define SUB 500
 
#include <MsTimer2.h>
#include <math.h>
int dtime = 50;
int delay_time = 0;
double speed, direction;
double speed_r, direction_r;
String speed_s, direction_s, angle_s;
String command, command_last;
char command_head;

void change_direction() {
  if (direction == 1) {
    digitalWrite(DIR, HIGH);
    direction = 0;
  } else {
    digitalWrite(DIR, LOW);
    direction = 1;
  }
}


String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(DIR, OUTPUT);
  delayMicroseconds(50);
  pinMode(PUL, OUTPUT);
  digitalWrite(DIR, LOW);
  delayMicroseconds(50);

  MsTimer2::set(50000, change_direction);
  MsTimer2::start();
}




void loop() {

  // 等待串口接收到字符 'R'
  // 在等待过程中向串口发送字符 'S'，每隔 1 秒发送一次
  while (Serial.read() != 'R') {
    Serial.print("S");
    delay(1000);
  }

  double angle = 0, timer_control_d=0;
  
  int timer_control = 0;

  while (1) {
    command_head = Serial.read();
    switch(command_head){
      case 'R':{
        command = Serial.readString(); // 读取 "speed,dir"，speed单位为转/秒,最大转速为5转/秒
        // 如果指令不为空且不为 'a'，则解析指令
        if (command_last!=command && command != 'a' && command != NULL){
          
        // 从指令中获取 speed、direction两个数值，并转换为 double 类型
        speed_s = getValue(command,',',0);
        speed_r = speed_s.toDouble();
        direction_s = getValue(command,',',1);
        direction_r = direction_s.toDouble();
        MsTimer2::stop();
        //更新speed与direction的值
        speed = speed_r;
        speed = 0.0003*pow(speed,4)-0.0001*pow(speed,3)+0.0237*pow(speed,2)+0.9542*speed;//进行转速修正,该修正公式仅在（1~20Hz）下适用,误差为5%
        direction = direction_r;
        delay(dtime);

        // 输出当前速度和转动方向信息到串口
        Serial.print("R"); Serial.print((float)speed_r); Serial.print(","); Serial.println((int
        )direction);         
          
        // 将 command 设置为 'a'，防止重复执行指令
        command = 'a';
        command_last = command;        
        break;}
      }
      case 'O':{
        command = Serial.readString(); // 读取 "speed,dir"，speed单位为转/秒,最大转速为5转/秒
        // 如果指令不为空且不为 'a'，则解析指令
        if (command_last!=command && command != 'a' && command != NULL){
          
        speed_s = getValue(command, ',', 0);
        speed_r = speed_s.toDouble();
        direction_s = getValue(command, ',', 1);
        direction_r = direction_s.toDouble();
        angle_s = getValue(command, ',', 2);
        //更新speed与direction的值
        speed = speed_r;
        direction = direction_r;
        angle = angle_s.toDouble();
        //angle = angle + speed*40; //修正量
        delay(dtime);

        // 输出当前速度和转动方向信息到串口
        Serial.print("O");
        Serial.print(speed);
        Serial.print(",");
        Serial.print(direction);
        Serial.print(",");
        Serial.println(angle);
        // 延迟 dtime 毫秒s
        delay(dtime);
        
        timer_control = (angle / (speed * 360)) * 1000;
        MsTimer2::set(timer_control, change_direction);
        MsTimer2::start();

        // 将 command 设置为 'a'，防止重复执行指令
        command = 'a';
        command_last = command;
        break;
        }
      }
      case 'P':{
        speed = 0;
        MsTimer2::stop();
        break;
      }
      default:{// 如果接收到的字符不是 'N'，则清空串口缓冲区
        while (Serial.available() != 0) {
          Serial.read();
          delay(10);}    
          break;    
      }
      }

    //控制电机转动方向
    if (direction == 1)
      digitalWrite(DIR, HIGH);
    else
      digitalWrite(DIR, LOW);

    //更新脉冲宽度
    delay_time = 1000000 / (speed * 2 * 1.15 * SUB);
    if (speed != 0) {
      //电机运行
      digitalWrite(PUL, HIGH);
      delayMicroseconds(delay_time);
      digitalWrite(PUL, LOW);
      delayMicroseconds(delay_time);
    }
  }
}

// SEND 'R' TO START THE MOTOR
//(ROTATION)     R SPEED,DIRECTION
//(OSICILLATION) O SPEED,DERECTION,ANGLE
//(PAUSE)        P
