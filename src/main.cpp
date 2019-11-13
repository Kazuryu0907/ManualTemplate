#include "mbed.h"
#include "sensor/MPU9250.h"
#include "driver/PIDController.h"
#include "driver/WheelKinematics.h"


#define DEBUG


typedef enum
{
  STCONN,CONN,NOCONN
}PS4Status;


PwmOut RFA(PB_4);
PwmOut RFB(PB_5);
PwmOut LFA(PC_8);
PwmOut LFB(PC_9);
PwmOut RBA(PD_12);
PwmOut RBB(PD_13);
PwmOut LBA(PD_14);
PwmOut LBB(PD_15);

struct
{
  double MaxPwm = 0.2;
}Pwms;

PwmOut WheelPins[8] = {
  RFA,RFB,LFA,LFB,RBA,RBB,LBA,LBB
};

double driverPWMOutput[4]; 

struct
{
  PinName IMUSDA = PB_11;
  PinName IMUSCL = PB_10;
} I2CPin;

struct baudRate
{
  const long HardwareSerial = 256000;
  const long I2C = 400000;
  const long SoftwareSerial = 115200;
} SerialBaud;

struct
{
  int LocationX;
  int LocationY;
  int Yaw;
  bool TRIANGLE;
  bool CIRCLE;
  bool CROSS;
  bool SQUARE;
  PS4Status Status = NOCONN;
  int HangerY;
  int R1;
  int L1;
  int preR1;
  double RollTowelPwm;
  double RollSheetPwm;
  bool Share;
  bool Options;
  double UnfoldPwm;
}ManualVaris;//arduinoからくるDualShockの信号


MPU9250 IMU(I2CPin.IMUSDA, I2CPin.IMUSCL, SerialBaud.I2C,i2c);

WheelKinematics wheelKinematics(WheelKinematics::Mechanum4WD,Pwms.MaxPwm);

PIDController pidObX(0.03, 0.0001, 0);
PIDController pidObY(0.03, 0.0001, 0);
PIDController pidObYaw(0.03, 0.005, 0);

Serial serial(USBTX, USBRX);


void ResetPacket(char *p,int l)
{
  p[0] = 127;
  p[1] = 127;
  for(int i = 2;i<6;i++)p[i] = 0;
  p[6] = 127;
  p[7] = 0;
  p[8] = 0;
}
void ReceivePacket()//arduinoから信号を受け取る
{
  static int PacketLen = 9;
  char buf[PacketLen];
  IMU.i2c_->read(0x08<<1,buf,PacketLen);
  
  if(buf[0] == 1)
  {
      ManualVaris.LocationY = abs(buf[1] - 127) < 10?0:-buf[1] + 127;
      ManualVaris.LocationX = abs(buf[2] - 127) < 10?0:buf[2] - 127;
      ManualVaris.Yaw = -buf[3] + buf[4];
      ManualVaris.TRIANGLE = buf[5] & 0b1000;
      ManualVaris.CIRCLE = buf[5] & 0b0100;
      ManualVaris.CROSS = buf[5] & 0b0010;
      ManualVaris.SQUARE = buf[5] & 0b0001;
      ManualVaris.HangerY = -(abs(buf[6] - 127) < 10?0:buf[6] - 127);
      ManualVaris.R1 = buf[7] & 0b10;
      ManualVaris.L1 = buf[7] & 0b01;
      ManualVaris.Share = buf[8] & 0b10;
      ManualVaris.Options = buf[8] & 0b01;
  }else{
    //接続状態じゃなかったら初期化
    ManualVaris.LocationY = 0;
    ManualVaris.LocationX = 0;
    ManualVaris.Yaw = 0;
    ManualVaris.TRIANGLE = 0;
    ManualVaris.CIRCLE = 0;
    ManualVaris.CROSS = 0;
    ManualVaris.SQUARE = 0;
    ManualVaris.HangerY = 0;
    ManualVaris.R1 = 0;
    ManualVaris.L1 = 0;
    ManualVaris.Share = 0;
    ManualVaris.Options = 0;
  }
  
}


void setup(){
  pidObX.setOutputLimit(Pwms.MaxPwm);
  pidObY.setOutputLimit(Pwms.MaxPwm);
  pidObX.setOutputLimit(Pwms.MaxPwm);
  serial.printf("%s","setupIMU:");
  IMU.setup();
  serial.printf("%s\n","END");
  for(int i = 0;i<8;i++)WheelPins[i].period_ms(1);
}  


void update(int XLocation,int YLocation,int Yaw){

  double pidYaw;
  pidYaw = (double)Yaw*0.001;
  wheelKinematics.getScale((double)XLocation*0.002,(double)YLocation*0.002,pidYaw,IMU.getYaw(),driverPWMOutput);
  wheelKinematics.controlMotor(WheelPins,driverPWMOutput);
}


int main()
{
  setup();
  while(1){
    IMU.update();
    ReceivePacket();
    update(ManualVaris.LocationX,ManualVaris.LocationY,ManualVaris.Yaw);
    serial.printf("%d%s%d%s%d%s%d%s%d%s%d%s%d\n",ManualVaris.LocationX,":",ManualVaris.LocationY,":",ManualVaris.Yaw,":",ManualVaris.TRIANGLE,":",ManualVaris.CIRCLE,":",ManualVaris.CROSS,":",ManualVaris.SQUARE);
  } 

}

