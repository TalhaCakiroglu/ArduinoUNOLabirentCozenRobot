//Gerekli kütüphaneler
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define min_speed 30
SoftwareSerial bluetoothIletisim(10,9);//RX,TX -> Arduino da ters
MPU6050 dengee; 
                       
bool dmpDurum = true; //veri paketi              
uint8_t mpuIntStatus;               
uint8_t devStatus; //çalışırsa 0 çalışmazsa 1 değeri dönmesi sağlanacak
uint8_t fifoBuffer[64];                    
uint16_t packetSize;                
uint16_t fifoSayac;                 
           
                                
Quaternion q;                       
VectorFloat gravity;               
float ypr[3];                       

double originalSetpoint = 178;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.08;
double giris, cikis;

//PID tanımlaması
double Kp = 500;
double Kd = 100;
double Ki = 0.5;
PID pid(&giris, &cikis, &setpoint, Kp, Ki, Kd, DIRECT);

double solMotor = 0.8;
double sagMotor = 0.6;

//arduino pin bağlantıları
const int sag_i=7;
const int sag_g=6;
const int sol_i=5;
const int sol_g=4;
const int enb=3;
const int ena=8;
const int sol_sensor = 11;
const int sag_sensor = 12;
const int bitirme_sensor = 13;
int sol_durum, sag_durum, bitirme_durum;
LMotorController motorController(ena, sag_i, sag_g, enb, sol_i, sol_g, solMotor, sagMotor);

//jiroskop kesmesini ilk başta false olarak tanımlıyoruz
volatile bool mpuInterrupt = false;                 

void dmpDataReady()
{   //eğer veri paketi gönderimi sağlanırsa jiroskop kesmesini true yapıyoruz 
    mpuInterrupt = true;
}

void setup()
{
     //Bluetooth bağlantısı için setup
     Serial.begin(9600);
     bluetoothIletisim.begin(9600);
     //Mpu6050 setup
    Wire.begin();
    dengee.initialize();
    devStatus = dengee.dmpInitialize();
    //jiroskop x,y,z varsayılan değerlerimiz                               
    dengee.setXGyroOffset(200);
    dengee.setYGyroOffset(85);
    dengee.setZGyroOffset(-65);
    dengee.setZAccelOffset(1750);
    
    if (devStatus == 0)//çalışır
    {
      dengee.setDMPEnabled(true); //veri transferi -dmp- başlat
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = dengee.getIntStatus();
      dmpDurum = true;
      packetSize = dengee.dmpGetFIFOPacketSize();
      pid.SetMode(AUTOMATIC);
      pid.SetSampleTime(10);
      pid.SetSampleTime(5);
      pid.SetOutputLimits(-255, 255);
    }

    else
    {
      Serial.print(("DMP başlatılamadı..."));
      Serial.print(devStatus);
    }
  //TCRT5000L setup ve Dc motor setup
   pinMode(sag_i, OUTPUT);
   pinMode(sag_g, OUTPUT);
   pinMode(sol_i, OUTPUT);
   pinMode(sol_g, OUTPUT);
   pinMode(enb, OUTPUT);
   pinMode(ena, OUTPUT);
   pinMode(sag_sensor, INPUT); 
   pinMode(sol_sensor, INPUT);
   pinMode(bitirme_sensor, INPUT);
 
}


void labirent(){
   //kızılötesi sensörlerden okunan değerleri kaydetme
   sol_durum = digitalRead(sol_sensor); 
   sag_durum = digitalRead(sag_sensor);
   bitirme_durum = digitalRead(bitirme_sensor);
   digitalWrite(enb, HIGH);
   digitalWrite(ena, HIGH);
 
   if (sol_durum == LOW && sag_durum == LOW) //ikiside siyah okursa
   { digitalWrite(enb, HIGH);
     digitalWrite(ena, HIGH);    
     digitalWrite(sag_i, HIGH);
     digitalWrite(sag_g, LOW);
     digitalWrite(sol_i, HIGH);
     digitalWrite(sol_g, LOW);
  }
  else if (sol_durum == LOW && sag_durum == HIGH)  // sol siyah okursa
  { digitalWrite(enb, HIGH);
    digitalWrite(ena, HIGH);
    digitalWrite(sag_i, HIGH);
    digitalWrite(sag_g, HIGH);
    digitalWrite(sol_i, HIGH);
    digitalWrite(sol_g, LOW);
  }
  else if (sol_durum == HIGH && sag_durum == LOW) //sağ siyah okursa
  { digitalWrite(enb, HIGH);
    digitalWrite(ena, HIGH);
    digitalWrite(sag_i, HIGH);
    digitalWrite(sag_g, LOW);
    digitalWrite(sol_i, HIGH);
    digitalWrite(sol_g, HIGH);
  }
  else if (sol_durum == HIGH && sag_durum == HIGH && bitirme_durum == LOW) //sağ, sol siyah okumaz bitirmeSensör okursa dur.
  {
    digitalWrite(enb, HIGH);
    digitalWrite(ena, HIGH);
    digitalWrite(sag_i, LOW);
    digitalWrite(sag_g, LOW);
    digitalWrite(sol_i, LOW);
    digitalWrite(sol_g, LOW);
  }  
  else  //hiçbirine girmezse yine dur
  { digitalWrite(enb, HIGH);
    digitalWrite(ena, HIGH);
    digitalWrite(sag_i, LOW);
    digitalWrite(sag_g, LOW);
    digitalWrite(sol_i, LOW);
    digitalWrite(sol_g, LOW);
  }
}


void denge(){
    if (!dmpDurum)//veri transferi yoksa başlama
    {
      return;
    }
    while (!mpuInterrupt && fifoSayac < packetSize)
    {
        pid.Compute();
        motorController.move(cikis, min_speed);    
    }
   
    mpuInterrupt = false;
    mpuIntStatus = dengee.getIntStatus();
    fifoSayac = dengee.getFIFOCount();
   //taşma kontrol
  if ((mpuIntStatus & 0x10) || fifoSayac == 1024)
    {
      dengee.resetFIFO();
      Serial.println(("FIFO taştı!"));
    }
  else if (mpuIntStatus & 0x02)
    {
      while (fifoSayac < packetSize) fifoSayac = dengee.getFIFOCount();
      dengee.getFIFOBytes(fifoBuffer, packetSize);
      fifoSayac -= packetSize;
      dengee.dmpGetQuaternion(&q, fifoBuffer);
      dengee.dmpGetGravity(&gravity, &q);
      dengee.dmpGetYawPitchRoll(ypr, &q, &gravity);
      giris = ypr[1] * 180/M_PI + 180;
    }
}


void loop() 
 {
       denge();
       if(mpuIntStatus  == false) //kesme bilgisi yoksa, robot dengededir bu yüzden labirent çöz
        {
          labirent();
          bluetoothIletisim.println("Veri Gönderimi");
        }
       denge();
 }

 //by T@lha_Ç@kıroğlu All Rights Reserved...



    
