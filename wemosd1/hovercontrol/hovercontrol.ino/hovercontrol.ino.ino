/* hoverboard control from wemos d1 r2 (over wifi)
    12 aug 2018
      added over the air update
      MPU6050 Triple Axis Gyroscope & Accelerometer. Simple Gyroscope Example.GIT: https://github.com/jarzebski/Arduino-MPU6050
    13 aug 2018
      added esp8266fptserver to readin/readout SPIFFS through FTP: https://github.com/nailbuster/esp8266FTPServer
      create folder "data" (ctrl+K), filezille: ftp to IP address port 21, no encryption, 1 simultaneous connection
    15 aug 2018:
      redid MPU6050 reading since default library gave bullshit physical values
*/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <ESP8266FtpServer.h>

const char* ssid = "**";
const char* password = "**";

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;
// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

// struct to read out propoerties MPU6050 sensor
struct MPU6050_config {
  float GyroScaleFactor;
  float AccelScaleFactor;
};
struct MPU6050_config mpu6050_1;

struct MPU6050data {
  double Ax;
  double Ay;
  double Az;
  double T;
  double Gx;
  double Gy;
  double Gz;
};
struct MPU6050data kinematics;
struct MPU6050data MPU_offsets = {0, 0, 0, 0, 0, 0, 0};

float pitch = 0;
float roll = 0;
float yaw = 0;

// ftp server to get recorded data
FtpServer ftpSrv;

//IR receiver, seems to work with 3.3 and 5 V
int RECV_PIN = 11;
IRrecv irrecv(RECV_PIN);
decode_results results;


bool ledon = false;


//uart2, uart2 is the long cable closest to the buzzer
int16_t speed;
int16_t steer;
int16_t speedchange;
uint8_t cntr_uart2;
uint8_t checksum_uart2;
uint32_t timeout_uart2 = 0;

// different task rates
unsigned long time_scheduler1 = 0;
int taskrate1 = 2;

unsigned long time_scheduler2 = 0;
int taskrate2 = 20;

unsigned long time_scheduler3 = 0;
int taskrate3 = 50000;

void setup() {
  Serial.begin(19200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  pinMode(LED_BUILTIN, OUTPUT);

  //ftp connection with spiffs
  if (SPIFFS.begin())
  {
    Serial.println("SPIFFS opened!");
    ftpSrv.begin("wemosd1", "esp8266");   //username, password for ftp.  set ports in ESP8266FtpServer.h  (default 21, 50009 for PASV)
  }

  Wire.begin();

  MPU6050_Init();
  uint8_t scale_gyro = 3;
  uint8_t scale_accel = 0;
  setScale(MPU6050SlaveAddress, scale_gyro, scale_accel);
  mpu6050_1 = getMPU6050scales(MPU6050SlaveAddress);
  Serial.println("calibration sensor, don't move!");
  delay(1000);
  MPU_offsets = Read_PhysicalValues(mpu6050_1.GyroScaleFactor, mpu6050_1.AccelScaleFactor, MPU_offsets);

  irrecv.enableIRIn(); // Start the receiver
  steer = 0;
  speed = 0;

}


void loop() {
  ArduinoOTA.handle();
  ftpSrv.handleFTP();

  //2ms, only works at 2ms if rest of program doesn't take too long to calculate
  if (millis() >= time_scheduler1 + taskrate1)
  {
    time_scheduler1 = millis();
    //interpret commands IR receiver
    //values using LG remote: MKJ40653802
    if (irrecv.decode(&results)) {
      //      Serial.println(results.value, DEC);

      if (results.value == 551502015)
      {
        Serial.println("faster");
        speed = speed + 100;
        timeout_uart2 = 0;
      }
      else if (results.value == 551534655)
      {
        Serial.println("slower");
        speed = speed - 100;
        timeout_uart2 = 0;
      }
      else if (results.value == 551485695)
      {
        Serial.println("turn right");
        steer = steer + 100;
        timeout_uart2 = 0;
      }
      else if (results.value == 551518335)
      {
        Serial.println("turn left");
        steer = steer - 100;
        timeout_uart2 = 0;
      }
      else if (results.value == 551489775)
      {
        Serial.println("turn off");
        steer = 0;
        speed = 0;
        timeout_uart2 = 0;
      }



      irrecv.resume(); // Receive the next value
    }
    else
    {
      timeout_uart2++;
      uint32_t timeout_cal = 1000;
      timeout_cal /= taskrate1;
      Serial.print("timeout remote");
      Serial.println(timeout_uart2, DEC);
      //      Serial.println(timeout_cal,DEC);

      if (timeout_uart2 > timeout_cal)
      {
        Serial.println("no input received for too long");
        steer = 0;
        speed = 0;
      }
    }


    Serial.print("speed ");
    Serial.println(speed);

    Serial.print("steer ");
    Serial.println(steer);

  }


  if (millis() >= time_scheduler2 + taskrate2)
  {
    time_scheduler2 = millis();

    kinematics = Read_PhysicalValues(mpu6050_1.GyroScaleFactor, mpu6050_1.AccelScaleFactor, MPU_offsets);

    float timeStep = 0.001 * (float)taskrate2;
    pitch = pitch + kinematics.Gy * timeStep;
    roll = roll + kinematics.Gx * timeStep;
    yaw = yaw + kinematics.Gz * timeStep;

    Serial.print("pitch: ");
    Serial.print(pitch, DEC);


    Serial.print("roll: ");
    Serial.print(roll, DEC);


    Serial.print("yaw: ");
    Serial.print(yaw, DEC);

    Serial.println();

  }

  //blink led to indicate program is running
  if (millis() >= time_scheduler3 + taskrate3)
  {
    time_scheduler3 = millis();

    if (ledon == false)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      ledon = true;
      //      Serial.println("pin high");
    }
    else
    {
      digitalWrite(LED_BUILTIN, LOW);
      ledon = false;
      //      Serial.println("pin low");
    }


    //SPIFFS writing
    // open file for writing
    File f = SPIFFS.open("/f.txt", "w");
    if (!f) {
      Serial.println("file open failed");
    }
    Serial.println("====== Writing to SPIFFS file =========");
    // write 10 strings to file
    for (int i = 1; i <= 10; i++) {
      f.print("Millis() : ");
      f.println(millis());
      Serial.println(millis());
    }

    f.close();

    // open file for reading
    f = SPIFFS.open("/f.txt", "r");
    if (!f) {
      Serial.println("file open failed");
    }  Serial.println("====== Reading from SPIFFS file =======");
    for (int i = 1; i <= 10; i++) {
      String s = f.readStringUntil('\n');
      Serial.print(i);
      Serial.print(":");
      Serial.println(s);
    }
    f.close();

  }

}






// all helper functions


struct MPU6050data Read_PhysicalValues(float GyroScaleFactor, float AccelScaleFactor, MPU6050data offsets)
{
  //read out MPU6050 sensor and return gyroscope data in deg/s; accelerometer in g
  struct MPU6050data_raw {
    int16_t Ax;
    int16_t Ay;
    int16_t Az;
    int16_t T;
    int16_t Gx;
    int16_t Gy;
    int16_t Gz;
  };

  struct MPU6050data_raw kin ;
  struct MPU6050data w;

  Wire.beginTransmission(MPU6050SlaveAddress);
  Wire.write(MPU6050_REGISTER_ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050SlaveAddress, (uint8_t)14);
  kin.Ax = (((int16_t)Wire.read() << 8) | Wire.read());
  kin.Ay = (((int16_t)Wire.read() << 8) | Wire.read());
  kin.Az = (((int16_t)Wire.read() << 8) | Wire.read());
  kin.T = (((int16_t)Wire.read() << 8) | Wire.read());
  kin.Gx = (((int16_t)Wire.read() << 8) | Wire.read());
  kin.Gy = (((int16_t)Wire.read() << 8) | Wire.read());
  kin.Gz = (((int16_t)Wire.read() << 8) | Wire.read());


  //divide each with their sensitivity scale factor and correct for offsets
  w.Ax = kin.Ax / AccelScaleFactor - offsets.Ax;
  w.Ay = kin.Ay / AccelScaleFactor - offsets.Ay;
  w.Az = kin.Az / AccelScaleFactor - offsets.Az;
  w.T = kin.T / 340 + 36.53; //temperature formula
  w.Gx = kin.Gx / GyroScaleFactor - offsets.Gx;
  w.Gy = kin.Gy / GyroScaleFactor - offsets.Gy;
  w.Gz = kin.Gz / GyroScaleFactor - offsets.Gz;

  //      Serial.println("raw values");
  //    Serial.print("Ax: "); Serial.print(kin.Ax);
  //    Serial.print(" Ay: "); Serial.print(kin.Ay);
  //    Serial.print(" Az: "); Serial.print(kin.Az);
  //    Serial.print(" T: "); Serial.print(kin.T);
  //    Serial.print(" Gx: "); Serial.print(kin.Gx);
  //    Serial.print(" Gy: "); Serial.print(kin.Gy);
  //    Serial.print(" Gz: "); Serial.println(kin.Gz);

  //    Serial.println("converted values");
  Serial.print("Ax: "); Serial.print(w.Ax);
  Serial.print(" Ay: "); Serial.print(w.Ay);
  Serial.print(" Az: "); Serial.print(w.Az);
  Serial.print(" T: "); Serial.print(w.T);
  Serial.print(" Gx: "); Serial.print(w.Gx);
  Serial.print(" Gy: "); Serial.print(w.Gy);
  Serial.print(" Gz: "); Serial.print(w.Gz);

  return w;

}

void MPU6050_Init() {
  //configure MPU6050
  delay(100);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);

  delay(100);

}


void setScale(byte addr, uint8_t scale_gyro, uint8_t scale_acc)
// set scaling factors for gyro and accelerometer reading
{

  uint8_t value;
  if (scale_gyro == 0)
  {
    value = 0x00;
    Serial.println("scale set to 250 deg/s");
  }
  else if (scale_gyro == 3) {
    value = 0x18;
    Serial.println("scale set to 2000 deg/s");
  }
  else
  {
    value = 0x00;
    Serial.println("scale set to 250 deg/s");
  }
  Wire.beginTransmission(MPU6050SlaveAddress);
  Wire.write(MPU6050_REGISTER_GYRO_CONFIG);
  Wire.write(value);
  Wire.endTransmission();
  delay(200);


  if (scale_acc == 0)
  {
    value = 0x00;
    Serial.println("scale set to 2g");
  }
  else if (scale_acc == 3) {
    value = 0x18;
    Serial.println("scale set to 16g");
  }
  else
  {
    value = 0x00;
    Serial.println("scale set to 2 deg/s");
  }
  Wire.beginTransmission(MPU6050SlaveAddress);
  Wire.write(MPU6050_REGISTER_ACCEL_CONFIG);
  Wire.write(value);
  Wire.endTransmission();
  delay(100);

}


struct MPU6050_config getMPU6050scales(byte addr) {
  Wire.beginTransmission(addr);
  //  Wire.write(0x1B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.write(MPU6050_REGISTER_GYRO_CONFIG);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 2, true); // request a total of 14 registers
  uint8_t Gyro = (Wire.read() & (bit(3) | bit(4))) >> 3;
  uint8_t Accl = (Wire.read() & (bit(3) | bit(4))) >> 3;

  float GyroScaleFactor;
  float AccelScaleFactor;

  if (Gyro == 0)
  {
    GyroScaleFactor = 131.0;
  }
  else if (Gyro == 3)
  {
    GyroScaleFactor = 16.4;
  }

  if (Accl == 0)
  {
    AccelScaleFactor = 16384;
  }
  else if (Accl == 3)
  {
    AccelScaleFactor = 2048;
  }

  //  Serial.println(GyroScaleFactor, DEC);
  //  Serial.println(AccelScaleFactor, DEC);

  MPU6050_config r = {GyroScaleFactor, AccelScaleFactor};
  return r;

  //  const float MPU_ACCL_2_SCALE = 16384.0;
  //const float MPU_ACCL_4_SCALE = 8192.0;
  //const float MPU_ACCL_8_SCALE = 4096.0;
  //const float MPU_ACCL_16_SCALE = 2048.0;
  //  const float MPU_GYRO_250_SCALE = 131.0;
  //const float MPU_GYRO_500_SCALE = 65.5;
  //const float MPU_GYRO_1000_SCALE = 32.8;
  //const float MPU_GYRO_2000_SCALE = 16.4;

}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}


