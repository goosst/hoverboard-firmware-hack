/* hoverboard control using wemos d1 r2 (over wifi)
    12 aug 2018
      added over the air update
      MPU6050 Triple Axis Gyroscope & Accelerometer. Simple Gyroscope Example.GIT: https://github.com/jarzebski/Arduino-MPU6050
    13 aug 2018
      added esp8266fptserver to read in/read out SPIFFS through FTP: https://github.com/nailbuster/esp8266FTPServer
      create folder "data" (ctrl+K), filezille: ftp to IP address port 21, no encryption, 1 simultaneous connection
    15 aug 2018:
      re-did MPU6050 reading since default library gave bullshit physical values
    17 aug 2018:
      sending and receiving messages trhough UDP works, using the program "Packet Sender" on pc. UDP defines steer and speed commands.
      uart2 is the cable closest to the buzzer
      commented all Serial.print since there is only one serial port on the board
    19 aug 2018:
      added proper taskscheduler for different taskrates
      uncommented spiffs
    23 aug 2018
      added software serial for uart3 (debugging uart) since wemos d1 only has one hardware serial port
    1 sep 2018
      restructured functions to read out two MPU6050's + acceleration based pitch/roll calculation + general code cleanup
*/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <TaskScheduler.h>
#include <Wificonfig.h> //custom file to store SSID and password
#include <SoftwareSerial.h>
#include <math.h>


SoftwareSerial swSer(D6, D7, false, 127); //D6 Rx, D7 Tx

const char* ssid = WIFI_SSID ;
const char* password = WIFI_PASSWD ;

WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char incomingPacket[255];
char replyPacket[] = "Hi there! Got the message :-)";

//gets overwritten once UDP message arrive to the wemos
IPAddress ip_pc(192, 168, 0, 205);
uint16_t port_pc = 40911;


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

// struct to read out properties MPU6050 sensor
struct MPU6050_config
{
  float GyroScaleFactor;
  float AccelScaleFactor;
};
struct MPU6050_config mpu6050_1;
struct MPU6050_config mpu6050_2;

struct MPU6050data
{
  double Ax;
  double Ay;
  double Az;
  double T;
  double Gx;
  double Gy;
  double Gz;
};
struct MPU6050data MPU_offsets = {0, 0, 0, 0, 0, 0, 0};
struct MPU6050data MPU_offsets2 = {0, 0, 0, 0, 0, 0, 0};

struct Kinematic
{
  float pitch = 0;
  float roll = 0;
  float yaw = 0;
  float pitchAcc = 0;
  float rollAcc = 0;
};
struct Kinematic Sens1;
struct Kinematic Sens2;
float rad2deg = 180 / 3.14159265359;


bool ledon = false;

int16_t speed_increments = 100;
int16_t steer_increments = 50;

//uart2, uart2 is the long cable closest to the buzzer
int16_t speed;
int16_t steer;
int16_t speedchange;
uint8_t cntr_uart2;
uint8_t checksum_uart2;
uint32_t timeout_uart2 = 0;

//uart3, short cable closes to the red led
int16_t volt_bat;
int16_t speed_left;
int16_t speed_right;
int16_t checksum;
int16_t checksum_calc;
uint8_t cntr_uart3; uint8_t cntr_uart3_prev; uint8_t debnce_uart3_ticks;

// different task rates
int taskrate1 = 200;
int taskrate2 = 10;
int taskrate3 = 1000;
int taskrate4 = 20;

Task t1(taskrate2, TASK_FOREVER, &MPU6050_68);
Task t2(taskrate2, TASK_FOREVER, &MPU6050_69);
Task t3(taskrate4, TASK_FOREVER, &SetpointsHoverboard);
Task t5(taskrate3, TASK_FOREVER, &broadcastStatus);
Task t4(taskrate1, TASK_FOREVER, &readDebugUart);
Task t6(taskrate3, TASK_FOREVER, &blinkLed);

Scheduler runner;


void MPU6050_69()
{
  digitalWrite(D5, HIGH);       // set AD0 on MPU high to get other I2C address for MPU6050
  uint8_t MPU6050SlaveAddress = 0x69;
  Sens2 = getKinematics(MPU6050SlaveAddress, MPU_offsets2,mpu6050_2, Sens2);
}

void MPU6050_68()
{
  uint8_t MPU6050SlaveAddress = 0x68;
  Sens1 = getKinematics(MPU6050SlaveAddress, MPU_offsets,mpu6050_1, Sens1);
}

struct Kinematic getKinematics(uint8_t MPU6050SlaveAddress, MPU6050data MPU_offsets, MPU6050_config mpu6050_1, Kinematic kinematics)
{

  char replyPacket[100];
  struct MPU6050data Sensorvals;
  Sensorvals = Read_PhysicalValues(MPU6050SlaveAddress, mpu6050_1.GyroScaleFactor, mpu6050_1.AccelScaleFactor, MPU_offsets);

  //process data
  float timeStep = 0.001 * (float)taskrate2;
  kinematics.roll = kinematics.roll + Sensorvals.Gx * timeStep;
  kinematics.pitch = kinematics.pitch + Sensorvals.Gy * timeStep;
  kinematics.yaw = kinematics.yaw + Sensorvals.Gz * timeStep;

  float gravity = Sensorvals.Ax * Sensorvals.Ax + Sensorvals.Ay * Sensorvals.Ay + Sensorvals.Az * Sensorvals.Az;
  kinematics.rollAcc = atan2f(Sensorvals.Ay, Sensorvals.Az) * rad2deg;
  kinematics.pitchAcc = atan2f(-Sensorvals.Ax, sqrt(Sensorvals.Ay * Sensorvals.Ay + Sensorvals.Az * Sensorvals.Az)) * rad2deg;

  //      pitch = pitch * 0.98 + pitchAcc * 0.02;
  //      roll = roll * 0.98 * rollAcc * 0.02;

  return kinematics;
  yield();

}



void readDebugUart()
{
  int16_t debugdata[12];
  bool read_serial = true;
  char inChar = -1;
  char replyPacket3[255];
  String message = "";

  //  sprintf(replyPacket3, "%s", "uart3 reading started");
  //  Udp.beginPacket(ip_pc, port_pc);
  //  Udp.write(replyPacket3);
  //  Udp.endPacket();

  bool start_new_message = false;

  while (swSer.available() > 0 && read_serial) // Don't read unless there you know there is data
  {
    //search for start of new string
    while (start_new_message == false && swSer.available() > 0)
    {
      inChar = swSer.read();
      if (inChar == '\n')
      {
        start_new_message = true;
        //        Serial.println("new message:");
      }
    }


    int16_t index = 0;
    while (start_new_message && index < 12 && swSer.available() > 0)
    {

      inChar = swSer.peek();
      if (inChar == '\n')
      {

        start_new_message = false;
        //        Serial.println("message ended");
        //        for (int i = 0; i < 12; i++)
        //        {
        //          Serial.print(i, DEC);
        //          Serial.print(":");
        //          Serial.print(debugdata[i], DEC);
        //          Serial.print(":");
        //          Serial.println(debugdata[i], BIN);
        //        }

        cntr_uart3 = debugdata[0] ;
        int16_t speed_left_temp = debugdata[1] | (debugdata[2] << 8);
        int16_t speed_right_temp = debugdata[3] | (debugdata[4] << 8);
        int16_t volt_bat_temp = debugdata[5] | (debugdata[6] << 8);
        checksum = debugdata[7] | (debugdata[8] << 8);
        checksum_calc = speed_left_temp + speed_right_temp + volt_bat_temp;



        //        delay(1);

        //        sprintf(replyPacket3, "%s%d", "cntr uart3:", cntr_uart3);
        //        Udp.beginPacket(ip_pc, port_pc);
        //        Udp.write(replyPacket3);
        //        Udp.endPacket();


        if (checksum == checksum_calc)
        {
          //          sprintf(replyPacket3, "%s", "uart3 valid message");
          //          Udp.beginPacket(ip_pc, port_pc);
          //          Udp.write(replyPacket3);
          //          Udp.endPacket();
          speed_left = speed_left_temp;
          speed_right = speed_right_temp;
          volt_bat = volt_bat_temp;

          //          sprintf(replyPacket3, "%s%d%s%d%s%d", "batt", volt_bat, " spd l:", speed_left, " spd r:", speed_right);
          //          Udp.beginPacket(ip_pc, port_pc);
          //          Udp.write(replyPacket3);
          //          Udp.endPacket();
        }
        else
        {
          //          Serial.println("garbage message");
          sprintf(replyPacket3, "%s", "uart3 garbage message");
          Udp.beginPacket(ip_pc, port_pc);
          Udp.write(replyPacket3);
          Udp.endPacket();
        }

        if (cntr_uart3 == cntr_uart3_prev)
        {
          sprintf(replyPacket3, "%s", "uart3 hangs");
          Udp.beginPacket(ip_pc, port_pc);
          Udp.write(replyPacket3);
          Udp.endPacket();
          debnce_uart3_ticks++;
        }
        else
        {
          debnce_uart3_ticks = 0;
        }
        cntr_uart3_prev = cntr_uart3;

      }
      else
      {
        debugdata[index] = swSer.read();
        index++;
      }
      yield();
    }


    //clear buffer to stop program from just reading serial data, data is sent quicker than arduino can handle

    while (swSer.available() > 0)
    {
      //      swSer.read();
      byte number = swSer.read();
      yield();
    }
    read_serial = false;
  }

  //  sprintf(replyPacket3, "%s", "uart3 reading ended");
  //  Udp.beginPacket(ip_pc, port_pc);
  //  Udp.write(replyPacket3);
  //  Udp.endPacket();

}

void SetpointsHoverboard()
{
  //  uint32_t starttimer = millis();
  //  uint32_t endtimer;

  // interpret incoming udp packages
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets
    //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {

      strlwr(incomingPacket);
      if (strstr(incomingPacket, "right") || strstr(incomingPacket, "r"))
      {
        //Serial.print("Received message Right");
        steer += steer_increments;
        timeout_uart2 = 0;
      }

      else if (strstr(incomingPacket, "fast") || strstr(incomingPacket, "faster") || (strstr(incomingPacket, "f") && !strstr(incomingPacket, "l")))
      {
        //Serial.print("Received message faster");
        speed += speed_increments;
        timeout_uart2 = 0;
      }

      else if (strstr(incomingPacket, "slow") || strstr(incomingPacket, "slower") || strstr(incomingPacket, "sl"))
      {
        //Serial.print("Received message slower");
        speed -= speed_increments;
        timeout_uart2 = 0;
      }
      else if (strstr(incomingPacket, "left") || strstr(incomingPacket, "l"))
      {
        //Serial.print("Received message Left");
        //        sprintf(replyPacket, "%s", "left message received");
        //        Udp.beginPacket(ip_pc, port_pc);
        //        Udp.write(replyPacket);
        //        Udp.endPacket();

        steer -= steer_increments;
        timeout_uart2 = 0;
      }
      else if (strstr(incomingPacket, "stop") || strstr(incomingPacket, "s") || strstr(incomingPacket, "x"))
      {
        //Serial.print("Received message stop");
        steer = 0;
        speed = 0;
        timeout_uart2 = 0;
      }

      ip_pc = Udp.remoteIP();
      port_pc = Udp.remotePort();


    }
    //      Serial.printf("UDP packet contents: %s\n", incomingPacket);
    //      char replyPacket[255];
    //      sprintf(replyPacket, "%s%d%s%d", "steer:", steer, " speed:", speed);

    //      Udp.beginPacket(ip_pc, port_pc);

    //      Udp.write(replyPacket);
    //      Udp.endPacket();
    delay(0);

  }
  else
  {
    timeout_uart2++;
    uint32_t timeout_cal = 100; //s
    timeout_cal /= taskrate2;
    timeout_cal = timeout_cal * 1000;

    //      Serial.print("timeout remote");
    //      Serial.println(timeout_uart2, DEC);
    //      Serial.println(timeout_cal,DEC);

    if (timeout_uart2 > timeout_cal)
    {
      //Serial.println("no input received for too long");
      steer = 0;
      speed = 0;
    }
  }

  //    Serial.print("speed ");
  //    Serial.println(speed);
  //
  //    Serial.print("steer ");
  //    Serial.println(steer);


  // uart2 commands + checks
  Serial.write((uint8_t *) &steer, sizeof(steer));
  Serial.write((uint8_t *) &speed, sizeof(speed));
  Serial.write((uint8_t *) &cntr_uart2, sizeof(cntr_uart2));
  checksum_uart2 = steer + speed + cntr_uart2;
  Serial.write((uint8_t *) &checksum_uart2, sizeof(checksum_uart2));


  //  Serial.print("counter uart2:");
  //  Serial.println(cntr_uart2, DEC);
  cntr_uart2++;

  //check chronometrics
  //  endtimer = millis();
  //  char replyPacket3[255];
  //  sprintf(replyPacket3, "%s%d\t%d", "uart2:", (uint8_t *) &steer, sizeof(steer));
  //  Udp.beginPacket(ip_pc, port_pc);
  //  Udp.write(replyPacket3);
  //  Udp.endPacket();


}

void broadcastStatus()
{
  char replyPacket3[255];

  //steer and speed signals
  sprintf(replyPacket3, "%s%d%s%d", "steer:", steer, " speed:", speed);
  Udp.beginPacket(ip_pc, port_pc);
  Udp.write(replyPacket3);
  Udp.endPacket();
  yield();

  sprintf(replyPacket, "%s%.3f%s%.3f%s%.3f", "pitch1:", Sens1.pitch, " roll:", Sens1.roll, " yaw:", Sens1.yaw);
  Udp.beginPacket(ip_pc, port_pc);
  Udp.write(replyPacket);
  Udp.endPacket();
  yield();

  sprintf(replyPacket, "%s%.3f%s%.3f%s%.3f", "pitch2:", Sens2.pitch, " roll:", Sens2.roll, " yaw:", Sens2.yaw);
  Udp.beginPacket(ip_pc, port_pc);
  Udp.write(replyPacket);
  Udp.endPacket();
  yield();

  sprintf(replyPacket, "%s%d%s%d%s%d", "spd_lft:", speed_left, " spd_rght:", speed_right, " volt_bat:", volt_bat);
  Udp.beginPacket(ip_pc, port_pc);
  Udp.write(replyPacket);
  Udp.endPacket();
  delay(0);

}

void blinkLed()
{

  //to show program is running
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

  delay(0);

}



void setup() {

  Serial.begin(19200);
  swSer.begin(9600);
  pinMode(D5, OUTPUT);           // set pin to input


  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    ////Serial.println("Connection Failed! Rebooting...");
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
    //Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    //Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      //Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      //Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      //Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      //Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      //Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  //Serial.println("Ready");
  //Serial.print("IP address: ");
  //Serial.println(WiFi.localIP());


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // put your setup code here, to run once:
  Wire.begin();
  delay(100);


  uint8_t MPU6050SlaveAddress = 0x68;
  MPU6050_Init(MPU6050SlaveAddress);
  uint8_t scale_gyro = 3;
  uint8_t scale_accel = 0;
  setScale(MPU6050SlaveAddress, scale_gyro, scale_accel);
  mpu6050_1 = getMPU6050scales(MPU6050SlaveAddress);

  //Serial.println("calibration sensor, don't move!");
  delay(100);
  //need to make something more fancy
  MPU_offsets = Read_PhysicalValues(MPU6050SlaveAddress, mpu6050_1.GyroScaleFactor, mpu6050_1.AccelScaleFactor, MPU_offsets);
  MPU_offsets.Ax = 0;
  MPU_offsets.Ay = 0;
  MPU_offsets.Az = 0;
  //  Serial.println("MPU_offsets");
  //  Serial.println(MPU_offsets.Gx);
  //  Serial.println(MPU_offsets.Gy);
  //  Serial.println(MPU_offsets.Gz);
  //  Serial.println(MPU_offsets.T);


  MPU6050SlaveAddress = 0x69;
  MPU6050_Init(MPU6050SlaveAddress);
  setScale(MPU6050SlaveAddress, scale_gyro, scale_accel);
  mpu6050_2 = getMPU6050scales(MPU6050SlaveAddress);

  //Serial.println("calibration sensor, don't move!");
  delay(100);
  //need to make something more fancy
  MPU_offsets2 = Read_PhysicalValues(MPU6050SlaveAddress, mpu6050_2.GyroScaleFactor, mpu6050_2.AccelScaleFactor, MPU_offsets2);
  MPU_offsets2.Ax = 0;
  MPU_offsets2.Ay = 0;
  MPU_offsets2.Az = 0;

  //  Serial.println("MPU_offsets2");
  //  Serial.println(MPU_offsets2.Gx);
  //  Serial.println(MPU_offsets2.Gy);
  //  Serial.println(MPU_offsets2.Gz);
  //  Serial.println(MPU_offsets2.T);

  steer = 0;
  speed = 0;

  Udp.begin(localUdpPort);
  // wait till first udp message arrives to start
  bool Nomessagereceived = true;
  while (Nomessagereceived)
  {
    int packetsize = Udp.parsePacket();
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      ip_pc = Udp.remoteIP();
      port_pc = Udp.remotePort();
      char replyPacket3[200];
      sprintf(replyPacket3, "%s", "sketch started");
      Udp.beginPacket(ip_pc, port_pc);
      Udp.write(replyPacket3);
      Udp.endPacket();
      Nomessagereceived = false;
    }
    else
    {
      delay(200);
    }
  }

  while (swSer.available() > 0) {
    swSer.read();
    yield();
  }
  //    Serial.println("uart3 buffer cleared");
  char replyPacket3[50];
  sprintf(replyPacket3, "%s", "uart3 buffer cleared");
  Udp.beginPacket(ip_pc, port_pc);
  Udp.write(replyPacket3);
  Udp.endPacket();


  runner.init();
  runner.addTask(t1);
  runner.addTask(t2);
  runner.addTask(t3);
  runner.addTask(t4);
  runner.addTask(t5);
  runner.addTask(t6);

  t1.enable();
  t2.enable();
  t3.enable();
  t4.enable();
  t5.enable();
  t6.enable();


  yield();//not sure if this has any use
}


void loop() {
  ArduinoOTA.handle();
  runner.execute();
  delay(0);
}






// all MPU6050 helper functions

struct MPU6050data Read_PhysicalValues(uint8_t MPU6050SlaveAddress, float GyroScaleFactor, float AccelScaleFactor, MPU6050data offsets)
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

  delay(0);
  //divide each with their sensitivity scale factor and correct for offsets
  w.Ax = kin.Ax / AccelScaleFactor - offsets.Ax;
  w.Ay = kin.Ay / AccelScaleFactor - offsets.Ay;
  w.Az = kin.Az / AccelScaleFactor - offsets.Az;
  w.T = kin.T / 340 + 36.53; //temperature formula
  w.Gx = kin.Gx / GyroScaleFactor - offsets.Gx;
  w.Gy = kin.Gy / GyroScaleFactor - offsets.Gy;
  w.Gz = kin.Gz / GyroScaleFactor - offsets.Gz;

  //  Serial.println("raw values");
  //  Serial.print("Ax: "); Serial.print(kin.Ax);
  //  Serial.print(" Ay: "); Serial.print(kin.Ay);
  //  Serial.print(" Az: "); Serial.print(kin.Az);
  //  Serial.print(" T: "); Serial.print(kin.T);
  //  Serial.print(" Gx: "); Serial.print(kin.Gx);
  //  Serial.print(" Gy: "); Serial.print(kin.Gy);
  //  Serial.print(" Gz: "); Serial.println(kin.Gz);

  //  Serial.println("converted values");
  //  Serial.print("Ax: "); Serial.print(w.Ax);
  //  Serial.print(" Ay: "); Serial.print(w.Ay);
  //  Serial.print(" Az: "); Serial.print(w.Az);
  //  Serial.print(" T: "); Serial.print(w.T);
  //  Serial.print(" Gx: "); Serial.print(w.Gx);
  //  Serial.print(" Gy: "); Serial.print(w.Gy);
  //  Serial.print(" Gz: "); Serial.print(w.Gz);

  return w;

}

void MPU6050_Init(uint8_t MPU6050SlaveAddress) {
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


void setScale(uint8_t MPU6050SlaveAddress, uint8_t scale_gyro, uint8_t scale_acc)
// set scaling factors for gyro and accelerometer reading
{

  uint8_t value;
  if (scale_gyro == 0)
  {
    value = 0x00;
    //    Serial.println("scale set to 250 deg/s");
  }
  else if (scale_gyro == 3) {
    value = 0x18;
    //    Serial.println("scale set to 2000 deg/s");
  }
  else
  {
    value = 0x00;
    //    Serial.println("scale set to 250 deg/s");
  }
  Wire.beginTransmission(MPU6050SlaveAddress);
  Wire.write(MPU6050_REGISTER_GYRO_CONFIG);
  Wire.write(value);
  Wire.endTransmission();
  delay(200);


  if (scale_acc == 0)
  {
    value = 0x00;
    //    Serial.println("scale set to 2g");
  }
  else if (scale_acc == 3) {
    value = 0x18;
    //    Serial.println("scale set to 16g");
  }
  else
  {
    value = 0x00;
    //    Serial.println("scale set to 2 deg/s");
  }
  Wire.beginTransmission(MPU6050SlaveAddress);
  Wire.write(MPU6050_REGISTER_ACCEL_CONFIG);
  Wire.write(value);
  Wire.endTransmission();
  delay(100);

}


struct MPU6050_config getMPU6050scales(uint8_t MPU6050SlaveAddress)
{
  Wire.beginTransmission(MPU6050SlaveAddress);
  //  Wire.write(0x1B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.write(MPU6050_REGISTER_GYRO_CONFIG);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050SlaveAddress, 2, true); // request a total of 14 registers
  uint8_t Gyro = (Wire.read() & (bit(3) | bit(4))) >> 3;
  uint8_t Accl = (Wire.read() & (bit(3) | bit(4))) >> 3;

  float GyroScaleFactor = 1; //stupid way to avoid ever having zero's
  float AccelScaleFactor = 1; //stupid way to avoid ever having zero's


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

  //  Serial.println("scaling factor");
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

