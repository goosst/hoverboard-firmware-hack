/* hoverboard control from wemos d1 r2 (over wifi)
    12 aug 2018
      over the air update
      MPU6050 Triple Axis Gyroscope & Accelerometer. Simple Gyroscope Example.GIT: https://github.com/jarzebski/Arduino-MPU6050
    13 aug 2018
      added esp8266fptserver to readin/readout SPIFFS through FTP: https://github.com/nailbuster/esp8266FTPServer
      create data folder (ctrl+K), 
      filezille: ftp to IP address port 21, no encryption, 1 simultaneous connection

*/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESP8266FtpServer.h>

const char* ssid = "-";
const char* password = "-";


FtpServer ftpSrv;

MPU6050 mpu;

bool ledon = false;


unsigned long time_scheduler2 = 0;
int taskrate2 = 20;

unsigned long time_scheduler3 = 0;
int taskrate3 = 500;

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
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

  //mpu6050, gy-521
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  // If you want, you can set gyroscope offsets
  // mpu.setGyroOffsetX(155);
  // mpu.setGyroOffsetY(15);
  // mpu.setGyroOffsetZ(15);

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  // Check settings
  checkSettings();


  if (SPIFFS.begin())
  {
    Serial.println("SPIFFS opened!");
    ftpSrv.begin("wemosd1", "esp8266");   //username, password for ftp.  set ports in ESP8266FtpServer.h  (default 21, 50009 for PASV)
  }


}

void loop() {
  ArduinoOTA.handle();
  ftpSrv.handleFTP();

  if (millis() >= time_scheduler2 + taskrate2)
  {
    time_scheduler2 = millis();

    Vector rawGyro = mpu.readRawGyro();
    Vector normGyro = mpu.readNormalizeGyro();

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

      delay(3);
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



void checkSettings()
{
  Serial.println();

  Serial.print(" * Sleep Mode:        ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Clock Source:      ");
  switch (mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }

  Serial.print(" * Gyroscope:         ");
  switch (mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
  }

  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());

  Serial.println();
}
