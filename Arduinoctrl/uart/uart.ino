/* initial date: 2 august 2018
   author: goosst
   sketch for arduino due (3.3V based uart etc.)

   uart2 from hoverboard is connected to serial 1 from the due
   uart3 from hoverboard is connected to serial 2
   2 aug 18: added distance measurement with VL53L0X
   3 aug 18:
      added GS 521 gyroscope reading
      added IR remote using LG remote: MKJ40653802 and VS1838b IR receiver, using this library for the arduino due https://github.com/enternoescape/Arduino-IRremote-Due/blob/master/IRremote2.h
   4 aug 18: added speed and steer corrections based on IR input, removed usage of delay and replaced by millis(); mpu6050.calcGyroOffsets(true) should be fixed due to annoying timeout
*/

#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050_tockn.h>
#include <IRremote2.h>

int ledPin = 13;
bool ledon = false;

VL53L0X sensor;
uint16_t distance1_mm;

MPU6050 mpu6050(Wire);

//uart2 on serial 1, uart2 is the long cable on closest to the buzzer
int16_t speed;
int16_t steer;
int16_t speedchange;
uint8_t cntr_uart2;
uint8_t checksum_uart2;
uint32_t timeout_uart2 = 0;

// uart3 information on serial 2, uart3 is short cable closes to the red led
int16_t volt_bat;
int16_t speed_left;
int16_t speed_right;
int16_t checksum;
int16_t checksum_calc;
uint8_t cntr_uart3; uint8_t cntr_uart3_prev; uint8_t debnce_uart3_ticks;
char inChar = -1;
int16_t debugdata[12];



int RECV_PIN = 11;//IR receiver, seems to work with 3.3 and 5 V
IRrecv irrecv(RECV_PIN);
decode_results results;

unsigned long time_scheduler1 = 0;
int taskrate1 = 2;

unsigned long time_scheduler2 = 0;
int taskrate2 = 20;

unsigned long time_scheduler3 = 0;
int taskrate3 = 500;

uint8_t startup;

void setup() {
  pinMode(ledPin, OUTPUT);

  // put your setup code here, to run once:
  Serial1.begin(19200); //uart2
  Serial2.begin(19200); //uart3
  Serial.begin(19200); //computer debugging


  //  Serial.print("start program");
  //  Serial.println(millis());
  speedchange = 0;

  //vl53 settings
  Wire1.begin();
  //  Serial.print("step 1:");
  //  Serial.println(millis());

  sensor.init();
  sensor.setTimeout(500);
  //  sensor.setSignalRateLimit(0.25);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor.setMeasurementTimingBudget(20000); //measuring time us
  //  Serial.print("step 2:");
  //  Serial.println(millis());

  //mpu6050, gy-521
  Wire.begin();
  mpu6050.begin();
  //  Serial.print("step 3:");
  //  Serial.println(millis());

  mpu6050.calcGyroOffsets(true); //this is a bloody annoying function if a sensor is not connected, it only times out after ages

  irrecv.enableIRIn(); // Start the receiver

  steer = 0;
  speed = 0;

  //  Serial.print("step 4:");
  //  Serial.println(millis());

  //clear serial buffer uart3
  while (Serial2.available() > 0) {
    Serial2.read();
  }
  //  Serial.print("buffer cleared");

  //  Serial.print("step 5:");
  //  Serial.println(millis());
}

void loop() {

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


  // some test profile commanding speed and steer
  //  if (speedchange < 300) {
  //    speed = 150;
  //  }
  //  else {
  //    speed = 700;
  //  }

  //  speedchange++;
  //  if (speedchange > 800) {
  //    speedchange = 0;
  //  }
  //  //Serial.println(speedchange);
  //  steer = 0;



  if (millis() >= time_scheduler2 + taskrate2)
  {
    time_scheduler2 = millis();
    // uart2 commands + checks
    Serial1.write((uint8_t *) &steer, sizeof(steer));
    Serial1.write((uint8_t *) &speed, sizeof(speed));
    Serial1.write((uint8_t *) &cntr_uart2, sizeof(cntr_uart2));
    checksum_uart2 = steer + speed + cntr_uart2;
    Serial1.write((uint8_t *) &checksum_uart2, sizeof(checksum_uart2));
    //  Serial.print("counter uart2:");
    //  Serial.println(cntr_uart2, DEC);
    cntr_uart2++;



    // uart3 reading and conversion
    //    Serial.println("--------------");
    //  Serial.println(Serial2.available());

    bool read_serial = true;
    while (Serial2.available() > 0 && read_serial) // Don't read unless there you know there is data
    {
      //search for start of new string

      bool start_new_message = false;
      while (start_new_message == false && Serial2.available() > 0)
      {
        inChar = Serial2.read();
        if (inChar == '\n')
        {
          start_new_message = true;
          //        Serial.println("new message:");
        }
      }

      int16_t index = 0;
      while (start_new_message && index < 12 && Serial2.available() > 0)
      {

        inChar = Serial2.peek();
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
          speed_left = debugdata[1] | (debugdata[2] << 8);
          speed_right = debugdata[3] | (debugdata[4] << 8);
          volt_bat = debugdata[5] | (debugdata[6] << 8);
          checksum = debugdata[7] | (debugdata[8] << 8);
          checksum_calc = speed_left + speed_right + volt_bat;

          Serial.print("batt volt:");
          Serial.println(volt_bat, DEC);

          Serial.print("speed_left ");
          Serial.println(speed_left, DEC);

          Serial.print("speed_right ");
          Serial.println(speed_right, DEC);

          //        Serial.print("checksum ");
          //        Serial.println(checksum, DEC);
          //        Serial.println(checksum_calc, DEC);

          //        Serial.print("counter ");
          //        Serial.println(cntr_uart3, DEC);

          if (checksum == checksum_calc)
          {
            Serial.println("valid message");
          }
          else
          {
            Serial.println("garbage message");
          }

          if (cntr_uart3 == cntr_uart3_prev)
          {
            Serial.println("uart communication is hanging");
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
          debugdata[index] = Serial2.read();
          index++;
        }
      }

      //clear buffer to stop program from just reading serial data, data is sent quicker than arduino can handle
      //    if (index >= 12)
      //    {
      while (Serial2.available() > 0) {
        Serial2.read();
      }
      Serial.println("uart3 buffer cleared");
      read_serial = false;
      //    }

    }

    if (false)
    {

      // distance measurements
      distance1_mm = sensor.readRangeSingleMillimeters();
      Serial.print("distance: ");
      Serial.println(distance1_mm, DEC);
      if (sensor.timeoutOccurred()) {
        Serial.println(" TIMEOUT");
      }


      //gyroscope
      Serial.print("gyroAngleX : "); Serial.print(mpu6050.getGyroAngleX());
      Serial.print("\tgyroAngleY : "); Serial.print(mpu6050.getGyroAngleY());
      Serial.print("\tgyroAngleZ : "); Serial.println(mpu6050.getGyroAngleZ());

    }

  }


  //blink led to indicate program is running
  if (millis() >= time_scheduler3 + taskrate3)
  {
    time_scheduler3 = millis();

    if (ledon == false)
    {
      digitalWrite(ledPin, HIGH);
      ledon = true;
      //      Serial.println("pin high");
    }
    else
    {
      digitalWrite(ledPin, LOW);
      ledon = false;
      //      Serial.println("pin low");
    }
  }

}





