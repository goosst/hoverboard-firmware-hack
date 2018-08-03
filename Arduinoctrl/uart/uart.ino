/* initial date: 2 august 2018
   author: goosst
   sketch for arduino due (3.3V based uart etc.)

   uart2 from hoverboard is connected to serial 1 from the due
   uart3 from hoverboard is connected to serial 2
   2 aug 18: added distance measurement with VL53L0X 
   3 aug 18: 
    * added GS 521 gyroscope reading
    * added IR remote using LG remote: MKJ40653802 and VS1838b IR receiver
*/

#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050_tockn.h>
#include <IRremote2.h>

VL53L0X sensor;
uint16_t distance1_mm;

MPU6050 mpu6050(Wire1);

//uart2
int16_t speed;
int16_t steer;
int16_t speedchange;
uint8_t cntr_uart2;
uint8_t checksum_uart2;

// uart3 information:
int16_t volt_bat;
int16_t speed_left;
int16_t speed_right;
int16_t checksum;
int16_t checksum_calc;
uint8_t cntr_uart3; uint8_t cntr_uart3_prev; uint8_t debnce_uart3_ticks;
char inChar = -1;
int16_t debugdata[12];



int RECV_PIN = 11;//IR receiver
IRrecv irrecv(RECV_PIN);
decode_results results;


void setup() {
  // put your setup code here, to run once:
  Serial1.begin(19200);
  Serial.begin(19200);
  Serial2.begin(19200);

  speedchange = 0;

  //clear serial buffer
  while (Serial2.available() > 0) {
    Serial2.read();
  }
  Serial.print("buffer cleared");

  //vl53 settings
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
  //  sensor.setSignalRateLimit(0.25);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  sensor.setMeasurementTimingBudget(20000); //measuring time us

  Wire1.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  irrecv.enableIRIn(); // Start the receiver
}

void loop() {

  // some profile commanding speed and steer
  if (speedchange < 300) {
    speed = 150;
  }
  else {
    speed = 700;
  }

  speedchange++;
  if (speedchange > 800) {
    speedchange = 0;
  }
  //Serial.println(speedchange);
  steer = 0;



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
  Serial.print("--------------");
  Serial.println(Serial2.available());

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
    Serial.println("buffer cleared");
    read_serial = false;
    //    }

  }



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

  if (irrecv.decode(&results)) {
    Serial.println(results.value, DEC);

    //values using LG remote: MKJ40653802
    if (results.value == 551502015)
    {
      Serial.println("faster");
    }
    else if (results.value == 551534655)
    {
      Serial.println("slower");
    }
    else if(results.value == 551485695)
    {
      Serial.println("turn right");
    }
    else if(results.value == 551518335)
    {
      Serial.println("turn left");
    }


    irrecv.resume(); // Receive the next value
  }
  

  delay(10);
}





