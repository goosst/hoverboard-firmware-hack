char inData[128]; // Allocate some space for the string
char inChar = -1; // Where to store the character read
int16_t debugdata[12];

int16_t speed;
int16_t speedchange;

// uart 3 information:
int16_t volt_bat;
int16_t speed_left;
int16_t speed_right;
int16_t checksum;
int16_t checksum_calc;
uint8_t cntr_uart3; uint8_t cntr_uart3_prev; uint8_t debnce_uart3_ticks;

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

}

void loop() {



  // put your main code here, to run repeatedly:
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


  int16_t steer = 0;

  Serial1.write((uint8_t *) &steer, sizeof(steer));
  Serial1.write((uint8_t *) &speed, sizeof(speed));

  //  Serial.println(Serial2.available());

  Serial.print("--------------");
  Serial.println(Serial2.available());

  bool read_serial = true;
  while (Serial2.available() > 0 && read_serial) // Don't read unless there you know there is data
  {
    //search for start of new string

    bool start_new_message = false;
    while (start_new_message == false)
    {
      inChar = Serial2.read();
      if (inChar == '\n')
      {
        start_new_message = true;
        //        Serial.println("new message:");
      }
    }

    int16_t index = 0;
    while (start_new_message && index < 12)
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

    //clear buffer to stop program from just reading serial data
    //    if (index >= 12)
    //    {
    while (Serial2.available() > 0) {
      Serial2.read();
    }
    Serial.println("buffer cleared");
    read_serial = false;
    //    }

  }
  delay(2);
}





