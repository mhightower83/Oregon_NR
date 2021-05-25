#include <Oregon_NR.h>

#if defined ( ESP8266 ) || ( ESP32 )// For Wemos
  // Oregon_NR oregon(13, 13,          // receiver at pin D7 (GPIO13)
  //                       2, true,    // LED on D2 pulled up to + power (true). If the LED is not needed, then the pin number is 255
  //                       50, true);  // Buffer for receiving a parcel of 50 nibls, package assembly is enabled for v2
  Oregon_NR oregon (4, 4,           // receiver at pin D2 (GPIO04)
                    2, true,        // LED on D4 pulled up to + power (true). If the LED is not needed, then the pin number is 255
                    50, true);      // Buffer for receiving a parcel of 50 nibls, package assembly is enabled for v2

#else                               // For AVR
  Oregon_NR oregon(2, 0,            // Receiver on pin D2, Interrupt 0,
                     13, false);    // Receive LED on pin 13
                                    // By default, the Buffer for receiving a parcel is standard - for 24 nibbles, package assembly is enabled for v2

// Oregon_NR oregon(2, 0);             // Default constructor, with standard buffer and no LED
#endif

void setup() {
   Serial.begin(115200);
   Serial.println();
   if (oregon.no_memory)
   {
    Serial.println("NO MEMORY");   // Not enough memory
    do yield();
    while(true);
   }

  // enable listening to the radio channel
  oregon.start();
  oregon.receiver_dump = 0;       // true - Turns on the "oscilloscope" - displaying the data received from the receiver
}

void loop() {
  (void)RECEIVER_PIN;

  //////////////////////////////////////////////////////////////////////
  // Capture the package, //////////////////////////////////////////////
  oregon.capture(0); // 1 - output service information to Serial

  // The captured data is good until the next capture call
  // Processing the resulting package //////////////////////////////////
  if (oregon.captured)  {
    // Output information to Serial
    Serial.print ((float) millis() / 1000, 1); // Time
    Serial.print ("s\t\t");
    // Protocol version
    if (oregon.ver == 2) Serial.print("  ");
    if (oregon.ver == 3) Serial.print("3 ");

    // Information about package recovery
    if (oregon.restore_sign & 0x01) Serial.print("s");  // single ticks restored
    else  Serial.print(" ");
    if (oregon.restore_sign & 0x02) Serial.print("d");  // double ticks restored
    else  Serial.print(" ");
    if (oregon.restore_sign & 0x04) Serial.print("p "); // fixed error when recognizing the package version
    else  Serial.print("  ");
    if (oregon.restore_sign & 0x08) Serial.print("r "); // compiled from two packages (for build mode in v.2)
    else  Serial.print("  ");

    // Output the received packet.
    for (int q = 0;q < oregon.packet_length; q++)
      if (oregon.valid_p[q] == 0x0F) Serial.print(oregon.packet[q], HEX);
      else Serial.print(" ");

    // Package processing time
    Serial.print("  ");
    Serial.print(oregon.work_time);
    Serial.print("ms ");

    if ((oregon.sens_type == THGN132 ||
    (oregon.sens_type & 0x0FFF) == RTGN318 ||
    (oregon.sens_type & 0x0FFF) == RTHN318 ||
    oregon.sens_type == THGR810 ||
    oregon.sens_type == THN132 ||
    oregon.sens_type == THN800 ||
    oregon.sens_type == BTHGN129 ||
    oregon.sens_type == BTHR968 ||
    oregon.sens_type == THGN500) && oregon.crc_c){
      Serial.print("\t");



      if (oregon.sens_type == THGN132) Serial.print("THGN132N");
      if (oregon.sens_type == THGN500) Serial.print("THGN500 ");
      if (oregon.sens_type == THGR810) Serial.print("THGR810 ");
      if ((oregon.sens_type & 0x0FFF) == RTGN318) Serial.print("RTGN318 ");
      if ((oregon.sens_type & 0x0FFF) == RTHN318) Serial.print("RTHN318 ");
      if (oregon.sens_type == THN132 ) Serial.print("THN132N ");
      if (oregon.sens_type == THN800 ) Serial.print("THN800  ");
      if (oregon.sens_type == BTHGN129 ) Serial.print("BTHGN129");
      if (oregon.sens_type == BTHR968 ) Serial.print("BTHR968 ");

      if (oregon.sens_type != BTHR968 && oregon.sens_type != THGN500)
      {
        Serial.print(" CHNL: ");
        Serial.print(oregon.sens_chnl);
      }
      else Serial.print("        ");
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print("ID: ");
      Serial.print(oregon.sens_id, HEX);

      if (oregon.sens_tmp >= 0 && oregon.sens_tmp < 10) Serial.print(" TMP:  ");
      if (oregon.sens_tmp < 0 && oregon.sens_tmp >-10) Serial.print(" TMP: ");
      if (oregon.sens_tmp <= -10) Serial.print(" TMP:");
      if (oregon.sens_tmp >= 10) Serial.print(" TMP: ");
      Serial.print((oregon.sens_tmp * 1.8 + 32), 1);
      Serial.print("F ");
      if (oregon.sens_type == THGN132 ||
          oregon.sens_type == THGR810 ||
          oregon.sens_type == BTHGN129 ||
          oregon.sens_type == BTHR968 ||
          (oregon.sens_type & 0x0FFF) == RTGN318 ||
          oregon.sens_type == THGN500 ) {
        Serial.print("HUM: ");
        Serial.print(oregon.sens_hmdty, 0);
        Serial.print("%");
      }
      else Serial.print("        ");

      if (oregon.sens_type == BTHGN129 ||  oregon.sens_type == BTHR968)
      {
      Serial.print(" PRESS: ");
      Serial.print(oregon.get_pressure(), 1);
      Serial.print("Hgmm ");
      }
    }

  if (oregon.sens_type == WGR800 && oregon.crc_c){
      Serial.print("\tWGR800  ");
      Serial.print("        ");
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print("ID: ");
      Serial.print(oregon.sens_id, HEX);

      Serial.print(" AVG: ");
      Serial.print(oregon.sens_avg_ws, 1);
      Serial.print("m/s  MAX: ");
      Serial.print(oregon.sens_max_ws, 1);
      Serial.print("m/s  DIR: "); //N = 0, E = 4, S = 8, W = 12
      switch (oregon.sens_wdir)
      {
      case 0: Serial.print("N"); break;
      case 1: Serial.print("NNE"); break;
      case 2: Serial.print("NE"); break;
      case 3: Serial.print("NEE"); break;
      case 4: Serial.print("E"); break;
      case 5: Serial.print("SEEE"); break;
      case 6: Serial.print("SE"); break;
      case 7: Serial.print("SSE"); break;
      case 8: Serial.print("S"); break;
      case 9: Serial.print("SSW"); break;
      case 10: Serial.print("SW"); break;
      case 11: Serial.print("SWW"); break;
      case 12: Serial.print("W"); break;
      case 13: Serial.print("NWW"); break;
      case 14: Serial.print("NW"); break;
      case 15: Serial.print("NNW"); break;
      }

    }

    if (oregon.sens_type == UVN800 && oregon.crc_c){
      Serial.print("\tUVN800  ");
      Serial.print("        ");
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print("ID: ");
      Serial.print(oregon.sens_id, HEX);

      Serial.print(" UV IDX: ");
      Serial.print(oregon.UV_index);

    }


    if (oregon.sens_type == RFCLOCK && oregon.crc_c){
      Serial.print("\tRF CLOCK");
      Serial.print(" CHNL: ");
      Serial.print(oregon.sens_chnl);
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print("ID: ");
      Serial.print(oregon.sens_id, HEX);
      Serial.print(" TIME: ");
      Serial.print((oregon.packet[6] & 0x0F), HEX);
      Serial.print((oregon.packet[6] & 0xF0) >> 4, HEX);
      Serial.print(':');
      Serial.print((oregon.packet[5] & 0x0F), HEX);
      Serial.print((oregon.packet[5] & 0xF0) >> 4, HEX);
      Serial.print(':');
      Serial.print(':');
      Serial.print((oregon.packet[4] & 0x0F), HEX);
      Serial.print((oregon.packet[4] & 0xF0) >> 4, HEX);
      Serial.print(" DATE: ");
      Serial.print((oregon.packet[7] & 0x0F), HEX);
      Serial.print((oregon.packet[7] & 0xF0) >> 4, HEX);
      Serial.print('.');
      if ((oregon.packet[8] & 0x0F) == 1 || (oregon.packet[8] & 0x0F) == 3)   Serial.print('1');
      else Serial.print('0');
      Serial.print((oregon.packet[8] & 0xF0) >> 4, HEX);
      Serial.print('.');
      Serial.print((oregon.packet[9] & 0x0F), HEX);
      Serial.print((oregon.packet[9] & 0xF0) >> 4, HEX);

    }

    if (oregon.sens_type == PCR800 && oregon.crc_c){
      Serial.print("\tPCR800  ");
      Serial.print("        ");
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print(" ID: ");
      Serial.print(oregon.sens_id, HEX);
      Serial.print("   TOTAL: ");
      Serial.print(oregon.get_total_rain(), 1);
      Serial.print("mm  RATE: ");
      Serial.print(oregon.get_rain_rate(), 1);
      Serial.print("mm/h");

    }

#if ADD_SENS_SUPPORT == 1
      if ((oregon.sens_type & 0xFF00) == THP && oregon.crc_c) {
      Serial.print("\tTHP     ");
      Serial.print(" CHNL: ");
      Serial.print(oregon.sens_chnl);
      Serial.print(" BAT: ");
      Serial.print(oregon.sens_voltage, 2);
      Serial.print("V");
      if (oregon.sens_tmp > 0 && oregon.sens_tmp < 10) Serial.print(" TMP:  ");
      if (oregon.sens_tmp < 0 && oregon.sens_tmp > -10) Serial.print(" TMP: ");
      if (oregon.sens_tmp <= -10) Serial.print(" TMP:");
      if (oregon.sens_tmp >= 10) Serial.print(" TMP: ");
      Serial.print(oregon.sens_tmp, 1);
      Serial.print("C ");
      Serial.print("HUM: ");
      Serial.print(oregon.sens_hmdty, 1);
      Serial.print("% ");
      Serial.print("PRESS: ");
      Serial.print(oregon.sens_pressure, 1);
      Serial.print("Hgmm");

    }
#endif
    Serial.println();
  }

  // yield();
}
