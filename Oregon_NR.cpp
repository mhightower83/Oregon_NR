#include "Oregon_NR.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the Arduino OREGON_NR library.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// The MIT License (MIT)
//
// Copyright (c) 2021 Sergey Zawislak
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Этот файл - часть библиотеки OREGON_NR
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021 Сергей Зависляк
//
// Данная лицензия разрешает лицам, получившим копию данного программного обеспечения и сопутствующей документации
// (в дальнейшем именуемыми «Программное Обеспечение»), безвозмездно использовать Программное Обеспечение без ограничений,
// включая неограниченное право на использование, копирование, изменение, слияние, публикацию, распространение, сублицензирование
// и/или продажу копий Программного Обеспечения, а также лицам, которым предоставляется данное Программное Обеспечение, при соблюдении следующих условий:
//
// Указанное выше уведомление об авторском праве и данные условия должны быть включены во все копии или значимые части данного Программного Обеспечения.
//
// ДАННОЕ ПРОГРАММНОЕ ОБЕСПЕЧЕНИЕ ПРЕДОСТАВЛЯЕТСЯ «КАК ЕСТЬ», БЕЗ КАКИХ-ЛИБО ГАРАНТИЙ, ЯВНО ВЫРАЖЕННЫХ ИЛИ ПОДРАЗУМЕВАЕМЫХ, ВКЛЮЧАЯ ГАРАНТИИ ТОВАРНОЙ
// ПРИГОДНОСТИ, СООТВЕТСТВИЯ ПО ЕГО КОНКРЕТНОМУ НАЗНАЧЕНИЮ И ОТСУТСТВИЯ НАРУШЕНИЙ, НО НЕ ОГРАНИЧИВАЯСЬ ИМИ. НИ В КАКОМ СЛУЧАЕ АВТОРЫ ИЛИ ПРАВООБЛАДАТЕЛИ
// НЕ НЕСУТ ОТВЕТСТВЕННОСТИ ПО КАКИМ-ЛИБО ИСКАМ, ЗА УЩЕРБ ИЛИ ПО ИНЫМ ТРЕБОВАНИЯМ, В ТОМ ЧИСЛЕ, ПРИ ДЕЙСТВИИ КОНТРАКТА, ДЕЛИКТЕ ИЛИ ИНОЙ СИТУАЦИИ,
// ВОЗНИКШИМ ИЗ-ЗА ИСПОЛЬЗОВАНИЯ ПРОГРАММНОГО ОБЕСПЕЧЕНИЯ ИЛИ ИНЫХ ДЕЙСТВИЙ С ПРОГРАММНЫМ ОБЕСПЕЧЕНИЕМ.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
  yield() - will only allow the Espressif NONOS SDK to run.
  Arduion ESP8266 core does not get to run.

  delay(0) - both the Espressif NONOS SDK and the Arduino ESP8266 core get to
  run.
*/
// static __always_inline void YIELD() { delay(0); }
static __always_inline void YIELD() { yield(); }


// Everything related to interruption /////////////////////////////////////
#ifndef Oregon_NR_int
#define Oregon_NR_int
static volatile unsigned long pm;
static volatile unsigned long pl, timer_mark;

#if defined ( ESP8266 )
void IRAM_ATTR receiver_interruption(void) {
#elif defined ( ESP32 )
void IRAM_ATTR receiver_interruption(void) {
#else
void receiver_interruption(void) {
#endif

  if (digitalRead(RECEIVER_PIN)) {
    // Impulse start
    pl = 0;
    pm = micros();
  }
  else {
    // End of impulse
    // End time and length are calculated
    pl = micros() - pm;
    //pm += pl;
  }
  // YIELD(); // cannot have this in an ISR
}
#endif

//////////////////////////////////////////////////////////////////////
Oregon_NR::Oregon_NR(byte MHZ, byte MHZ_INT)
{
  INT_NO = MHZ_INT;
  RECEIVER_PIN = MHZ;
  pinMode(MHZ, INPUT); // The output to which the receiver is connected
  packet = new byte[packet_length];
  valid_p = new byte[packet_length];
  decode_tacts = new byte[no_read_tacts];
  collect_data = new byte[no_read_tacts];
  collect_data2 = new byte[no_read_tacts];
}

Oregon_NR::Oregon_NR(byte MHZ, byte MHZ_INT, byte led, bool pull_up)
{
  INT_NO = MHZ_INT;
  LED = led;
  PULL_UP = pull_up;
  RECEIVER_PIN = MHZ;
  pinMode(MHZ, INPUT);    // GPIO pin connected to Receiver Data pin
  pinMode(LED, OUTPUT);   // GPIO pin connected to LED
  packet = new byte[packet_length];
  valid_p = new byte[packet_length];
  decode_tacts = new byte[no_read_tacts];
  collect_data = new byte[no_read_tacts];
  collect_data2 = new byte[no_read_tacts];
}
Oregon_NR::Oregon_NR(byte MHZ, byte MHZ_INT, byte led, bool pull_up, int p_size, bool assembl)
{
  is_assemble = assembl;
  INT_NO = MHZ_INT;
  LED = led;
  PULL_UP = pull_up;
  RECEIVER_PIN = MHZ;
  pinMode(MHZ, INPUT);    // Вывод, на который подключён приёмник
  pinMode(LED, OUTPUT);   // Вывод светодиода
  if (p_size < PACKET_LENGTH) p_size = PACKET_LENGTH;
  packet_length = p_size;
  no_read_bits = (p_size + 8) * 4;
  no_read_tacts = (p_size + 6) * 8;
  packet = new byte[packet_length];
  valid_p = new byte[packet_length];
  decode_tacts = new byte[no_read_tacts];
  if (is_assemble)  collect_data = new byte[no_read_tacts];
  else collect_data = new byte;
  collect_data2 = new byte[no_read_tacts];
  if (packet == NULL || valid_p == NULL || decode_tacts == NULL || collect_data == NULL || collect_data2 == NULL) no_memory = true;

}



//////////////////////////////////////////////////////////////////////
void Oregon_NR::start()
{
  packet_number = 0;
  packets_received = 0;
  start_pulse_cnt = 0;
  receive_status = FIND_PACKET;
  led_light(false);
  attachInterrupt(INT_NO, receiver_interruption, CHANGE);
}
//////////////////////////////////////////////////////////////////////
void Oregon_NR::stop()
{
  detachInterrupt(INT_NO);
}
//////////////////////////////////////////////////////////////////////
// Capture and parse the packet
// DEBUG_INFO - Serial displays information about data capture
//////////////////////////////////////////////////////////////////////
void Oregon_NR::capture(bool DEBUG_INFO)
{
  ////////////////////////////////////////////////////////
  // Return to the original state
  //maybe_packet = 0;
  packets_received = 0;
  sens_type = 0;
  crc_c = 0;
  captured = 0;
  data_val = 0;
  data_val2 = 0;


  ////////////////////////////////////////////////////////
  // Read data from the receiver
  noInterrupts();
  pulse_length = pl;
  pl = 0;
  pulse_marker = pm;
  interrupts();

  ////////////////////////////////////////////////////////
  // An impulse has arrived
  if (pulse_length != 0 && receive_status == FIND_PACKET) {
    // If the impulse came too late for a specific protocol version, then this is the first impulse.
    if ((pulse_marker - pre_marker) > (PER_LENGTH2 * 2 + LENGTH_TOLERANCE) && ver == 2) start_pulse_cnt = 0;
    if ((pulse_marker - pre_marker) > (PER_LENGTH3 * 2 + LENGTH_TOLERANCE) && ver == 3) start_pulse_cnt = 0;


    // The first "correct" pulse is found - we determine the type of protocol
    if (start_pulse_cnt == 0) {

      if (pulse_length < (MAX_LENGTH2 + LENGTH_TOLERANCE) && pulse_length > (MIN_LENGTH2 -  LENGTH_TOLERANCE) && catch2 ) {
        start_pulse_cnt = 1;
        pre_marker = pulse_marker;
        ver = 2;
        pulse1 = pulse_length;
      }

      if (pulse_length < (MAX_LENGTH3 + LENGTH_TOLERANCE) && pulse_length > (MIN_LENGTH3 -  LENGTH_TOLERANCE) && catch3 && packet_number == 0) {
        start_pulse_cnt = 1;
        pre_marker = pulse_marker;
        ver = 3;
        pulse1 = pulse_length;
      }
      return;
    }

    // Found the next "correct" impulse
    else {

      // version 2
      if (pulse_length < (MAX_LENGTH2 + LENGTH_TOLERANCE) && pulse_length > (MIN_LENGTH2 -  LENGTH_TOLERANCE) && catch2 ) {

        // If the pulse is in the right place, then add the counter of the found start pulses
        if ((pulse_marker - pre_marker) > (PER_LENGTH2 * 2 - LENGTH_TOLERANCE) && (pulse_marker - pre_marker) < (PER_LENGTH2 * 2 + LENGTH_TOLERANCE) && ver == 2)
        {
          start_pulse_cnt++;
          pre_marker = pulse_marker;
          pulse2 = pulse_length;
          if (start_pulse_cnt != CATCH_PULSES) return;
          pulse2 = pulse_length;
          pulse_length = 0;
        }
        // The next impulse is in the wrong place
        // Assign it first
        else
        {
          start_pulse_cnt = 1;
          pre_marker = pulse_marker;
          pulse1 = pulse_length;
          ver = 2;
          return;
        }
      }

      // version 3
      if (pulse_length < (MAX_LENGTH3 + LENGTH_TOLERANCE) && pulse_length > (MIN_LENGTH3 -  LENGTH_TOLERANCE) && catch3 && packet_number == 0) {

        // If the pulse is in the right place, then add the counter of the found start pulses
        if ((pulse_marker - pre_marker) > (PER_LENGTH3 * 2 - LENGTH_TOLERANCE) && (pulse_marker - pre_marker) < (PER_LENGTH3 * 2 + LENGTH_TOLERANCE) && ver == 3)
        {
          start_pulse_cnt++;
          pre_marker = pulse_marker;
          pulse2 = pulse_length;
          if (start_pulse_cnt != CATCH_PULSES) return;
          pulse_length = 0;

        }
        // The next impulse is in the wrong place
        // Assign it first
        else
        {
          start_pulse_cnt = 1;
          pre_marker = pulse_marker;
          pulse1 = pulse_length;
          ver = 3;
          return;
        }
      }
    }
  }



  //////////////////////////////////////////////////////////////////////
  // If Found the right number of correct pulses in the right places, then maybe it's a packet. We start COLLECTING DATA

  if (start_pulse_cnt == CATCH_PULSES && receive_status == FIND_PACKET) {
    start_pulse_cnt = 0;

    if (packet_number == 0)
    {
      read_tacts = collect(collect_data);
      first_packet_end = millis();
      packet_number = 1;

    }
    else
    {
      read_tacts2 = collect(collect_data2);
      packet_number = 2;
      receive_status = ANALIZE_PACKETS;

    }

  }


  //*************************************************************************************
  /////////////////////////////////////////////////////////////////////
  // If the first packet is found and the waiting time for the second one has expired
  // Do not wait for the second, but switch to the analysis mode
  // ALSO do not wait for the second packet if packet splice mode is disabled
  if (packet_number == 1 && (millis() - first_packet_end) > 200) receive_status = ANALIZE_PACKETS;
  if (packet_number == 1 && (!is_assemble || ver == 3 )) receive_status = ANALIZE_PACKETS;


  //////////////////////////////////////////////////////////////////////
  // Data Analysis /////////////////////////////////////////////////////
  if  (receive_status == ANALIZE_PACKETS) {
    // Serial.print("ver an");
    // Serial.println(ver);
    //////////////////////////////////////////////////////////////////////
    // If only a piece of the parcel has arrived, then it is not worth processing
    if ((ver == 2 && read_tacts < 136 && read_tacts2 < 136) || (ver == 3 && read_tacts < 80))
    {
      receive_status = FIND_PACKET;
      start_pulse_cnt = 0;
      packet_number = 0;
      return;
    }
    // Disable interrupt to reduce processing time
    detachInterrupt(INT_NO);

    led_light(true);
    restore_sign = 0;
    work_time = millis();    // Calculate the processing time of the packet
    // Dump the collected data
    // To send without interference, the impurity values ​​should be approximately
    // v2 - 87 07 and occasionally 86 06, because pulse length 883ms and 395ms
    // v3 - 86 06 and occasionally 87 07 because pulse length 838 and 350ms
    if (DEBUG_INFO && receiver_dump) {
      Serial.println(" ");
      Serial.print("SCOPE1 ");
      byte* bt = collect_data;
      for (int i = 0; i < read_tacts; i++) {
        Serial.print((*bt & 0xF0) >> 4, HEX);
        Serial.print(*bt & 0x0F, HEX);
        Serial.print(' ');
        bt++;
      }
      Serial.println(" ");
      if ( packet_number == 2)
      {
        Serial.print("SCOPE2 ");
        byte* bt = collect_data2;
        for (int i = 0; i < read_tacts2; i++) {
          Serial.print((*bt & 0xF0) >> 4, HEX);
          Serial.print(*bt & 0x0F, HEX);
          Serial.print(' ');
          bt++;
        }
        Serial.println(" ");
      }

    }


    //////////////////////////////////////////////
    // Process the first record
    // Decrypt the entry. We will save the data in decode_tacts []
    get_tacts(collect_data, read_tacts);
    bool halfshift;

    if (get_data(0, ver, collect_data) > get_data(1, ver, collect_data)) {
      data_val = get_data(0, ver, collect_data);
      halfshift = 0;
    }
    else {
      data_val = get_data(1, ver, collect_data);
      halfshift = 1;
    }

    (void)halfshift; // does not appear to be used at this time - mjh 05/24/21

    //////////////////////////////////////////////
    // Looking for the sync position
    synchro_pos = get_synchro_pos(collect_data);
    //////////////////////////////////////////////
    // Display the parcel
    if (DEBUG_INFO) {
      if (packet_number == 2)   Serial.print("1)     ");
      if (packet_number == 1)   Serial.print("RESULT ");
      byte* bt = collect_data;
      for (int i = 0; i < no_read_bits; i++) {
        if ((ver == 3 && i <= read_tacts) || (ver == 2 && i <= read_tacts / 2)) {
          if (*bt > 128 + 1) Serial.print('I');
          if (*bt < 128 - 1) Serial.print('O');
          if (*bt == 128 + 1) Serial.print('i');
          if (*bt == 128 - 1) Serial.print('o');
          if (*bt == 128) Serial.print('.');
          if (receiver_dump && ver == 2) Serial.print("     ");
          if (receiver_dump && ver == 3) Serial.print("  ");
        }
        else Serial.print(' ');
        bt++;
      }
      Serial.print(" OSV:");
      Serial.print(ver);
      Serial.print(" SYN:");
      if (synchro_pos < 255)
        Serial.print(synchro_pos);
      else  Serial.print("NO");
      Serial.print(" TIME:");
      Serial.println (millis() / 1000);
    }
    //////////////////////////////////////////////
    // Process the second record in the same way
    if (packet_number == 2) {
      get_tacts(collect_data2, read_tacts2);

      if (get_data(0, ver, collect_data2) > get_data(1, ver, collect_data2)) {
        data_val2 = get_data(0, ver, collect_data2);
        halfshift = 0;
      }
      else {
        data_val2 = get_data(1, ver, collect_data2);
        halfshift = 1;
      }

      synchro_pos2 = get_synchro_pos(collect_data2);
      if (DEBUG_INFO) {
        Serial.print("2)     ");
        byte* bt = collect_data2;
        for (int i = 0; i < no_read_bits; i++) {
          if (i <= read_tacts2 / 2) {
            if (*bt > 128 + 1) Serial.print('I');
            if (*bt < 128 - 1) Serial.print('O');
            if (*bt == 128 + 1) Serial.print('i');
            if (*bt == 128 - 1) Serial.print('o');
            if (*bt == 128) Serial.print('.');
            if (receiver_dump && ver == 2) Serial.print("     ");
            if (receiver_dump && ver == 3) Serial.print("  ");
          }
          else Serial.print(' ');
          bt++;
        }

        Serial.print(" OSV:");
        Serial.print(ver);
        Serial.print(" SYN:");
        if (synchro_pos2 < 255)
          Serial.print(synchro_pos2);
        else
          Serial.print("NO");;
      }
    }
    byte* result_data = nullptr;
    //D byte* result_data_start, aux_data;
    int correlation;



    //////////////////////////////////////////////
    // COMPARISON OF PACKAGES
    // If there is only one package, then there is nothing to match
    if (packet_number == 1)
    {
      result_size = read_tacts;
      result_data = collect_data;
    }
    //////////////////////////////////////////////
    // But if there are two, then PACKAGE ASSEMBLY is needed
    // calculate the optimal "offset" of packets relative to each other
    if (packet_number == 2) {

      correlation = correlate_data(collect_data, collect_data2);
      if (DEBUG_INFO) {
        Serial.print(" COR: ");
        Serial.println(correlation);
      }
      //////////////////////////////////////////////
      // Collect the data into the package where the sync was found earlier
      //////////////////////////////////////////////

      if (synchro_pos >= synchro_pos2)
      {
        result_size = read_tacts2;
        result_data = collect_data2;
        correlation = -correlation;
        assemble_data(collect_data2, collect_data, correlation);
      }
      else
      {
        result_size = read_tacts;
        result_data = collect_data;
        assemble_data(collect_data, collect_data2, correlation);
      }
    }
    //////////////////////////////////////////////
    // Output of the finished parcel
    if (DEBUG_INFO && packet_number == 2) {
      Serial.print("RESULT ");
      byte* rdt = result_data;
      for (int bt = 0; bt < no_read_bits; bt++) {
        if (bt <= result_size / 2) {
          if (*rdt > 128 + 1) Serial.print('I');
          if (*rdt < 128 - 1) Serial.print('O');
          if (*rdt == 128 + 1) Serial.print('i');
          if (*rdt == 128 - 1) Serial.print('o');
          if (*rdt == 128) Serial.print('.');
          if (receiver_dump && ver == 2) Serial.print("     ");
          if (receiver_dump && ver == 3) Serial.print("  ");
        }
        else Serial.print(' ');
        rdt++;
      }
      Serial.println(" ");
    }



    // Check if splicing did anything - disabled. It only gives you a flag, but it takes a long time.
    //////////////////////////////////////////////

    // if (get_data(halfshift, ver, result_data) > data_val && get_data(halfshift, ver, result_data) > data_val2 && ver == 2)
    if (packet_number == 2)

      //////////////////////////////////////////////
      // Extract the bits from the clock sequence
      sens_type = 0;
    if (result_data)
    {
      if (get_info_data(result_data, packet, valid_p))
      {
        sens_type = get_sensor(packet);  // Determine the type of package by the type of sensor
        restore_data(packet, sens_type); // Recover data by sensor type
        crc_c = check_CRC(packet, sens_type); // Check CRC, if it is true, then make all doubtful bits confident
        // If not all bytes are determined with certainty, it cannot be assumed that the packet is correct
        // The packet is captured only if the start sequence is found (sync nibl)
        // If there were no synchromes, then there is nothing to talk about at all
        if ( synchro_pos != 255 && packet_number == 1)  captured = 1;
        if ( (synchro_pos2 != 255 || synchro_pos2 != 255) && packet_number == 2)  captured = 1;
        // Capturing a piece of the parcel is not counted
        if ((ver == 2 && read_tacts < 136) || (ver == 3 && read_tacts < 80))   captured = 0;
      }
    }
    else
    {
      captured = 0; // Added 05/24/21 - mjh
    }


#if ADD_SENS_SUPPORT == 1
    sens_tmp2 = 404;
    sens_CO = 255;
    sens_CH = 255;
    sens_pressure = 0;
    sens_voltage = 0;
    sens_current = 0;
    sens_ip22  = 0;
    sens_ip72 = 0;
    sens_lockalarm = 0;
    sens_pump_count = 0;
    sens_capacitance = 0;
    sens_drop_counter = 0;
#endif
    sens_id = 0;
    sens_chnl = 0;
    sens_battery = 0;
    sens_tmp = 0;
    sens_hmdty = 0;
    UV_index = 0;
    lightness = 0;
    sens_avg_ws = 0;
    sens_max_ws = 0;
    sens_wdir = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // Decoding Oregon sensors - Расшифровка датчиков Орегон
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    if ((sens_type == THGN132           ||
         sens_type == THN132            ||
         sens_type == THN800            ||
         (sens_type  & 0x0FFF) == RTGN318 ||
         (sens_type  & 0x0FFF) == RTHN318 ||
         sens_type == THGR810           ||
         sens_type == BTHGN129          ||
         sens_type == BTHR968           ||
         sens_type == THGN500) && crc_c)
    {

      sens_id = get_id(packet);
      sens_chnl = get_channel(packet);
      sens_battery = get_battery(packet);
      sens_tmp = get_temperature(packet);

      if (sens_type == THGN132 ||
          (sens_type  & 0x0FFF) == RTGN318 ||
          sens_type == THGR810           ||
          sens_type == BTHGN129          ||
          sens_type == BTHR968           ||
          sens_type == THGN500)
        sens_hmdty = get_humidity(packet);
      else sens_hmdty = 0;
    }

    if (sens_type == PCR800 && crc_c) {
      sens_id = get_id(packet);
      sens_battery = get_battery(packet);
    }

    if (sens_type == RFCLOCK && crc_c) {
      sens_id = get_id(packet);
      sens_chnl = get_channel(packet);
      sens_battery = get_battery(packet);
    }

    if (sens_type == WGR800 && crc_c) {
      sens_id = get_id(packet);
      sens_battery = get_battery(packet);
      sens_avg_ws = get_avg_windspeed(packet);
      sens_max_ws = get_max_windspeed(packet);
      sens_wdir = get_winddirection(packet);
    }

    if (sens_type == UVN800 && crc_c) {
      sens_id = get_id(packet);
      sens_battery = get_battery(packet);
      UV_index = get_UV(packet);
      lightness = get_light(packet);
    }

    if (sens_type == PCR800 && crc_c) {
      sens_id = get_id(packet);
      sens_battery = get_battery(packet);
      // The rest of the parameters are retrieved directly from the package
      // Остальные параметры извлекаюся непосредственно из пакета
    }




#if ADD_SENS_SUPPORT == 1
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Decoding complex gas sensors - Расшифровка комплексных газовых датчиков
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    if ((sens_type & 0xFF00) == GAS && crc_c) {
      sens_id = 0;
      sens_battery = 0;
      sens_chnl = get_gas_channel(packet);
      sens_tmp = get_gas_temperature_out(packet);
      if (packet[9] == 0x0F) sens_tmp = 404;
      sens_tmp2 = get_gas_temperature_in(packet);
      if (packet[12] == 0x0F) sens_tmp2 = 404;
      sens_hmdty = get_gas_hmdty(packet);
      sens_CO = get_gas_CO(packet);
      sens_CH = get_gas_CH(packet);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Decoding of fire alarm sensors - Расшифровка датчиков пожарной сигнализации
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    if ((sens_type & 0xFF00) == FIRE && crc_c) {
      sens_id = 0;
      sens_battery = 0;

      sens_chnl = get_gas_channel(packet);
      sens_ip22 = get_fire_ip22(packet);
      sens_ip72 = get_fire_ip72(packet);
      sens_lockalarm = get_fire_lockalarm(packet);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Decoding of THP sensors - Расшифровка датчиков THP
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    if ((sens_type & 0xFF00) == THP && crc_c) {
      sens_chnl = get_gas_channel(packet);
      sens_voltage = get_thp_voltage(packet);
      sens_tmp = get_thp_temperature(packet);
      sens_hmdty = get_thp_humidity(packet);
      sens_pressure = get_thp_pressure(packet);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Decoding current and voltage sensors - Расшифровка датчиков тока и напряжения
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    if ((sens_type & 0xFF00) == CURRENT && crc_c) {
      sens_id = 0;
      sens_battery = 0;

      sens_chnl = get_gas_channel(packet);
      sens_current = get_current(packet);
      sens_voltage = get_voltage(packet);
      sens_pump_count = get_pump_count(packet);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////
    // Decoding of capacitive precipitation sensors - Расшифровка датчиков осадков емкостного типа
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    if ((sens_type & 0xFF00) == CAPRAIN && crc_c) {
      sens_id = 0;
      sens_battery = 0;

      //sens_heater =
      sens_drop_counter = get_dropcounter(packet);
      sens_capacitance = get_capacitance(packet);
    }

#endif
    ////////////////////////////////////////////////////////////////////////////////
    // Rest of calculations
    // Return everything to its original state and turn on listening to the receiver
    work_time = millis() - work_time;
    packets_received = 0;
    if (data_val >= 64 && synchro_pos != 255) packets_received++;
    if (data_val2 >= 64 && synchro_pos2 != 255) packets_received++;
    receive_status = FIND_PACKET;
    start_pulse_cnt = 0;
    packet_number = 0;
    led_light(false);
    //Serial.print("LED = ");
    //Serial.println(LED);
    attachInterrupt(INT_NO, receiver_interruption, CHANGE);
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Retrieves the clock sequence from the recording
// Parameters: cdptr - pointer to the recorded clock sequence
// The result is written to the decode_tacts array
////////////////////////////////////////////////////////////////////////////////////////////////////
void Oregon_NR::get_tacts(byte* cdptr, int bitsize) {

  // Reset arrays
  for (int bt = 0 ; bt < bitsize; bt++) decode_tacts[bt] = 2;     // Initially, the clock is unknown - Изначально такт неизвестен

  // Decoding the bars
  byte* cdp = cdptr;
  for (int bt = 0 ; bt < bitsize; bt++)
  {
    if (ver == 2 && decode_method == 1)
    {
      if ((*cdp & 0xf0) > 0x20 && (*cdp & 0x0f) > 0x03) decode_tacts[bt] = 1;
      if ((*cdp & 0xf0) < 0x30 && (*cdp & 0x0f) < 0x05) decode_tacts[bt] = 0;
      if ((*cdp & 0xf0) < 0x20 && (*cdp & 0x0f) > 0x04) decode_tacts[bt] = 4;
      if ((*cdp & 0xf0) > 0x40 && (*cdp & 0x0f) < 0x02) decode_tacts[bt] = 3;
    }
    if (ver == 2 && decode_method == 2)
    {
      if (*cdp == 0x88 || *cdp == 0x87  || *cdp == 0x86 || *cdp == 0x85 || *cdp == 0x84 || *cdp == 0x83 || *cdp == 0x78 ||  *cdp == 0x77 || *cdp == 0x68 || *cdp == 0x58 ) decode_tacts[bt] = 1;              // Такт 11 (В ИДЕАЛЕ 87, НО ИЗ ЗА СДВИГА НА 3 ТАКТА МОЖЕТ БЫТЬ ОТ 58 А 84)
      if (*cdp == 0x00 || *cdp == 0x01  || *cdp == 0x02 || *cdp == 0x03 || *cdp == 0x10 || *cdp == 0x20 || *cdp == 0x30) decode_tacts[bt] = 0;                                                         // Такт 00 (В ИДЕАЛЕ 00, НО ИЗ ЗА СДВИГА НА 3 ТАКТА МОЖЕТ БЫТЬ ОТ 30 ДА 03)
      if (*cdp == 0x05 || *cdp == 0x06  || *cdp == 0x07 || *cdp == 0x08 || *cdp == 0x15 || *cdp == 0x16 || *cdp == 0x17 || *cdp == 0x24 || *cdp == 0x25 || *cdp == 0x34 || *cdp == 0x35) decode_tacts[bt] = 4; // Такт 01 (В ИДЕАЛЕ 07, НО ИЗ ЗА СДВИГА НА 3 ТАКТА МОЖЕТ БЫТЬ ДО 34)
      if (*cdp == 0x50 || *cdp == 0x60  || *cdp == 0x70 || *cdp == 0x80 || *cdp == 0x51 || *cdp == 0x61 || *cdp == 0x71 || *cdp == 0x42 || *cdp == 0x52 || *cdp == 0x43 || *cdp == 0x53) decode_tacts[bt] = 3; // Такт 10 (В ИДЕАЛЕ 70, НО ИЗ ЗА СДВИГА НА 3 ТАКТА МОЖЕТ БЫТЬ ДО 43)
    }
    if (ver == 3 && decode_method == 1)
    {
      if ((*cdp & 0xf0) > 0x20 && (*cdp & 0x0f) > 0x04) decode_tacts[bt] = 1;
      if ((*cdp & 0xf0) < 0x30 && (*cdp & 0x0f) < 0x05) decode_tacts[bt] = 0;
      if ((*cdp & 0xf0) < 0x20 && (*cdp & 0x0f) > 0x02) decode_tacts[bt] = 4;
      if ((*cdp & 0xf0) > 0x20 && (*cdp & 0x0f) < 0x02) decode_tacts[bt] = 3;
    }
    if (ver == 3 && decode_method == 2)
    {
      if (*cdp == 0x87 || *cdp == 0x86  || *cdp == 0x85 || *cdp == 0x84 || *cdp == 0x83 || *cdp == 0x82 || *cdp == 0x78 ||  *cdp == 0x77 || *cdp == 0x76 || *cdp == 0x68 || *cdp == 0x67 ) decode_tacts[bt] = 1;              // Такт 11 (В ИДЕАЛЕ 87, НО ИЗ ЗА СДВИГА НА 3 ТАКТА МОЖЕТ БЫТЬ ОТ 58 А 84)
      if (*cdp == 0x00 || *cdp == 0x01  || *cdp == 0x02 || *cdp == 0x03 || *cdp == 0x10 || *cdp == 0x20 || *cdp == 0x30) decode_tacts[bt] = 0;                                                         // Такт 00 (В ИДЕАЛЕ 00, НО ИЗ ЗА СДВИГА НА 3 ТАКТА МОЖЕТ БЫТЬ ОТ 30 ДА 03)
      if (*cdp == 0x05 || *cdp == 0x06  || *cdp == 0x07 || *cdp == 0x08 || *cdp == 0x15 || *cdp == 0x16 || *cdp == 0x17 || *cdp == 0x24 || *cdp == 0x25 || *cdp == 0x34 || *cdp == 0x35) decode_tacts[bt] = 4; // Такт 01 (В ИДЕАЛЕ 07, НО ИЗ ЗА СДВИГА НА 3 ТАКТА МОЖЕТ БЫТЬ ДО 34)
      if (*cdp == 0x50 || *cdp == 0x60  || *cdp == 0x70 || *cdp == 0x80 || *cdp == 0x51 || *cdp == 0x61 || *cdp == 0x71 || *cdp == 0x42 || *cdp == 0x52 || *cdp == 0x43 || *cdp == 0x53) decode_tacts[bt] = 3; // Такт 10 (В ИДЕАЛЕ 70, НО ИЗ ЗА СДВИГА НА 3 ТАКТА МОЖЕТ БЫТЬ ДО 43)
    }
    if (decode_method == 3)
    {
      if ((((*cdp) >> 4) + (*cdp & 0x0F)) < 5)  decode_tacts[bt] = 0;
      if (((((*cdp) >> 4) + (*cdp & 0x0F)) > 4) && ((((*cdp) >> 4) + (*cdp & 0x0F)) < 10))
      {
        if (((*cdp) >> 4) > (*cdp & 0x0f))  decode_tacts[bt] = 3;
        if (((*cdp) >> 4) < (*cdp & 0x0f))  decode_tacts[bt] = 4;
        if (((*cdp) >> 4) == (*cdp & 0x0f) && (*(cdp - 1) & 0x0F) < 4 )  decode_tacts[bt] = 4;
        if (((*cdp) >> 4) == (*cdp & 0x0f) && (*(cdp - 1) & 0x0F) > 4 )  decode_tacts[bt] = 3;
      }
      if ((((*cdp) >> 4) + (*cdp & 0x0F)) > 10) decode_tacts[bt] = 1;

    }
    // *cdp++;
    cdp++;
  }

  // Print expansion
  if (receiver_dump)
  {
    byte* cdp = cdptr;
    Serial.print("BEFORE ");


    for (int bt = 0 ; bt < bitsize; bt++)
    {

      if (decode_tacts[bt] == 1) Serial.print("II");
      if (decode_tacts[bt] == 0) Serial.print("OO");
      if (decode_tacts[bt] == 2) Serial.print("__");
      if (decode_tacts[bt] == 3) Serial.print("IO");
      if (decode_tacts[bt] == 4) Serial.print("OI");
      Serial.print(" ");
      // *cdp++;
      cdp++;
    }
    Serial.println();
  }


  // Deciphered everything we could on the fly
  // Check if the clock sequence is valid

  for (int bt = 1; bt < bitsize; bt++)
  {
    //    if (decode_tacts[bt] == 2)
    {
      //Х0 0X - invalid
      if ((decode_tacts[bt - 1]  == 0  || decode_tacts[bt - 1]  == 3) && (decode_tacts[bt] == 0 || decode_tacts[bt] == 4)) decode_tacts[bt] = 2;
      //Х1 1X - not allowed
      if ((decode_tacts[bt - 1]  == 1  || decode_tacts[bt - 1]  == 4) && (decode_tacts[bt] == 1 || decode_tacts[bt] == 3)) decode_tacts[bt] = 2;
    }
  }

  // Restore single ticks
  for (int bt = 1; bt < (bitsize - 1); bt++)
  {
    if (decode_tacts[bt] == 2)
    {
      //Х0 __ 0Х
      //Х0 11 0Х
      if ((decode_tacts[bt - 1]  == 0  || decode_tacts[bt - 1]  == 3) && (decode_tacts[bt + 1] == 0 || decode_tacts[bt + 1] == 4)) {
        decode_tacts[bt] = 1;
        restore_sign ^= 2;
      }
      //Х0 __ 1Х
      //Х0 10 1Х
      if ((decode_tacts[bt - 1]  == 0  || decode_tacts[bt - 1]  == 3) && (decode_tacts[bt + 1] == 1 || decode_tacts[bt + 1] == 3)) {
        decode_tacts[bt] = 3;
        restore_sign ^= 2;
      }
      //Х1 __ 0Х
      //Х1 01 0Х
      if ((decode_tacts[bt - 1]  == 1  || decode_tacts[bt - 1]  == 4) && (decode_tacts[bt + 1] == 0 || decode_tacts[bt + 1] == 4)) {
        decode_tacts[bt] = 4;
        restore_sign ^= 2;
      }
      //Х1 __ 1Х
      //Х1 00 1Х
      if ((decode_tacts[bt - 1]  == 1  || decode_tacts[bt - 1]  == 4) && (decode_tacts[bt + 1] == 1 || decode_tacts[bt + 1] == 3)) {
        decode_tacts[bt] = 0;
        restore_sign ^= 2;
      }
    }
  }

  // recover lost half-cycles
  cdp = cdptr;
  for (int bt = 1 ; bt < (bitsize - 1); bt++)
  {
    if (decode_tacts[bt] == 2)
    {
      //Х0 _0
      //Х0 10
      if ((*cdp & 0x0f) < 0x05 && (decode_tacts[bt - 1] == 0 || decode_tacts[bt - 1] == 3)) {
        decode_tacts[bt] = 3;
        restore_sign ^= 1;
      }
      //Х1 _1
      //Х1 01
      if ((*cdp & 0x0f) > 0x04 && (decode_tacts[bt - 1] == 1 || decode_tacts[bt - 1] == 4)) {
        decode_tacts[bt] = 4;
        restore_sign ^= 1;
      }
      //0_ 0X
      //01 0X
      if ((*cdp & 0xF0) < 0x50 && (decode_tacts[bt + 1] == 0 || decode_tacts[bt + 1] == 4)) {
        decode_tacts[bt] = 4;
        restore_sign ^= 1;
      }
      //1_ 1X
      //10 1X
      if ((*cdp & 0xF0) > 0x40 && (decode_tacts[bt + 1] == 1 || decode_tacts[bt + 1] == 3)) {
        decode_tacts[bt] = 3;
        restore_sign ^= 1;
      }
    }
    // *cdp++;
    cdp++;
  }

  // Again we check if the clock sequence is valid, but what we restored there is unknown :)

  for (int bt = 1; bt < bitsize; bt++)
  {
    {
      //Х0 0X - invalid
      if ((decode_tacts[bt - 1]  == 0  || decode_tacts[bt - 1]  == 3) && (decode_tacts[bt] == 0 || decode_tacts[bt] == 4)) decode_tacts[bt] = 2;
      //Х1 1X - not allowed
      if ((decode_tacts[bt - 1]  == 1  || decode_tacts[bt - 1]  == 4) && (decode_tacts[bt] == 1 || decode_tacts[bt] == 3)) decode_tacts[bt] = 2;
    }
    // *cdp++;
    cdp++;
  }


  // Determine the package version by the preamble
  // If the preamble is confidently recognized for several clock cycles, then you can judge the version of the package

  //001100110011 -> v2
  if (/*decode_tacts[0] == 0 && decode_tacts[1] == 1  &&*/ decode_tacts[2] == 0 && decode_tacts[3] == 1 && decode_tacts[4] == 0 && decode_tacts[5] == 1 && ver == 3) {
    ver = 2;
    restore_sign ^= 4;
  }
  //110011001100 -> v2
  if (/*decode_tacts[0] == 1 && decode_tacts[1] == 0  &&*/ decode_tacts[2] == 1 && decode_tacts[3] == 0 && decode_tacts[4] == 1 && decode_tacts[5] == 0 && ver == 3) {
    ver = 2;
    restore_sign ^= 4;
  }
  //010101010101 -> v3
  if (/*decode_tacts[0] == 4 && decode_tacts[1] == 4  &&*/ decode_tacts[2] == 4 && decode_tacts[3] == 4 && decode_tacts[4] == 4 && decode_tacts[5] == 4 && ver == 2) {
    ver = 3;
    restore_sign ^= 4;
  }
  //101010101010 -> v3
  if (/*decode_tacts[0] == 3 && decode_tacts[1] == 3  &&*/ decode_tacts[2] == 3 && decode_tacts[3] == 3 && decode_tacts[4] == 3 && decode_tacts[5] == 3 && ver == 2) {
    ver = 3;
    restore_sign ^= 4;
  }

  // Print expansion
  if (receiver_dump)
  {
    byte* cdp = cdptr;
    Serial.print("AFTER  ");
    for (int bt = 0 ; bt < bitsize; bt++)
    {
      if (decode_tacts[bt] == 1) Serial.print("II");
      if (decode_tacts[bt] == 0) Serial.print("OO");
      if (decode_tacts[bt] == 2) Serial.print("__");
      if (decode_tacts[bt] == 3) Serial.print("IO");
      if (decode_tacts[bt] == 4) Serial.print("OI");
      Serial.print(" ");
      // *cdp++;
      cdp++;
    }
    Serial.println();
  }


  return;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Retrieves the bit from the clock sequence
// Parameters: cdptr - pointer to the written data
// btt - offset in ticks. An offset per cycle in parsing can help restore a packet that has a damaged beginning.
// The function returns the quality or "suitability" of the decryption - the number of confidently recognized bars.
// Comparing the suitability with btt = 0 and btt = 1, choose the best
////////////////////////////////////////////////////////////////////////////////////////////////////
int Oregon_NR::get_data(int btt, byte p_ver, byte* cdptr) {
  // btt - offset per clock during parsing can help restore a package whose beginning is destroyed

  byte* cdp = cdptr;
  // Clean up the array
  for (int bt = 0 ; bt < no_read_bits; bt++) {

    *cdp = 128;
    cdp++;
  }
  cdp = cdptr;

  *cdp = (128 + 2); // The first bit is always one !!!
  cdp++;
  int packet_validity = 0;

  if (p_ver == 2) {
    for (int bt = 1 ; bt < no_read_bits; bt++) {

      if (decode_tacts[bt * 2 - btt] == 0) *cdp -= 1; // If 00 - then maybe 0 here
      //if(decode_tacts[bt*2-btt]==0) *cdp-=1; // initial weight 1 discards out-of-sync sensors. 2 - catches with them
      if (decode_tacts[bt * 2 - btt] == 1) *cdp += 1; // If 11 - then presumably here 1
      if (decode_tacts[bt * 2 - 2 - btt] == 1 && decode_tacts[bt * 2 - 1 - btt] == 4) *cdp -= 1; // If 1101 was confident before, then this adds confidence that 0 is here
      if (decode_tacts[bt * 2 - 2 - btt] == 0 && decode_tacts[bt * 2 - 1 - btt] == 3) *cdp += 1;
      if (decode_tacts[bt * 2 - 2 - btt] == 0 && decode_tacts[bt * 2 - 1 - btt] == 1) *cdp -= 1;
      if (decode_tacts[bt * 2 - 2 - btt] == 1 && decode_tacts[bt * 2 - 1 - btt] == 0) *cdp += 1;

      if (decode_tacts[bt * 2 + 2 - btt] == 1 && decode_tacts[bt * 2 + 1 - btt] == 3) *cdp -= 1; // If after that comes 1011 - then this adds confidence that here is 0
      if (decode_tacts[bt * 2 + 2 - btt] == 0 && decode_tacts[bt * 2 + 1 - btt] == 4) *cdp += 1;
      if (decode_tacts[bt * 2 + 2 - btt] == 0 && decode_tacts[bt * 2 + 1 - btt] == 1) *cdp -= 1;
      if (decode_tacts[bt * 2 + 2 - btt] == 1 && decode_tacts[bt * 2 + 1 - btt] == 0) *cdp += 1;

      // Count the number of valid bits in the packet
      if (*cdp > (129)) packet_validity += *cdp - 128;
      if (*cdp < (127)) packet_validity += 128 - *cdp;
      cdp++;
      YIELD();
    }
    return packet_validity; // return the number of valid bytes
  }

  if (p_ver == 3) {
    for (int bt = 1 ; bt < no_read_bits; bt++) {

      if (*(cdp - 1) > 128) // if there was 1 before
      {
        if (decode_tacts[bt - btt] == 0 || decode_tacts[bt - btt] == 1) *cdp -= 2; // If 00 or 11 - then 0 here
        if (decode_tacts[bt - btt] == 3 || decode_tacts[bt - btt] == 4) *cdp += 2; // If 01 or 10 - then here 1
      }
      if (*(cdp - 1) < 128) // if there was 0 before
      {
        if (decode_tacts[bt - btt] == 0 || decode_tacts[bt - btt] == 1) *cdp += 2; // If 00 or 11 - then here 1
        if (decode_tacts[bt - btt] == 3 || decode_tacts[bt - btt] == 4) *cdp -= 2; // If 01 or 10 - then here 0
      }

      // if it is not clear before that, then most likely it was not possible to restore two whole clock cycles. look a few bits back

      // Restore the order when skipping the spirit of the beats to a row -- google translate issue spirit may be ghost
      if (*(cdp - 1) == 128 && *(cdp - 2) == 128)
      {
        // 0 __ __ 0 - does not change
        if ((decode_tacts[bt - btt] == 0 || decode_tacts[bt - btt] == 3) && (decode_tacts[bt - btt - 2] == 0 || decode_tacts[bt - btt - 2] == 4))
        {
          if (*(cdp - 2) > 128) *cdp += 1;
          if (*(cdp - 2) < 128) *cdp -= 1;
        }
        else
        {
          // 1 __ __ 0 or 0 __ __ 1 - changes
          if (*(cdp - 2) > 128) *cdp -= 1;
          if (*(cdp - 2) < 128) *cdp += 1;
        }
      }

      // Restore the order when skipping three bits to a row
      if (*(cdp - 1) == 128 && *(cdp - 2) == 128  && *(cdp - 3) == 128)
      {
        // 0 __ __ 0 - does not change
        if ((decode_tacts[bt - btt] == 0 || decode_tacts[bt - btt] == 3) && (decode_tacts[bt - btt - 3] == 0 || decode_tacts[bt - btt - 3] == 4))
        {
          if (*(cdp - 2) > 128) *cdp += 1;
          if (*(cdp - 2) < 128) *cdp -= 1;
        }
        else
        {
          // 1 __ __ 0 or 0 __ __ 1- changes
          if (*(cdp - 2) > 128) *cdp -= 1;
          if (*(cdp - 2) < 128) *cdp += 1;
        }
      }

      // Restore the order when skipping four bits to a row
      if (*(cdp - 1) == 128 && *(cdp - 2) == 128  && *(cdp - 3) == 128  && *(cdp - 4) == 128)
      {
        // 0 __ __ 0 - does not change
        if ((decode_tacts[bt - btt] == 0 || decode_tacts[bt - btt] == 3) && (decode_tacts[bt - btt - 4] == 0 || decode_tacts[bt - btt - 4] == 4))
        {
          if (*(cdp - 2) > 128) *cdp += 1;
          if (*(cdp - 2) < 128) *cdp -= 1;
        }
        else
        {
          // 0 __ __ 0 - does not change
          if (*(cdp - 2) > 128) *cdp -= 1;
          if (*(cdp - 2) < 128) *cdp += 1;
        }
      }
      // More nibble using the checksum still cannot be restored.


      // Count the number of valid bits in the packet
      if (*cdp > (129))  packet_validity += *cdp - 128;
      if (*cdp < (127)) packet_validity += 128 - *cdp;
      cdp++;
      YIELD();
    }
    return packet_validity; // return the number of valid bytes
  }
  return 0;

}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Listen to a channel with a sampling rate of 16384Hz
// i.e. recording once every 61μs
// Each clock has a byte. The measure is divided into two half measures. 8 measurements are taken in each half-cycle.
// If there is a signal, the measurement adds 1 to the corresponding nibble. No signal - 0x00. The presence of a signal in the 0x88 cycle.
// cdptr - pointer to the memory area where the signal is written
// dtl - pointer to the number of read ticks
////////////////////////////////////////////////////////////////////////////////////////////////////
int Oregon_NR::collect(byte* cdptr) {

  //D bool cdp_prev_null;
  byte* cdp = cdptr;
  byte nulls_found = 0;
  int bt2 = 0;
  //////////////////////////////////////////////////////
  // Start recording from this moment (end of the last hook pulse + 1/16 bar)
  if (ver == 2)
  {
    pre_marker += 946; // two bars
    *cdp = 0x87;   // The first two bars are known - 11. We caught the impulse!
    cdp++;
  }

  if (ver == 3)
  {
    pre_marker += 1434; // three bars
    *cdp = 0x07;       // The first four bars are known - 0101. We caught the impulse!
    *(cdp + 1) = 0x07;
    cdp += 2;
  }

  //////////////////////////////////////////////////////
  // Start reading data into memory
  // The maximum length of a link for v3 is 104 BITS, THN132 is 76 BITs + at least 3 bits 111, which we already found
  int bt;

  for (bt = 0 ; bt < no_read_tacts; bt++) {
    *cdp = 0;
    for (byte ckl = 0; ckl < 8; ckl++) {            // Read 8 times in a half-cycle
      pre_marker += 61;
      while (micros() < pre_marker);
      if (digitalRead(RECEIVER_PIN)) *cdp += 0x10;  // Measurements are written to the high nibble
    }
    for (byte ckl = 0; ckl < 8; ckl++) {
      pre_marker += 61;
      while (micros() < pre_marker);
      if (digitalRead(RECEIVER_PIN)) *cdp += 1;     // In the next half-cycle, the measurements are written to the lowest nibble. This saves memory.

    }
    bt2++;
    // Ideal period 976.5625
    //   Every 7 clock cycles add 4μs to align the period from 976μs to 976.5714μs + 0.009%
    if (bt2 == 7)
    {
      pre_marker += (4 + timing_correction) ;
      bt2 = 0;
    }
   // Do not give up the processor while collecting data !!!
    // YIELD();
    /////////////////////////////////////////////
    // There is a time until the next half-cycle arrives
    // You can check if the package is over
    // If the channel has recently been empty or weak interference, then this adds confidence that we are seeing the end of the packet

    if ((*cdp & 0xf0) < 0x30 && (*cdp & 0x0f) < 0x05)  nulls_found++;
    else nulls_found = 0;
    cdp++;

    /////////////////////////////////////////////
    // If there are more empty_space empty fields in the record, then
    // this is most likely the end of the message. There is no point in reading further
    // empty_space - empirical number, depends on the type of receiver and signal level
    // If reduced, may be confused with packet corruption
    // If you increase, then you can not stop reading and start recording noise
    if (nulls_found > empty_space ) return bt;










    /////////////////////////////////////////////
    // We are waiting for the arrival of the time of the next half-cycle

    while (micros() < pre_marker);
  }
  YIELD();
  return bt;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//Определение смещения пакетов друг относительно друга
//В качестве параметров передаются указатели на массивы данных
// Возваращаяется смещение
// >0 - второй пакет начался раньше, <0 - Первый пакет начался раньше
////////////////////////////////////////////////////////////////////////////////////////////////////
int Oregon_NR::correlate_data(byte* ser1, byte* ser2) {

  byte best_correl = 0;
  int best_shift = 0;
  byte best_correl_back = 0;
  int best_shift_back = 0;
  byte shift_score[no_read_bits];
  byte* s1;
  byte* s2;
  byte* s2t = ser2;
  //смещаем первый пакет относительно второго
  for (int sht = 0; sht < no_read_bits; sht++) {
    s1 = ser1;
    s2 = s2t;
    shift_score[sht] = 0;
    for (int sp = 0; sp < no_read_bits - sht; sp++) {
      if ((*s1 > (128 + 1) && *s2 > (128 + 1)) || (*s1 < (128 - 1) && *s2 < (128 - 1)) ) shift_score[sht]++;
      s2++;
      s1++;
    }
    YIELD();
    s2t++;
  }
  for (int i = 0; i < no_read_bits; i++) {

    if (shift_score[i] > best_correl) {
      best_correl = shift_score[i];
      best_shift = i;
    }
  }

  //Теперь наоборот -втрой пакет относительно первого

  byte* s1t = ser1;
  for (int sht = 0; sht < no_read_bits; sht++) {
    s2 = ser2;
    s1 = s1t;
    shift_score[sht] = 0;
    for (int sp = 0; sp < no_read_bits - sht; sp++) {

      if ((*s1 > (128 + 1) && *s2 > (128 + 1)) || (*s1 < (128 - 1) && *s2 < (128 - 1)) ) shift_score[sht]++;
      s2++;
      s1++;
    }
    YIELD();
    s1t++;
  }
  // Ищем наилучшее совпадение для обоих вариантов

  for (int i = 0; i < no_read_bits; i++) {

    if (shift_score[i] > best_correl_back) {
      best_correl_back = shift_score[i];
      best_shift_back = i;
    }
  }
  //И возвращаем самое лучшее из двух
  if (best_correl_back > best_correl) return -best_shift_back;
  else return best_shift;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//Сборка из двух пакетов
//В качестве параметров передаются указатели на массивы данных
// Причём первым должен идти результирующий пакет, т.е. тот который имеет более длинную преамбулу.
//shift - смещение втрого пакета относительного первого
////////////////////////////////////////////////////////////////////////////////////////////////////
void Oregon_NR::assemble_data(byte* m1, byte* m2, int shift) {

  byte* s1 = m1;
  byte* s2 = m2;

  if (shift >= 0) {
    for (int g = 0; g < shift; g++) s2++;
    for (int i = 0; i < no_read_bits - shift; i++) {
      if (*s1 < (128 + 2) && *s1 > (128 - 2) && (*s2 > (128 + 1) || *s2 < (128 - 1))) {
        if (*s1 == 128) restore_sign ^= 8;
        *s1 = *s2;
      }
      s1++;
      s2++;
    }
  }
  else {
    for (int g = 0; g < -shift; g++) s1++;
    for (int i = 0; i < no_read_bits + shift; i++) {
      if (*s1 < (128 + 2) && *s1 > (128 - 2) && (*s2 > (128 + 1) || *s2 < (128 - 1))) {
        if (*s1 == 128) restore_sign ^= 8;
        *s1 = *s2;
      }
      s1++;
      s2++;
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//Возвращает позицию синхронибла в посылке. 0xFF - нет синхронибла
//
//code - указатель на расшифрованную битовую последовательность
//result - указатель на кодовую посылку
////////////////////////////////////////////////////////////////////////////////////////////////////
int Oregon_NR::get_synchro_pos(byte* code) {

  bool syn_found = false;
  byte* cp = code;
  int i = 0;
  for (i = 0; i < no_read_bits - 8; i++) {
    if (!consist_synchro && (*cp < 128 && *(cp + 1) > 128 && *(cp + 2) < 128 && *(cp + 3) > 128)) {
      syn_found = true;
      break;
    }

    if (consist_synchro && (*cp < 127 && *(cp + 1) > 129 && *(cp + 2) < 127 && *(cp + 3) > 129)) {
      syn_found = true;
      break;
    }

    cp++;
  }
  if (!syn_found) return 0xFF;
  //Последовательность нашли, но надо убедиться, что перед этим идёт перамбула, т. е. уверенные единицы
  // В преамбуле не может быть нулей! - это главное
  //Преамбулу надо просматривать на 16-3 = 13 битов назад. Ну хотя бы на 10!!!

  for (byte g = i; i - g < 10 && g > 0; g --) {
    cp --;
    if (*cp < 127) return 0xFF; // Перед синхрониблом в преамбуле не может быть уверенного нуля. Нет тут синхронибла!
  }
  return (byte) i;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Creates a code message
// code - pointer to the decrypted bit sequence
// result - a pointer to a code message
// valid - pointer to the validity map of the code message
//
// Создаёт кодовую посылку
// code - указатель на расшифрованную битовую последовательность
// result - указатель на кодовую посылку
// valid - указатель на карту достоверности кодовой посылки
////////////////////////////////////////////////////////////////////////////////////////////////////
int Oregon_NR::get_info_data(byte* code, byte* result, byte* valid) {

  byte* rd = result;
  byte* vd = valid;
  // Clean up the arrays
  //Чистим массивы
#if 0
  for (int l = 0; l < packet_length; l++) {
    *vd = 0;
    *rd = 0;
    vd++;
    rd++;
  }
  rd = result;
  vd = valid;
#else
  for (int l = 0; l < packet_length; l++) {
    vd[l] = 0;
    rd[l] = 0;
  }
#endif

  int csm;
  for (csm = 0; csm < 30; csm++) {
    if ( !consist_synchro && (*code < 128 && *(code + 1) > 128 && *(code + 2) < 128 && *(code + 3) > 128)) break; // Sequence 0101 found - Найдена последовательность 0101
    if (  consist_synchro && (*code < 127 && *(code + 1) > 129 && *(code + 2) < 127 && *(code + 3) > 129)) break;
    code++;
  }
  // Synchronous in the first 20 bits was not found, such a packet cannot be decrypted in the second version of the protocol!
  // Синхронибл в первых 20 битах не найден, такой пакет не расшифруешь во второй версии протокола!
  //  if (ver == 2 && csm > 22) return 0;
  // For the third version of the protocol, the number is different
  // ДЛя третьей версии протокола цифра иная
  //  if (ver == 3 && csm > 30) return 0;
  // Go to the beginning of reading
  // Переходим на начало считывания
  code += 4;
  int ii = 0;
  for (int i = 0; i < no_read_bits - csm; i++)
  {
    // Not to go out of bounds
    // Чтобы не выйти за пределы
    if (i >= packet_length * 4 || (ver == 2 && i > result_size / 2 - csm - 4) || (ver == 3 && i > result_size - csm - 4)) break;

    int multipl;
    switch (ii) {
      case 0: {
          multipl = 0x01;
          break;
        }
      case 1: {
          multipl = 0x02;
          break;
        }
      case 2: {
          multipl = 0x04;
          break;
        }
      case 3: {
          multipl = 0x08;
          break;
        }
    }

    if (*code > 128 ) {
      *rd += multipl;
      *vd += multipl;
    }
    if (*code < 128 ) *vd += multipl;
    code ++;
    ii ++;
    if (ii == 4) {
      ii = 0;
      vd++;
      rd++;
    }
  }
  //        Serial.println();
  //        Serial.println(result_size);
  //        Serial.println(csm);
  //        Serial.println((result_size/2 - csm - 4) / 4);

  return 1;
}








//
////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for decrypting data from sensors
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns the temperature value
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_temperature(byte* oregon_data) {

  float tmprt = 0;
  if (((sens_type & 0x0FFF) == RTGN318 ||
       (sens_type & 0x0FFF) == RTHN318 ||
       sens_type == THGR810 ||
       sens_type == THGN132 ||
       sens_type == THGN500 ||
       sens_type == THN132 ||
       sens_type == BTHGN129 ||
       sens_type == BTHR968  ||
       sens_type == THN800) && crc_c)
  {

    oregon_data += 8;
    //исправляем возможные ошибки:
    for (int g = 0; g < 4; g++)  if (*(oregon_data + g) > 9) *(oregon_data + g) = *(oregon_data + g) - 8;
    tmprt += *(oregon_data) * 0.1;
    tmprt += *(oregon_data + 1);
    tmprt += *(oregon_data + 2) * 10;
    if (*(oregon_data + 3)) tmprt = -tmprt;
  }
  return tmprt;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns the type of the sensor.
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
word Oregon_NR::get_sensor(byte* oregon_data) {

  return (word)(*(oregon_data)) * 0x1000 + (*(oregon_data + 1)) * 0x0100 + (*(oregon_data + 2)) * 0x10 + *(oregon_data + 3);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns the value of the channel
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_channel(byte* oregon_data) {
  if (crc_c)
  {
    byte channel = 0;
    //    word sens_type = get_sensor(oregon_data);
    if (sens_type == THGN132 || sens_type == THN132)
    {
      switch (*(oregon_data + 4))
      {
        case 1:
          channel = 1;
          break;
        case 2:
          channel = 2;
          break;
        case 4:
          channel = 3;
          break;
      }
    }
    if ((sens_type & 0x0FFF) == RTGN318 ||
        (sens_type & 0x0FFF) == RTHN318 ||
        (sens_type & 0x0FFF) == RFCLOCK ||
        sens_type == THGR810  ||
        sens_type == THN800 ||
        sens_type == BTHGN129 ||
        sens_type == BTHR968  ||
        sens_type == THGN500)
      channel = *(oregon_data + 4);
    return channel;
  }
  else return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_battery(byte* oregon_data) {
  if (((sens_type & 0x0FFF) == RTGN318 ||
       (sens_type & 0x0FFF) == RTHN318 ||
       (sens_type & 0x0FFF) == RFCLOCK ||
       sens_type == THGR810 ||
       sens_type == THGN132 ||
       sens_type == THGN500  ||
       sens_type == THN132 ||
       sens_type == BTHGN129 ||
       sens_type == BTHR968  ||
       sens_type == THN800 ||
       sens_type == WGR800 ||
       sens_type == PCR800 ||
       sens_type == UVN800) && crc_c)
    return (*(oregon_data + 7) & 0x4) ? 0 : 1;
  else  return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns the moisture value
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_humidity(byte* oregon_data) {

  if (((sens_type & 0x0FFF) == RTGN318 ||
       sens_type == THGR810 ||
       sens_type == BTHGN129 ||
       sens_type == BTHR968  ||
       sens_type == THGN132)  && crc_c ) {
    byte tmprt = 0;
    oregon_data += 12;
    // fix possible errors:
    for (int g = 0; g < 2; g++)  if (*(oregon_data + g) > 9) *(oregon_data + g) = *(oregon_data + g) - 8;
    tmprt = *(oregon_data);
    tmprt += *(oregon_data + 1) * 10;
    return (float)tmprt;
  }
  else return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns the id of the sensor
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_id(byte* oregon_data) {
  if (((sens_type & 0x0FFF) == RTGN318 ||
       (sens_type & 0x0FFF) == RTHN318 ||
       sens_type == THGR810 ||
       sens_type == THGN132 ||
       sens_type == THGN500  ||
       sens_type == THN132 ||
       sens_type == BTHGN129 ||
       sens_type == BTHR968  ||
       sens_type == THN800 ||
       sens_type == WGR800 ||
       sens_type == PCR800 ||
       sens_type == UVN800) && crc_c)

  {
    byte tmprt;
    oregon_data += 5;
    tmprt = *(oregon_data) * 0x10;
    tmprt += *(oregon_data + 1);
    return tmprt;
  }
  else return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns the average wind value in m / s
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_avg_windspeed(byte* oregon_data)
{
  if (sens_type == WGR800 && crc_c) {
    float tmprt;
    tmprt = *(oregon_data + 15) * 0x10;
    tmprt += *(oregon_data + 14);
    return tmprt / 10;
  }
  else return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns the value of the maximum wind gust in m / s
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_max_windspeed(byte* oregon_data)
{
  if (sens_type == WGR800 && crc_c) {
    float tmprt;
    tmprt = *(oregon_data + 12) * 0x10;
    tmprt += *(oregon_data + 11);
    return tmprt / 10;
  }
  else return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns wind direction in quadrants
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_winddirection(byte* oregon_data)
{
  if (sens_type == WGR800 && crc_c) {
    //D byte tmprt;
    return *(oregon_data + 8) & 0x0F;
    //Квадранты  0-N, 1-NNE, 2-NE, 3-ENE, 4-E, 5-ESE, 6-SE, 7-SSE, 8-S, 9-SSW, A-SW, B-WSW, C-W, D-WNW, E-NW,F-NNW
  }
  else return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns the UV index
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_UV(byte* oregon_data)
{
  if (sens_type == UVN800 && crc_c) {
    byte tmprt;
    tmprt = *(oregon_data + 9) * 0x10;
    tmprt += *(oregon_data + 8);
    return tmprt;
  }
  else return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns the illumination in arbitrary units
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_light(byte* oregon_data)
{
  if (sens_type == UVN800 && crc_c) {
    byte tmprt;
    tmprt = *(oregon_data + 11) * 0x10;
    tmprt += *(oregon_data + 10);
    tmprt -= 0x6F;
    return tmprt;
  }
  else return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns the pressure
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_pressure()
{
  if (sens_type == BTHGN129 && crc_c) {
    return (float)(*(packet + 15) + (*(packet + 16) << 4)  + ((*(packet + 17) & 0x01) << 8) + 545) * 0.75;
  }
  if (sens_type == BTHR968 && crc_c) {
    return (float)(*(packet + 15) + (*(packet + 16) << 4)  + ((*(packet + 17) & 0x01) << 8) + 600) * 0.75;
  }
  else return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns data from the rain sensor
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_total_rain()
{
  if (sens_type == PCR800 && crc_c) {
    float tmprt;
    tmprt = *(packet + 17) * 100000;
    tmprt += *(packet + 16) * 10000;
    tmprt += *(packet + 15) * 1000;
    tmprt += *(packet + 14) * 100;
    tmprt += *(packet + 13) * 10;
    tmprt += *(packet + 12);
    return tmprt * 0.0254;
  }
  else return 0;
}

float Oregon_NR::get_rain_rate()
{
  if (sens_type == PCR800 && crc_c) {
    float tmprt;
    tmprt  = *(packet + 8) * 1000;
    tmprt += *(packet + 9) * 100;
    tmprt += *(packet + 10) * 10;
    tmprt += *(packet + 11);
    return tmprt * 0.0254;
  }
  else return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// CRC check
// oregon_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Oregon_NR::check_CRC(byte* oregon_data, word sens_type) {

  if (sens_type == THN132)
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0XD6, 16, false) ;
  }

  if (sens_type == THGN132)
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0X3C, 19, false) ;
  }

  if (sens_type == THGN500)
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0XD8, 19, false) ;
  }

  if ((sens_type & 0x0FFF) == RTGN318)
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0X00, 19, false) ;
  }

  if ((sens_type & 0x0FFF) == RFCLOCK)
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0X00, 19, false) ;
  }

  if (sens_type == BTHGN129)
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0X00, 23, false) ;
  }

  if (sens_type == BTHR968)
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0XA1, 23, false) ;
  }

  if ((sens_type & 0x0FFF) == RTHN318)
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0X00, 16, false) ;
  }

  if (sens_type == THGR810)
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0X00, 19, true) ;
  }

  if (sens_type == UVN800 )
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0X0, 17, true) ;
  }


  if (sens_type == WGR800)
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0X00, 21, true) ;
  }

  if (sens_type == PCR800)
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0X00, 22, true) ;
  }

  if (sens_type == THN800)
  {
    return check_oregon_crcsum(oregon_data, 0X07, 0X00, 16, true) ;
  }

#if ADD_SENS_SUPPORT == 1

  if ((sens_type & 0xFF00) == GAS || (sens_type & 0xFF00) == THP || (sens_type & 0xFF00) == FIRE || (sens_type & 0xFF00) == CURRENT  || (sens_type & 0xFF00) == CAPRAIN)
  {
    return check_own_crcsum(oregon_data, 19) ;
  }

#endif

  return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Procedure for calculating CRC8 AND checksum for Oregon sensors
// oregon_data - a pointer to a code message
// CCIT_POLY - CRC generating polynomial
// CCIT_START - initial CRC value
// p_length - packet length
// v3 - set to 1 if the third version of the protocol
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Oregon_NR::check_oregon_crcsum(byte* oregon_data, byte CCIT_POLY, byte CCIT_START, byte p_length, bool v3)

{
  byte* pp = oregon_data;
  byte cksum = 0, crc = CCIT_START,  recived_cksum, recived_crc;
  for (int x = 0; x < p_length - 4; x++)
  {
    cksum += *pp;
    if ( v3 || (x != 5 && x != 6))
    {
      crc ^= *pp;
      for (byte i = 0; i < 4; i++)
        if (crc & 0x80) crc = (crc << 1) ^ CCIT_POLY;
        else crc <<= 1;
    }
    pp++;
  }

  for (byte i = 0; i < 4; i++)
    if (crc & 0x80) crc = (crc << 1) ^ CCIT_POLY;
    else crc <<= 1;

  recived_cksum = *pp + *(pp + 1) * 0x10;
  recived_crc = *(pp + 2) + *(pp + 3) * 0x10;
  YIELD();
  return (recived_crc == crc && recived_cksum == cksum) ? 1 : 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Procedure for calculating CRC8 AND checksum for Oregon sensors
// oregon_data - a pointer to a code message
// CCIT_POLY - CRC generating polynomial
// CCIT_START - initial CRC value
// p_length - packet length
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Oregon_NR::check_own_crcsum(byte* oregon_data, byte p_length)

{
  byte* pp = oregon_data;
  byte cksum = 0, crc = 0,  recived_cksum, recived_crc;
  for (int x = 0; x < p_length - 4; x++)
  {
    cksum += *pp;
    crc ^= *pp;
    for (byte i = 0; i < 4; i++)
      if (crc & 0x80) crc = (crc << 1) ^ 7;
      else crc <<= 1;
    pp++;
  }
  for (byte i = 0; i < 4; i++)
    if (crc & 0x80) crc = (crc << 1) ^ 7;
    else crc <<= 1;
  recived_cksum = *pp + *(pp + 1) * 0x10;
  recived_crc = *(pp + 2) + *(pp + 3) * 0x10;
  return (recived_crc == crc && recived_cksum == cksum) ? 1 : 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Restore data by sensor type
////////////////////////////////////////////////////////////////////////////////////////////////////
void Oregon_NR::restore_data(byte* oregon_data, word sens_type) {

  byte* pp = oregon_data;
  if (sens_type == THGN132) {
    pp += 8;
    for (int x = 0; x < 6; x++) {
      if (*pp > 9 && x != 3) *pp -= 8;
      pp++;
    }
  }
  if (sens_type == THN132 || sens_type == THN800) {
    pp += 8;
    for (int x = 0; x < 3; x++) {
      if (*pp > 9) *pp -= 8;
      pp++;
    }
  }
  return;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
void Oregon_NR::led_light(bool led_on) {
  if (LED != 0xFF) {
    if (PULL_UP && led_on) digitalWrite(LED, LOW);
    if (PULL_UP && !led_on) digitalWrite(LED, HIGH);
    if (!PULL_UP && led_on) digitalWrite(LED, HIGH);
    if (!PULL_UP && !led_on) digitalWrite(LED, LOW);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
//
#if ADD_SENS_SUPPORT == 1
////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for decoding data from GAS sensors
// gas_data - a pointer to a code message
////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_gas_channel(byte* gas_data) {

  return gas_data[2];
}
////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_gas_temperature_out(byte* gas_data) {

  int temperat = gas_data[9] * 0x0100 + gas_data[8] * 0x0010 + gas_data[7];
  return ((float)(-1000 + temperat)) / 10;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_gas_temperature_in(byte* gas_data) {

  int temperat = gas_data[12] * 0x0100 + gas_data[11] * 0x0010 + gas_data[10];
  return ((float)(-1000 + temperat)) / 10;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_gas_hmdty(byte* gas_data) {

  return gas_data[14] * 0x10 + gas_data[13];
}
////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_gas_CO(byte* gas_data) {

  return gas_data[6] * 0x10 + gas_data[5];
}
////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_gas_CH(byte* gas_data) {

  return gas_data[4] * 0x10 + gas_data[3];
}

////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_fire_ip22(byte* fire_data) {

  return fire_data[4] * 0x10 + fire_data[5];
}
////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_fire_ip72(byte* fire_data) {

  return fire_data[6] * 0x10 + fire_data[7];
}
////////////////////////////////////////////////////////////////////////////////////////////////////
byte Oregon_NR::get_fire_lockalarm(byte* fire_data) {

  return fire_data[8] * 0x10 + fire_data[9];
}


////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_current(byte* current_data) {

  return ((float)(current_data[4] * 0x1000 + current_data[5] * 0x0100  + current_data[6] * 0x0010  + current_data[7])) / 1000;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_voltage(byte* voltage_data) {

  return ((float)(voltage_data[8] * 0x1000 + voltage_data[9] * 0x0100  + voltage_data[10] * 0x0010  + voltage_data[11])) / 10;
}

word Oregon_NR::get_pump_count(byte* voltage_data) {

  return (voltage_data[12] * 0x0100 + voltage_data[13] * 0x0010  + voltage_data[14]);
}
unsigned long Oregon_NR::get_dropcounter(byte* packetdata) {

  return (packetdata[10] * 0x10000  + packetdata[11] * 0x1000  + packetdata[12] * 0x100 + packetdata[13] * 0x10 + packetdata[14]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int Oregon_NR::get_capacitance(byte* packetdata) {

  return (packetdata[6] * 0x1000  + packetdata[7] * 0x100  + packetdata[8] * 0x10 + packetdata[9]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_thp_temperature(byte* current_data) {

  return (float)(current_data[3] * 0x0100 + current_data[4] * 0x0010  + current_data[5] * 0x0001) / 10 - 100;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_thp_pressure(byte* current_data) {

  return (float)(current_data[9] * 0x0100 + current_data[10] * 0x0010  + current_data[11] * 0x0001) / 10 + 500;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_thp_voltage(byte* current_data) {

  return (float)(current_data[12] * 0x0100 + current_data[13] * 0x0010  + current_data[14] * 0x0001) * 0.01;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
float Oregon_NR::get_thp_humidity(byte* current_data) {

  return (float)(current_data[6] * 0x0100 + current_data[7] * 0x0010  + current_data[8] * 0x0001) / 10;
}
#endif
#if 0
/*
    Addendum to support Ambient Weather Module

    Manchester coding is used at the physical layer by this sensor.
    Clock rate is 1024Hz within a very small window of variation.

    The preamble contains a total of 13 bits; the first 11 are ones, followed by
    a 01 sequence.

    The next bit begins the data frame. Each data frame is six bytes.

    The entire message including preamble is sent three times with no delay
    between repetitions.

    All data is sent bigendian order, bits and bytes.

    Because the preamble is 13 bits, each successive message repetition is
    shifted one bit relative to byte or nibble boundaries.
    Extracting the repeated messages therefore requires a one or two-bit shift
    for the 2nd and 3rd copies respectively.

*/

// Hash calculation https://eclecticmusingsofachaoticmind.wordpress.com/2015/01/21/home-automation-temperature-sensors/

uint8_t  hashLFSR(int length, uint8_t *buff)
{
    uint8_t mask = 0x7C;
    uint8_t checksum = 0x64;
    uint8_t data;
    int byteCnt;

    for ( byteCnt=0; byteCnt < length; byteCnt++)
    {
        int bitCnt;
        data = buff[byteCnt];

        for ( bitCnt= 7; bitCnt >= 0 ; bitCnt-- )
        {
            uint8_t bit;

            // Rotate mask right
            bit = mask & 1;
            mask = (mask >> 1 ) | (mask << 7);
            if ( bit )
            {
                mask ^= 0x18;
            }

            // XOR mask into checksum if data bit is 1
            if ( data & 0x80 )
            {
                checksum ^= mask;
            }
            data <<= 1;
        }
    }
    return checksum;
}


#endif
