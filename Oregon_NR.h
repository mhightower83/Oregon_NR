#include <Arduino.h>
#ifndef Oregon_NR_h
#define Oregon_NR_h

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
//

// Recognize packets from the following Oregon Scientific sensors:
//
#define THGN132     0x1D20      // Temperature, humidity, 3 channels,
#define THGN500     0x1D30      // Temperature, humidity,
#define THN132      0xEC40      // Temperature, 3 channels,
#define RTGN318     0x0CC3      // Temperature, humidity, time, 5 channels,
#define RTHN318     0x0CD3      // Temperature, time, 5 channels,
#define RFCLOCK     0x0CF3      //
#define BTHGN129    0x5D53      // Temperature, humidity, pressure, 5 channels,
#define BTHR968     0x5D60      // Temperature, humidity, pressure,
#define THGR810     0xF824      // Temperature, humidity, 10 channels,
#define THN800      0xC844      // Temperature, 3 channels,
#define WGR800      0x1984      // Wind Direction And Speed
#define UVN800      0xD874      // UV index, illuminance (thanks to XOR for the data provided).
#define PCR800      0x2914      // precipitation counter

//
// Proprietary sensors:
#define THP         0x5500      // Temperature, humidity, atm pressure, 8 channels, operation from 3 AA batteries, (documentation is attached)
#define GAS         0xAA00      // Temperature, humidity, concentration of CO and CH4, 8 channels,
#define FIRE        0xBB00      // Control of signals of fire lines of sensors IP212-22 and IP212-72
#define CURRENT     0xEE00      // Current, voltage, 8 channels,
#define CAPRAIN     0x8800      // Capacitive all-weather precipitation sensor
//
// These parameters can be played around to save resources
#define  ADD_SENS_SUPPORT 1     // Support for additional types of sensors of our own design - disabling slightly saves RAM

//
// These parameters can be played around to tweak the best reception

#define MAX_LENGTH2 976         // The maximum pulse length of the preamble v2 is not less than the period to catch the signal from different emulators
#define MIN_LENGTH2 883         // Minimum pulse length when capturing (for v2 pulses are shortened by 93μs), i.e. should be no more than 883μs,

#define MAX_LENGTH3 488         // The maximum pulse length of the preamble v3 is at least half a period to catch a signal from different emulators
#define MIN_LENGTH3 330         // Minimum pulse length when capturing (for v2 pulses are shortened by 138μs), i.e. should be no more than 350μs,

#define LENGTH_TOLERANCE 100    // Tolerance for pulse length. Depends on the receiver
                                // Depending on the signal level, the pulse length may "float"

#define CATCH_PULSES 3          // How many to look for correct pulses to start capturing. Recommended 2 - 4.
                                // More - you don't need to catch the packet in a noisy environment
                                // Less - you can skip the packet, much distracted by the noise analysis
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Don't touch these parameters!

#define IS_ASSEMBLE 1           // Try to build one whole from two damaged packages (for v2) - disabling saves RAM a lot!


#define PACKET_LENGTH 26        // Packet length in nibls excluding preamble and sync,
                                // maximum for PCR800 is 22 nibls
                                // For capturing longer packets can be increased


#define READ_BITS ((PACKET_LENGTH + 8) * 4)

                                // Maximum packet length in bits
                                // preamble for v2 is 5 nibls (FFFFA), the longest packet is 19nibls (THGN132)
                                // total (19 + 5) * 4 = 96 bits
                                // preamble for v3 - 7 nibls (FFFFFFA), the longest packet is 22 nibls PCR900
                                // total (22 + 7) * 4 = 116 bits

#define READ_TACTS ((PACKET_LENGTH + 6) * 8)
                                // Maximum packet length per cycle
                                // v2 - 96 * 2 = 192
                                // v3 - 116 = 116




#define FIND_PACKET   1
#define ANALIZE_PACKETS 2
#define PER_LENGTH2 976         // Data transfer period. For v2 and v3 976.56μs (1024Hz)
#define PER_LENGTH3 488


static int RECEIVER_PIN;

class Oregon_NR
{
  public:

    int packet_length = PACKET_LENGTH;
    int no_read_bits = (PACKET_LENGTH + 8) * 4;
    int no_read_tacts = (PACKET_LENGTH + 6) * 8;
    bool is_assemble = IS_ASSEMBLE;
    bool no_memory = false;

    //Данные датчика
    word sens_type;               // Sensor type


    float sens_tmp,               // Temperature
    sens_hmdty;                   // Humidity

    byte sens_chnl,               // Channel number
    sens_id,                      // ID
    sens_battery;                 // Battery status

    byte ver = 0;                 // Protocol version

    bool crc_c = 0;               // CRC check result. Resets on capture. Exposed when the correct packet is received.
    bool captured = 0;            // Capture data flag. Exposed if data was read into memory.

    unsigned long work_time;      // Capture time
    byte* packet;                 // Result packet
    byte* valid_p;                // Validity mask - mask of confident bit recognition
    byte packets_received = 0;    // Number of received packets in block (0 ... 2)
    byte received_CRC;            // Calculated СRC

    Oregon_NR(byte, byte);        // Constructor. Parameters:
    Oregon_NR(byte, byte, byte, bool);             // (receiver pin, interrupt number, LED pin, pull up)
    Oregon_NR(byte, byte, byte, bool, int, bool);  // (receiver pin, interrupt number, LED pin, pull up, buffer size)
    void start();                 // Start listening receiver
    void stop();                  // Stop listening receiver. So as not to occupy the processor when not needed
    void capture(bool);           // Capture packet. if parameter is true function dumps capture data to Serial.

    bool consist_synchro = false; // When searching for synchronization, should we rely on confirmed or questionable data?

    byte empty_space = 3;         // How many "empty" measures are needed to determine the end of the message?
                                  // The parameter is determined by the signal level and the AGC speed of the receiver.
                                  // The better they are, the lower the number. BUT less than two is not recommended
                                  // Satra version had 5
    bool catch2 = 1, catch3 = 1;  // which protocol versions to accept
    int timing_correction = 0;    // Correction of the receiving frequency (from -10 to +10)
    byte decode_method = 3;       // Method for decoding clock cycles
                                  // 1 - traditional
                                  // 3 - with frequency deviation

    // Wind meter
    float sens_avg_ws, sens_max_ws;
    byte  sens_wdir;
    float get_avg_windspeed(byte*);
    float get_max_windspeed(byte*);
    byte get_winddirection(byte*);
    // UV
    byte UV_index, lightness;
    byte get_UV(byte*);
    byte get_light(byte*);
    // precipitation counter
    float get_total_rain();
    float get_rain_rate();
    // Barometer
    float get_pressure();

    byte restore_sign;            // Bit field informing about successful methods of package restoration

                                  // 0 - restored single ticks
                                  // 1 - double ticks restored
                                  // 2 - fix the protocol version when disassembling the package
                                  // 3 - restored by splice method (v2) - disabled to save resources

    bool receiver_dump = 0;       // Whether to dump the channel to Serial. only works if capture (true)
                                  // in fact, this is an oscillogram of the envelope of the signal from the receiver
                                  // Also output clock sequences before and after recovery

    unsigned long pulse1, pulse2, pulse3;

#if ADD_SENS_SUPPORT == 1
    float sens_pressure,          // Pressure
    sens_voltage,                 // voltage in V (for CURRENT and THP sensors)
    sens_tmp2;                    // Temperature2 (for GASv2 sensor)
    byte sens_CO,                 // CO (ppm * 10) (for GASv2 sensor)
    sens_CH;                      // CH4 (ppm * 100) (ppm)
    byte sens_ip22,               // IP22 channel data (for FIRE sensor)
    sens_ip72,                    // IP72 channel data (for FIRE sensor)
    sens_lockalarm;               // LOCK_ALARM channel data (for FIRE sensor)
    float sens_current;           // current in A (for CURRENT sensor)

    word  sens_pump_count;         // pump counter
    unsigned long sens_drop_counter;// drop counter (for CAPRAIN sensor)
    int sens_capacitance;         // Sensor capacity (for CAPRAIN sensor)
#endif
    bool check_oregon_crcsum(byte*, byte , byte , byte, bool );

  private:


    byte read_tacts, read_tacts2, result_size;
    byte LED = 0xFF;            // LED output that blinks when receiving
    bool PULL_UP;               // where the LED is connected
    byte packet_number = 0;     // The number of received packets in the parcel
    int INT_NO;                 // Receiver interrupt number
    // bool reciever_ctrl = true; // Receiver control flag (set when an impulse arrives, cleared in the timer)

   // Data arrays for recording data from the channel and received bits
    byte* decode_tacts;        // Array of ticks. meaning
    //                          0 = zero
    //                          1 = unit
    //                          2 = unknown
    //                          3 = transition +
    //                          4 = transition

    byte* collect_data;         // Memory for collecting data from the receiver
#if IS_ASSEMBLE
    byte* collect_data2;
#else
    byte* collect_data2;
#endif
    // And when it becomes an array of received bits, then the values ​​are:
    //                             128 - unknown
    //                           > 128 - one
    //                           < 128 - zero

    byte receive_status = FIND_PACKET;
    byte start_pulse_cnt = 0;
    unsigned long pulse_length, timer_marklong;
    unsigned long pulse_marker, right_pulse_marker;
    unsigned long pre_marker;       // To store preamble timestamps when capturing a packet
    unsigned long first_packet_end;
    int data_val, data_val2;        // Package quality
    int synchro_pos, synchro_pos2;  // Positions of synchromes in the record

    byte get_gas_CH(byte* gas_data);
    byte get_gas_CO(byte* gas_data);
    byte get_gas_hmdty(byte* gas_data);
    float get_gas_temperature_in(byte* gas_data);
    float get_gas_temperature_out(byte* gas_data);
    byte get_gas_channel(byte* gas_data);
    void restore_data(byte* oregon_data, word sensor_type);
    bool check_CRC(byte* oregon_data, word sensor_type);
    bool check_own_crcsum(byte* , byte);
    byte get_id(byte* oregon_data);
    float get_humidity(byte* oregon_data);
    byte get_battery(byte* oregon_data);
    byte get_channel(byte* oregon_data);
    word get_sensor(byte* oregon_data);
    float get_temperature(byte* oregon_data);
    int get_info_data(byte* code, byte* result, byte* valid);
    void assemble_data(byte* s1, byte* s2, int shift);
    int correlate_data(byte* ser1, byte* ser2);
    int collect(byte* cdptr);
    int get_data(int btt, byte p_ver, byte* cdptr);
    void get_tacts(byte*, int);
    int get_synchro_pos(byte* code);
    void led_light(bool);

#if ADD_SENS_SUPPORT == 1
    byte get_fire_ip22(byte* fire_data);
    byte get_fire_ip72(byte* fire_data);
    byte get_fire_lockalarm(byte* fire_data);
    float get_current(byte* curent_data);
    float get_voltage(byte* voltage_data);
    word  get_pump_count(byte* voltage_data);
    unsigned long get_dropcounter(byte* packetdata);
    int get_capacitance(byte* packetdata);
    float get_thp_humidity(byte* oregon_data);
    float get_thp_temperature(byte* oregon_data);
    float get_thp_pressure(byte* oregon_data);
    float get_thp_voltage(byte* oregon_data);
#endif

};

#endif
