//Basic Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>


#include <SPI.h>
#include <SD.h>

//Screen
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <spl_logo.h>

//Servos
#include <Servo.h>

//IMU
#include <Adafruit_BNO055.h>
//Light sensor
#include <hp_BH1750_dual.h>


//RTC
#include "RTClib.h"

#define VERSION_STRING "Main V0.5"

RTC_PCF8523 rtc;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo my_azi_servo;
Servo my_alt_servo;

class Cubesat_Board {
private:
    const int power_up_delay = 100; //in milliseconds
    const int button_a_pin = 9;
    const int button_b_pin = 6;
    const int button_c_pin = 5;
    const int servo_en_pin = 13;
    const int azi_pin = 10;
    const int alt_pin = 11;
    const int hmi_dis_pin = A4;
    const int payload_dis_pin = A5;
    const int light_sensor_pin = A0;

    const int display_width = 64;
    const int display_height = 128;

    const byte light_addr[4] = { BH1750_TO_VCC,
                                BH1750_TO_VCC,
                                BH1750_TO_GROUND,
                                BH1750_TO_GROUND };

    const byte light_ch[4] = { 1,
                                0,
                                0,
                                1 };
    hp_BH1750 light_sensor[4];

public:
    Adafruit_SH1107* display;

    void payload_power(bool is_on) {
        digitalWrite(payload_dis_pin, !is_on);  //0: enable, 1:disable
        delay(power_up_delay);
    }
    void hmi_power(bool is_on) {
        digitalWrite(hmi_dis_pin, !is_on);  //0: enable, 1:disable
        delay(power_up_delay);
    }
    void servo_power(bool is_on) {
        digitalWrite(servo_en_pin, is_on);  //1: enable, 0:disable
        delay(power_up_delay);
    }

    bool read_button_a(void) {
        return !digitalRead(button_a_pin);
    }
    bool read_button_b(void) {
        return !digitalRead(button_b_pin);
    }
    bool read_button_c(void) {
        return !digitalRead(button_c_pin);
    }

    float read_analog_light(void) {
        return ((float)analogRead(light_sensor_pin)) / 1023.0;;
    }

    Cubesat_Board(void) {
        pinMode(button_a_pin, INPUT_PULLUP);
        pinMode(button_b_pin, INPUT_PULLUP);
        pinMode(button_c_pin, INPUT_PULLUP);

        pinMode(hmi_dis_pin, OUTPUT);
        pinMode(payload_dis_pin, OUTPUT);
        pinMode(servo_en_pin, OUTPUT);

        payload_power(0);
        servo_power(0);
        hmi_power(1);

        display = new Adafruit_SH1107(display_width, display_height, &Wire);
    }

    void begin_display(bool display_buffer) {
        display->begin(0x3C, true); // Address 0x3C default
        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(SH110X_WHITE);

        if (display_buffer) {
            display->setRotation(0);
            display->drawBitmap((display_width - imageWidth) / 2, (display_height - imageHeight - 7) / 2, bitmap, imageWidth, imageHeight, 1);
            display->setCursor(0, 120);
            display->println(VERSION_STRING);
            display->display();
            delay(1000);
        }
        display->setRotation(1);
        display->clearDisplay();
        display->display();
        display->setCursor(0, 0);
    }

    void begin_light_sensors(void) {
        byte t_addr, t_ch;

        for (byte ls = 0; ls < 4; ls++) {
            t_addr = light_addr[ls];
            t_ch = light_ch[ls];

            display->print(ls);
            display->print(":BH1750 ");
            display->print(t_addr == BH1750_TO_GROUND ? 'G' : 'V');
            display->print(t_ch);
            display->print('-');

            if (!light_sensor[ls].begin(t_addr, t_ch)) {
                display->println("X");
                display->display();
            }
            else {
                display->println("OK");
                display->display();
            }
            // actually display all of the above
        }
        delay(5000);
    }

    void begin(long baud_rate) {
        Serial.begin(baud_rate);
        delay(1000);

        begin_display(true); //if this is the first time running, it will display the splashscreen for 1s, and then clear it

        begin_light_sensors();
    }

    int get_lux_from_sensor_id(uint8_t id) {
        if (id < 4) {
            light_sensor[id].start();
            while (!light_sensor[id].hasValue(true)) {
                //kill time until it reads
            }
            return light_sensor[id].getLux();
        }
        else {
            return -1;
        }
    }
};