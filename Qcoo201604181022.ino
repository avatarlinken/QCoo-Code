 #include <EEPROM.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include "Adafruit_NeoPixel.h"
#include <avr/wdt.h>
//************************const variable for hardware*********************************//

#define MAX_LED_NUM       75
#define LED_PIN           6
#define LED_PIN2          5
#define ONE_FACE_LED_NUM  25
#define BRTS         10
#define BCTS         9
#define POWER_ON_OFF      7
#define SHARE_MOTOR_PORT       3
#define VOLTAGE_DETECT_PORT    A1
#define CHARGE_PORT            2
#define STANDBY_PORT           4
#define IS_IN_CHARGE           A3
#define GET 1
#define RUN 2
#define RESET 4
#define START 5

#define MPU_ADDR 0x68
#define MEM_START_ADDR 0x6E
#define MEM_R_W 0x6F



int deviceState = 0;
int is_dice_mode = 0;
int initKey = 0, address = 0;
int off_brightness = 60;
int off_delaytime = 1000;
Adafruit_NeoPixel led = Adafruit_NeoPixel(MAX_LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800 );
Adafruit_NeoPixel led2 = Adafruit_NeoPixel(MAX_LED_NUM, LED_PIN2, NEO_GRB + NEO_KHZ800 );
//Command
#define SET_INDEX_COLOR                  0x01
#define READ_INDEX_COLOR                 0x02
#define RGB_SHOW                         0x03
#define RGB_BRIGHTNESS                   0x04
#define RGB_SET_FX                       0x05

#define SAVE_BRIGHTNESS                  0x44

#define MPU_INIT                         0x43

#define DIC_MODE                         0x42

#define RGB_SET_COLOR_0                  0x41
#define RGB_SET_COLOR_1                  0x11
#define RGB_SET_COLOR_2                  0x12
#define RGB_SET_COLOR_3                  0x13
#define RGB_SET_COLOR_4                  0x14
#define POWER_OFF                        0x15
#define SHAKE_MOTOR                      0x16
#define VOLTAGE_DETECT                   0x17
#define CHARG_STATUS                     0x18
#define GET_BRIGHTNESS                   0x19
#define CONNECTION_STATE                 0X20

//*******value for dic ************************//

int seed[] =
{
    1, 2, 3, 4, 5, 6
};
int randomTmp[] =
{
    0, 0, 0, 0, 0, 0
};

int randomNum = 1;

int timecount = 0;

float tmpx = 0;
float tmpy = 0;
float tmpz = 0;

int diceAnKey = 0;
int diceAnCount = 0;
int diceAnLengh = 90;
int diceAnflame = 0;
//*******value for dic ************************//

uint8_t device_is_in_charge = 0;

uint8_t plugin_state = 0;
uint8_t fx_value = 0;
uint8_t last_fx_state = 0;
uint8_t dice_on = 1;
uint8_t brightness_value = 60;
uint8_t keep_time = 0;
int time_to_end_motor = 0;
int16_t bri = 0, charge_bri = 0, st = 0, charge_st = 0, bat_count = 0;
int need_to_be_charged = 0;
float bat_voltage = 0;

int dic_line[6][4] =
{
    {
        0, 0, 16, 0
    }
    ,
    {
        0, 1, 1, 0
    }
    ,
    {
        0, 1, 8, 64
    }
    ,
    {
        0, 5, 1, 64
    }
    ,
    {
        0, 5, 17, 64
    }
    ,
    {
        0, 10, 130, 160
    }
};

int rect_line[5][4]=
{
    {1, 0, 192, 48},
    {0, 129, 32, 72},
    {0, 66, 16, 132},
    {0, 36, 9, 2},
    {0, 24, 6, 1},
};
int arrows[3][4]=
{
    {0, 69, 68, 0},
    {0, 2, 42, 32},
    {0, 0, 17, 81}
};
int re_arrows[3][4]=
{
    {0, 0, 69, 68},
    {0, 8, 168, 128},
    {1, 21, 16, 0}
};
uint8_t color_tab[29][3] =
{
    {
        0x00, 0x00, 0x00
    }
    ,
    {
        242, 88, 30
    }
    ,
    {
        242, 147, 37
    }
    ,
    {
        242, 207, 46
    }
    ,
    {
        214, 242, 51
    }
    ,
    {
        155, 240, 47
    }
    ,
    {
        98, 241, 45
    }
    ,
    {
        52, 240, 44
    }
    ,
    {
        39, 240, 54
    }
    ,
    {
        39, 241, 97
    }
    ,
    {
        40, 241, 152
    }
    ,
    {
        41, 242, 209
    }
    ,
    {
        36, 215, 241
    }
    ,
    {
        26, 156, 239
    }
    ,
    {
        17, 97, 239
    }
    ,
    {
        11, 47, 239
    }
    ,
    {
        32, 34, 239
    }
    ,
    {
        88, 34, 240
    }
    ,
    {
        147, 35, 240
    }
    ,
    {
        206, 36, 239
    }
    ,
    {
        240, 34, 213
    }
    ,
    {
        240, 24, 253
    }
    ,
    {
        240, 18, 96
    }
    ,
    {
        240, 13, 43
    }
    ,
    {
        240, 32, 27
    }
    ,
    {
        255, 0, 0
    }
    ,
    {
        0, 255, 0
    }
    ,
    {
        0, 0, 255
    }
    ,
    {
        255, 255, 255
    }
    ,
};

int X_energy = 0, Y_energy = 0, Z_energy = 0;
long power_off_count = 0;
//******************variable for commands**********************//
 
union
{
    byte byteVal[4];
    float floatVal;
    long longVal;
}
val;

String mVersion = "2.0.2";
boolean isAvailable = false;

int len = 52;
char buffer[52];
byte index = 0;
byte dataLen;
byte modulesLen = 0;
boolean isStart = false;
unsigned char irRead;
char serialRead;

unsigned char prevc = 0;
long lastTime = 0;
long dic_last_time = 0;
long currentTime = 0;
//******************variable for commands**********************//

int fx_vriable = 0;
uint8_t fx_idx ;
float gyro [3];         // [gyrox,gyroy,gyroz]
MPU6050 mpu;
byte received_packet[50];
byte temp = 0;
byte fifoCountL = 0;
byte fifoCountL2 = 0;
byte packetCount = 0x00;
boolean longPacket = false;
boolean firstPacket = true;

unsigned const char dmpMem[8][16][16] PROGMEM =
{
    {
        {
            0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00
        }
        ,
        {
            0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01
        }
        ,
        {
            0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01
        }
        ,
        {
            0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00
        }
        ,
        {
            0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00
        }
        ,
        {
            0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82
        }
        ,
        {
            0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00
        }
        ,
        {
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0
        }
        ,
        {
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC
        }
        ,
        {
            0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4
        }
        ,
        {
            0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10
        }
    }
    ,
    {
        {
            0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8
        }
        ,
        {
            0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7
        }
        ,
        {
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C
        }
        ,
        {
            0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C
        }
        ,
        {
            0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0
        }
    }
    ,
    {
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
        ,
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        }
    }
    ,
    {
        {
            0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F
        }
        ,
        {
            0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2
        }
        ,
        {
            0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF
        }
        ,
        {
            0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C
        }
        ,
        {
            0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1
        }
        ,
        {
            0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01
        }
        ,
        {
            0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80
        }
        ,
        {
            0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C
        }
        ,
        {
            0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80
        }
        ,
        {
            0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E
        }
        ,
        {
            0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9
        }
        ,
        {
            0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24
        }
        ,
        {
            0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0
        }
        ,
        {
            0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86
        }
        ,
        {
            0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1
        }
        ,
        {
            0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86
        }
    }
    ,
    {
        {
            0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA
        }
        ,
        {
            0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C
        }
        ,
        {
            0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8
        }
        ,
        {
            0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3
        }
        ,
        {
            0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84
        }
        ,
        {
            0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5
        }
        ,
        {
            0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3
        }
        ,
        {
            0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1
        }
        ,
        {
            0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5
        }
        ,
        {
            0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D
        }
        ,
        {
            0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9
        }
        ,
        {
            0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D
        }
        ,
        {
            0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9
        }
        ,
        {
            0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A
        }
        ,
        {
            0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8
        }
        ,
        {
            0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87
        }
    }
    ,
    {
        {
            0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8
        }
        ,
        {
            0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68
        }
        ,
        {
            0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D
        }
        ,
        {
            0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94
        }
        ,
        {
            0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA
        }
        ,
        {
            0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56
        }
        ,
        {
            0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9
        }
        ,
        {
            0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA
        }
        ,
        {
            0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A
        }
        ,
        {
            0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60
        }
        ,
        {
            0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97
        }
        ,
        {
            0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04
        }
        ,
        {
            0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78
        }
        ,
        {
            0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79
        }
        ,
        {
            0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68
        }
        ,
        {
            0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68
        }
    }
    ,
    {
        {
            0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04
        }
        ,
        {
            0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66
        }
        ,
        {
            0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31
        }
        ,
        {
            0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60
        }
        ,
        {
            0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76
        }
        ,
        {
            0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56
        }
        ,
        {
            0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD
        }
        ,
        {
            0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91
        }
        ,
        {
            0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8
        }
        ,
        {
            0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE
        }
        ,
        {
            0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9
        }
        ,
        {
            0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD
        }
        ,
        {
            0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E
        }
        ,
        {
            0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8
        }
        ,
        {
            0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89
        }
        ,
        {
            0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79
        }
    }
    ,
    {
        {
            0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8
        }
        ,
        {
            0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA
        }
        ,
        {
            0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB
        }
        ,
        {
            0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3
        }
        ,
        {
            0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3
        }
        ,
        {
            0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3
        }
        ,
        {
            0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3
        }
        ,
        {
            0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC
        }
        ,
        {
            0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
        }
    }
};

//DMP update transmissions (Bank, Start Address, Update Length, Update Data...)

static byte dmp_updates[29][9] =
{
    {
        0x03, 0x7B, 0x03, 0x4C, 0xCD, 0x6C
    } //FCFG_1 inv_set_gyro_calibration
    ,
    {
        0x03, 0xAB, 0x03, 0x36, 0x56, 0x76
    } //FCFG_3 inv_set_gyro_calibration
    ,
    {
        0x00, 0x68, 0x04, 0x02, 0xCB, 0x47, 0xA2
    } //D_0_104 inv_set_gyro_calibration
    ,
    {
        0x02, 0x18, 0x04, 0x00, 0x05, 0x8B, 0xC1
    } //D_0_24 inv_set_gyro_calibration
    ,
    {
        0x01, 0x0C, 0x04, 0x00, 0x00, 0x00, 0x00
    } //D_1_152 inv_set_accel_calibration
    ,
    {
        0x03, 0x7F, 0x06, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97
    } //FCFG_2 inv_set_accel_calibration
    ,
    {
        0x03, 0x89, 0x03, 0x26, 0x46, 0x66
    } //FCFG_7 inv_set_accel_calibration
    ,
    {
        0x00, 0x6C, 0x02, 0x20, 0x00
    } //D_0_108 inv_set_accel_calibration
    ,
    {
        0x02, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00
    } //CPASS_MTX_00 inv_set_compass_calibration
    ,
    {
        0x02, 0x44, 0x04, 0x00, 0x00, 0x00, 0x00
    } //CPASS_MTX_01
    ,
    {
        0x02, 0x48, 0x04, 0x00, 0x00, 0x00, 0x00
    } //CPASS_MTX_02
    ,
    {
        0x02, 0x4C, 0x04, 0x00, 0x00, 0x00, 0x00
    } //CPASS_MTX_10
    ,
    {
        0x02, 0x50, 0x04, 0x00, 0x00, 0x00, 0x00
    } //CPASS_MTX_11
    ,
    {
        0x02, 0x54, 0x04, 0x00, 0x00, 0x00, 0x00
    } //CPASS_MTX_12
    ,
    {
        0x02, 0x58, 0x04, 0x00, 0x00, 0x00, 0x00
    } //CPASS_MTX_20
    ,
    {
        0x02, 0x5C, 0x04, 0x00, 0x00, 0x00, 0x00
    } //CPASS_MTX_21
    ,
    {
        0x02, 0xBC, 0x04, 0x00, 0x00, 0x00, 0x00
    } //CPASS_MTX_22
    ,
    {
        0x01, 0xEC, 0x04, 0x00, 0x00, 0x40, 0x00
    } //D_1_236 inv_apply_endian_accel
    ,
    {
        0x03, 0x7F, 0x06, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97
    } //FCFG_2 inv_set_mpu_sensors
    ,
    {
        0x04, 0x02, 0x03, 0x0D, 0x35, 0x5D
    } //CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
    ,
    {
        0x04, 0x09, 0x04, 0x87, 0x2D, 0x35, 0x3D
    } //FCFG_5 inv_set_bias_update
    ,
    {
        0x00, 0xA3, 0x01, 0x00
    } //D_0_163 inv_set_dead_zone
    ,
    //SET INT_ENABLE at i=22
    {
        0x07, 0x86, 0x01, 0xFE
    } //CFG_6 inv_set_fifo_interupt
    ,
    {
        0x07, 0x41, 0x05, 0xF1, 0x20, 0x28, 0x30, 0x38
    } //CFG_8 inv_send_quaternion
    ,
    {
        0x07, 0x7E, 0x01, 0x30
    } //CFG_16 inv_set_footer
    ,
    {
        0x07, 0x46, 0x01, 0x9A
    } //CFG_GYRO_SOURCE inv_send_gyro
    ,
    {
        0x07, 0x47, 0x04, 0xF1, 0x28, 0x30, 0x38
    } //CFG_9 inv_send_gyro -> inv_construct3_fifo
    ,
    {
        0x07, 0x6C, 0x04, 0xF1, 0x28, 0x30, 0x38
    } //CFG_12 inv_send_accel -> inv_construct3_fifo
    ,
    {
        0x02, 0x16, 0x02, 0x00, 0x0A
    } //D_0_22 inv_set_fifo_rate
};

//*******************functions of protocol********************//
uint8_t readBuffer(int index)
{
    return buffer[index];
}

void writeBuffer(int index, unsigned char c)
{
    buffer[index] = c;
}
void writeHead(uint8_t mode)
{
    writeSerial(0xf0);
    writeSerial(0x55);
    writeSerial(mode);
}
void writeEnd()
{
    Serial.println();
}
void writeSerial(unsigned char c)
{
    Serial.write(c);
}
void readSerial()
{
    isAvailable = false;
    if (Serial.available() > 0)
    {
        if(initKey == 0)
        {
            initKey = 1;
            is_dice_mode = 1;
            dice_on = 0;
            deviceState = 1;
            // all_off();
        }
        isAvailable = true;
        serialRead = Serial.read();
    }
}
void parseData()
{

    isStart = false;
    int idx = readBuffer(3);
    int action = readBuffer(4);
    int device = readBuffer(5);

    switch (action)
    {
    case GET:
    {
        writeHead(0x01);

        writeSerial(idx);
        readSensor(device);
        writeEnd();
    }
    break;
    case RUN:
    {
        runModule(device);
        fx_idx = idx;
    }
    break;
    }
}
void callOK(uint8_t index, uint8_t errorcode)
{
    // if(digitalRead(BCTS) == 0){
    writeSerial(0xf0);
    writeSerial(0x55);
    writeSerial(0x01);
    writeSerial(index);
    writeSerial(errorcode);
    writeEnd();
    // }

}
void sendByte(char c)
{
    writeSerial(0x01);
    writeSerial(c);
}

void sendFloat(float value)
{
    writeSerial(0x2);
    val.floatVal = value;
    writeSerial(val.byteVal[0]);
    writeSerial(val.byteVal[1]);
    writeSerial(val.byteVal[2]);
    writeSerial(val.byteVal[3]);
}
void runModule(int device)
{
    switch (device)
    {
    case SAVE_BRIGHTNESS:
    {
        uint8_t brightness = readBuffer(6);
        if(brightness<=10){
            return;
        }
        EEPROM.write(address, brightness);
        callOK(fx_idx, 0x10);
    }
    break;
    case MPU_INIT:
    {
        uint8_t init_state = readBuffer(6);
        if (init_state == 0x01)
        {
            mem_init();
            callOK(fx_idx, 0x10);//////////////////////////////////////////
        }

    }
    break;
    case DIC_MODE:
    {
        uint8_t dice_switch = readBuffer(6);
        if (dice_switch == 0x01)
        {
            mem_init();
            dice_on = 1;
            is_dice_mode = 0;
            deviceState = 0;
            callOK(fx_idx, 0x10);//////////////////////////////////////////
            // Serial.println("hi bro you have got the dice on then go ahead");
            // is_dice_mode = 1;
            // dice_on = 0;
            // deviceState=1;


        }
        if (dice_switch == 0x00)
        {
            is_dice_mode = 1;
            dice_on = 1;
            deviceState = 1;
            all_off();
            callOK(fx_idx, 0x10);//////////////////////////////////////////
            Serial.println("yewll so terriable");
        }
    }
    break;

    case RGB_SHOW:
    {
        led.show();
        led2.show();

    }
    break;

    case RGB_BRIGHTNESS:
    {
        brightness_value = readBuffer(6);
        if(brightness_value <=10)
        {
            brightness_value = 10;
        }
        led2.setBrightness(brightness_value);
        led.setBrightness(brightness_value);
        led.show();
        led2.show();
        callOK(fx_idx, 0x10);//////////////////////////////////////////
    }
    break;
    case RGB_SET_FX:///////////////////////////////////////////////////////////////////////////////////set effect
    {
        fx_value = readBuffer(6);
        all_off();
        led.setBrightness(brightness_value);
        led2.setBrightness(brightness_value);
        callOK(fx_idx, 0x10);//////////////////////////////////////////
    }
    break;

    case RGB_SET_COLOR_0:
    {
        uint8_t color_num = readBuffer(6);
        uint32_t led_index = readBuffer(7);
        // Serial.println(led_index);
        if (led_index > MAX_LED_NUM)
        {
            led_index = led_index - 75;
            // all_off();
            led2.setPixelColor(led_index - 1 , color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
            led2.show();
        }
        else
        {
            // all_off();
            led.setPixelColor(led_index - 1 , color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
            led.show();
        }
    }
    break;
    case RGB_SET_COLOR_1:
    {
        uint8_t face_num = readBuffer(6);
        uint8_t color_num = readBuffer(7);
        uint32_t led_index1 = readBuffer(8);
        uint32_t led_index2 = readBuffer(9);
        uint32_t led_index3 = readBuffer(10);
        uint32_t led_index4 = readBuffer(11);
        uint8_t led_num_start = (face_num - 1) * 25;
        uint32_t led_index = (led_index1 << 24) + (led_index2 << 16) + (led_index3 << 8) + led_index4;
        uint32_t led_index_temp;
        // all_off();
        if (led_num_start >= MAX_LED_NUM)
        {
            led_num_start = led_num_start - 75;
            for (uint8_t i = 0; i < ONE_FACE_LED_NUM; i++)
            {

                led_index_temp = (led_index >> i) & 0x01;
                if (led_index_temp == 1)
                {

                    led2.setPixelColor(led_num_start + i, color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
                    // led2.show();
                }
            }
        }
        else
        {
            for (uint8_t i = 0; i < ONE_FACE_LED_NUM; i++)
            {
                led_index_temp = (led_index >> i) & 0x01;
                if (led_index_temp == 1)
                {
                    led.setPixelColor(led_num_start + i, color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);

                }
            }
        }
        led.show();
        led2.show();
    }
    break;
    case RGB_SET_COLOR_2:
    {
        uint8_t face_num = readBuffer(6);
        uint8_t color_num = readBuffer(7);
        uint8_t led_num_start = (face_num - 1) * 25;
        if (led_num_start >= MAX_LED_NUM)
        {
            led_num_start = led_num_start - 75;
            for (uint8_t i = 0; i < ONE_FACE_LED_NUM; i++)
            {
                led2.setPixelColor(led_num_start + i, color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
            }
        }
        else
        {
            for (uint8_t i = 0; i < ONE_FACE_LED_NUM; i++)
            {
                led2.setPixelColor(led_num_start + i, color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
            }
        }
    }
    break;
    case RGB_SET_COLOR_3:
    {
        uint8_t color_num = readBuffer(6);
        for (uint8_t i = 0; i < MAX_LED_NUM; i++)
        {
            led.setPixelColor(i, color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
            led2.setPixelColor(i, color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
        } 
    }
    break;
    case RGB_SET_COLOR_4:
    {
        uint8_t color_num = readBuffer(6);
        for (uint8_t i = 0; i < MAX_LED_NUM; i++)
        {
            led.setPixelColor(i, color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
            led2.setPixelColor(i, color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
        }
        led.show();
        led2.show();
    }
    break;
    case POWER_OFF:
    {
        uint8_t power_val = readBuffer(6);
        if (power_val == 0x0a && device_is_in_charge == 0)
        {
            led.setBrightness(60);
            led2.setBrightness(60);
            for (uint8_t i = 0; i < MAX_LED_NUM; i++)
            {
                led.setPixelColor(i, off_brightness, 0, 0);
                led2.setPixelColor(i, off_brightness, 0, 0);
            }
            led.show();
            led2.show();
            delay(off_delaytime);
            digitalWrite(POWER_ON_OFF, HIGH);
        }
    }
    break;
    case SHAKE_MOTOR:
    {
        uint8_t speed_val = readBuffer(6);
        keep_time = readBuffer(7);
        time_to_end_motor = keep_time * 50;
        // Serial.println(speed_val);
        analogWrite(SHARE_MOTOR_PORT, speed_val);
        // Serial.println("fucking in");
        callOK(fx_idx, 0x10);//////////////////////////////////////////

        mem_init();
    }
    break;
    case VOLTAGE_DETECT:
    {
        sendFloat(bat_voltage);
    }
    break;
    case CHARG_STATUS:
    {
        // uint8_t charge_val = digitalRead(CHARGE_PORT);
        // uint8_t standby_val = digitalRead(STANDBY_PORT);
        // uint8_t charge_status = charge_val || standby_val;
        // sendByte(charge_status);
        Serial.println(digitalRead(IS_IN_CHARGE));
    }
    case GET_BRIGHTNESS:
    {
        sendByte(brightness_value);
    }
    break;
    case CONNECTION_STATE:
    {
        uint8_t connect_state = readBuffer(6);
        if (connect_state == 0x01)
        {
            callOK(fx_idx, 0x10);//////////////////////////////////////////
            deviceState = 1;
            is_dice_mode = 1;
            all_off();
            led.setBrightness(brightness_value);
            led2.setBrightness(brightness_value);

        }
        if (connect_state == 0x00)
        {
            deviceState = 0;
            callOK(fx_idx, 0x10);//////////////////////////////////////////
        }

    }
    break;
    default:
        break;
    }
}
void readSensor(int device)
{

}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait)
{
    // if (fx_value != 1)
    // {
    //   return;
    // }
    for (uint16_t i = 0; i < 2 * led.numPixels(); i++)
    {
        if (i >= MAX_LED_NUM)
        {
            readSerial();
            if (isAvailable)
            {
                read_serial();
                // goto outside1;
                return;
            }
            led2.setPixelColor(i - MAX_LED_NUM, c);
            led2.show();
        }
        else
        {
            readSerial();
            if (isAvailable)
            {
                read_serial();
                //goto outside1;
                return;
            }
            led.setPixelColor(i, c);
            led.show();
        }
        readSerial();
        if (isAvailable)
        {
            read_serial();
            return;
        }
        delay(wait);
    }
}

void rainbow(uint8_t wait)
{
    outside3:
    if (fx_value != 3)
    {
        return;
    }
    uint16_t i, j;
    for (j = 0; j < 256; j++)
    {
        for (i = 0; i < 2 * led.numPixels(); i++)
        {
            if (i >= MAX_LED_NUM)
            {
                led2.setPixelColor(i - MAX_LED_NUM, Wheel((i + j) & 255));
                readSerial();
                if (isAvailable)
                {
                    read_serial();
                    goto outside3;
                    return;
                }
            }
            else
            {
                led.setPixelColor(i, Wheel((i + j) & 255));
                readSerial();
                if (isAvailable)
                {
                    read_serial();
                    goto outside3;
                    return;
                }
            }
        }
        led.show();
        led2.show();
        readSerial();
        if (isAvailable)
        {
            read_serial();
            goto outside3;
            return;
        }
        delay(wait);
    }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait)
{
    outside4:
    uint16_t i, j;
    if (fx_value != 4)
    {
        return;
    }
    for (j = 0; j < 256 * 5; j++)
    {
        // 5 cycles of all colors on wheel
        for (i = 0; i < 2 * led.numPixels(); i++)
        {
            if (i >= MAX_LED_NUM)
            {
                led2.setPixelColor((i - MAX_LED_NUM), Wheel(((i * 256 / led.numPixels()) + j) & 255));
                readSerial();
                if (isAvailable)
                {
                    read_serial();
                    goto outside4;
                    return;
                }
            }
            else
            {
                led.setPixelColor(i, Wheel(((i * 256 / led.numPixels()) + j) & 255));
                readSerial();
                if (isAvailable)
                {
                    read_serial();
                    goto outside4;
                    return;
                }
            }
            wdt_reset();
        }
        led.show();
        led2.show();
        readSerial();
        if (isAvailable)
        {
            read_serial();
            goto outside4;
            return;
        }
        delay(wait);
    }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait)
{
    // outside2: //fx_value = 0;
    if (fx_value != 2)
    {
        Serial.println("crash here");
        return;
    }
    for (int j = 0; j < 10; j++)
    {
        for (int q = 0; q < 3; q++)
        {
            for (int i = 0; i < 2 * led.numPixels(); i = i + 3)
            {
                if (i >= MAX_LED_NUM)
                {
                    led2.setPixelColor((i - MAX_LED_NUM) + q, c);
                }
                else
                {
                    led.setPixelColor(i + q, c);
                }
                readSerial();
                read_serial();
            }
            led.show();
            led2.show();
            readSerial();
            if (isAvailable)
            {
                read_serial();
                return;
            }
            delay(wait);//UNPOPULAR DELAY
            for (int i = 0; i < 2 * led.numPixels(); i = i + 3)
            {
                if (i >= MAX_LED_NUM)
                {
                    led2.setPixelColor((i - MAX_LED_NUM) + q, c);
                }
                else
                {
                    led.setPixelColor(i + q, c);
                }
            }

        }
    }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait)
{
    outside5:
    if (fx_value != 5)
    {
        return;
    }
    for (int j = 0; j < 256; j++)
    {
        // cycle all 256 colors in the wheel
        for (int q = 0; q < 3; q++)
        {
            //turn every third pixel on
            for (int i = 0; i < 2 * led.numPixels(); i = i + 3)
            {
                if (i >= MAX_LED_NUM)
                {
                    led2.setPixelColor((i - MAX_LED_NUM) + q, Wheel( (i + j) % 255));
                    readSerial();
                    if (isAvailable)
                    {
                        read_serial();
                        Serial.println("jump to outside");
                        goto outside5;
                        return;
                    }
                }
                else
                {
                    led.setPixelColor(i + q, Wheel( (i + j) % 255));
                    readSerial();
                    if (isAvailable)
                    {
                        read_serial();
                        goto outside5;
                        return;
                    }
                }
            }
            led.show();
            led2.show();
            readSerial();
            if (isAvailable)
            {
                read_serial();
                goto outside5;
                return;
            }
            delay(wait);
            for (int i = 0; i < 2 * led.numPixels(); i = i + 3)
            {
                //turn every third pixel off
                if (i >= MAX_LED_NUM)
                {
                    led2.setPixelColor((i - MAX_LED_NUM) + q, 0);
                    readSerial();
                    if (isAvailable)
                    {
                        read_serial();
                        goto outside5;
                        return;
                    }
                }
                else
                {
                    led.setPixelColor(i + q, 0);
                    readSerial();
                    if (isAvailable)
                    {
                        read_serial();
                        goto outside5;
                        return;
                    }
                }
            }
        }
    }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85)
    {
        return led.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170)
    {
        WheelPos -= 85;
        return led.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return led.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}



void breath_light(uint8_t wait)
{
    outside6:
    if (fx_value != 6)
    {
        return;
    }
    if (bri >= 200)
    {
        st = 1;
    }
    if (bri <= 0)
    {
        st = 0;
    }

    if (st)
    {
        bri = bri - 2;
    }
    else
    {
        bri = bri + 2 ;
    }
    //bri = 255;// change the effect to White pure.20160406
    for (int16_t t = 0; t < MAX_LED_NUM; t++)
    {
        read_serial();
        led.setPixelColor(t, bri, bri, bri); /* parameter description: led number, red, green, blue, flash mode */
        led2.setPixelColor(t, bri, bri, bri);
        readSerial();
        if (isAvailable)
        {
            read_serial();
            goto outside6;
            return;
        }
    }
    led.show();
    led2.show();
    if (isAvailable)
    {
        read_serial();
        return;
    }
}

void read_serial(void)

{
    while (isAvailable)
    {
        // deviceState = 1;
        unsigned char c = serialRead & 0xff;
        if ((c == 0x55) && (isStart == false))
        {
            if (prevc == 0xf0)
            {
                index = 1;
                isStart = true;
            }
        }
        else
        {
            prevc = c;
            if (isStart)
            {
                if (index == 2)
                {
                    dataLen = c;
                }
                else if (index > 2)
                {
                    dataLen--;
                }
                // cleanBuffer();
                writeBuffer(index, c);
            }
        }
        index++;
        if (index > 51)
        {
            index = 0;
            isStart = false;
        }
        if (isStart && (dataLen == 0) && (index > 3))
        {
            isStart = false;
            parseData();
            index = 0;
        }
        readSerial();
    }
}

void rect_line_time()
{

    int color_rect = random(1, 23);
    int show_time = 200;


    terminal_effect(4, -1, show_time, color_rect);
    body_rect_effect(1, show_time, color_rect);
    terminal_effect(3, 1, show_time, color_rect);
    // all_off();
    color_rect = color_rect + 5;
    terminal_effect(3, -1, show_time, color_rect);
    body_rect_effect(-1, show_time, color_rect);
    terminal_effect(4, 1, show_time, color_rect);

    readSerial();
    if (isAvailable)
    {
        read_serial();
        return;


    }
}
void make_arrows(int direction, int delaytime, int show_color)
{
    if (direction == 1)
    {
        for (int i = 2; i >= 0 ; i-- )
        {
            usual_set_fect(1, show_color, arrows[i][0], arrows[i][1], arrows[i][2], arrows[i][3], 1);
            delay(delaytime);
            usual_set_fect(2, show_color, arrows[i][0], arrows[i][1], arrows[i][2], arrows[i][3], 1);
            delay(delaytime);
            usual_set_fect(5, show_color, arrows[i][0], arrows[i][1], arrows[i][2], arrows[i][3], 1);
            delay(delaytime);
            usual_set_fect(6, show_color, arrows[i][0], arrows[i][1], arrows[i][2], arrows[i][3], 1);
            delay(delaytime);
            readSerial();
            if (isAvailable)
            {
                read_serial();
                return;
            }
        }
    }
    if (direction == -1)
    {
        for (int i = 2; i >= 0 ; i-- )
        {
            usual_set_fect(1, show_color, re_arrows[i][0], re_arrows[i][1], re_arrows[i][2], re_arrows[i][3], 1);
            delay(delaytime);
            usual_set_fect(2, show_color, re_arrows[i][0], re_arrows[i][1], re_arrows[i][2], re_arrows[i][3], 1);
            delay(delaytime);
            usual_set_fect(5, show_color, re_arrows[i][0], re_arrows[i][1], re_arrows[i][2], re_arrows[i][3], 1);
            delay(delaytime);
            usual_set_fect(6, show_color, re_arrows[i][0], re_arrows[i][1], re_arrows[i][2], re_arrows[i][3], 1);
            delay(delaytime);
            all_off();
            readSerial();
            if (isAvailable)
            {
                read_serial();
                return;
            }
        }
    }

}

void body_rect_effect(int direction, int delaytime, int show_color)
{
    if(fx_value != 2)
    {
        goto outside2;
    }
    if (direction == 1)
    {
        for (int i = 4; i >= 0 ; i-- )
        {
            if(fx_value != 2)
            {
                goto outside2;
            }
            usual_set_fect(1, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3], 1); //usual_set_fect(face_num, color_num, led_index1, led_index2, led_index3, led_index4);
            usual_set_fect(2, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3], 1);
            usual_set_fect(5, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3], 1);
            usual_set_fect(6, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3], 1);
            delay(delaytime);
        }
    }
    if (direction == -1)
    {
        for (int i = 0; i <= 4 ; i++ )
        {
            if(fx_value != 2)
            {
                goto outside2;
            }
            usual_set_fect(1, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3], 1); //usual_set_fect(face_num, color_num, led_index1, led_index2, led_index3, led_index4);
            usual_set_fect(2, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3], 1);
            usual_set_fect(5, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3], 1);
            usual_set_fect(6, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3], 1);
            delay(delaytime);
        }
    }
    outside2:
    ;
}
void terminal_effect(int face, int direction, int delaytime, int show_color)
{
    if(fx_value != 2)
    {
        goto outside1;
    }

    if (direction == 1)
    {
        if(fx_value != 2)
        {
            goto outside1;
        }
        usual_set_fect(face, show_color, 1, 248, 198, 63, 1);
        delay(delaytime);
        if(fx_value != 2)
        {
            goto outside1;
        }
        usual_set_fect(face, show_color, 0, 7, 41, 192, 1);
        delay(delaytime);
        if(fx_value != 2)
        {
            goto outside1;
        }
        usual_set_fect(face, show_color, 0, 0, 16, 0, 1);
        delay(delaytime);

    }
    if (direction == -1)
    {
        if(fx_value != 2)
        {
            goto outside1;
        }
        usual_set_fect(face, show_color, 0, 0, 16, 0, 1);
        delay(delaytime);
        if(fx_value != 2)
        {
            goto outside1;
        }
        usual_set_fect(face, show_color, 0, 7, 41, 192, 1);
        delay(delaytime);
        if(fx_value != 2)
        {
            goto outside1;
        }
        usual_set_fect(face, show_color, 1, 248, 198, 63, 1);
        delay(delaytime);

    }
    outside1:
    ;
}

// void fixed_delay()
// {

// }
void Special_light()
{
    GetRandomArr();
    for (int i = 1; i <= 6 ; i++ )
    {
        if(fx_value != 1)
        {
            goto endfx1;
        }
        usual_set_fect(i, randomTmp[i - 1] * 4 - 3, 255, 255, 255, 255, 1);
    }
    endfx1:
    ;
}

void show_the_fx(uint8_t fx_value)
{
    wdt_reset();
    switch (fx_value)
    {
    case 0:
        // resetFunc();
        all_off();
        break;
    case 1:
        Special_light();
        break;
    case 2:
        rect_line_time();
        break;
    case 3:
        make_arrows(1, 40, 2);
        // all_off();
        make_arrows(-1, 40, 2);
        // all_off();
        break;
    case 4:
        rainbowCycle(20);
        // colorWipe(led.Color(255, 0, 0), 50);
        // colorWipe(led.Color(255, 255, 0), 50);
        // colorWipe(led.Color(255, 0, 255), 50);
        break;
    case 5:
        theaterChaseRainbow(50);
        break;
    case 6:
        // DiceMode(gyro[0], gyro[1], gyro[2]);
        // digitalClockDisplay();
        break;
    default:
        power_off_count = 0;
        break;
    }
}
//************************functions of protocol****************************//

void dmp_init()
{

    for (int i = 0; i < 7; i++)
    {
        bank_sel(i);
        for (byte j = 0; j < 16; j++)
        {

            byte start_addy = j * 0x10;

            Wire.beginTransmission(MPU_ADDR);
            Wire.write(MEM_START_ADDR);
            Wire.write(start_addy);
            Wire.endTransmission();

            Wire.beginTransmission(MPU_ADDR);
            Wire.write(MEM_R_W);
            for (int k = 0; k < 16; k++)
            {
                unsigned char byteToSend = pgm_read_byte( & (dmpMem[i][j][k]));
                Wire.write((byte) byteToSend);
            }
            Wire.endTransmission();
        }
    }

    bank_sel(7);

    for (byte j = 0; j < 8; j++)
    {

        byte start_addy = j * 0x10;

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(MEM_START_ADDR);
        Wire.write(start_addy);
        Wire.endTransmission();

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(MEM_R_W);
        for (int k = 0; k < 16; k++)
        {
            unsigned char byteToSend = pgm_read_byte( & (dmpMem[7][j][k]));
            Wire.write((byte) byteToSend);
        }
        Wire.endTransmission();
    }

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MEM_START_ADDR);
    Wire.write(0x80);
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MEM_R_W);
    for (int k = 0; k < 9; k++)
    {
        unsigned char byteToSend = pgm_read_byte( & (dmpMem[7][8][k]));
        Wire.write((byte) byteToSend);
    }
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MEM_R_W);
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.requestFrom(MPU_ADDR, 9);
    // Wire.endTransmission();
    byte incoming[9];
    for (int i = 0; i < 9; i++)
    {
        incoming[i] = Wire.read();
    }
}

void mem_init()
{
    dmp_init();
    for (byte i = 0; i < 22; i++)
    {
        bank_sel(dmp_updates[i][0]);
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(MEM_START_ADDR);
        Wire.write(dmp_updates[i][1]);
        Wire.endTransmission();

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(MEM_R_W);
        for (byte j = 0; j < dmp_updates[i][2]; j++)
        {
            Wire.write(dmp_updates[i][j + 3]);
        }
        Wire.endTransmission();
    }

    regWrite(0x38, 0x32);

    for (byte i = 22; i < 29; i++)
    {
        bank_sel(dmp_updates[i][0]);
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(MEM_START_ADDR);
        Wire.write(dmp_updates[i][1]);
        Wire.endTransmission();

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(MEM_R_W);
        for (byte j = 0; j < dmp_updates[i][2]; j++)
        {
            Wire.write(dmp_updates[i][j + 3]);
        }
        Wire.endTransmission();
    }

    temp = regRead(0x6B);
    //Serial.println(temp, HEX);
    temp = regRead(0x6C);
    // Serial.println(temp, HEX);

    regWrite(0x38, 0x02);
    regWrite(0x6B, 0x03);
    // regWrite(0x6B, 0x70);
    // regWrite(0x38, 0x38);
    // regWrite(0x6B, 0x73);
    regWrite(0x19, 0x04);
    regWrite(0x1B, 0x18);
    regWrite(0x1A, 0x0B);
    regWrite(0x70, 0x03);
    regWrite(0x71, 0x00);
    regWrite(0x00, 0x00);
    regWrite(0x01, 0x00);
    regWrite(0x02, 0x00);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x13);
    for (byte i = 0; i < 6; i++)
    {
        Wire.write(0x00);
    }
    Wire.endTransmission();

    // regWrite(0x24, 0x00);

    bank_sel(0x01);
    regWrite(0x6E, 0xB2);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6F);
    Wire.write(0xFF);
    Wire.write(0xFF);
    Wire.endTransmission();

    bank_sel(0x01);
    regWrite(0x6E, 0x90);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6F);
    Wire.write(0x09);
    Wire.write(0x23);
    Wire.write(0xA1);
    Wire.write(0x35);
    Wire.endTransmission();

    temp = regRead(0x6A);

    regWrite(0x6A, 0x04);

    //Insert FIFO count read?
    fifoReady();

    regWrite(0x6A, 0x00);
    regWrite(0x6B, 0x03);

    delay(2);

    temp = regRead(0x6C);
    // Serial.println(temp, HEX);
    regWrite(0x6C, 0x00);
    temp = regRead(0x1C);
    // Serial.println(temp, HEX);
    regWrite(0x1C, 0x00);
    delay(2);
    temp = regRead(0x6B);
    // Serial.println(temp, HEX);
    regWrite(0x1F, 0x02);
    regWrite(0x21, 0x9C);
    regWrite(0x20, 0x50);
    regWrite(0x22, 0x00);
    regWrite(0x6A, 0x04);
    regWrite(0x6A, 0x00);
    regWrite(0x6A, 0xC8);

    bank_sel(0x01);
    regWrite(0x6E, 0x6A);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6F);
    Wire.write(0x06);
    Wire.write(0x00);
    Wire.endTransmission();

    bank_sel(0x01);
    regWrite(0x6E, 0x60);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6F);
    for (byte i = 0; i < 8; i++)
    {
        Wire.write(0x00);
    }
    Wire.endTransmission();
    bank_sel(0x00);
    regWrite(0x6E, 0x60);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6F);
    Wire.write(0x40);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();
}

void regWrite(byte addy, byte regUpdate)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(addy);
    Wire.write(regUpdate);
    Wire.endTransmission();
}

byte regRead(byte addy)
{  
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(addy);
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.requestFrom(MPU_ADDR, 1);
    // Wire.endTransmission();
    while (!Wire.available())
    {
    }
    byte incoming = Wire.read();
    return incoming;
}

void getPacket()
{
    if (fifoCountL > 32)
    {
        fifoCountL2 = fifoCountL - 32;
        longPacket = true;
    }
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x74);
    Wire.endTransmission();
    // Wire.requestFrom(MPU_ADDR, 42);
    // for(byte i = 0; i < fifoCountL; i++){
    if (longPacket)
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.requestFrom(MPU_ADDR, 32);
        for (byte i = 0; i < 32; i++)
        {
            received_packet[i] = Wire.read();
        }
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x74);
        Wire.endTransmission();
        Wire.beginTransmission(MPU_ADDR);
        Wire.requestFrom(MPU_ADDR, (unsigned int)fifoCountL2);
        for (byte i = 32; i < fifoCountL; i++)
        {
            received_packet[i] = Wire.read();
        }
        longPacket = false;
    }
    else
    {
        Wire.beginTransmission(MPU_ADDR);
        Wire.requestFrom(MPU_ADDR, (unsigned int)fifoCountL);
        for (byte i = 0; i < fifoCountL; i++)
        {
            received_packet[i] = Wire.read();
        }
    }
}

boolean fifoReady()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x72);
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.requestFrom(MPU_ADDR, 2);
    // Wire.endTransmission();
    byte fifoCountH = Wire.read();
    fifoCountL = Wire.read();
    //Serial.print(fifoCountL);
    if (fifoCountL == 42 || fifoCountL == 44)
    {
        return 1;
    }
    else return 0;
}

void check_MPU()
{

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75);
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.requestFrom(MPU_ADDR, 1);
    byte aByte = Wire.read();
    if (aByte == 0x68)
    {
        Serial.println("Found MPU6050");
    }
    else
    {
        Serial.println("Didn't find MPU6050");
    }
}
void bank_sel(byte bank)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6D);
    Wire.write(bank);
    Wire.endTransmission();
}

void GetRandomArr ()
{

    for (int i = 0; i < 6; i++)
    {
        seed[i] = i + 1;
    }

    int s1 = random(0, 6);

    randomTmp[0] = seed [s1];

    seed [s1] = -1;
    int t2[] =
    {
        0, 0, 0, 0, 0
    };
    int index = 0;
    for (int i = 0; i < 6; i++)
    {
        if (seed [i] != -1)
        {
            t2 [index] = seed [i];
            index++;
        }
    }
    int s2 = random (0, 5);
    for (int i = 0; i < 6; i++)
    {
        if (seed [i] == t2 [s2])
        {
            seed [i] = -1;
        }
    }

    randomTmp[1] = t2 [s2];


    int t3[] =
    {
        0, 0, 0, 0
    };
    index = 0;
    for (int i = 0; i < 6; i++)
    {
        if (seed [i] != -1) 
        {
            t3 [index] = seed [i];
            index++;
        }
    }
    int s3 = random (0, 4);

    for (int i = 0; i < 6; i++)
    {
        if (seed [i] == t3 [s3])
        {
            seed [i] = -1;
        }
    }
    randomTmp[2] = t3 [s3];
    int t4[] =
    {
        0, 0, 0
    };
    index = 0;
    for (int i = 0; i < 6; i++)
    {
        if (seed [i] != -1)
        {
            t4 [index] = seed [i];
            index++;
        }
    }
    int s4 = random (0, 3);
    for (int i = 0; i < 6; i++)
    {
        if (seed [i] == t4 [s4])
        {
            seed [i] = -1;
        }
    }
    randomTmp[3] = t4 [s4];
    int t5[] =
    {
        0, 0
    };
    index = 0;
    for (int i = 0; i < 6; i++)
    {
        if (seed [i] != -1)
        {
            t5 [index] = seed [i];
            index++;
        }
    }
    int s5 = random (0, 2);
    for (int i = 0; i < 6; i++)
    {
        if (seed [i] == t5 [s5])
        {
            seed [i] = -1;
         }
    }
    randomTmp[4] = t5 [s5];
    for (int i = 0; i < 6; i++)
    {
        if (seed [i] != -1)
        {
            randomTmp[5] = seed [i];
        }
    }
}

void try_to_power_off(int energy)
{
    // Serial.println(power_off_count);
    if (energy <= 25)// && digitalRead(BCTS) == 1
    {
        power_off_count++;
    }
    else
        power_off_count = 0;
    if (power_off_count >= 3600 && device_is_in_charge == 0)   //about three minutes 20 seconds .
    {
        led.setBrightness(60);
        led2.setBrightness(60);
        for (uint8_t i = 0; i < MAX_LED_NUM; i++)
        {
            led.setPixelColor(i, off_brightness, 0, 0);
            led2.setPixelColor(i, off_brightness, 0, 0);
        }
        led.show();
        led2.show();
        delay(off_delaytime);

        digitalWrite(POWER_ON_OFF, HIGH);
    }
}

int gesture_energy_calculate(float gyrox, float gyroy, float gyroz)
{
    int X_tmp_energy = 0;
    int Y_tmp_energy = 0;
    int Z_tmp_energy = 0;
    X_tmp_energy += gyrox;//gx+ax
    Y_tmp_energy += gyroy;
    Z_tmp_energy += gyroz;
    X_energy = abs(X_tmp_energy);
    Y_energy = abs(Y_tmp_energy);
    Z_energy = abs(Z_tmp_energy);
    return (X_energy + Y_energy + Z_energy);
}


void DiceMode(float gyrox, float gyroy, float gyroz)
{


    // dice_on = 1;
    // is_dice_mode = 0;
    // deviceState = 0;

    // Serial.println("what a fucking function that i can funing gointo yes balabala");
    if(dice_on == 1)
    {
        all_off();
        GetRandomArr();
        randomNum = random(1, 25);
        for (int i = 1; i <= 6 ; i++ )
        {
            matrix_effect(i, randomNum, randomTmp[i - 1], dic_line); //face、color、dice number、face msg
        }
    }
    dice_on = 0;
    if (deviceState == 0 || is_dice_mode == 0)
    {
        tmpx += abs(gyrox);
        tmpy += abs(gyroy); // @34123;
        tmpz += abs(gyroz);
        timecount++;
        if (timecount > 30)
        {
            timecount = 0;
            if (tmpx > 150 || tmpy > 150 || tmpz > 150)
            {

                tmpx = 0;
                tmpy = 0;
                tmpz = 0;
                diceAnKey = true;
            }
            else
            {

                tmpx = 0;
                tmpy = 0;
                tmpz = 0;

                diceAnLengh = 30;
            }
        }

        if (diceAnKey)
        {

            diceAnflame++;

            if (diceAnflame < diceAnLengh)
            {
                diceAnCount++;
                if ((diceAnCount > 1))
                {

                    all_off();
                    diceAnCount = 0;
                    GetRandomArr();
                    randomNum = random(1, 25);
                    for (int i = 1; i <= 6 ; i++ )
                    {
                        readSerial();
                        if (isAvailable)
                        {
                            read_serial();
                        }
                        if(deviceState == 1)
                        {
                            goto end_dice_mode;
                        }
                        matrix_effect(i, randomNum, randomTmp[i - 1], dic_line); //face、color、dice number、face msg
                    }

                }
            }
            else
            {
                diceAnKey = false;
                diceAnflame = 0;
                tmpx = 0;
                tmpy = 0;
                tmpz = 0;
            }
        }
    }
    end_dice_mode:
    ;
}

void all_off()
{
    for (uint8_t i = 0; i < MAX_LED_NUM; i++)
    {
        led.setPixelColor(i, 0, 0, 0);
        led2.setPixelColor(i, 0, 0, 0);
    }
    led.show();
    led2.show();
}

//**************//
void usual_set_fect(uint8_t faceNum, uint8_t colorNum, uint32_t index1, uint32_t index2, uint32_t index3, uint32_t index4, uint8_t ishow)
{
    wdt_reset();
    uint8_t face_num = faceNum;
    uint8_t color_num = colorNum;
    uint32_t led_index1 = index1;//dic[dicNum - 1][0];
    uint32_t led_index2 = index2;//dic[dicNum - 1][1];
    uint32_t led_index3 = index3;//dic[dicNum - 1][2];
    uint32_t led_index4 = index4;//dic[dicNum - 1][3];

    uint8_t led_num_start = (face_num - 1) * 25;
    uint32_t led_index = (led_index1 << 24) + (led_index2 << 16) + (led_index3 << 8) + led_index4;
    uint32_t led_index_temp;
    if (led_num_start >= MAX_LED_NUM)
    {
        led_num_start = led_num_start - 75;
        for (uint8_t i = 0; i < ONE_FACE_LED_NUM; i++)
        {
            led_index_temp = (led_index >> i) & 0x01;
            if (led_index_temp == 1)
            {
                led2.setPixelColor(led_num_start + i, color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
                readSerial();
                if (isAvailable)
                {
                    read_serial();
                    return;
                }

            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < ONE_FACE_LED_NUM; i++)
        {
            led_index_temp = (led_index >> i) & 0x01;
            if (led_index_temp == 1)
            {
                led.setPixelColor(led_num_start + i, color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
                readSerial();
                if (isAvailable)
                {
                    read_serial();
                    return;
                }
            }
        }
    }
    if(ishow == 1)
    {
        readSerial();
        if (isAvailable)
        {
            read_serial();
            return;
        }

        led.show();
        led2.show();

    }
}
//*****************//
void matrix_effect(uint8_t faceNum, uint8_t colorNum, int dicNum, int dic[6][4])
{
    uint8_t face_num = faceNum;
    uint8_t color_num = colorNum;
    uint32_t led_index1 = dic[dicNum - 1][0];
    uint32_t led_index2 = dic[dicNum - 1][1];
    uint32_t led_index3 = dic[dicNum - 1][2];
    uint32_t led_index4 = dic[dicNum - 1][3];
    usual_set_fect(face_num, color_num, led_index1, led_index2, led_index3, led_index4, 1);
}
void setup()
{
    // if(EEPROM.read(address) != 0 && EEPROM.read(address) != NULL)
    // {
    //     brightness_value = EEPROM.read(address);
    // }
     wdt_disable();
    brightness_value = 30;
    initKey = 0;
    led.begin();
    led2.begin();
    led.setBrightness(brightness_value);
    led2.setBrightness(brightness_value);


    // randomNum = EEPROM.read(address);

    // if (randomNum >= 255)
    // {
    //     charge_st = 1;
    //     EEPROM.write(1, charge_st);
    // }
    // if (randomNum <= 0)
    // {
    //     charge_st = 0;
    //     EEPROM.write(1, charge_st);
    // }
    // charge_st = EEPROM.read(1);
    // if (charge_st)
    // {
    //     randomNum = randomNum - 10;
    //     EEPROM.write(address, randomNum);
    // }
    // else
    // {
    //     randomNum = randomNum + 10 ;
    //     EEPROM.write(address, randomNum);
    // }
    // randomNum = map(randomNum, 0, 255, 1, 28);

    usual_set_fect(1, randomNum, 0, 7, 41, 193, 1);
    usual_set_fect(2, randomNum, 0, 7, 41, 192, 1);
    usual_set_fect(3, randomNum, 0, 0, 0, 0, 1);
    usual_set_fect(4, randomNum, 0, 0, 0, 0, 1);
    usual_set_fect(5, randomNum, 0, 7, 41, 64, 1);
    usual_set_fect(6, randomNum, 0, 7, 41, 192, 1);
    led.show();
    led2.show();
    Serial.begin(9600);
    Wire.begin();
    delay(1);
    check_MPU();
    regWrite(0x6B, 0xC0);
    regWrite(0x6C, 0x00);
    delay(10);
    regWrite(0x6B, 0x00);
    regWrite(0x6D, 0x70);
    regWrite(0x6E, 0x06);
    temp = regRead(0x6F);
    regWrite(0x6D, 0x00);
    temp = regRead(0x00);
    temp = regRead(0x01);
    temp = regRead(0x02);
    temp = regRead(0x6A);
    regWrite(0x37, 0x32);
    temp = regRead(0x6B);
    delay(5);
    // mpu.setXGyroOffset(-185);
    // mpu.setYGyroOffset(7006);
    // mpu.setZGyroOffset(-85);
    // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    mem_init();
    Serial.print("Version: ");
    Serial.println(mVersion);

    analogReference(INTERNAL);
    pinMode(CHARGE_PORT, INPUT);

    pinMode(IS_IN_CHARGE, INPUT);
    pinMode(STANDBY_PORT, INPUT);

    pinMode(BRTS, OUTPUT);


  
    digitalWrite(BRTS, LOW);
    pinMode(BCTS, INPUT);
    // digitalWrite(BCTS, LOW);
    pinMode(POWER_ON_OFF, OUTPUT);
    digitalWrite(POWER_ON_OFF, LOW);
    lastTime = millis();
    wdt_enable(WDTO_4S);
}
void loop()
{
    wdt_reset();
    if(time_to_end_motor != 0)
    {

        time_to_end_motor--;
    }

    if(time_to_end_motor <= 0)
    {
        analogWrite(SHARE_MOTOR_PORT, 255);
        time_to_end_motor = 0;
    }
    // Serial.println(time_to_end_motor);
    if (fx_value != 0)
    {

        readSerial();
        if (isAvailable)
        {
            read_serial();
        }


        if(last_fx_state != fx_value)
        {
            all_off();
            // Serial.println("changed !!!!!*************");
        }

        last_fx_state = fx_value;

        mem_init();
        analogWrite(SHARE_MOTOR_PORT, 255);
        show_the_fx(fx_value);
    }

    if (millis() - lastTime > 33)
    {
        if (fifoReady())
        {
            getPacket();
            temp = regRead(0x3A);
            if (firstPacket)
            {
                delay(1);
                bank_sel(0x00);
                regWrite(0x6E, 0x60);
                Wire.beginTransmission(MPU_ADDR);
                Wire.write(0x6F);
                Wire.write(0x04);
                Wire.write(0x00);
                Wire.write(0x00);
                Wire.write(0x00);
                Wire.endTransmission();
                bank_sel(1);
                regWrite(0x6E, 0x62);
                Wire.beginTransmission(MPU_ADDR);
                Wire.write(0x6F);
                Wire.endTransmission();
                Wire.beginTransmission(MPU_ADDR);
                Wire.requestFrom(MPU_ADDR, 2);
                temp = Wire.read();
                temp = Wire.read();
                firstPacket = false;
                fifoReady();
            }

            gyro[0] = (received_packet[16] << 8) + received_packet[17];
            gyro[1] = (received_packet[20] << 8) + received_packet[21];
            gyro[2] = (received_packet[24] << 8) + received_packet[25];


            lastTime = millis();

             Serial.print(gyro[0] / 16.4f);
             Serial.print(",");
             Serial.print(gyro[1] / 16.4f);
             Serial.print(",");
             Serial.println(-gyro[2] / 16.4f);

            if (device_is_in_charge == 0 &&  is_dice_mode == 0 )
            {
                int energy = gesture_energy_calculate(gyro[0], gyro[1], gyro[2]);
                DiceMode(gyro[0], gyro[1], gyro[2]);
                try_to_power_off(energy);
            }
        }
        power_managment();
    }
    readSerial();
    if (isAvailable)
    {
        read_serial();
    }
    

}
void power_managment()
{
    int plug_read = digitalRead(IS_IN_CHARGE);
    uint8_t chargeport = digitalRead(CHARGE_PORT);
    uint8_t standbyport = digitalRead(STANDBY_PORT);


    bat_voltage = (analogRead(VOLTAGE_DETECT_PORT) / 1024.0) * 1.1 * 10.0;


    // Serial.println(bat_voltage);

//(chargeport == 0 || standbyport == 0) ||
    if ( plug_read == 1 ) //其中有一个是低电平说明USB已经插入；
    {
        need_to_be_charged = 0;
        device_is_in_charge = 1;
        power_off_count = 0;
    }
//(chargeport == 1 && standbyport == 1) ||
    if ( plug_read == 0) //无任何设备插入的情况（设备指USB）
    {
        device_is_in_charge = 0;
        if (deviceState == 1)
        {
            power_off_count = 0;
        }
    }

    if (bat_voltage < 3)
    {
        need_to_be_charged++;
    }
    else
        need_to_be_charged = 0;

    if (need_to_be_charged >= 200)
    {
        if(deviceState == 1)
        {
            all_off();
        }
        led.setBrightness(60);
        led2.setBrightness(60);
        usual_set_fect(3, 25, 1, 85, 85, 85, 1); //facenum = 3 = top ;color = 25 = red ;the full effect;
        usual_set_fect(6, 25, 0, 12, 15, 0, 1);
        deviceState = 0;
        delay(2000);
        digitalWrite(POWER_ON_OFF, HIGH);
    }
    if (device_is_in_charge == 1 && deviceState == 0)//whrere there is a USB port plugin or a wireless charge voltage plugin.
    {

        if (plugin_state == 0)//restore the paramaters to the origin state.make it excute once.
        {
            all_off();
            charge_bri = 0;
            // aculate_once = 0;
        }

        if (chargeport == 0  && standbyport == 1)// charging state of port charge and port stand.
        {
            if (charge_bri >= 50)
            {
                charge_st = 1;
            }
            if (charge_bri <= 10)
            {
                charge_st = 0;
            }

            if (charge_st)
            {
                charge_bri = charge_bri - 1;
            }
            else
            {
                charge_bri = charge_bri + 1 ;
            }
            led2.setBrightness(charge_bri);

            usual_set_fect(6, 2, 0, 12, 15, 0, 1);

            led2.show();
        }

        if (chargeport == 1 && standbyport == 0 && plug_read == 1)
        {
            led2.setBrightness(brightness_value);
            usual_set_fect(6, 26, 0, 12, 15, 0, 1);
        }
        plugin_state = 1;
    }
    if (device_is_in_charge == 0 && deviceState == 0 && plug_read == 0) //whrere the extension power is off from the device.
    {
        if (plugin_state == 1)
        {
            all_off();

            for (uint8_t i = 0; i < MAX_LED_NUM; i++)
            {
                led.setPixelColor(i, color_tab[8][0], color_tab[8][1], color_tab[8][2]);
                led2.setPixelColor(i, color_tab[8][0], color_tab[8][1], color_tab[8][2]);
            }
            led.show();
            led2.show();
            led.setBrightness(brightness_value / 2);
            led2.setBrightness(brightness_value / 2);
            plugin_state = 0;
        }
    }
}
