//1.8.2
#include <Wire.h>
#include "Adafruit_NeoPixel.h"
// #include "Filters.h"
//************************const variable for hardware*********************************//

#define MAX_LED_NUM                 75
#define ONE_FACE_LED_NUM            25
#define POWER_ON_OFF                7
#define VOLTAGE_DETECT_PORT         A1
#define LED_PIN                     6
#define LED_PIN2                    5
#define CONNECT                     4
#define IS_IN_CHARGE                A3
// #define MULTIFUNCTION               2

#define GET                         1
#define RUN                         2
#define RESET                       4
#define START                       5
#define BMG160_ADDR                 0x68
#define BUFFSIZE                    33
long deltaTime = 0;
boolean is_in_dev_mode = 0;
boolean deviceState = 0;
boolean is_dice_mode = 0;
uint8_t initKey = 0, address = 0;
int powerCounter = 0;
uint8_t last_fx_value = 0;
boolean isAvailable = false;
Adafruit_NeoPixel led = Adafruit_NeoPixel(MAX_LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800 );
Adafruit_NeoPixel led2 = Adafruit_NeoPixel(MAX_LED_NUM, LED_PIN2, NEO_GRB + NEO_KHZ800 );
// filters out changes faster that 2.22 Hz.
float filterFrequency = 2.22 ;
int  last_key_value = 0;
// unsigned char key_value = 0;
// create a one pole (RC) INTEGRATOR filter
// FilterOnePole IntegratorFilterX( INTEGRATOR, filterFrequency );   
// FilterOnePole IntegratorFilterY( INTEGRATOR, filterFrequency );   
// FilterOnePole IntegratorFilterZ( INTEGRATOR, filterFrequency );   

//Command

#define SET_INDEX_COLOR                  0x01
#define READ_INDEX_COLOR                 0x02
#define RGB_SHOW                         0x03
#define RGB_BRIGHTNESS                   0x04
#define RGB_SET_FX                       0x05
#define DEV_MODE                         0X06
#define SAVE_BRIGHTNESS                  0x44

#define MPU_INIT                         0x43

#define DIC_MODE                         0x42

#define RGB_SET_COLOR_0                  0x41
#define RGB_SET_COLOR_1                  0x11
#define RGB_SET_COLOR_2                  0x12

#define RGB_SET_COLOR_4                  0x14
#define POWER_OFF                        0x15
#define SHAKE_MOTOR                      0x16
#define VOLTAGE_DETECT                   0x17
#define CHARG_STATUS                     0x18
#define GET_BRIGHTNESS                   0x19
#define CONNECTION_STATE                 0X20

//*******value for dic ************************//
int life = 0;
int rect_line[5][4]=
{
    {1, 0, 192, 48},
    {0, 129, 32, 72},
    {0, 66, 16, 132},
    {0, 36, 9, 2},
    {0, 24, 6, 1},
};
unsigned char seed[] =
{
    1, 2, 3, 4, 5, 6
};
unsigned char randomTmp[] =
{
    0, 0, 0, 0, 0, 0
};
int tmpx = 0;
int tmpy = 0;
int tmpz = 0;
uint8_t randomNum = 1;

uint8_t timecount = 0;

uint8_t diceAnKey = 0;
uint8_t diceAnCount = 0;
uint8_t diceAnLengh = 90;
uint8_t diceAnflame = 0;
//*******value for dic ************************//

boolean is_dev_in_charge = 0;

uint8_t fx_value = 0;
boolean last_fx_state = 0;
boolean dice_on = 1;

uint8_t brightness_value = 25;

int gyrox_global = 0;
int gyroy_global = 0;
int gyroz_global = 0;
int last_gyrox = 0;
int last_gyroy = 0;
int last_gyroz = 0;

int16_t bri = 0, charge_bri = 0, st = 0, charge_st = 0, bat_count = 0;
// int need_to_be_charged = 0;
float bat_voltage = 0;

unsigned char dic_line[6][4] =
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

const uint8_t color_tab[29][3] =
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


String mVersion = "7.0.0";

// uint8_t len = 52;
char buffer[BUFFSIZE];
byte index = 0;
byte dataLen;
byte modulesLen = 0;
boolean isStart = false;
unsigned char irRead;
char serialRead;

unsigned char prevc = 0;
long lastTime = 0;


uint8_t fx_idx ;

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
        isAvailable = true;
        serialRead = Serial.read();
    }
}
void parseData()
{
    //F0550400020501
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
    writeSerial(0x02);
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
void sendString(String s)
{
    int l = s.length();
    writeSerial(0x04);
    writeSerial(l);
    for (int i = 0; i < l; i++)
    {
        writeSerial(s.charAt(i));
    }
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
    {   //F0550400020501
        case DEV_MODE:
        {
            is_in_dev_mode = readBuffer(6); 
            all_off();
        }
        break;
        case DIC_MODE:
        {
            uint8_t dice_switch = readBuffer(6);
            if (dice_switch == 0x01)
            {
                fx_value  = 3;
                dice_on = 1;
                is_dice_mode = 0;
            // deviceState = 0;
                callOK(fx_idx, 0x10);//////////////////////////////////////////
            }
            if (dice_switch == 0x00)
            {
                fx_value  = 0;
                is_dice_mode = 1;
                dice_on = 1;
                // deviceState = 1;
                all_off();
                callOK(fx_idx, 0x10);//////////////////////////////////////////
            }
        }
        break;
        case RGB_BRIGHTNESS:
        {
            brightness_value = readBuffer(6);
            if(brightness_value <= 10)
            {
                brightness_value = 10;
            }
            if(brightness_value >= 130)
            {
                brightness_value = 130;
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
            callOK(fx_idx, 0x10);//////////////////////////////////////////
        }
        break;

        case RGB_SET_COLOR_0:
        {
            // fx_value = 0;
            uint8_t color_num = readBuffer(6);
            uint16_t led_index = readBuffer(7);
            // Serial.println(led_index);
            if (led_index > MAX_LED_NUM)
            {
                led_index = led_index - 75;
                led2.setPixelColor(led_index - 1 , color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
                led2.show();
            }
            else
            {
                led.setPixelColor(led_index - 1 , color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
                led.show();
            }
        }
        break;
        case RGB_SET_COLOR_1:
        {
            // fx_value = 0;
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
            // fx_value = 0;
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
                    led.setPixelColor(led_num_start + i, color_tab[color_num][0], color_tab[color_num][1], color_tab[color_num][2]);
                }
            }
            led.show();
            led2.show();
        }
        break;
        case RGB_SET_COLOR_4:
        {
            // fx_value = 0;
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
            if (power_val == 0x0a && is_dev_in_charge == 0)
            {
                for (uint8_t i = 0; i < MAX_LED_NUM; i++)
                {
                    led.setPixelColor(i, 123, 0, 0);
                    led2.setPixelColor(i, 123, 0, 0);
                }
                led.show();
                led2.show();
                delay(1000);
                digitalWrite(POWER_ON_OFF, HIGH);
            }
        }
        break;
        default:
        ;
    }
}
void terminal_effect(int face, int direction, int delaytime, int show_color,uint8_t type )
{
    if(fx_value != type)
    {
        goto outside1;
    }
    if (direction == 1)
    {
        usual_set_fect(face, show_color, 1, 248, 198, 63);
        delay(delaytime);

        usual_set_fect(face, show_color, 0, 7, 41, 192);
        delay(delaytime);

        usual_set_fect(face, show_color, 0, 0, 16, 0);
        delay(delaytime);
    }
    if (direction == -1)
    {
        usual_set_fect(face, show_color, 0, 0, 16, 0);
        delay(delaytime);
        usual_set_fect(face, show_color, 0, 7, 41, 192);
        delay(delaytime);
        usual_set_fect(face, show_color, 1, 248, 198, 63);
        delay(delaytime);
    }
    outside1:
    ;
}
void body_rect_effect(int direction, int delaytime, int show_color,uint8_t type)
{
    if(fx_value != type)
    {
        goto outside2;
    }
    if (direction == 1)
    {
        for (int i = 4; i >= 0 ; i-- )
        {
            if(fx_value != type)
            {
                goto outside2;
            }
            usual_set_fect(1, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3]); 
            usual_set_fect(2, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3]);
            usual_set_fect(5, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3]);
            usual_set_fect(6, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3]);
            delay(delaytime);
        }
    }
    if (direction == -1)
    {
        for (int i = 0; i <= 4 ; i++ )
        {
            if(fx_value != type)
            {
                goto outside2;
            }
            usual_set_fect(1, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3]); //usual_set_fect(face_num, color_num, led_index1, led_index2, led_index3, led_index4);
            usual_set_fect(2, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3]);
            usual_set_fect(5, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3]);
            usual_set_fect(6, show_color, rect_line[i][0], rect_line[i][1], rect_line[i][2], rect_line[i][3]);
            delay(delaytime);
        }
    }
    outside2:
    ;
}
void rect_line_time(uint8_t type)
{
    int color_rect = random(1, 23);
    int show_time = 100;

    if (fx_value != type)
    {
        goto endTheFx;
        return;
    }
    terminal_effect(4, -1, show_time, color_rect,type);
    body_rect_effect(1, show_time, color_rect,type);
    terminal_effect(3, 1, show_time, color_rect,type);
    // all_off();
    color_rect = color_rect + 5;
    terminal_effect(3, -1, show_time, color_rect,type);
    body_rect_effect(-1, show_time, color_rect,type);
    terminal_effect(4, 1, show_time, color_rect,type);

    readSerial();
    if (isAvailable)
    {
        read_serial();
        return;
    }
    endTheFx:
    ;
}
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait,uint8_t type)
{   
    if(fx_value!=type){return;}
    for (uint16_t i = 0; i < 150; i++)
    {
        if (fx_value != type)
        {
            goto outsidex;
        }
        if (i >= MAX_LED_NUM)
        {
            led2.setPixelColor(i - MAX_LED_NUM, c);
            led2.show();
        }
        else
        {
            led.setPixelColor(i, c);
            led.show();
        }
        delay(wait);
    }
    outsidex:
    ;
}

void rainbow(uint8_t wait)
{
    if (fx_value != 1)
    {
        return;
    }
    uint16_t i, j;
    for (j = 0; j < 256; j++)
    {
        for (i = 0; i < 150; i++)
        {
            if (i >= MAX_LED_NUM)
            {
                led2.setPixelColor(i - MAX_LED_NUM, Wheel((i + j) & 255));
            }
            else
            {
                led.setPixelColor(i, Wheel((i + j) & 255));
                readSerial();
            }
            // intButton();
            readSerial();
            if (isAvailable)
            {
                read_serial();
                goto outside3;
                return;
            }
        }
        led.show();
        led2.show();
        delay(wait);
    }
    // last_key_value = digitalRead(MULTIFUNCTION);
    outside3:
    ;
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait,uint8_t type)
{

    uint16_t i, j;
    if (fx_value != type)
    {
        return;
    }
    for (j = 0; j < 256 * 5; j++)
    {
        // 5 cycles of all colors on wheel
        for (i = 0; i < 2 * led.numPixels(); i++)
        {
            if (fx_value != type)
            {
                goto outside4;
            }
            if (i >= MAX_LED_NUM)
            {
                led2.setPixelColor((i - MAX_LED_NUM), Wheel(((i * 256 / led.numPixels()) + j) & 255));
            }
            else
            {
                led.setPixelColor(i, Wheel(((i * 256 / led.numPixels()) + j) & 255));
            }
            // intButton();
            readSerial();
            if (isAvailable)
            {
                read_serial();
                goto outside4;
            }
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
    }
    outside4:
    ;
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait)
{
    // outside2: //fx_value = 0;
    for (int j = 0; j < 10; j++)
    {
        for (int q = 0; q < 3; q++)
        {
            for (int i = 0; i <150; i = i + 3)
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
            led.show();
            led2.show();
            delay(wait);//UNPOPULAR DELAY
            for (int i = 0; i < 150; i = i + 3)
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
void theaterChaseRainbow(uint8_t wait,uint8_t type)
{
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

                }
                else
                {
                    led.setPixelColor(i + q, Wheel( (i + j) % 255));
                }
                if (fx_value != type)
                {
                    goto outside5;
                }
                // intButton();
                readSerial();
                if (isAvailable)
                {
                    read_serial();
                    // Serial.println("jump to outside");
                    goto outside5;
                    return;
                }
            }
            led.show();
            led2.show();
            
            delay(wait);
            for (int i = 0; i < 2 * led.numPixels(); i = i + 3)
            {
                //turn every third pixel off
                if (i >= MAX_LED_NUM)
                {
                    led2.setPixelColor((i - MAX_LED_NUM) + q, 0);
                    
                }
                else
                {
                    led.setPixelColor(i + q, 0);
                }
                if (fx_value != type)
                {
                    goto outside5;
                }
                // intButton();
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
    outside5:
    ;
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

void breath_light(uint8_t wait,uint8_t min,uint8_t max, uint8_t color = 0)
{
    if (bri >= max)
    {
        st = 1;
    }
    if (bri <= min)
    {
        st = 2;
    }

    if (st==1)
    {
        bri = bri - 2;
    }
    if(st==2)
    {
      bri = bri + 2 ;  
  }
    //bri = 255;// change the effect to White pure.20160406
  for (uint8_t t = 0; t < MAX_LED_NUM; t= t+st)
  {
        // read_serial();
        led.setPixelColor (t,0, 0, bri+color); /* parameter description: led number, red, green, blue, flash mode */
    led2.setPixelColor(t,0, 0, bri+color);
}
led.show();
led2.show();
outside6:
;
}

void read_serial(void)
{
    while (isAvailable)
        {   power_off_count = 0;
            deviceState = 1;
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
            if (index > BUFFSIZE - 1)
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

    void Special_light(uint8_t type)
    {
        GetRandomArr();

        for (int i = 1; i <= 6 ; i++ )
        {
            if(fx_value != type)
            {
                goto endfx1;
            }
            usual_set_fect(i, randomTmp[i - 1] * 4 - 3, 255, 255, 255, 255);
            delay(100);
        }
        endfx1:
        ;
    }

    void show_the_fx(uint8_t fx_value)
    {
        switch (fx_value)
        {
            case 0:
            all_off();
            // Serial.println("funcking in hewew");
            break;
            case 1:
            dice_on = 1;
            life = 1;
            Special_light(1);
            break;
            case 2:
            dice_on = 1;
            life = 1;
            theaterChaseRainbow(50,2);
            break;
            case 5:
            dice_on = 1;
            life = 1;
            rainbowCycle(20,5);
            break;
            case 4:
            dice_on = 1;
            life = 1;
            rect_line_time(4);
            break;
            case 3:
            DiceMode(gyrox_global,gyroy_global, gyroz_global);
            life = 1;
            break;
            default:
            power_off_count = 0;

        }
    }
//************************functions of protocol****************************//



    void GetRandomArr ()
    {
        for (int i = 0; i < 6; i++)
        {
            seed[i] = i + 1;
        }

        int s1 = random(0, 5);

        randomTmp[0] = seed [s1];

        seed [s1] = 10;
        int t2[] =
        {
            0, 0, 0, 0, 0
        };
        int index = 0;
        for (int i = 0; i < 6; i++)
        {
            if (seed [i] != 10)
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
                seed [i] = 10;
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
            if (seed [i] != 10)
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
                seed [i] = 10;
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
            if (seed [i] != 10)
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
                seed [i] = 10;
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
            if (seed [i] != 10)
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
                seed [i] = 10;
            }
        }
        randomTmp[4] = t5 [s5];
        for (int i = 0; i < 6; i++)
        {
            if (seed [i] != 10)
            {
                randomTmp[5] = seed [i];
            }
        }
    }

    void try_to_power_off(int energy)
    {
        if (energy <= 2 && digitalRead(IS_IN_CHARGE) == 0)
        {
             power_off_count = millis()-deltaTime;
 
        }

        else
        deltaTime = millis();

        if (power_off_count - 12000000 >= 2)   //about one minutes 15 seconds .
        {
            for (uint8_t i = 0; i < MAX_LED_NUM; i++)
            {
                led.setPixelColor(i, 123, 0, 0);
                led2.setPixelColor(i, 123, 0, 0);
            }
            led.show();
            led2.show();
            digitalWrite(POWER_ON_OFF, HIGH);
        }
    }
int gesture_energy_calculate(int gyrox, int gyroy, int gyroz)/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
void DiceMode(int gyrox, int gyroy, int gyroz)
{
    if(dice_on == 1)
    {
        all_off();
        GetRandomArr();
        randomNum = random(1, 25);
        for (int i = 1; i <= 6 ; i++ )
        {
            matrix_effect(i, randomNum, randomTmp[i - 1], dic_line); //face、color、dice number、face msg
        }
        dice_on = 0;
    }
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
                if ((diceAnCount > 3))
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
                    if(fx_value != 3)
                    {
                        goto end_dice_mode;
                    }
                        matrix_effect(i, randomNum, randomTmp[i - 1], dic_line); //face、color、dice number、face msg
                        Serial.println("funcking in bro");
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

    // intButton();
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
void usual_set_fect(uint8_t faceNum, uint8_t colorNum, uint32_t index1, uint32_t index2, uint32_t index3, uint32_t index4)
{
    // intButton();
    readSerial();
    if (isAvailable)
    {
        read_serial();
    }
    buffer[6] = faceNum;
    buffer[7] = colorNum;
    buffer[8] = index1;
    buffer[9] = index2;
    buffer[10] = index3;
    buffer[11] = index4;
    runModule(RGB_SET_COLOR_1);
    memset(buffer, 0x00, sizeof(char) * BUFFSIZE);
}
//*****************//
void matrix_effect(uint8_t faceNum, uint8_t colorNum, uint8_t dicNum, unsigned char dic[6][4])
{
    usual_set_fect(faceNum, colorNum, dic[dicNum - 1][0], dic[dicNum - 1][1], dic[dicNum - 1][2], dic[dicNum - 1][3]);
}
int bat_low_counter = 0;
void power_managment()
{
    int energy = gesture_energy_calculate(gyrox_global, gyroy_global, gyroz_global);


    bat_voltage = analogRead(VOLTAGE_DETECT_PORT)*3.3/1024.0*10;
    try_to_power_off(energy);
 
    if(bat_voltage<=3.5 && digitalRead(IS_IN_CHARGE) == 0)
    {
        bat_low_counter++;
        if((bat_low_counter>=200) || (fx_value!=0) && (bat_low_counter>=20)&& (fx_value != 3))
        {
            for(int i=0; i<75; i++)
            {
                led.setPixelColor(i,200,0,0);
                led2.setPixelColor(i,200,0,0);
            }  
            led.show();
            led2.show();
            delay(2000);
            digitalWrite(POWER_ON_OFF, HIGH);
        } 
    }
    // else 
    //     bat_low_counter=0;

    if(is_dev_in_charge)
    {
        fx_value = 0;
        // power_off_count = 0;
        breath_light(0,10,180,2);
    }
}
void setup()
{
    initKey = 0;
    led.begin();
    led2.begin();
    led.setBrightness(brightness_value);
    led2.setBrightness(brightness_value);
    delay(200);
    randomNum = (analogRead(A0)+analogRead(A2))%5;
    usual_set_fect(1, randomNum, 0, 7, 41, 193);
    usual_set_fect(2, randomNum, 0, 7, 41, 192);
    usual_set_fect(3, randomNum, 0, 0, 0, 0);
    usual_set_fect(4, randomNum, 0, 0, 0, 0);
    usual_set_fect(5, randomNum, 0, 7, 41, 64);
    usual_set_fect(6, randomNum, 0, 7, 41, 192);
    Serial.begin(9600);

    // pinMode(MULTIFUNCTION, INPUT);
    pinMode(IS_IN_CHARGE, INPUT);
    pinMode(POWER_ON_OFF, OUTPUT);
    digitalWrite(POWER_ON_OFF, LOW);
    lastTime = millis();
    Serial.print(F("Version: "));
    Serial.println(mVersion);
    Wire.begin();
    // Initialise Serial Communication, set baud rate = 9600


    // Start I2C Transmission
    Wire.beginTransmission(BMG160_ADDR);
    // Select Range register
    Wire.write(0x0F);
    // Configure 80 scale range 2000 dps
    // Configure 81 scale range 1000 dps
    // Configure 82 scale range 500 dps
    // Configure 83 scale range 250 dps
    // Configure 84 scale range 125 dps
    Wire.write(0x80);
    // Stop I2C Transmission
    Wire.endTransmission();

    // Start I2C Transmission
    Wire.beginTransmission(BMG160_ADDR);
    // Select Bandwidth register
    Wire.write(0x10);
    // Set bandwidth 04 = 200 Hz
    // Set bandwidth 03 = 400 Hz
    // Set bandwidth 02 = 1000 Hz
    Wire.write(0x04); 
    // Stop I2C Transmission
    Wire.endTransmission();
    delay(1000);
    // attachInterrupt(digitalPinToInterrupt(2), intFunc, FALLING );
}


// inline void intButton()
// {
//     // dice_on = 1;
//     if(digitalRead(MULTIFUNCTION) == 0 && last_key_value == 1)
//     {
//         fx_value++;
//     }

//     if(fx_value>5)
//     {
//         fx_value = 1;
//     }
//     if(digitalRead(IS_IN_CHARGE)== 1 && last_charge_value == 0)
//     {
//         fx_value = 0;
//         all_off();
//     }
//     if(digitalRead(IS_IN_CHARGE)== 0 && last_charge_value == 1)
//     {
//         fx_value = 0;
//         all_off();
//     }
//     last_key_value = digitalRead(MULTIFUNCTION);
//     MULTIFUNCTION;
// }

void loop()
{
   
    is_dice_mode = 0;
 
    readSerial();
    if (isAvailable)
    {
        read_serial();
    }
    if(last_fx_value !=  fx_value)
    {
        all_off();
    }
    if(fx_value == 0 && life == 1){
        all_off();
        life = 0;
    }
    if(fx_value != 0)
    {
       show_the_fx(fx_value); 
    }

    last_fx_value = fx_value;
    unsigned int data[6];
    // Start I2C Transmission
    Wire.beginTransmission(BMG160_ADDR);
    // Select Gyrometer data register
    Wire.write(0x02);
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 6 bytes of data
    Wire.requestFrom(BMG160_ADDR, 6);
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb

    if(Wire.available() == 6)
    {
        data[0] = Wire.read();
        data[1] = Wire.read();
        data[2] = Wire.read();
        data[3] = Wire.read();
        data[4] = Wire.read();
        data[5] = Wire.read();
    }
    int xGyro = ((data[1] * 256) + data[0]);
    int yGyro = ((data[3] * 256) + data[2]);
    int zGyro = ((data[5] * 256) + data[4]);
    gyrox_global = xGyro>>6;
    gyroy_global = yGyro>>6;
    gyroz_global = zGyro>>6;

    gyrox_global = (gyrox_global/8.0);
    gyroy_global = (gyroy_global/8.0);
    gyroz_global = (gyroz_global/8.0);


    // gyrox_global = IntegratorFilterX.input(gyrox_global);
    // gyroy_global = IntegratorFilterY.input(gyroy_global);
    // gyroz_global = IntegratorFilterZ.input(gyroz_global);
    if(!is_in_dev_mode)
    {
        power_managment();
    }
    if(is_in_dev_mode)
    {
        deltaTime = millis(); 
    }
    
    if ((millis() - lastTime >= 33))//&&is_dev_in_charge==0)
    {
        lastTime = millis();
//        Serial.print(-gyroy_global);
//        Serial.print(F(","));
//        Serial.print(gyrox_global);
//        Serial.print(F(","));
//        Serial.println(-gyroz_global); 

         Serial.write(0xf0);
         Serial.write(0x55);
         Serial.write(0x06);
         Serial.write(data[0]);
         Serial.write(data[1]);
         Serial.write(data[2]);
         Serial.write(data[3]);
         Serial.write(data[4]);
         Serial.write(data[5]);
         Serial.write(0xff);
// Serial.println(digitalRead(IS_IN_CHARGE));
    }
    last_gyrox = gyrox_global;
    last_gyroy = gyroy_global;
    last_gyroz = gyroz_global;
    //last_key_value = digitalRead(IS_IN_CHARGE);

    if(is_dev_in_charge == 1 && digitalRead(IS_IN_CHARGE) == 0 && is_in_dev_mode == 1)
    {
        is_in_dev_mode = 0;
        Serial.println("OUT OF DEV MODE");
    }
     is_dev_in_charge = digitalRead(IS_IN_CHARGE);
}

