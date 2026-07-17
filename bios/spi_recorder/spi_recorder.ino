// wiring example for ripping a PMW3360 SROM: https://i.imgur.com/EspAlvz.jpeg
// set the board to 240mhz or higher for best results (WARNING: higher than 240mhz only works with USB if you overvolt the MCU)
// this implements reading SPI mode 3. if you want a different mode, you need to edit these two lines:
//        uint32_t clockval = (1 << pin_clock);
//            if (newclock && !clockval && buff_i < buffsize)


#include <pico/stdlib.h>

#define buffsize 50000
#define pin_ncs 19
#define pin_dat1 17
#define pin_dat2 18
#define pin_clock 16

uint8_t buff[buffsize];

void setup()
{
    Serial.begin(2000000);
    
    pinMode(pin_ncs, INPUT);
    pinMode(pin_dat1, INPUT);
    pinMode(pin_dat2, INPUT);
    pinMode(pin_clock, INPUT);
    
    delay(3000);
    Serial.println("booted");
    
    memset(buff, 0, buffsize);
}

void loop()
{
    uint32_t pins = gpio_get_all();
    if (!(pins & (1 << pin_ncs)))
    {   
        uint32_t buff_i = 0;
        uint32_t clockval = (1 << pin_clock);
        uint32_t start = micros();
        uint32_t n = 0;
        uint8_t bit = 0;
        
        noInterrupts();
        do
        {
            n++;
            uint32_t newclock = pins & (1 << pin_clock);
            if (newclock && !clockval && buff_i < buffsize)
            {
                buff[buff_i] |= (!!(pins & (1 << pin_dat1))) << (7-bit);
                bit += 1;
                if (bit == 8)
                {
                    bit = 0;
                    buff_i += 1;
                }
            }
            clockval = newclock;
            pins = gpio_get_all();
        } while (!(pins & (1 << pin_ncs)));
        interrupts();
        
        uint32_t end = micros();
        
        if (buff_i > 100)
        {
            Serial.printf("len: %d -- micros %d - khz %d - khz limit %d\n", buff_i, end-start, buff_i*8*1000/(end-start), n*1000/(end-start)/2);
            for (size_t i = 0; i < buff_i; i++)
                Serial.printf("%02X, ", buff[i]);
            Serial.println("");
        }
        memset(buff, 0, buffsize);
    }
}