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
// Abort a stuck capture if NCS never rises (bad wiring)
#define CAPTURE_TIMEOUT_US 5000000UL

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
        bool timed_out = false;
        
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
            if ((micros() - start) > CAPTURE_TIMEOUT_US) {
                timed_out = true;
                break;
            }
        } while (!(pins & (1 << pin_ncs)));
        interrupts();
        
        uint32_t end = micros();
        uint32_t elapsed = end - start;
        
        if (timed_out) {
            Serial.println("capture timed out waiting for NCS high; check wiring");
        } else if (buff_i > 100 && elapsed > 0)
        {
            Serial.printf("len: %d -- micros %d - khz %d - khz limit %d\n", buff_i, elapsed, buff_i*8*1000/elapsed, n*1000/elapsed/2);
            for (size_t i = 0; i < buff_i; i++)
                Serial.printf("%02X, ", buff[i]);
            Serial.println("");
        }
        memset(buff, 0, buffsize);
    }
}
