#pragma once

#include "AC_Dancing_LED.h"
#include <SRV_Channel/SRV_Channel.h>
#include "AP_Math/AP_Math.h"

class PWM3Led : public Dancing_LED_Backend
{
public:
    PWM3Led(AC_Dancing_LED& led);

protected:

    bool hw_init(void) override;
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;
    void hw_get_rgb(uint8_t *r, uint8_t *g, uint8_t *b) override;

private :

    SRV_Channel *_r;
    SRV_Channel *_g;
    SRV_Channel *_b;

};


