#pragma once

#include "AC_Dancing_LED.h"
#include <AP_HAL/I2CDevice.h>

#define DANCING_LED_I2C_ADDRESS

class I2CLed : public Dancing_LED_Backend
{
public:
 I2CLed(AC_Dancing_LED &led);

protected:


    bool hw_init(void) override;
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;
    void hw_get_rgb(uint8_t *r, uint8_t *g, uint8_t *b) override;

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    void _timer(void);

    struct {
        uint8_t r, g, b;
    } rgb;

    bool _need_update;
};

