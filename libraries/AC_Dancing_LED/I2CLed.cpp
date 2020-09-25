#include "I2CLed.h"
#include <utility>

extern const AP_HAL::HAL& hal;

I2CLed::I2CLed(AC_Dancing_LED &led) : Dancing_LED_Backend(led){}

bool I2CLed::hw_init()
{

    _dev = std::move(hal.i2c_mgr->get_device(1, _led._i2c_address));

     if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
     }

    // give back i2c semaphore
    _dev->get_semaphore()->give();

    _dev -> register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&I2CLed::_timer, void));

    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool I2CLed::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb.r = red;
    rgb.g = green;
    rgb.b = blue;
    _need_update = true;
    return true;
}

void I2CLed::_timer(void)
{
    if (!_need_update) {
        return;
    }
    _need_update = false;

    uint8_t transaction[] = {rgb.r, rgb.g, rgb.b};

    _dev->transfer(transaction, sizeof(transaction), nullptr, 0);
}

void  I2CLed::hw_get_rgb(uint8_t *r, uint8_t *g, uint8_t *b) {
    // cannot get real values from i2c led
    // so take last values that was set with set_color method
    *r = _red_curr;
    *g = _green_curr;
    *b = _blue_curr;
}
