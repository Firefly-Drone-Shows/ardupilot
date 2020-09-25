#ifndef AP_RTCM_COUNTER_H
#define AP_RTCM_COUNTER_H

#include <AP_HAL/AP_HAL.h>
#include <Filter/Filter.h>
#include "AP_GPS.h"

#define FILTER_SIZE 20

typedef AverageFilter<float,float,FILTER_SIZE> PackageAverage;

class RTCM_Counter
{
    struct PkgStat
    {
        // Mavlink channel index corresponding to serial port
        uint8_t channel_index{0};
        // Keeps periodicaly calculated packets amount
        PackageAverage average;
        // Last rtcm package sum reteived from gps
        uint16_t last_counted{0};
        // Average packages sum after insert
        float result{0};
        // Last time package sum read from gps
        uint32_t last_pckg_time_ms{0};
    };

public:
    RTCM_Counter(const AP_GPS &gps);

    // Returns 8bit state.  1 - bit represents status of rtcm channel (0 -
    uint8_t get_state(uint8_t index) const;

    void every_second_update();

private:
    const AP_GPS &gps;

    PkgStat pkg_stat[2];

    int first_channel_index{-1};
    int second_channel_index{-1};

    void detect_channel();
};

#endif // AP_RTCM_COUNTER_H
