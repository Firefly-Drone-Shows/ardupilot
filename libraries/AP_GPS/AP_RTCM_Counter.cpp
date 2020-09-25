#include <GCS_MAVLink/GCS.h>
#include "AP_RTCM_Counter.h"

extern const AP_HAL::HAL &hal;

RTCM_Counter::RTCM_Counter(const AP_GPS &gps_) : gps(gps_)
{

}

uint8_t RTCM_Counter::get_state(uint8_t index) const {
    // We are intersing only in 0 and 1 indexes
    if (index != 0 && index != 1)
        return 0;

    // rtcm recevie status
    uint8_t rtcm_status = 0;
    // Healthy?
    if (pkg_stat[index].last_pckg_time_ms > 0 && AP_HAL::millis() - pkg_stat[index].last_pckg_time_ms < 10000) {
        rtcm_status = 1 << 7;
    }

    // In general this rate is about 2.4 - 2.5 packets per sec. Multiply it by 10 and set it to remaining 7 bits
    float rate = pkg_stat[index].result;
    uint8_t rate10 = static_cast<uint8_t>(rate * 10) & 0x7F;

    rtcm_status |= rate10;

    return rtcm_status;
}


void RTCM_Counter::every_second_update() {

     if (first_channel_index < 0) {
         detect_channel();
     }

     if (second_channel_index < 0) {
         detect_channel();
     }

     if (first_channel_index > 0 && second_channel_index > 0) {
         if (first_channel_index < second_channel_index) {
            // Do swap channels
            int tmp = first_channel_index;
            first_channel_index = second_channel_index;
            second_channel_index = tmp;
         }
     }

     for (uint8_t i = 0; i < 2; i++) {
        uint16_t rtcm_pkgs_tmp = gps.get_rtcm_count(pkg_stat[i].channel_index);

        if (rtcm_pkgs_tmp > pkg_stat[i].last_counted) {
            pkg_stat[i].last_pckg_time_ms = AP_HAL::millis();
        }

        // In case of overflow
        if (pkg_stat[i].last_counted > rtcm_pkgs_tmp) {
            pkg_stat[i].last_counted = 0;
        }

        pkg_stat[i].result = pkg_stat[i].average.apply(rtcm_pkgs_tmp - pkg_stat[i].last_counted);
        pkg_stat[i].last_counted = rtcm_pkgs_tmp;
     }
}

void RTCM_Counter::detect_channel() {
    uint8_t active = gps.get_rtcm_channel_mask();

    for (uint8_t i = 0; i < 8; i++) {
        if ((active & (1U << i)) > 0 ) {
            // Active channel detected;
            if (first_channel_index < 0) {
                first_channel_index = i;
                pkg_stat[0].channel_index = i;
                hal.console->printf("First channel deteced %d\n", first_channel_index);
            }

            if (second_channel_index < 0 && first_channel_index != i) {
                second_channel_index = i;
                pkg_stat[1].channel_index = i;
                hal.console->printf("Second channel deteced %d\n", second_channel_index);

                // Do we need to swap channels to let primary channel obtain lowers channel index.
                if (second_channel_index < first_channel_index) {
                    PkgStat tmp = pkg_stat[0];
                    pkg_stat[0] = pkg_stat[1];
                    pkg_stat[1] = tmp;
                    first_channel_index = pkg_stat[0].channel_index;
                    second_channel_index = pkg_stat[1].channel_index;
                     hal.console->printf("Swapped!!!!!!!!!!\n");
                }
            }
        }
    }
}

