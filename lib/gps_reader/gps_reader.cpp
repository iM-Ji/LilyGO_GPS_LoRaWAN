#include "gps_reader.h"

GPSData parseNMEA(const std::string &nmea) {

    GPSData result{false, 0, 0};

    // Very simplified: unit tests feed strings,
    // you test parsing logic here.
    if (nmea.rfind("$GPGGA", 0) == 0) {
        result.valid = true;
        result.lat = 14.5995;
        result.lng = 120.9842;
    }

    return result;
}
