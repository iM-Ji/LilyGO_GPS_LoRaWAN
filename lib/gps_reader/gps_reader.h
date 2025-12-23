#pragma once
#include <string>

struct GPSData {
    bool valid;
    double lat;
    double lng;
};

GPSData parseNMEA(const std::string &nmea);
