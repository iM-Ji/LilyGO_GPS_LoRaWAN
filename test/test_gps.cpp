#include <Arduino.h>
#include <unity.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;

void test_gps_parsing(void) {
    const char *nmea = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

    for(size_t i = 0; i < strlen(nmea); i++){
        gps.encode(nmea[i]);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.0001, 48.1173, gps.location.lat());
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 11.5166667, gps.location.lng());
}

void setup() {
    Serial.begin(115200);
    UNITY_BEGIN();
    RUN_TEST(test_gps_parsing);
    UNITY_END();
}

void loop() {}
