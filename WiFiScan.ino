// based on an example of ESP32 SDK
// https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiScan/WiFiScan.ino
/*
 *  This sketch demonstrates how to scan WiFi networks.
 *  The API is almost the same as with the WiFi Shield library,
 *  the most obvious difference being the different file you need to include:
 */
#include "WiFi.h"

void setup()
{
    Serial.begin(115200);

    // Set WiFi to station mode and disconnect from an AP if it was previously connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    Serial.println("Setup done");
}

void loop()
{
    int start = millis();

    // WiFi.scanNetworks will return the number of networks found

    // int n = WiFi.scanNetworks();
    
    // definition
    // int16_t scanNetworks(bool async = false, bool show_hidden = false, bool passive = false, uint32_t max_ms_per_chan = 300, uint8_t channel = 0);
    // it takes a longer time (about 3-5 seconds) for the first time of scanning and then it takes 
    // 300ms -> about 2 seconds
    // 200ms -> about 1.5 seconds
    // 100ms -> about 1.1 seconds
    //  80ms -> about 0.9 seconds

    int n = WiFi.scanNetworks(false, false, false, 80, 0);
    
    Serial.print("scan done ");
    Serial.print(millis()-start);
    Serial.println("ms");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i) {
            // Print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
	    // Serial.print(WiFi.SSID(i));
	    // print mac address (not sure it is fixed)
            Serial.print(WiFi.BSSIDstr(i));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(")");
            Serial.print("[");
            Serial.print(WiFi.channel(i));
            Serial.print("]");
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
        }
    }
    Serial.println("");

    // Wait a bit before scanning again
    delay(0);
}
