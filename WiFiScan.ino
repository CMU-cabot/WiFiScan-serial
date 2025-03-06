// Copyright (c) 2020  Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

//
// This Arduino code will publish WiFi scan result nas string message
// topic: wifi_scan_str
// format: BSSID,SSID,Channel,RSSI,sec,nsec
//

#include "CaBotHandle.hpp"  // alternative implementation not using ros.h

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include "Arduino.h"
#include "WiFi.h"
#include "esp_wifi.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET 4
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// baud rate for serial connection
#define BAUDRATE (115200)

// maximum number of channels
#define MAX_CHANNEL (13)

// 11 ch in US, 13 ch in other countries
#define DEFAULT_N_CHANNEL (11)

// typical WiFi SSID beacon is 102.4 ms
#define DEFAULT_SCAN_DURATION (105)

// need some interval if you want to transport via WiFi
#define DEFAULT_SCAN_INTERVAL (5)

// maximum wait cycle
// if you have APs in 3 channels in your environment, the maximum wait time is
// like (3*DEFAULT_MAX_SKIP)*(DEFAULT_SCAN_DURATION+SCAN_INTERVAL) = (3*14)*105
// = 42*105 =
#define DEFAULT_MAX_SKIP (14)

// maximum queue size, ignore if exceeds
#define MAX_WAITING (128)

// verbosity
#define DEFAULT_VERBOSITY (1)

// declare reset function @ address 0
// this can cause undesirable result with multiple core system like ESP32
// void(* resetFunc) (void) = 0;

#define PARAM_TIMEOUT (500)

bool is_display_available = false;

cabot::Handle ch;

int max_skip = DEFAULT_MAX_SKIP;
int n_channel = DEFAULT_N_CHANNEL;
int scan_duration = DEFAULT_SCAN_DURATION;
int scan_interval = DEFAULT_SCAN_INTERVAL;
int verbose = DEFAULT_VERBOSITY;

bool isScanning = false;
unsigned long scanningStart = 0;
int channel = 0;
int skip[MAX_CHANNEL];
int count[MAX_CHANNEL];
int aps[MAX_CHANNEL];
unsigned long lastseen[MAX_CHANNEL];
char buf[256];

// BSSID=17, SSID=32, CH=2, RSSI=4, sec=10, nsec=10, commas=5, total 80 + margin
// 20
char msg_buf[MAX_WAITING][100];
int waiting = 0;
int all_zero_count = 0;

void loginfo(char *buf) {
  ch.loginfo(buf);

  if (!is_display_available) {
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F(buf));
  display.display();
}

void showText(char *buf, int row) {
  if (!is_display_available) {
    return;
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 8 * row);
  display.println(F(buf));
}

void configure_param(char *name, int *val) {
  char buf[64];

  snprintf(buf, sizeof(buf), "param %s: ", name);
  if (!ch.getParam(name, val, 1, PARAM_TIMEOUT)) {
    snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "(default) ");
  }
  snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "%d", *val);
  ch.loginfo(buf);
}

void configure() {
  configure_param("verbose", &verbose);
  configure_param("max_skip", &max_skip);
  configure_param("n_channel", &n_channel);
  configure_param("scan_duration", &scan_duration);
  configure_param("scan_interval", &scan_interval);

  if (verbose) {
    loginfo("you can suppress loginfo by _verbose:=0");
  }

  loginfo("Configuration updated");
}

void showAppStatus() {
  display.clearDisplay();
  showText("WiFi Scanner Ready", 0);
  showText("Waiting Connection", 1);
  sprintf(buf, "Time: %7.1f", millis() / 1000.0);
  showText(buf, 2);
  display.display();
}

void setup() {
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    is_display_available = true;
    showAppStatus();
  }

  // init hardware
  ch.setBaudRate(BAUDRATE);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // init rosserial
  ch.init();

  int wait = millis();
  while (!ch.connected()) {
    showAppStatus();
    ch.spinOnce();
    delay(100);

    // TODO
    // very rarely, it can be astate that the serial cannot be connected
    // so reset every 10 seconds
    if (millis() - wait > 10 * 1000) {
      restart();
    }
  }

  configure();

  // init internal state
  scanningStart = millis();
  memset(skip, 0, sizeof(skip));
  memset(count, 0, sizeof(count));
  memset(aps, 0, sizeof(count));
  memset(lastseen, 0, sizeof(lastseen));

  loginfo("Setup done");
}

void loop() {
  handleScan();
  ch.spinOnce();
}

void showScanStatus() {
  if (!is_display_available) {
    return;
  }
  sprintf(buf, "");
  for (int i = 0; i < n_channel; i++) {
    sprintf(buf + strlen(buf), "%2d:%3d|", i + 1, aps[i],
            (millis() - lastseen[i]) / 1000.0);
  }
  sprintf(buf + strlen(buf), "%2d", channel + 1);
  sprintf(buf + strlen(buf), "%s", ch.connected() ? "*" : "-");
  sprintf(buf + strlen(buf), ",%s", isScanning ? "*" : "-");
  sprintf(buf + strlen(buf), "%2d", all_zero_count);
  display.clearDisplay();
  showText(buf, 0);
  display.display();
}

/*
 * kick wifi scan if needed
 * when a scan is completed put message strings into the waiting queue
 * handle waiting queue while waiting scan
 */
void handleScan() {
  if (isScanning == false) {
    if (!ch.connected()) {
      restart();
    }
    // TODO
    // not sure why, but when the serial is disconnected
    // sometimes it can be a strange state that nh.connected() == true and WiFi
    // Scan does not work restart the hardware if the WiFi scan returns no
    // result for 10 consequtive cycles
    if (channel == 0) {
      checkZeroScan(10);
    }
    if (millis() <= scanningStart) {
      // during scan interval
      checkQueue();
      return;
    }

    if (count[channel] >= skip[channel]) {
      // start scan for the current channcel
      //
      // definition
      // int16_t scanNetworks(bool async = false,
      //                      bool show_hidden = false,
      //                      bool passive = false,
      //                      uint32_t max_ms_per_chan = 300,
      //                      uint8_t channel = 0);
      int n = WiFi.scanNetworks(true, false, false, scan_duration, channel + 1);
      scanningStart = millis();
      count[channel] = 0;
      isScanning = true;
      showScanStatus();
    } else {
      // skip until skip count
      count[channel] += 1;
      channel = (channel + 1) % n_channel;
    }
  } else {
    int n = 0;
    if ((n = WiFi.scanComplete()) >= 0) {
      // scan completed
      aps[channel] = n;
      showScanStatus();
      if (verbose) {
        sprintf(buf, "[ch:%2d][%3dAPs][skip:%2d/%2d]%3dms,%5dms", channel + 1,
                n, skip[channel], max_skip, millis() - scanningStart,
                millis() - lastseen[channel]);
        ch.loginfo(buf);
      }
      lastseen[channel] = millis();
      scanningStart = millis() + scan_interval;

      if (n == 0) {
        // increments skip count if no AP is found at the current channel
        skip[channel] = min(skip[channel] + 1, max_skip);
        ch.sync();
      } else {
        // if APs are found, put string into the queue
        skip[channel] = 0;
        for (int i = 0; i < n && waiting < MAX_WAITING; ++i) {
          String name = WiFi.SSID(i);
          name.replace(",", " ");
          sprintf(msg_buf[waiting], "%s,%s,%d,%d,%d,%d",
                  WiFi.BSSIDstr(i).c_str(), name.c_str(), WiFi.channel(i),
                  WiFi.RSSI(i), ch.now().sec, ch.now().nsec);
          waiting++;
        }
      }

      channel = (channel + 1) % n_channel;
      isScanning = false;
    } else {
      // waiting scan result
      checkQueue();
    }
  }
}

void checkQueue() {
  if (waiting > 0) {
    waiting--;
    ch.publish(0x17, msg_buf[waiting], strlen(msg_buf[waiting]));
  }
}

void checkZeroScan(int maximum) {
  bool all_zero = true;
  for (int i = 0; i < n_channel; i++) {
    all_zero = all_zero && aps[i] == 0;
  }
  if (all_zero) {
    all_zero_count++;
    if (all_zero_count > maximum) {
      restart();
    }
  } else {
    all_zero_count = 0;
  }
}

void restart() {
  display.clearDisplay();
  showText("Restart ESP", 0);
  display.display();
  ESP.restart();
}
