// Copyright (c) 2020  Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
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

// https://github.com/ros-drivers/rosserial/pull/448
// #define ESP_SERIAL
// still not included in released version 0.9.1
// the following is a workaround
#undef ESP32
#include "ros.h"
#define ESP32

#include "Arduino.h"
#include "std_msgs/String.h"
#include "WiFi.h"
#include "esp_wifi.h"

// baud rate for serial connection
#define BAUDRATE (115200)

// maximum number of channels
#define MAX_CHANNEL (13)

// 11 ch in US, 13 ch in other countries
#define DEFAULT_N_CHANNEL (11)

// typical WiFi SSID beacon is 102.4 ms
#define DEFAULT_SCAN_DURATION (105)

// need some interval if you want to transport via WiFi
#define DEFAULT_SCAN_INTERVAL (0)

// maximum wait cycle
// if you have APs in 3 channels in your environment, the maximum wait time is like
// (3*DEFAULT_MAX_SKIP)*(DEFAULT_SCAN_DURATION+SCAN_INTERVAL) = (3*14)*105 = 42*105 = 
#define DEFAULT_MAX_SKIP (14)

// maximum queue size, ignore if exceeds
#define MAX_WAITING (128)

// verbosity
#define DEFAULT_VERBOSITY (true)

//declare reset function @ address 0
void(* resetFunc) (void) = 0;


ros::NodeHandle nh;
std_msgs::String wifi_scan_msg;
ros::Publisher wifi_scan_pub("wifi_scan_str", &wifi_scan_msg);

int max_skip = DEFAULT_MAX_SKIP;
int n_channel = DEFAULT_N_CHANNEL;
int scan_duration = DEFAULT_SCAN_DURATION;
int scan_interval = DEFAULT_SCAN_INTERVAL;
bool verbose = DEFAULT_VERBOSITY;


bool isScanning = false;
int scanningStart = 0;
int channel = 0;
int skip[MAX_CHANNEL];
int count[MAX_CHANNEL];
unsigned long lastseen[MAX_CHANNEL];
char buf[256];

// BSSID=17, SSID=32, CH=2, RSSI=4, sec=10, nsec=10, commas=5, total 80 + margin 20
char msg_buf[MAX_WAITING][100]; 
int waiting = 0;

void configure()
{
  if (!nh.getParam("~verbose", &verbose, 1, 500)) {
    verbose = DEFAULT_VERBOSITY;
  }
  if (!nh.getParam("~max_skip", &max_skip, 1, 500)) {
    max_skip = DEFAULT_MAX_SKIP;
  }
  if (!nh.getParam("~n_channel", &n_channel, 1, 500)) {
    n_channel = DEFAULT_N_CHANNEL;
  }
  if (!nh.getParam("~scan_duration", &scan_duration, 1, 500)) {
    scan_duration = DEFAULT_SCAN_DURATION;
  }
  if (!nh.getParam("~scan_interval", &scan_interval, 1, 500)) {
    scan_interval = DEFAULT_SCAN_INTERVAL;
  }

  if (verbose) {
    nh.loginfo("you can suppress loginfo by _verbose:=false");
    sprintf(buf, "max_skip:=%d", max_skip);
    nh.loginfo(buf);
    sprintf(buf, "n_channel:=%d", n_channel);
    nh.loginfo(buf);
    sprintf(buf, "scan_duration:=%d", scan_duration);
    nh.loginfo(buf);
    sprintf(buf, "scan_interval:=%d", scan_interval);
    nh.loginfo(buf);
  }

  nh.loginfo("Configuration updated");
  nh.spinOnce();
}

void setup()
{
  // init hardware
  nh.getHardware()->setBaud(BAUDRATE);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // init rosserial
  nh.initNode();
  nh.advertise(wifi_scan_pub);
  while(!nh.connected()) {
    nh.spinOnce();
    delay(100);
  }

  configure();

  // init internal state
  scanningStart = millis();
  memset(skip, 0, sizeof(skip));
  memset(count, 0, sizeof(count));
  memset(lastseen, 0, sizeof(lastseen));

  nh.loginfo("Setup done");
}

void loop()
{
  if (!nh.connected()) {
    // if it is disconnected reset
    resetFunc();
  }

  handleScan();
}

/*
 * kick wifi scan if needed
 * when a scan is completed put message strings into the waiting queue
 * handle waiting queue while waiting scan
 */
void handleScan()
{
  if (isScanning == false) {
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
      int n = WiFi.scanNetworks(true, false, false, scan_duration, channel+1);
      scanningStart = millis();
      count[channel] = 0;
      isScanning = true;
    } else {
      // skip until skip count
      count[channel] += 1;
      channel = (channel+1)%n_channel;
    }    
  }
  else {
    int n = 0;
    if ((n = WiFi.scanComplete()) >= 0) {
      // scan completed
      if (verbose) {
        sprintf(buf, "[ch:%2d][skip:%2d/%2d][%d APs] %3d ms, %5d ms since lastseen, ",
                channel+1, skip[channel], max_skip, n, millis()-scanningStart, millis()-lastseen[channel]);
        nh.loginfo(buf);
      }
      lastseen[channel] = millis();
      scanningStart = millis()+scan_interval;

      if (n == 0) {
	// increments skip count if no AP is found at the current channel
        skip[channel] = min(skip[channel]+1, max_skip);
      } else {
	// if APs are found, put string into the queue
        skip[channel] = 0;
        for (int i = 0; i < n && waiting < MAX_WAITING; ++i) {
          sprintf(msg_buf[waiting], "%s,%s,%d,%d,%d,%d", WiFi.BSSIDstr(i).c_str(), WiFi.SSID(i).c_str(),
                  WiFi.channel(i), WiFi.RSSI(i), nh.now().sec, nh.now().nsec);
	  waiting++;
        }
      }

      channel = (channel+1) % n_channel;
      isScanning = false;
    } else {
      // waiting scan result
      checkQueue();
    }
  }
}

void checkQueue()
{
  if (waiting > 0) {
    waiting--;
    wifi_scan_msg.data = msg_buf[waiting];
    wifi_scan_pub.publish(&wifi_scan_msg);
    nh.spinOnce();
  }
}
