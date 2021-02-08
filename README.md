# WiFi Scanner for ESP32/rosserial

This program scans WiFi (2.4GHz only) APs with ESP32-WROOM-32D board and publish `/wifi_scan_str` topic on ROS via rosserial.

### Optional hardware

- [Adafruit OLED 128x32 monochrome display](https://www.adafruit.com/product/931) to show scanning status
  - connect with Quiic (or I2C)

### `/wifi_scan_str` format
```
type: std_msgs/String
data: "<BSSID>,<SSID>,<CH>,<RSSI>,<sec>,<nsec>"
```

#### Example
```
data: "F3:42:EB:1B:D1:1A,NETWORK,1,-69,1612237458,30490966"
```

### Usage
- compile the program and write to your ESP32 board
```
$ rosrun rosserial_python serial_node.py _port:=<path/to/tty> _baud:=115200
```

### ROS Parameters
- **verbose** (bool): set verbosity (default **true**)
- **n_channel** (in): set number of channels (default **11** for US, 13 can be used in other countries)
- **scan_duration** (int): max scanning duration per channel (default **105**ms)
- **scan_interval** (int): scanning interval (default **0**ms)
- **max_skip** (int): scanner increments one for skip count up to max_skip if no APs is found at a certain channel. It can increase frequency to scan active channels. Please check the code for the detail. (default **14**)

### Requirements
- Espressif ESP32-WROOM-32D (tested)
- Serial connection (USB)

### Limitation
- WiFi transport cannot be used while WiFi is scanning.
- You can increase scan interval time to secure WiFi transmission time.

### License
MIT
