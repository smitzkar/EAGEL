# Welcome to EAGEL[^2]!  

A simple DIY "drone"[^1] streaming either live camera or infrared feed using Seeedstudio's Xiao ESP32-S3 Sense. Part of the [TechTales exhibition at UNI_VERSUM.](https://www.lifolab.de/techtales). 

[^1]: "Technically" NOT considered a drone -> potentially skirts legal restrictions on where it can be flown (check your local restrictions)

[^2]: Acronym: 	<ins>E</ins>AGEL 	<ins>A</ins>erostat, 	<ins>G</ins>iving 	<ins>E</ins>lectronics a 	<ins>L</ins>ift  
English "eagle" homophone to German "Igel" = "hedgehog" 
What better animal to associate with a balloon-lifted aerial camera than a near-blind spiky little critter?

## Features 

Choose between: 
1. Video livestream to browser with:  
a) included camera module  
b) upgraded camera (for example [OV5640](https://www.seeedstudio.com/OV5640-Camera-for-XIAO-ESP32S3-Sense-With-Heat-Sink-p-5739.html))

2. Display upscaled live feed from 8x8 infrared sensor (used in exhibition)  
[to do: measure runtime on full battery]

3. Replace our simple examples with your own ideas! Use it as a repeater for your ESP-NOW network, add various sensors, give it a little propeller to steer it, ... 

## Hardware

- [Seeed Studio XIAO ESP32S3 Sense](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)
- [Adafruit AMG8833 IR Thermal Camera Breakout](https://www.adafruit.com/product/3538)
- 340mAh LiPo battery
- 22in mylar ballon (or similar) -> doublecheck what we actually used!
- kite string
- optionally add NeoPixels or similar LEDs for visual feedback
- raspberry pi and [round display](https://www.waveshare.com/3.4inch-dsi-lcd-c.htm) (for infrared)
- browser on phone/tablet/laptop (for camera)

## Quick Start / Documentation 

See [docs/Build-Guide.md](docs/Build-Guide.md) for details on the physical setup and some napkin maths on helium/balloon requirements. 

### -[] should probably move this to Getting-Started.md 


Select the desired option by commenting out the others (only one may be active)
```C
//MARK: Select Camera (don't trust that the choosing already works)
//#define USE_THERMAL // connect to I2C (pins D4 (SDA) and D5 (SCL) on Xiao)
//#define USE_OG_CAM
#define USE_OV5640
```

Set your WiFi credentials (here an example with two options: hotspot, home)
```C
//MARK: === WIFI / OTA ===
const int numNetworks = 2;
const char* ssidList[] = {"SSID_home",     "SSID_hotspot"};
const char* passList[] = {"PASSWORD_home", "PASSWORD_hotspot"};
const char* HOSTNAME = "EAGEL_1"; 
``` 

Select board and port, then compile and upload.

Browser stuff ... 

### Todo: -[] Pi setup for 

## Summer School 2025

The original pre-infrared version was used as the starting point to build on top of for the students participating in the [2025 CityLab Berlin's Summer School](https://citylab-berlin.org/de/blog/citylab-summer-school-2025-stadt-wald-labor/). 

The results can be found here:  

https://github.com/NurzhanSeitzhanov/Not-a-Drone_Big-Data

https://github.com/thitrunganhnguyen/InTheWoods

## Links  

[Our website lifolab](https://www.lifolab.de)

[Fachgebiet Nachrichtenübertragung an der Technischen Universität Berlin](https://www.tu.berlin/nue)


