//MARK: Select Camera (don't trust that the choosing already works)
#define USE_THERMAL // connect to I2C (pins D4 (SDA) and D5 (SCL) on Xiao)
//#define USE_OG_CAM
//#define USE_OV5640

//MARK: Give it a name
const char* HOSTNAME = "EAGEL_1";

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <FastLED.h>

#ifdef USE_THERMAL
    #include <Wire.h>
    #include <Adafruit_AMG88xx.h>
    // additions to enable 8x8 IR grideye / thermal camera
    Adafruit_AMG88xx amg;
    WiFiServer thermalServer(81);  // simple TCP server for 8×8 data stream
    void TaskThermal(void* pvParameters);
    #define TEMP_MIN 12 // degree C
    #define TEMP_MAX 32 // degree C
    #define TEMP_THRESHOLD 22 // for old threshold
    #define ALERT_DELTA 6.0 // for relative threshold
    #define SEND_RAW true  // set true to send float data (4 bytes × 64 = 256 bytes)
    volatile bool thermalAlert = false; // Shared flag to signal high temperature detected
#endif

#ifdef USE_OV5640
    #include "esp_camera.h"
    #define CAMERA_MODEL_XIAO_ESP32S3
    #include "camera_pins.h"
    #include "app_httpd.h"
    void TaskCamera(void* pvParameters);
    // Camera object
    OV5640 ov5640;
#endif

#ifdef USE_OG_CAM 
    #include "ESP32_OV5640_AF.h"
    #include "esp_camera.h"
    #define CAMERA_MODEL_XIAO_ESP32S3
    #include "camera_pins.h"
    #include "app_httpd.h"
#endif



//MARK: === WIFI / OTA ===
const char* ssidList[] = {"KarlPhone", "Healthy_Design_&_Sick_Machines"};
const char* passList[] = {"Aldebaran", "Bakterien_und_Viren"};
const int numNetworks = 2;
// const char* HOSTNAME = "EAGEL_1";
// const char* ssid = "KarlPhone";
// const char* password = "Aldebaran";



// === LED ===
#define DATA_PIN_1 4
#define DATA_PIN_2 9
#define NUM_LEDS_PER_STRIP 3
#define LED_CHILL_BRIGHTNESS 4 // regular blinking
#define LED_ALERT_BRIGHTNESS 255 // ALERT

CRGB leftArmLeds[NUM_LEDS_PER_STRIP];
CRGB rightArmLeds[NUM_LEDS_PER_STRIP];

// Forward declarations
void TaskOTA(void* pvParameters);
// void TaskCamera(void* pvParameters);
void TaskLED(void* pvParameters);
// void TaskThermal(void* pvParameters); // for grideye

struct FadeState {
    int brightness = 0;
    int fadeAmount = 5;
    CRGB lastColour = CRGB::Black;
    int lastSpeed = -1;
};
FadeState leftState, rightState;

struct RadiateState {
    int step = 0;
    int direction = 1; // 1 = outward, -1 = inward
    uint8_t brightness[3] = {0,0,0};
};

struct ArmState {
    float phase = 0.0;           // Current phase in the animation cycle (0.0 to 2π)
    uint8_t brightness[3] = {0, 0, 0};
};

// LED helper types
typedef void (*AnimationFunction)(CRGB*, int);
void controlStrip(CRGB* strip, int numLeds, AnimationFunction animation);
//void v_blinkFade(CRGB* strip, int numLeds, CRGB colour=CRGB::Blue, int speed = 50);
void v_blinkFade(CRGB* strip, int numLeds, CRGB colour, int speed, FadeState& state);
void v_blinkRadiate(CRGB* strip, int numLeds, CRGB colour, int speed);
void v_blinkRadiateEdge(CRGB* strip, int numLeds, CRGB colour, int speed, RadiateState& state);

//===================================================================================================

//MARK: SETUP
void setup() {
    Serial.begin(115200);
    delay(500);

    // LED setup (FastLED init will happen inside LED task)
    
    // OTA task - Core 0, higher priority
    xTaskCreatePinnedToCore(TaskOTA, "TaskOTA", 4096, NULL, 2, NULL, 0);

    // // Camera task - Core 0, medium priority
    // xTaskCreatePinnedToCore(TaskCamera, "TaskCamera", 8192, NULL, 1, NULL, 0);

    // Thermal task - Core 0, medium priority (as replacement for Camera)
    xTaskCreatePinnedToCore(TaskThermal, "TaskThermal", 4096, NULL, 1, NULL, 0);


    // LED task - Core 1, low priority
    xTaskCreatePinnedToCore(TaskLED, "TaskLED", 4096, NULL, 1, NULL, 1);
}

//MARK: LOOP
void loop() {
    // empty — tasks handle everything
}

//=====================================================================================================

/*=========================================================
  TASK: OTA (Core 0)
=========================================================*/
void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    for (int i = 0; i < numNetworks; i++) {
        Serial.printf("[WiFi] Trying network %s...\n", ssidList[i]);
        WiFi.begin(ssidList[i], passList[i]);

        unsigned long startAttempt = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 8000) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\n[WiFi] Connected to %s\n", ssidList[i]);
            Serial.println("[WiFi] IP address: " + WiFi.localIP().toString());
            return;
        } else {
            Serial.printf("\n[WiFi] Failed to connect to %s\n", ssidList[i]);
            WiFi.disconnect(true);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }

    Serial.println("[WiFi] No networks connected, retrying soon...");
}

void TaskOTA(void* pvParameters){
    Serial.println("[OTA] Task started on core " + String(xPortGetCoreID()));

    connectWiFi();

    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.begin();

    for(;;){
        ArduinoOTA.handle();

        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[WiFi] Disconnected, attempting reconnect...");
            connectWiFi();
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); // less frequent check to reduce CPU load
    }
}


/*=========================================================
  TASK: Thermal Camera / Grideye (Core 0)
=========================================================*/
#ifdef USE_THERMAL
void TaskThermal(void* pvParameters){
    Serial.println("[Thermal] Task started on core " + String(xPortGetCoreID()));

    connectWiFi();

    if(!amg.begin()){
        Serial.println("[Thermal] AMG8833 not found!");
        vTaskDelete(NULL);
    }

    thermalServer.begin();
    Serial.println("[Thermal] Server started on port 81");

    for(;;){
        WiFiClient client = thermalServer.available();
        if(client){
            Serial.println("[Thermal] Client connected");
            float pixels[64];
            uint8_t scaled[64];

            while (client.connected()) {
                amg.readPixels(pixels);
                bool alert = false; // reset 


                // for (int i = 0; i < 64; i++) {
                //     scaled[i] = map(pixels[i], TEMP_MIN, TEMP_MAX, 0, 255);
                //     if (pixels[i] > TEMP_THRESHOLD) { // adjust threshold to your environment
                //         alert = true;
                //     }
                // }

                // replace the alert logic to relative threshold:
                float minTemp = pixels[0];
                float maxTemp = pixels[0];
                for (int i = 0; i < 64; i++) {
                    if (pixels[i] < minTemp) minTemp = pixels[i];
                    if (pixels[i] > maxTemp) maxTemp = pixels[i];
                }
                // Alert if temperature delta is large (motion/person detected)
                float delta = maxTemp - minTemp;
                alert = (delta > ALERT_DELTA);  

                thermalAlert = alert;  // share state with LED task

                size_t bytesSent;
                if (SEND_RAW) {
                    // Send floats (256 bytes total)
                    bytesSent = client.write((uint8_t*)pixels, sizeof(pixels));
                } else {
                    // Send scaled (64 bytes total)
                    bytesSent = client.write(scaled, sizeof(scaled));
                }

                client.flush(); // not really necessary, but...

                if (bytesSent != (SEND_RAW ? sizeof(pixels) : sizeof(scaled))) {
                    Serial.println("[Thermal] Warning: partial write!");
                }
                vTaskDelay(100 / portTICK_PERIOD_MS); // ~10 fps
            }


            client.stop();
            Serial.println("[Thermal] Client disconnected");
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
#endif

/*=========================================================
  TASK: Camera (Core 0)
=========================================================*/
#ifdef USE_OV5640
void TaskCamera(void* pvParameters){
    Serial.println("[Camera] Task started on core " + String(xPortGetCoreID()));

    // --- Camera Init ---
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_UXGA;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 2;

    esp_err_t err = esp_camera_init(&config);
    if(err != ESP_OK){
        Serial.printf("Camera init failed with error 0x%x\n", err);
        vTaskDelete(NULL);
    }

    sensor_t* s = esp_camera_sensor_get();
    ov5640.start(s);
    ov5640.focusInit();
    ov5640.autoFocusMode();

    startCameraServer(); // MJPEG stream

    for(;;){
        // Optionally: monitor autofocus
        uint8_t rc = ov5640.getFWStatus();
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
#endif

/*=========================================================
  TASK: LED Animations (Core 1)
=========================================================*/

void TaskLED(void* pvParameters){
    Serial.println("[LED] Task started on core " + String(xPortGetCoreID()));

    FastLED.addLeds<WS2812B, DATA_PIN_1, GRB>(leftArmLeds, NUM_LEDS_PER_STRIP);
    FastLED.addLeds<WS2812B, DATA_PIN_2, GRB>(rightArmLeds, NUM_LEDS_PER_STRIP);
    int _brightness = LED_CHILL_BRIGHTNESS;
    FastLED.setBrightness(_brightness);  // set global brightness limit using "private" variable
    FastLED.clear();
    FastLED.show();

    ArmState leftState, rightState;
    // FadeState leftFadeState, rightFadeState;

    for(;;){
        if(thermalAlert){
            if (_brightness != LED_ALERT_BRIGHTNESS){
                _brightness = LED_ALERT_BRIGHTNESS;
                FastLED.setBrightness(_brightness); // max brightness for alert
            }
            v_radiateOutward(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 15, leftState);
            v_radiateOutward(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 15, rightState);
        } else {
            if (_brightness != LED_CHILL_BRIGHTNESS){
                _brightness = LED_CHILL_BRIGHTNESS;
                FastLED.setBrightness(_brightness); // back to regular
            }
            v_radiateOutward(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Green, 90, leftState);
            v_radiateOutward(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 90, rightState);
        }
    
        // if (thermalAlert) {
        //     // 🔥 Warning animation
        //     FastLED.setBrightness(LED_ALERT_BRIGHTNESS);  // set global brightness limit
        //     v_blinkFade(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 5, leftFadeState);
        //     v_blinkFade(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 5, rightFadeState);
        // } else {
        //     // 🌈 Normal animation
        //     FastLED.setBrightness(LED_CHILL_BRIGHTNESS);  // set global brightness limit
        //     v_blinkFade(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Green, 40, leftFadeState);
        //     v_blinkFade(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 40, rightFadeState);
        // }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


// void TaskLED(void* pvParameters){
//     Serial.println("[LED] Task started on core " + String(xPortGetCoreID()));

//     FastLED.addLeds<WS2812B, DATA_PIN_1, GRB>(leftArmLeds, NUM_LEDS_PER_STRIP);
//     FastLED.addLeds<WS2812B, DATA_PIN_2, GRB>(rightArmLeds, NUM_LEDS_PER_STRIP);
//     FastLED.clear();
//     FastLED.show();

//     for(;;){
//         v_blinkFade(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Green);
//         v_blinkFade(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red);
//     }
// // }

// void TaskLED(void* pvParameters){
//     Serial.println("[LED] Task started on core " + String(xPortGetCoreID()));

//     FastLED.addLeds<WS2812B, DATA_PIN_1, GRB>(leftArmLeds, NUM_LEDS_PER_STRIP);
//     FastLED.addLeds<WS2812B, DATA_PIN_2, GRB>(rightArmLeds, NUM_LEDS_PER_STRIP);
//     FastLED.clear();
//     FastLED.show();

//     static RadiateState leftState, rightState;

//     for(;;){
//         // if (thermalAlert) {
//         //     // 🔥 Warning animation
//         //     v_blinkFade(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 5);
//         //     //v_blinkFade(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 5);
//         // } else {
//         //     // 🌈 Normal animation
//         //     v_blinkFade(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Green, 40);
//         //     //v_blinkFade(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 40);
//         // }

//         // if (thermalAlert) {
//         //     v_blinkFade(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 5, leftState);
//         //     v_blinkFade(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 5, rightState);
//         // } else {
//         //     v_blinkFade(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Green, 40, leftState);
//         //     v_blinkFade(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 40, rightState);
//         // }

//         // if (thermalAlert) {
//         //     v_blinkRadiate(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 50);
//         //     v_blinkRadiate(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 50);
//         // } else {
//         //     v_blinkRadiate(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Green, 80);
//         //     v_blinkRadiate(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 80);
//         // }

//          if(thermalAlert){
//             v_blinkRadiateEdge(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 20, leftState);
//             v_blinkRadiateEdge(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 20, rightState);
//         } else {
//             v_blinkRadiateEdge(leftArmLeds, NUM_LEDS_PER_STRIP, CRGB::Green, 200, leftState);
//             v_blinkRadiateEdge(rightArmLeds, NUM_LEDS_PER_STRIP, CRGB::Red, 200, rightState);
//         }

//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
// }

/*=========================================================
  LED Helper Functions
=========================================================*/
void controlStrip(CRGB* strip, int numLeds, AnimationFunction animation){
    animation(strip, numLeds);
    FastLED.show();
}


void v_radiateOutward(CRGB* strip, int numLeds, CRGB colour, int speed, ArmState& state) {
    // Speed controls how fast the phase advances (smaller = faster animation)
    const float phaseIncrement = 0.05 * (50.0 / speed);
    
    // Advance phase
    state.phase += phaseIncrement;
    if(state.phase > TWO_PI) state.phase -= TWO_PI;
    
    // Calculate brightness for each LED based on phase
    // Center LED (1) leads, then edges (0 and 2) follow with delay
    for(int i = 0; i < numLeds; i++){
        float ledPhase;
        
        if(i == 1) {
            // Center LED - no delay
            ledPhase = state.phase;
        } else {
            // Edge LEDs - delayed by π/2 (quarter cycle)
            ledPhase = state.phase - (PI / 2.0);
            if(ledPhase < 0) ledPhase += TWO_PI;
        }
        
        // Use sine wave for smooth fade (0 to 255)
        float sineValue = sin(ledPhase);
        // Map from [-1, 1] to [0, 255]
        uint8_t targetBrightness = (uint8_t)((sineValue + 1.0) * 127.5);
        
        // Smooth transition to target brightness
        int delta = targetBrightness - state.brightness[i];
        if(abs(delta) > 2) {
            state.brightness[i] += delta / 4;
        } else {
            state.brightness[i] = targetBrightness;
        }
        
        // Apply color and brightness
        strip[i] = colour;
        strip[i].fadeToBlackBy(255 - state.brightness[i]);
    }
    
    FastLED.show();
}

// Alternative version with sharper wave (optional)
void v_radiateOutwardSharp(CRGB* strip, int numLeds, CRGB colour, int speed, ArmState& state) {
    const float phaseIncrement = 0.05 * (50.0 / speed);
    
    state.phase += phaseIncrement;
    if(state.phase > TWO_PI) state.phase -= TWO_PI;
    
    for(int i = 0; i < numLeds; i++){
        float ledPhase;
        
        if(i == 1) {
            ledPhase = state.phase;
        } else {
            ledPhase = state.phase - (PI / 1.5);  // Tighter spacing
            if(ledPhase < 0) ledPhase += TWO_PI;
        }
        
        // Sharper wave using cubic sine
        float sineValue = sin(ledPhase);
        sineValue = sineValue * sineValue * sineValue;  // Cubic for sharper peaks
        if(sin(ledPhase) < 0) sineValue = -sineValue;   // Preserve sign
        
        uint8_t targetBrightness = (uint8_t)((sineValue + 1.0) * 127.5);
        
        int delta = targetBrightness - state.brightness[i];
        state.brightness[i] += delta / 3;
        
        strip[i] = colour;
        strip[i].fadeToBlackBy(255 - state.brightness[i]);
    }
    
    FastLED.show();
}

void v_blinkRadiateEdge(CRGB* strip, int numLeds, CRGB colour, int speed, RadiateState& state) {
    const uint8_t pattern[][3] = {
        {0,0,0},
        {64,32,0},
        {128,64,32},
        {192,128,64},
        {255,192,128}  // outermost peak
    };
    const int steps = sizeof(pattern)/sizeof(pattern[0]);

    // Smoothly approach target for current step
    for(int i=0;i<numLeds;i++){
        int target;
        if(state.direction > 0) target = pattern[state.step][i];       // outward
        else target = pattern[steps-1 - state.step][i];               // inward

        int delta = target - state.brightness[i];
        if(delta > 0) delta = max(1, delta / 6);
        else if(delta < 0) delta = min(-1, delta / 6);

        state.brightness[i] += delta;
    }

    // Temporary "edge pop" at the peak
    if((state.direction > 0 && state.step == steps-1) || (state.direction < 0 && state.step == 0)){
        state.brightness[0] = min(255, state.brightness[0] + 32);
        state.brightness[numLeds-1] = min(255, state.brightness[numLeds-1] + 32);
    }

    // Apply to strip
    for(int i=0;i<numLeds;i++){
        strip[i] = colour;
        strip[i].fadeToBlackBy(255 - state.brightness[i]);
    }
    FastLED.show();

    // Advance step
    state.step += state.direction;
    if(state.step >= steps-1) state.direction = -1;
    else if(state.step <= 0) state.direction = 1;

    vTaskDelay(speed / portTICK_PERIOD_MS);
}


// void v_blinkRadiateEdge(CRGB* strip, int numLeds, CRGB colour, int speed, RadiateState& state) {
//     const uint8_t target[][3] = {
//         {0, 0, 0},
//         {64, 0, 0},
//         {128, 64, 0},
//         {192, 128, 64},
//         {255, 192, 128}, // outermost peak
//     };
//     const int steps = sizeof(target) / sizeof(target[0]);

//     // Smoothly approach target
//     for(int i = 0; i < numLeds; i++){
//         int delta = (int)target[state.step][i] - state.brightness[i];
//         state.brightness[i] += delta / 4; // smoothing factor
//     }

//     // Temporary “edge pop”
//     if(state.step == steps-1){
//         state.brightness[0] = min(255, state.brightness[0]+32); // left edge brighter
//         state.brightness[numLeds-1] = min(255, state.brightness[numLeds-1]+32); // right edge
//     }

//     // Apply to LEDs
//     for(int i = 0; i < numLeds; i++){
//         strip[i] = colour;
//         strip[i].fadeToBlackBy(255 - state.brightness[i]);
//     }
//     FastLED.show();

//     // Advance step
//     state.step += state.direction;
//     if(state.step >= steps-1 || state.step <= 0) state.direction = -state.direction;

//     vTaskDelay(speed / portTICK_PERIOD_MS);
// }

// void v_blinkRadiate(CRGB* strip, int numLeds, CRGB colour, int speed) {
//     // Define the intensity steps for 3 LEDs
//     const uint8_t pattern[][3] = {
//         {0, 0, 0},
//         {1, 0, 0},
//         {2, 1, 0},
//         {2, 2, 1},
//         {2, 2, 2},
//         {1, 2, 3},
//         {0, 1, 2},
//         {0, 0, 1},
//         {0, 0, 0}
//     };
//     static int step = 0;

//     for (int i = 0; i < numLeds; i++) {
//         strip[i] = colour;
//         strip[i].fadeToBlackBy(255 - pattern[step][i] * 64); // scale 0-3 → 0-192
//     }

//     FastLED.show();
//     step = (step + 1) % (sizeof(pattern) / sizeof(pattern[0]));
//     vTaskDelay(speed / portTICK_PERIOD_MS);
// }


void v_blinkFade(CRGB* strip, int numLeds, CRGB colour, int speed, FadeState& state) {
    if (colour != state.lastColour || speed != state.lastSpeed) {
        state.brightness = 0;
        state.fadeAmount = 5;
        state.lastColour = colour;
        state.lastSpeed = speed;
    }

    state.brightness += state.fadeAmount;
    if (state.brightness <= 0 || state.brightness >= 255) state.fadeAmount = -state.fadeAmount;

    for (int i = 0; i < numLeds; i++) {
        strip[i] = colour;
        strip[i].fadeToBlackBy(255 - state.brightness);
    }

    FastLED.show();
    vTaskDelay(speed / portTICK_PERIOD_MS);
}


// void v_blinkFade(CRGB* strip, int numLeds, CRGB colour, int speed) {
//     static int brightness = 0;
//     static int fadeAmount = 5;
//     static CRGB lastColour = CRGB::Black;
//     static int lastSpeed = -1;

//     if (colour != lastColour || speed != lastSpeed) {
//         brightness = 0;
//         fadeAmount = 5;
//         lastColour = colour;
//         lastSpeed = speed;
//     }

//     brightness += fadeAmount;
//     if (brightness <= 0 || brightness >= 255) fadeAmount = -fadeAmount;

//     int center = numLeds / 2;
//     for (int i = 0; i < numLeds; i++) {
//         int distance = abs(i - center);
//         int localBrightness = brightness - distance * 60; // fade strength outward
//         if (localBrightness < 0) localBrightness = 0;

//         strip[i] = colour;
//         strip[i].fadeToBlackBy(255 - localBrightness);
//     }

//     FastLED.show();
//     vTaskDelay(speed / portTICK_PERIOD_MS);
// }



// void v_blinkFade(CRGB* strip, int numLeds, CRGB colour, int speed) {
//     // Persistent state between calls
//     static int brightness = 0;
//     static int fadeAmount = 5;
//     static CRGB lastColour = CRGB::Black;
//     static int lastSpeed = -1;

//     // Detect if the mode changed (color or speed changed)
//     if (colour != lastColour || speed != lastSpeed) {
//         brightness = 0;
//         fadeAmount = 5;
//         lastColour = colour;
//         lastSpeed = speed;
//     }

//     brightness += fadeAmount;
//     if (brightness <= 0 || brightness >= 255) fadeAmount = -fadeAmount;

//     for (int i = 0; i < numLeds; i++) {
//         strip[i] = colour;
//         strip[i].fadeToBlackBy(255 - brightness);
//     }

//     FastLED.show();
//     vTaskDelay(speed / portTICK_PERIOD_MS);
// }

// void v_blinkFade(CRGB* strip, int numLeds, CRGB colour, int speed = 50){
//     static int brightness = 0;
//     static int fadeAmount = 5;

//     brightness += fadeAmount;
//     if (brightness <= 0 || brightness >= 255) fadeAmount = -fadeAmount;

//     for (int i = 0; i < numLeds; i++) {
//         strip[i] = colour;
//         strip[i].fadeToBlackBy(255 - brightness);
//     }

//     FastLED.show();
//     vTaskDelay(speed / portTICK_PERIOD_MS);
// }



// // https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/


// #include "esp_camera.h"
// #include <WiFi.h>
// #include "ESP32_OV5640_AF.h"

// #define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

// #include "camera_pins.h"

// const char* ssid = "KarlPhone";
// const char* password = "Aldebaran";

// void startCameraServer();
// void setupLedFlash(int pin);
// OV5640 ov5640 = OV5640();

// void setup() {
//   Serial.begin(115200);
//   while(!Serial);
//   Serial.setDebugOutput(true);
//   Serial.println();

//   camera_config_t config;
//   config.ledc_channel = LEDC_CHANNEL_0;
//   config.ledc_timer = LEDC_TIMER_0;
//   config.pin_d0 = Y2_GPIO_NUM;
//   config.pin_d1 = Y3_GPIO_NUM;
//   config.pin_d2 = Y4_GPIO_NUM;
//   config.pin_d3 = Y5_GPIO_NUM;
//   config.pin_d4 = Y6_GPIO_NUM;
//   config.pin_d5 = Y7_GPIO_NUM;
//   config.pin_d6 = Y8_GPIO_NUM;
//   config.pin_d7 = Y9_GPIO_NUM;
//   config.pin_xclk = XCLK_GPIO_NUM;
//   config.pin_pclk = PCLK_GPIO_NUM;
//   config.pin_vsync = VSYNC_GPIO_NUM;
//   config.pin_href = HREF_GPIO_NUM;
//   config.pin_sscb_sda = SIOD_GPIO_NUM;
//   config.pin_sscb_scl = SIOC_GPIO_NUM;
//   config.pin_pwdn = PWDN_GPIO_NUM;
//   config.pin_reset = RESET_GPIO_NUM;
//   config.xclk_freq_hz = 20000000;
//   config.frame_size = FRAMESIZE_UXGA;
//   config.pixel_format = PIXFORMAT_JPEG; // for streaming
//   //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
//   config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
//   config.fb_location = CAMERA_FB_IN_PSRAM;
//   config.jpeg_quality = 12;
//   config.fb_count = 1;
  
//   // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
//   //                      for larger pre-allocated frame buffer.
//   if(config.pixel_format == PIXFORMAT_JPEG){
//     if(psramFound()){
//       config.jpeg_quality = 10;
//       config.fb_count = 2;
//       config.grab_mode = CAMERA_GRAB_LATEST;
//     } else {
//       // Limit the frame size when PSRAM is not available
//       config.frame_size = FRAMESIZE_SVGA;
//       config.fb_location = CAMERA_FB_IN_DRAM;
//     }
//   } else {
//     // Best option for face detection/recognition
//     config.frame_size = FRAMESIZE_240X240;
// #if CONFIG_IDF_TARGET_ESP32S3
//     config.fb_count = 2;
// #endif
//   }

//   // camera init
//   esp_err_t err = esp_camera_init(&config);
//   if (err != ESP_OK) {
//     Serial.printf("Camera init failed with error 0x%x", err);
//     return;
//   }

//   sensor_t * s = esp_camera_sensor_get();
//   ov5640.start(s);

//     if (ov5640.focusInit() == 0) {
//     Serial.println("OV5640_Focus_Init Successful!");
//   }

//   if (ov5640.autoFocusMode() == 0) {
//     Serial.println("OV5640_Auto_Focus Successful!");
//   }

// // Setup LED FLash if LED pin is defined in camera_pins.h
// #if defined(LED_GPIO_NUM)
//   setupLedFlash(LED_GPIO_NUM);
// #endif

//   WiFi.begin(ssid, password);
//   WiFi.setSleep(false);

//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("");
//   Serial.println("WiFi connected");

//   startCameraServer();

//   Serial.print("Camera Ready! Use 'http://");
//   Serial.print(WiFi.localIP());
//   Serial.println("' to connect");
// }

// void loop() {
//   uint8_t rc = ov5640.getFWStatus();
//   Serial.printf("FW_STATUS = 0x%x\n", rc);

//   if (rc == -1) {
//     Serial.println("Check your OV5640");
//   } else if (rc == FW_STATUS_S_FOCUSED) {
//     Serial.println("Focused!");
//   } else if (rc == FW_STATUS_S_FOCUSING) {
//     Serial.println("Focusing!");
//   }
// }
