// <<<<<<<<<<=== === ===SSD1306: 0x3C=== === ===>>>>>>>>>>
// 0.91inch OLED
bool screenDefaultMode = true;

String screenLine_0;
String screenLine_1;
String screenLine_2;
String screenLine_3;
String screenLine_3_1;

#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH   128 // OLED display width, in pixels
#define SCREEN_HEIGHT  32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
unsigned long currentTimeMillis = millis();
unsigned long lastTimeMillis = millis();

// init oled ctrl functions.
void initOLED(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.display();
}


// Updata all data and flash the screen.
void oled_update() {
  display.clearDisplay();
  display.setCursor(0,0);

  display.println(screenLine_0);
  display.println(screenLine_1);
  display.println(screenLine_2);
  display.println(screenLine_3);
  display.display();
}
// dev info update on oled.
void oledInfoUpdate() {
  currentTimeMillis = millis();
  if (currentTimeMillis - lastTimeMillis > 1000) {
    inaDataUpdate();
    lastTimeMillis = currentTimeMillis;
  } else {
    return;
  }
  if (!screenDefaultMode) {
    return;
  }

  screenLine_3 =screenLine_3_1 + "V:"+String(loadVoltage_V) ;
  oled_update();
}