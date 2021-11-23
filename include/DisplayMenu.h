#pragma once

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "bitmaps.h"
#include "globals.h"

/* Menu Enumerations */
typedef enum MenuContext
{
    MENU_SPLASH = 0,
    MENU_COMPASS,
    MENU_ACCEL,
    MENU_GYRO,
    MENU_BATTERY,
} MenuContext;

/* Menu Class */
class DisplayMenu
{
public:

    // Constructors
    DisplayMenu() {}
    DisplayMenu(int ScreenWidth, int ScreenHeight, TwoWire* I2C_bus, MenuContext defaultContext);
    ~DisplayMenu() {}
    
    // Functions
    void changeContext(MenuContext context);
    void draw();

private:

    // Member Vars
    Adafruit_SSD1306 _display;
    MenuContext _currContext;
    TwoWire* _wire;

    int _screenWidth;
    int _screenHeight;

    // Context Draw Functions
    void _splashDraw();
    void _compassDraw();
    void _accelDraw();
    void _gyroDraw();
    void _batteryDraw();

};