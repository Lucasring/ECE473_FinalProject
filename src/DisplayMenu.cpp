#include "DisplayMenu.h"

DisplayMenu::DisplayMenu(int ScreenWidth, int ScreenHeight, TwoWire* I2C_bus, MenuContext defaultContext)
{
    _currContext = defaultContext;
    _wire = I2C_bus;
    _screenHeight = ScreenHeight;
    _screenWidth = ScreenWidth;

    _display = Adafruit_SSD1306(_screenWidth, _screenHeight, _wire, -1);
    _display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    _display.cp437(true);
    _display.clearDisplay();
}
    
void DisplayMenu::changeContext(MenuContext context)
{
    _currContext = context;
    _display.clearDisplay();
}

void DisplayMenu::draw()
{

    switch (_currContext)
    {
        case MENU_SPLASH:
            _splashDraw();
            break;
        case MENU_COMPASS:
            _compassDraw();
            break;
        case MENU_ACCEL:
            _accelDraw();
            break;
        case MENU_GYRO:
            _gyroDraw();
            break;
        case MENU_BATTERY:
            _batteryDraw();
            break;
    }

    _display.display();

}

void DisplayMenu::_splashDraw()
{
    //_display.drawRect(10, 10, 20, 20, SSD1306_WHITE);
    _display.setTextSize(2);
    _display.setTextColor(WHITE);
    _display.setCursor(16, 14);
    _display.println("Lucas R.");
    _display.setCursor(10, 34);
    _display.println("Kai Leung");
}

void DisplayMenu::_compassDraw()
{

}

void DisplayMenu::_accelDraw()
{
    _display.setTextSize(1);
    _display.setTextColor(WHITE);
    _display.setCursor(0, 0);
    _display.println("Accel Menu");

}

void DisplayMenu::_gyroDraw()
{

}

void DisplayMenu::_batteryDraw()
{

}
