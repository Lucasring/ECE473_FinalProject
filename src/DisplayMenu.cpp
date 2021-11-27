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

void DisplayMenu::nextMenu()
{
    _currContext = (MenuContext)((_currContext + 1) % AMOUNT_OF_MENUS);
}

void DisplayMenu::prevMenu()
{
    _currContext = (MenuContext)((_currContext - 1) % AMOUNT_OF_MENUS);
}


void DisplayMenu::draw()
{

    _display.clearDisplay();

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

    _display.drawBitmap(64, 0, compassBitmaps[COMPASS_BACKGROUND], BITMAP_WIDTH, BITMAP_WIDTH, WHITE);
    _display.drawBitmap(64, 0, compassBitmaps[COMPASS_CONTEXT.cardinal], BITMAP_WIDTH, BITMAP_WIDTH, WHITE);

    _display.setCursor(0, 0);
    _display.setTextSize(1);
    _display.println("Compass\n");

    _display.println("Heading:");
    _display.println(COMPASS_CONTEXT.heading, 2);

}

void DisplayMenu::_accelDraw()
{

    _display.setTextSize(2);
    _display.setTextColor(WHITE);
    
    _display.setCursor(0, 0);
    _display.println("Accel Menu");
    
    // Print X Accel
    _display.print("X: ");
    _display.println(ACCEL_CONTEXT.accelX, 3);

    // Print Y Accel
    _display.print("Y: ");
    _display.println(ACCEL_CONTEXT.accelY, 3);

    // Print Z Accel
    _display.print("Z: ");
    _display.println(ACCEL_CONTEXT.accelZ, 3);

}

void DisplayMenu::_gyroDraw()
{

    _display.setTextSize(2);
    _display.setTextColor(WHITE);
    
    _display.setCursor(0, 0);
    _display.println("Gyro Menu");
    
    // Print roll
    if(g_OrientationEuler.roll >= 0)
    {
        _display.print("R: ");
        _display.println((180 / PI) * g_OrientationEuler.roll, 3);
    } else {
        _display.print("R:-");
        _display.println(-(180 / PI) * g_OrientationEuler.roll, 3);
    }

    // Print pitch
    if(g_OrientationEuler.pitch >= 0)
    {
        _display.print("P: ");
        _display.println((180 / PI) * g_OrientationEuler.pitch, 3);
    } else {
        _display.print("P:-");
        _display.println(-(180 / PI) * g_OrientationEuler.pitch, 3);
    }

}

void DisplayMenu::_batteryDraw()
{

    _display.setTextSize(2);
    _display.setTextColor(WHITE);

    _display.setCursor(0, 0);
    _display.println("Battery\nMenu");

}
