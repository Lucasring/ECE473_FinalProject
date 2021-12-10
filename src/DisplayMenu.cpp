#include "DisplayMenu.h"

/**
 * @brief Construct a new Display Menu:: Display Menu object
 * 
 * @param ScreenWidth       The width of the screen
 * @param ScreenHeight      The height of the screen
 * @param I2C_bus           An I2C bus object reference
 * @param defaultContext    The default menu to display
 */
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
    
/**
 * @brief   Changes the menu currently displayed
 * 
 * @param context   The menu to display
 */
void DisplayMenu::changeContext(MenuContext context)
{
    _currContext = context;
    _display.clearDisplay();
}

/**
 * @brief   Increments the menu index
 * 
 */
void DisplayMenu::nextMenu()
{
    _currContext = (MenuContext)((_currContext + 1) % AMOUNT_OF_MENUS);
}

/**
 * @brief   Decrements the menu index
 * 
 */
void DisplayMenu::prevMenu()
{
    _currContext = (_currContext == 0) ? _currContext = MENU_BATTERY : (MenuContext)((_currContext - 1) % AMOUNT_OF_MENUS);
}

/**
 * @brief   Draws the menu represented by the number in _currContext
 * 
 */
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

/**
 * @brief   Draws the splash screen
 * 
 */
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

/**
 * @brief   Draws the compass display screen
 * 
 */
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

/**
 * @brief Draws the accleration display menu
 * 
 */
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

/**
 * @brief   Draws the gyroscope / orientation display menu
 * 
 */
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

/**
 * @brief   Draws the battery display menu
 * 
 */
void DisplayMenu::_batteryDraw()
{

    _display.setTextSize(1);
    _display.setTextColor(WHITE);

    _display.setCursor(0, 0);
    _display.println("Battery Menu");
    _display.print("Voltage: ");
    _display.println(BATTERY_CONTEXT.value, 2);
    _display.print(BATTERY_CONTEXT.percentage*100, 0);
    _display.println("%");

}
