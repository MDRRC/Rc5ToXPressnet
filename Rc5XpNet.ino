/***********************************************************************************************************************
 * An RC5 to XpressNet converter for example to connect the Iris remote control of
 * Uhlenbrock (and any other RC5 remote control).
 **********************************************************************************************************************/

/***********************************************************************************************************************
   I N C L U D E S
 **********************************************************************************************************************/
#include "Arduino.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RC5.h>
#include <StateMachine.h>
#include <Wire.h>
#include <XpressNet.h>

/***********************************************************************************************************************
   D E F I N E S
 **********************************************************************************************************************/
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define XPRESSNET_ADDRESS 30 // XpNet address of device
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define XPRESSNET_PIN 3 // Read/write pin 485 chip
#else
#define XPRESSNET_PIN 2 // Read/write pin 485 chip
#endif
#define IR_PIN 7 // IR sensor pin

struct LocInfo
{
    uint16_t Address;
    uint8_t Speed;
    uint8_t Steps;
    uint8_t Direction;
    uint8_t Functions;
};

/***********************************************************************************************************************
   D A T A   D E C L A R A T I O N S (exported, local)
 **********************************************************************************************************************/

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static RC5 rc5(IR_PIN);
static XpressNetClass XPNet;
static StateMachine Stm = StateMachine();
static LocInfo locInfo;
static LocInfo locInfoPrevious;
static uint8_t Rc5Toggle               = 0;
static uint8_t Rc5TogglePrevious       = 0xFF;
static uint8_t Rc5Address              = 0;
static uint8_t Rc5Command              = 0;
static bool Rc5NewData                 = false;
static bool PowerOnStart               = true;
static uint32_t previousMillis         = 0;
static byte XpressNetPowerStat         = 0xFF;
static byte XpressNetPowerStatPrevious = 0xFF;
static uint16_t LocAddressSelect       = 0;
static uint8_t LocActualSpeed          = 0;
static uint8_t LocActualDirection      = 0;
static uint32_t LocActualFunctions     = 0;
static bool LocInfoChanged             = false;
static bool locInfoRefresh             = false;

/**
 * Forward direction loc symbol, see https://diyusthad.com/image2cpp for conversion of an image.
 */
const unsigned char locBitmapFw[] PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x3f, 0xfe, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00,
    0x78, 0x0e, 0x07, 0x00, 0x38, 0x0c, 0x0f, 0x00, 0x38, 0x0c, 0x0f, 0x00, 0x38, 0x0c, 0x0f, 0x00, 0x38, 0x1c, 0x0f,
    0x80, 0x7f, 0xff, 0xff, 0xc0, 0x7f, 0xff, 0xff, 0xc0, 0x7f, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xe0, 0x70, 0x3f,
    0xff, 0xc0, 0x20, 0x3f, 0xff, 0xc0, 0x60, 0x1f, 0xff, 0x80, 0x63, 0x1c, 0x78, 0xc0, 0x63, 0x1c, 0x38, 0x60, 0x20,
    0x18, 0x30, 0x60, 0x30, 0x3c, 0x38, 0x60, 0x18, 0x6e, 0x7c, 0xc0, 0x0f, 0xc7, 0xcf, 0x80, 0x00, 0x00, 0x00, 0x00 };

/**
 * Backward direction loc symbol.
 */
const unsigned char locBitmapBw[] PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x07, 0xff, 0xe0,
    0x0e, 0x07, 0x01, 0xe0, 0x0f, 0x03, 0x01, 0xc0, 0x0f, 0x03, 0x01, 0xc0, 0x0f, 0x03, 0x01, 0xc0, 0x1f, 0x03, 0x81,
    0xc0, 0x3f, 0xff, 0xff, 0xe0, 0x3f, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xe0, 0x3f, 0xff,
    0xc0, 0xe0, 0x3f, 0xff, 0xc0, 0x40, 0x1f, 0xff, 0x80, 0x60, 0x31, 0xe3, 0x8c, 0x60, 0x61, 0xc3, 0x8c, 0x60, 0x60,
    0xc1, 0x80, 0x40, 0x61, 0xc3, 0xc0, 0xc0, 0x33, 0xe7, 0x61, 0x80, 0x1f, 0x3e, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00 };

/**
 * States of the system.
 */
State* StmStateInit         = Stm.addState(&StateInit);
State* StmStateEmergency    = Stm.addState(&StateEmergency);
State* StmStateShortCircuit = Stm.addState(&StateShortCircuit);
State* StmStateServiceMode  = Stm.addState(&StateServiceMode);
State* StmStatePowerOff     = Stm.addState(&StatePowerOff);
State* StmStatePowerOn      = Stm.addState(&StatePowerOn);
State* StmStateGetLocInfo   = Stm.addState(&StateGetLocInfo);
State* StmStateSelectLoc    = Stm.addState(&StateSelectLoc);

/* Conversion table for normal speed to 28 steps DCC speed. */
const uint8_t SpeedStep28TableToDcc[29]
    = { 16, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8, 24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31 };

/* Conversion table for 28 steps DCC speed to normal speed. */
const uint8_t SpeedStep28TableFromDcc[32] = { 0, 0, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 0, 0, 2, 4, 6, 8,
    10, 12, 14, 16, 18, 20, 22, 24, 26, 28 };

/***********************************************************************************************************************
   L O C A L   F U N C T I O N S
 **********************************************************************************************************************/

/***********************************************************************************************************************
 */
void ShowInitSreen(void)
{
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 26);
    display.println(F(" RC5 XPNET"));
    display.print(F("   1.0.0"));
}

/***********************************************************************************************************************
 */
void UpdateStatusRow(const __FlashStringHelper* StatusRowPtr)
{
    display.fillRect(3, 0, 73, 10, SSD1306_BLACK);
    display.setTextSize(0);
    display.setCursor(3, 0);
    display.println(StatusRowPtr);
    display.display();
}

/***********************************************************************************************************************
 */
void ShowLocInfo()
{
    // Show address
    display.fillRect(0, 13, 127, 64, SSD1306_BLACK);

    display.setTextSize(2);
    display.setCursor(3, 14);
    display.print(locInfo.Address);
    display.setCursor(3, 34);

    // Show speed
    if (locInfo.Steps == 2)
    {
        display.print(SpeedStep28TableFromDcc[locInfo.Speed]);
    }
    else
    {
        display.print(locInfo.Speed);
    }

    // Show direction
    display.fillRect(69, 10, 29, 23, SSD1306_BLACK);
    if (locInfo.Direction == 1)
    {
        display.drawBitmap(70, 11, locBitmapFw, 28, 22, SSD1306_WHITE);
    }
    else
    {
        display.drawBitmap(70, 11, locBitmapBw, 28, 22, SSD1306_WHITE);
    }

    // Show decoder speed steps
    display.fillRect(110, 0, 17, 10, SSD1306_BLACK);
    display.setTextSize(0);
    display.setCursor(110, 0);
    switch (locInfo.Steps)
    {
    case 0: display.print("14"); break;
    case 2: display.print("28"); break;
    case 4: display.print("128"); break;
    default: break;
    }

    display.display();
    LocInfoChanged = false;
}

/***********************************************************************************************************************
 */
void StateInit()
{
    if (Stm.executeOnce)
    {
        locInfo.Address = 3;
        XPNet.getLocoInfo(locInfo.Address >> 8, locInfo.Address & 0xFF);
    }
    else
    {
    }
}

/***********************************************************************************************************************
 */
void StateEmergency()
{
    if (Stm.executeOnce)
    {
        XPNet.setPower(csEmergencyStop);
        UpdateStatusRow(F("EMERGENCY"));
        PowerOnStart = false;
    }
    else
    {
    }
}

/***********************************************************************************************************************
 */
void StateShortCircuit()
{
    if (Stm.executeOnce)
    {
        UpdateStatusRow(F("SHORT"));
        PowerOnStart = false;
    }
    else
    {
    }
}

/***********************************************************************************************************************
 */
void StateServiceMode()
{
    if (Stm.executeOnce)
    {
        UpdateStatusRow(F("SERVICE"));
        ShowInitSreen();
        PowerOnStart = false;
    }
    else
    {
    }
}

/***********************************************************************************************************************
 */
void StatePowerOff()
{
    if (Stm.executeOnce)
    {
        UpdateStatusRow(F("POWER OFF"));

        if (PowerOnStart == false)
        {
            locInfo.Speed = 0;
            XPNet.setLocoDrive(locInfo.Address >> 8, locInfo.Address & 0xFF, locInfo.Steps, 0);
            ShowLocInfo();
        }
        else
        {
            // After initial power on and power off state keep showing start screen.
            PowerOnStart = false;
        }
    }
}

/***********************************************************************************************************************
 */
void StatePowerOn()
{
    bool TransmitSpeedDirectionData = false;

    if (Stm.executeOnce)
    {
        locInfoRefresh = false;
        UpdateStatusRow(F("POWER ON"));
        ShowLocInfo();
        PowerOnStart = false;
    }
    else
    {
        if (LocInfoChanged == true)
        {
            ShowLocInfo();
        }

        if (Rc5NewData == true)
        {
            switch (Rc5Command)
            {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9: Rc5NewData = false; break;
            case 16:
                // Right button
                TransmitSpeedDirectionData = true;
                if (LocActualDirection == 0x0)
                {
                    LocActualDirection = 0x80;
                }
                break;
            case 17:
                // Left button
                TransmitSpeedDirectionData = true;
                if (LocActualDirection == 0x80)
                {
                    LocActualDirection = 0x0;
                }
                break;
            case 19:
                // Light off
                LocActualFunctions &= ~(1 << 0);
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF, LocActualFunctions & 0x01, 0);
                Rc5NewData = false;
                break;
            case 20:
                // F0 (Light) button on
                LocActualFunctions |= 1;
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF, LocActualFunctions & 0x01, 0);
                Rc5NewData = false;
                break;
            case 21:
                // F1 button
                LocActualFunctions ^= (1 << 1);
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF, (LocActualFunctions >> 1) & 0x01, 1);
                Rc5NewData = false;
                break;
            case 22:
                // F2 button
                LocActualFunctions ^= (1 << 2);
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF, (LocActualFunctions >> 2) & 0x01, 2);
                Rc5NewData = false;
                break;
            case 23:
                // F3 button
                LocActualFunctions ^= (1 << 3);
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF, (LocActualFunctions >> 3) & 0x01, 3);
                Rc5NewData = false;
                break;
            case 24:
                // F4 button
                LocActualFunctions ^= (1 << 4);
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF, (LocActualFunctions >> 4) & 0x01, 4);
                Rc5NewData = false;
                break;
            case 32:
                // + button
                TransmitSpeedDirectionData = true;
                if (LocActualDirection == 0x80)
                {
                    LocActualSpeed++;
                }
                else
                {
                    if (LocActualSpeed >= 1)
                    {
                        LocActualSpeed--;
                    }
                    else
                    {
                        // Speed 0, change direction.
                        LocActualDirection = 0x80;
                    }
                }
                break;
            case 33:
                // - button
                TransmitSpeedDirectionData = true;
                if (LocActualDirection == 0x80)
                {
                    if (LocActualSpeed >= 1)
                    {
                        LocActualSpeed--;
                    }
                    else
                    {
                        // Speed 0, change direction.
                        LocActualDirection = 0x0;
                    }
                }
                else
                {
                    LocActualSpeed++;
                }
                break;

            default: break;
            }

            if (TransmitSpeedDirectionData == true)
            {
                Rc5NewData = false;
                switch (locInfo.Steps)
                {
                case 1:
                    // 14 speed docoder.
                    if (LocActualSpeed > 14)
                    {
                        LocActualSpeed = 14;
                    }
                    XPNet.setLocoDrive(locInfo.Address >> 8, locInfo.Address & 0xFF, locInfo.Steps,
                        SpeedStep28TableToDcc[LocActualSpeed] | LocActualDirection);

                    break;
                case 2:
                    // 28 speed decoder.
                    if (LocActualSpeed > 28)
                    {
                        LocActualSpeed = 28;
                    }
                    XPNet.setLocoDrive(locInfo.Address >> 8, locInfo.Address & 0xFF, locInfo.Steps,
                        SpeedStep28TableToDcc[LocActualSpeed] | LocActualDirection);
                    break;
                case 4:
                    // 128 speed decoder.
                    if (LocActualSpeed > 127)
                    {
                        LocActualSpeed = 127;
                    }

                    XPNet.setLocoDrive(locInfo.Address >> 8, locInfo.Address & 0xFF, locInfo.Steps,
                        LocActualSpeed | LocActualDirection);
                    break;
                }

                XPNet.getLocoInfo(locInfo.Address >> 8, locInfo.Address & 0xFF);

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
                Serial.print("Speed : ");
                Serial.print(LocActualSpeed);
                Serial.print("Direction : ");
                Serial.print(LocActualDirection);
                Serial.print("Steps : ");
                Serial.print(locInfo.Steps);
                Serial.println("");

#endif
            }
        }
    }
}

/***********************************************************************************************************************
 */
void StateGetLocInfo()
{
    if (Stm.executeOnce)
    {
        UpdateStatusRow(F("GET LOC"));
        locInfoRefresh = true;
        PowerOnStart   = false;
    }
    else
    {
        if (LocInfoChanged == false)
        {
            XPNet.getLocoInfo(locInfo.Address >> 8, locInfo.Address & 0xFF);
        }
        else
        {
            if (locInfo.Steps == 2)
            {
                LocActualSpeed = SpeedStep28TableFromDcc[locInfo.Speed];
            }
            else
            {
                LocActualSpeed = locInfo.Speed;
            }

            locInfoRefresh = false;

            Stm.transitionTo(StmStatePowerOn);
        }
    }
}

/***********************************************************************************************************************
 */
void StateSelectLoc()
{
    if (Stm.executeOnce)
    {
        UpdateStatusRow(F("SELECT LOC"));

        LocAddressSelect = 0;

        display.fillRect(0, 10, 128, 64, SSD1306_BLACK);
        display.setTextSize(2);
        display.setCursor(1, 14);
        display.print("Loc  :");
        display.setCursor(75, 14);
        display.print(LocAddressSelect);
        display.display();
    }
    else
    {
        if (Rc5NewData == true)
        {
            switch (Rc5Command)
            {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                // Increase selected address.
                LocAddressSelect *= 10;
                LocAddressSelect += (uint16_t)(Rc5Command);

                // Limit
                if (LocAddressSelect > 9999)
                {
                    LocAddressSelect = 0;
                }

                // Display
                display.fillRect(74, 10, 51, 64, SSD1306_BLACK);
                display.setCursor(75, 14);
                display.print(LocAddressSelect);
                display.display();

                Rc5NewData = false;
                break;
            case 13:
                // Loc button to select loc. When address still zero keep old address.
                if (LocAddressSelect > 0)
                {
                    locInfo.Address = LocAddressSelect;
                }
                Rc5NewData = false;
                Stm.transitionTo(StmStateGetLocInfo);
                break;
            default: Rc5NewData = false; break;
            }
        }
    }
}

/***********************************************************************************************************************
 */
bool transitionEmergency()
{
    bool Result = false;
    if (XpressNetPowerStat == csEmergencyStop)
    {
        Result = true;
    }

    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionShortCircuit()
{
    bool Result = false;
    if (XpressNetPowerStat == csShortCircuit)
    {
        if (XpressNetPowerStatPrevious != XpressNetPowerStat)
        {
            XpressNetPowerStatPrevious = XpressNetPowerStat;
            Result                     = true;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionServiceMode()
{
    bool Result = false;
    if (XpressNetPowerStat == csServiceMode)
    {
        if (XpressNetPowerStatPrevious != XpressNetPowerStat)
        {
            XpressNetPowerStatPrevious = XpressNetPowerStat;
            Result                     = true;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionPowerOff()
{
    bool Result = false;
    if (XpressNetPowerStat == csTrackVoltageOff)
    {
        if (XpressNetPowerStatPrevious != XpressNetPowerStat)
        {
            XpressNetPowerStatPrevious = XpressNetPowerStat;
            Result                     = true;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionPowerOn()
{
    bool Result = false;
    if (XpressNetPowerStat == csNormal)
    {
        if (XpressNetPowerStatPrevious != XpressNetPowerStat)
        {
            XpressNetPowerStatPrevious = XpressNetPowerStat;
            Result                     = true;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionRc5StopButton()
{
    bool Result = false;
    if (Rc5NewData == true)
    {
        if (Rc5Command == 12)
        {
            // Based on central station state transmit power on or off.
            // Based on the status change applicable state will be entered.
            // When calling setPower() in states generated sometimes runarounds..
            switch (XpressNetPowerStat)
            {
            case csNormal: XPNet.setPower(csTrackVoltageOff); break;
            case csShortCircuit:
            case csEmergencyStop:
            case csTrackVoltageOff: XPNet.setPower(csNormal); break;
            case csServiceMode: XPNet.setPower(csTrackVoltageOff); break;
            }
            Rc5NewData = false;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionRc5SelectLocButton()
{
    bool Result = false;
    if (Rc5NewData == true)
    {
        if (Rc5Command == 13)
        {
            Result     = true;
            Rc5NewData = false;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */

/***********************************************************************************************************************
   E X P O R T E D   F U N C T I O N S
 **********************************************************************************************************************/

/**
 ***********************************************************************************************************************
   @brief   The setup function is called once at startup of the sketch
   @return  None
 **********************************************************************************************************************/
void setup()
{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    Serial.begin(57600);
    Serial.println("Starting");
#endif

    // Init display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        Serial.println(F("SSD1306 allocation failed"));
#endif
        for (;;)
            ; // Don't proceed, loop forever
    }

    ShowInitSreen();
    UpdateStatusRow(F("CONNECTING"));

    // Init Xpressnet.
    XPNet.start(XPRESSNET_ADDRESS, XPRESSNET_PIN);

    previousMillis = millis();

    locInfoPrevious.Address = 0xFFFF;

    // All transitions for the state machine.
    StmStateInit->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateInit->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateInit->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStateInit->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateInit->addTransition(&transitionShortCircuit, StmStateShortCircuit);

    StmStatePowerOff->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStatePowerOff->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStatePowerOff->addTransition(&transitionEmergency, StmStateEmergency);
    StmStatePowerOff->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStatePowerOff->addTransition(&transitionRc5StopButton, StmStateGetLocInfo);
    StmStatePowerOff->addTransition(&transitionRc5SelectLocButton, StmStateSelectLoc);

    StmStatePowerOn->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStatePowerOn->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStatePowerOn->addTransition(&transitionEmergency, StmStateEmergency);
    StmStatePowerOn->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStatePowerOn->addTransition(&transitionRc5StopButton, StmStatePowerOff);
    StmStatePowerOn->addTransition(&transitionRc5SelectLocButton, StmStateSelectLoc);

    StmStateEmergency->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateEmergency->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateEmergency->addTransition(&transitionServiceMode, StmStateEmergency);
    StmStateEmergency->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStateEmergency->addTransition(&transitionRc5StopButton, StmStatePowerOff);

    StmStateShortCircuit->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateShortCircuit->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateShortCircuit->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStateShortCircuit->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateShortCircuit->addTransition(&transitionRc5StopButton, StmStatePowerOff);

    StmStateServiceMode->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateServiceMode->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateServiceMode->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStateServiceMode->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateServiceMode->addTransition(&transitionRc5StopButton, StmStatePowerOff);

    StmStateGetLocInfo->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateGetLocInfo->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStateGetLocInfo->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateGetLocInfo->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStateGetLocInfo->addTransition(&transitionRc5StopButton, StmStatePowerOff);

    StmStateSelectLoc->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateSelectLoc->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStateSelectLoc->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateSelectLoc->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStateSelectLoc->addTransition(&transitionRc5StopButton, StmStatePowerOff);
}

/**
 ***********************************************************************************************************************
   @brief   The loop function is called in an endless loop
   @return  None
 **********************************************************************************************************************/
void loop()
{

    if (rc5.read((uint8_t*)(&Rc5Toggle), (uint8_t*)(&Rc5Address), (uint8_t*)(&Rc5Command)))
    {
        if (Rc5Toggle != Rc5TogglePrevious)
        {
            Rc5TogglePrevious = Rc5Toggle;
            Rc5NewData        = true;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
            Serial.print("a:");
            Serial.print(Rc5Address);
            Serial.print(" c:");
            Serial.print(Rc5Command);
            Serial.print(" t:");
            Serial.println(Rc5Toggle);
#endif
            Stm.run();
            previousMillis = millis();
        }
    }

    XPNet.receive();

    if (millis() - previousMillis >= 20)
    {
        Stm.run();
        previousMillis = millis();
    }
}

/***********************************************************************************************************************
 * XpressNet callback function for system status.
 */
void notifyXNetPower(uint8_t State)
{
    XpressNetPowerStat = State;
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    Serial.print("XnetPower : ");
    Serial.println(State);
#endif
}

/***********************************************************************************************************************
 * XpressNet callback function for loc data.
 */
void notifyLokAll(uint8_t Adr_High, uint8_t Adr_Low, boolean Busy, uint8_t Steps, uint8_t Speed, uint8_t Direction,
    uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3, boolean Req)
{
    bool LocDataValid = true;

    uint16_t Address = ((uint16_t)(Adr_High) << 8) | (uint16_t)(Adr_Low);

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    Serial.print("A: ");
    Serial.print(Address);
    Serial.print(" S: ");
    Serial.print(Speed);
    Serial.print(" D:  ");
    Serial.print(Steps);
    Serial.print(" Dir : ");
    Serial.print(Direction);
    Serial.print(" F0 : ");
    Serial.print(F0);
    Serial.print(" F1 : ");
    Serial.print(F1);
    Serial.print(" F2 : ");
    Serial.print(F2);
    Serial.print(" F3 : ");
    Serial.println(F3);
#endif
    if (locInfo.Address == Address)
    {
        locInfo.Address   = Address;
        locInfo.Speed     = Speed;
        locInfo.Direction = Direction;
        locInfo.Functions = (F0 >> 4) & 0x01;
        locInfo.Functions |= (F0 << 1) & 0x1E;
        locInfo.Functions |= ((uint32_t)(F1) << 5);
        locInfo.Functions |= ((uint32_t)(F2) << 13);
        locInfo.Functions |= ((uint32_t)(F3) << 21);

        switch (Steps)
        {
        case 0:
        case 2:
        case 4: locInfo.Steps = Steps; break;
        default: LocDataValid = false; break;
        }
    }

    if (LocDataValid == true)
    {
        if ((memcmp(&locInfo, &locInfoPrevious, sizeof(locInfo)) != 0) || (locInfoRefresh == true))
        {
            LocInfoChanged = true;
            memcpy(&locInfoPrevious, &locInfo, sizeof(locInfo));

            if (locInfoRefresh == true)
            {
                switch (locInfo.Steps)
                {
                case 0:
                case 4: LocActualSpeed = Speed; break;
                case 2:
                    // Convert DCC speed to normal speed
                    LocActualSpeed = SpeedStep28TableFromDcc[Speed];
                    break;
                }

                if (locInfo.Direction == 1)
                {
                    LocActualDirection = 128;
                }
                else
                {
                    LocActualDirection = 0;
                }
            }
        }
    }

    Busy = Busy;
    Req  = Req;
}
