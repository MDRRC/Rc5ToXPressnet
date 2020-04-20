# Rc5 To XPressNet converter (Work in progress!)

The Rc5 to XpressNet convertor is an interface between an RC5 remote control and the [XPressnet bus](www.lenzusa.com/1newsite1/Manuals/xpressnet.pdf). 

It is based on: 

 * An Ardiuno Mega 2560 Mini
 * An [Arduino Mega 2560](https://store.arduino.cc/arduino-mega-2560-rev3) will also work but the board is way bigger. 
 * RS485 driver IC.
 * 128*64 pixels OLED SSD1306 I2C display. 
 * TSOP4836 IR receiver.
 
The Rc5ToXPressnet is inteded to used with an [Uhlenbrock Iris infrared control](https://www.uhlenbrock.de/de_DE/produkte/digizen/I63D744D-001.htm!ArcEntryInfo=0004.9.I63D744D). Will also work with other RC5 remote controls for a TV but the layout and commands may e different, so adjustment of the code by yourself may be required.
 
## Tested with
 * [MDRRC-II (Lite)](https://robertdotevers.wordpress.com/). 
 * [Roco MultiMaus as master](https://www.roco.cc/en/product/5215-multimaus-0-0-0-0-0-004001-0/products.html).
 
## Connections
 
## Used library's
If you want to build the code yourself or update / change the code following library's are required.
 * [Rc5](https://github.com/guyc/RC5) for IR cobversion.
 * [StateMachine](https://github.com/jrullan/StateMachine) for the state machine (requires some updates, see below!) 
 * [XpressNet Client Library](http://pgahtow.de/wiki/index.php?title=XpressNet) for XpressNet protocol handling.
 * [Adafruit-SSD1306-Library](https://github.com/adafruit/Adafruit_SSD1306) to control the display.
 * [Adafruit-GFX-Library](https://github.com/adafruit/Adafruit-GFX-Library) for graphic control of the display.

More info will follow soon!    