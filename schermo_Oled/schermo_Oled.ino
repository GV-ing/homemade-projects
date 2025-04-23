#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, 4, 5, U8X8_PIN_NONE);

void screen(){
  u8g2.setFont(u8g_font_9x15B);
  u8g2.drawStr( 0, 20, "CO2:");
}

void setup() {
  u8g2_SetI2CAddress(u8g2.getU8g2(), 0x3C * 2);
  u8g2.begin();

}

void loop() {
  u8g2.firstPage();
    do {
      screen();
    } while ( u8g2.nextPage() );
}
