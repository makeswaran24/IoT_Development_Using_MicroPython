1] Alternating LED Blinking for 5 Seconds After Button Press
from machine import Pin
import time

led1 = Pin(15, Pin.OUT)
led2 = Pin(2, Pin.OUT)
button = Pin(4, Pin.IN, Pin.PULL_UP)

def alternate_blink():
    end_time = time.ticks_ms() + 5000
    while time.ticks_ms() < end_time:
        led1.value(1)
        led2.value(0)
        time.sleep(0.5)
        led1.value(0)
        led2.value(1)
        time.sleep(0.5)
    led1.value(0)
    led2.value(0)

try:
    while True:
        if button.value() == 0:
            print("Button pressed! Blinking LEDs...")
            alternate_blink()
            while button.value() == 0:
                pass
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user")
    led1.value(0)
    led2.value(0)
  2]HELLO WORLD ON I2C LCD display 
from machine import Pin, SoftI2C
from machine_i2c_lcd import I2cLcd
from time import sleep

I2C_ADDR = 0x27
I2C_NUM_ROWS = 2
I2C_NUM_COLS = 16

i2c = SoftI2C(sda=Pin(21), scl=Pin(22), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)

lcd.putstr("It's working :)")
sleep(4)

try:
    while True:
        lcd.clear()
        lcd.putstr("Hello World!")
        sleep(2)
        lcd.clear()
        lcd.move_to(0, 1)
        lcd.putstr("Hello World!")
        sleep(2)

except KeyboardInterrupt:
    print("Keyboard interrupt")
    lcd.backlight_off()
    lcd.display_off()
3]Analog Voltage Reading using ADC (ESP32)

from machine import ADC, Pin
import time

adc = ADC(Pin(34))
adc.width(ADC.WIDTH_12BIT)
adc.atten(ADC.ATTN_11DB)

while True:
    adc_value = adc.read()
    voltage = adc_value * (3.3 / 4095)
    print("ADC Value:", adc_value, "Voltage:", voltage, "V")
    time.sleep(1)

