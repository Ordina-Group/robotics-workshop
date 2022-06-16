import Adafruit_SSD1306
import subprocess
import time

from PIL import Image, ImageDraw, ImageDraw, ImageFont

display = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, gpio=1)

display.begin()
display.clear()
display.display()

width = display.width
height = display.height
image = Image.new('1', (width, height))

draw = ImageDraw.Draw(image)
draw.rectangle((0, 0, width, height), outline=0, fill=0)

font = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/UbuntuMono-B.ttf", 16, encoding="unic")

while True:
    ip_address = subprocess.check_output("hostname -I | cut -d\' \' -f1", shell=True)
    ssid = subprocess.check_output("iwgetid", shell=True).decode('utf-8').split('"')[1]

    draw.text((0, 10), str(ip_address.decode('utf-8')), font=font, fill=255)
    draw.text((0, 30), str(ssid), font=font, fill=255)

    display.image(image)
    display.display()
    time.sleep(1)