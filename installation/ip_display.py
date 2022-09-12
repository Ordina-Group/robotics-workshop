import Adafruit_SSD1306
import socket
import os

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

font = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/UbuntuMono-B.ttf", 14, encoding="unic")

def extract_ip():
    st = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        st.connect(('10.255.255.255', 1))
        IP = st.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        st.close()
    return IP

SSID = os.popen("sudo iwgetid -r").read()

draw.text((0, 10), extract_ip(), font=font, fill=255)
draw.text((0, 30), SSID, font=font, fill=255)

display.image(image)
display.display()
