# Prepare for creating service
* Make a folder on the robot desktop called: start_up
* Place the ip_display.py file in just made folder

# Install Python Pacakge
```
pip3 install Adafruit-SSD1306
```

# Create service
```
sudo systemctl edit --force --full start_display.service
```
Place this in the file that is just created:
```
[Unit]
Description=Robot Display Service
Wants=network-wait-online.target
After=network-wait-online.target

[Service]
Type=simple
User=jetson
WorkingDirectory=/home/jetson
ExecStartPre=/bin/sh -c 'until ping -c1 google.com; do sleep 1; done;'
ExecStart=/home/jetson/Desktop/start_up/start_display.sh

[Install]
WantedBy=multi-user.target
```

After creating the service you will need to enable it by using the following commando:

```
sudo systemctl enable start_display.service
```
