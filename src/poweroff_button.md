# Raspberry Pi GPIO Power Button (Long-Press Shutdown)
Target: Raspberry Pi 4 Model B
OS: Ubuntu 22.04

Holding a physical push button for 2.5 seconds performs a clean system shutdown.

--------------------------------------------------------------------

### TABLE OF CONTENTS

1. What This Does
2. Hardware Required
3. GPIO Wiring
4. Software Installation
5. Power Button Script
6. Sudo Permissions
7. systemd Autostart
8. Testing
9. Usage
10. Troubleshooting
11. Notes

--------------------------------------------------------------------

### 1. WHAT THIS DOES

- Reads a GPIO input
- Uses internal pull-up resistor
- Debounces the button
- Detects a long press (>= 2.5 seconds)
- Executes /sbin/poweroff
- Starts automatically at boot
- Prevents SD-card corruption

--------------------------------------------------------------------

### 2. HARDWARE REQUIRED

- Raspberry Pi 4 Model B
- Ubuntu 22.04
- Momentary push button (normally open)
- 2 jumper wires (female-female)

--------------------------------------------------------------------

### 3. GPIO WIRING

Pins used:

Button input: GPIO17 (BCM) — Physical pin 11
Ground: GND — Physical pin 6

Wiring:

- One leg of the button to GPIO17
- Other leg of the button to GND

Logic:

Released = HIGH
Pressed  = LOW

This logic is REQUIRED.

--------------------------------------------------------------------

### 4. SOFTWARE INSTALLATION

Install required packages, enable GPIO daemon:

``` bash
sudo apt install -y python3-gpiozero python3-pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
systemctl status pigpiod
```

--------------------------------------------------------------------

### 5. POWER BUTTON SCRIPT

Create the script file:

``` bash
sudo nvim /usr/local/bin/power_button.py
```

----- power_button.py -----

``` python
#!/usr/bin/env python3

from gpiozero import Button
from signal import pause
import subprocess

BUTTON_PIN = 17
HOLD_TIME = 2.5

button = Button(
    BUTTON_PIN,
    pull_up=True,
    bounce_time=0.1,
    hold_time=HOLD_TIME
)

def shutdown():
    subprocess.run(["/sbin/poweroff"])

button.when_held = shutdown

pause()
```

Make it executable:

``` bash
sudo chmod +x /usr/local/bin/power_button.py
```

--------------------------------------------------------------------

### 6. SUDO PERMISSIONS

Edit sudoers:

``` bash
sudo visudo
```
  
Add this line at the VERY BOTTOM:

hostuser ALL=(ALL) NOPASSWD: /sbin/poweroff

--------------------------------------------------------------------

### 7. SYSTEMD AUTOSTART

Create the service file:

``` bash
sudo nvim /etc/systemd/system/power-button.service
```
``` bash
[Unit]
Description=GPIO Power Button Shutdown
After=multi-user.target

[Service]
Type=simple
ExecStart=/usr/local/bin/power_button.py
Restart=always
User=root

[Install]
WantedBy=multi-user.target
```

Reload systemd:
``` bash
sudo systemctl daemon-reload
```

Enable service:
``` bash
sudo systemctl enable power-button.service
```

Start service:
``` bash
sudo systemctl start power-button.service
```

Verify:
``` bash
systemctl status power-button.service

```
--------------------------------------------------------------------

### 8. TESTING

Short press:
- Nothing should happen

Long press (>= 2.5 seconds):
- System shuts down cleanly
- HDMI goes blank
- SSH disconnects

--------------------------------------------------------------------

### 9. USAGE

- Short press: ignored
- Long press: shutdown
- Works headless
- Runs on every boot

After shutdown, power must be removed or toggled to restart.

--------------------------------------------------------------------

### 10. TROUBLESHOOTING

Immediate shutdown on boot:
- Button wired wrong
- Button is normally-closed
- GPIO shorted to GND

Button does nothing:
- Wrong GPIO number (must be BCM)
- Service not running
- pigpiod not running

Check:

``` bash
systemctl status pigpiod
systemctl status power-button.service
```

--------------------------------------------------------------------

### 11. NOTES

- This does NOT physically cut power
- Raspberry Pi cannot soft-power-on
- This is the safest shutdown method available