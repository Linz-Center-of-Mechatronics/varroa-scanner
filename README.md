# Varroa Drop Scanner (beeClient)

This package adds the functionality of scanning and clearing the droppings of a beehive to the beeClient.

## Hardware
The **drop-scanner** uses cameras to record images of the bee droppings. The droppings are cleared by operating a motor.

### Platform
A RaspberryPi is used as a platform.

### Cameras
#### USB
UVC compliant USB camera modules.
#### CSI
CSI Pi Cameras. If CSI camera does not work after applying the software setup check the camera to be enabled and properly installed ([Install Guide](https://www.raspberrypi.org/documentation/usage/camera/installing.md)):
- In GUI: Preferences - Interfaces - Camera (Enable)
- From CMD Line: sudo raspi-config Interface Options - Camera - Yes (Enable)

### Motor
A stepper motor is controlled via GPIO and a [TMC2209](https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf) [Module](https://www.bigtree-tech.com/products/bigtreetech-tmc2209-v1-2-uart-stepper-motor-driver.html) driver module.


## Software

From the root directory of this project one can simply install the package with:
```bash
sudo pip install .
```
any additional requirements can be installed by running
```bash
sudo pip install -r requirements_rpi.txt
sudo apt-get update
sudo apt-get install libatlas-base-dev python3-opencv python3-tk python-picamera python3-picamera
```
apply patches *autostart.patch* and *rotate_screen.patch*:
```bash
patch -p0 < beebox-drop-scanner/autostart.patch --verbose
sudo patch -p0 /boot/config.txt < beebox-drop-scanner/rotate_screen.patch --verbose
```

#### Helpful Tools
##### Florence
A screen keyboard like *florence* will help to use the *camera* function if there is no keyboard attached.
```bash
sudo apt-get install at-spi2-core florence
```

### Camera Box
```
user: pi
pw: beeclient
```
The provided USB-Drive **must** be named ```0000_Bee```!

### Automated Testing
Run tests from root directory with:
```bash
python3 -m pytest
```