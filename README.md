# beeClient (Varroa Drop Scanner)

This package adds the functionality of scanning and clearing the droppings of a beehive to the beeClient.

## Hardware
The **drop-scanner** uses cameras to record images of the bee droppings. The droppings are cleared by operating a motor.

### Platform
A RaspberryPi is used as a platform.

### Cameras
UVC compliant USB camera modules are used.

### Motor
A stepper motor is controlled via GPIO and a driver module.

[MKS SERVO42B](https://github.com/makerbase-mks/MKS-SERVO42B)


## Software

From the root directory of this project one can simply install the package with:
```bash
sudo pip install .
```
any additional requirements can be installed by running
```bash
sudo pip install -r requirements_rpi.txt
sudo apt-get install libatlas-base-dev python3-opencv python3-tk
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
