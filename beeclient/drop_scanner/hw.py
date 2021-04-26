"""Hardware module hw.py

Interfacing and controlling hardware.
"""

import time
import math
import threading
import logging
from abc import ABC, abstractmethod
from typing import Tuple, Type


class HardwareInterface(ABC):
    """Abstract hardware interfacing class.
    
    Interfaces the hardware to perform all actions requiring hardware.
    """
    
    steps_per_rev = 3200
    """Stepper motors number of steps per revolution
    """
    
    @abstractmethod
    def set_light_gpio(self, value: bool):
        """Set the level of the gpio controlling the light.
        
        Parameters
        ---------
        value
            Value to set light to
        """
        
        pass

    @abstractmethod
    def get_light_gpio(self) -> bool:
        """Get the level of the gpio controlling the light.
        
        RETURNS
        -------
        bool
            State of the light controlling gpio: True (light), False (no light).
        """
        
        pass
        
    @abstractmethod
    def set_stepper_motor(self, steps: int, ts: float):
        """Perform synchronously given steps with stepper motor.
        
        PARAMETERS
        ----------
        steps
            Steps for the stepper motor to perform.
        ts
            Sleep time between steps.
        """
        
        pass

    def set_stepper_motor_async(self, steps: int, ts: float):
        """Perform asynchronously given steps with stepper motor.
        
        PARAMETERS
        ----------
        steps
            Steps for the stepper motor to perform.
        ts
            Sleep time between steps.
        """
        
        threading.Thread(target=self.set_stepper_motor, args=(steps,)).start()

    @abstractmethod
    def get_stepper_motor_stat(self) -> Tuple[bool, bool, bool]:
        """Get status of stepper motor.
        
        RETURNS
        -------
        Tuple[bool, bool]
            State of stepper motor: enable, step, direction.
        """
        
        pass
    

class HardwareInterfaceDummy(HardwareInterface):
    """Dummy hardware interface for platforms without gpio access.
    """
    
    def __init__(self, *args, **kwargs):
        super(HardwareInterfaceDummy, self).__init__()
        self._gpio_light = False
        self._gpio_stepper_motor = False
    
    def set_light_gpio(self, value):
        self._gpio_light = value
    
    def get_light_gpio(self):
        return self._gpio_light
    
    def set_stepper_motor(self, steps, ts):
        self._gpio_stepper_motor = True
        time.sleep(steps * ts)
        self._gpio_stepper_motor = False
        
    def set_stepper_motor_async(self, steps, ts):
        threading.Thread(target=self.set_stepper_motor, args=(steps, ts,)).start()
        
    def get_stepper_motor_stat(self):
        return self._gpio_stepper_motor, True


class HardwareInterfacePi(HardwareInterface):
    """Hardware interface for RaspberryPi.
    
    Parameters
    ----------
    pin_light
        GPIO pin (board numbering) to control the light
    pin_en
        GPIO pin (board numbering) to enable the motor
    pin_step
        GPIO pin (board numbering) to perform a motor step
    pin_dir
        GPIO pin (board numbering) to control the motor direction
    """
    
    pulse_dur = 20.0e-6
    """Pulse duration for stepper motor control
    """
    
    def __init__(self, pin_light: int, pin_en: int, pin_step: int, pin_dir: int):
        from beeclient.drop_scanner.image import ImageProvider
        try:
            import RPi.GPIO as GPIO
            self._gpio = GPIO
        except RuntimeError:
            raise RuntimeError("Error importing RPi.GPIO! Have you added your user to the group gpio?")
        
        self._pin_light = pin_light
        self._pin_en = pin_en
        self._pin_step = pin_step
        self._pin_dir = pin_dir
        self._gpio.setmode(self._gpio.BOARD)
        self._gpio.setwarnings(False)
        self._gpio.setup([self._pin_dir, self._pin_en, self._pin_step, self._pin_light],
                         self._gpio.OUT, initial=self._gpio.LOW)
        
        self.image_provider_class = ImageProvider

        super(HardwareInterfacePi, self).__init__()

    def set_light_gpio(self, value):
        if value:
            self._gpio.output(self._pin_light, self._gpio.HIGH)
        else:
            self._gpio.output(self._pin_light, self._gpio.LOW)

    def get_light_gpio(self):
        return self._gpio.input(self._pin_light)

    def set_stepper_motor(self, steps, ts, t_en=10.0e-3, t_bias=180.0e-6):
        ts -= t_bias
        if ts <= 2 * self.pulse_dur:
            ts = 2 * self.pulse_dur
        t_on = ts / 2
        t_off = ts - t_on
        
        try:
            # enable motor
            logging.debug('en:  {} -> {}'.format(self._pin_en, 'HIGH'))
            self._gpio.output(self._pin_en, self._gpio.HIGH)
            # set direction
            logging.debug('dir: {} -> {}'.format(self._pin_dir, 'HIGH'))
            self._gpio.output(self._pin_dir, self._gpio.HIGH)
            
            # wait a bit
            time.sleep(t_en)

            # perform movement
            for _ in range(steps):
                self._gpio.output(self._pin_step, self._gpio.HIGH)
                time.sleep(t_on)
                self._gpio.output(self._pin_step, self._gpio.LOW)
                time.sleep(t_off)
                
        except KeyboardInterrupt:
            pass
        
        finally:
            # disable motor
            logging.debug('en:  {} -> {}'.format(self._pin_en, 'LOW'))
            self._gpio.output(self._pin_en, self._gpio.LOW)
            # reset pin states
            logging.debug('dir: {} -> {}'.format(self._pin_dir, 'LOW'))
            self._gpio.output(self._pin_dir, self._gpio.LOW)
            logging.debug('stp: {} -> {}'.format(self._pin_step, 'LOW'))
            self._gpio.output(self._pin_step, self._gpio.LOW)

    def get_stepper_motor_stat(self):
        return self._gpio.input(self._pin_en), self._gpio.input(self._pin_step), self._gpio.input(self._pin_dir)


class HardwareAbstraction(object):
    """An hardware abstraction class implemented as Singleton
    """
    _instance = None
    
    def __new__(cls, hw_interface_class: Type[HardwareInterface], *args, **kwargs) -> 'HardwareAbstraction':
        """Instantiate hardware abstraction singleton
        
        Parameters
        ----------
        hw_interface_class
            Hardware interface to instantiate
        
        Returns
        -------
        HardwareAbstraction
            Hardware abstraction instance
        """
        
        if cls._instance is None:
            cls._instance = super(HardwareAbstraction, cls).__new__(cls)
            cls._instance._init(hw_interface_class, *args, **kwargs)
        return cls._instance
    
    def _init(self, hwi_class: Type[HardwareInterface], *args, **kwargs):
        """Init the hardware interface
        
        Parameters
        ----------
        hwi_class
            The hardware interface class to init
        """
        
        self.hwi = hwi_class(*args, **kwargs)
    
    def set_light(self, on: bool = False):
        """Switch light on/off
        
        PARAMETERS
        ----------
        on
            Flag to turn on/off light.
        """
        
        logging.debug('switch_light {}'.format(on))
        self.hwi.set_light_gpio(on)
    
    def get_light_status(self) -> bool:
        """Get state of light.
        
        RETURNS
        -------
        state
            State of light True (on), False (off).
        """
        
        return self.hwi.get_light_gpio()
    
    @staticmethod
    def calc_steps_for_distance(dist: float, radius: float, steps_pre_rev: float) -> int:
        """Calculate number of steps to move distance.
        
        PARAMETERS
        ----------
        dist
            Distance to travel.
        radius
            Radius of transport wheel.
        steps_pre_rev
            Steps per revolution.
            
        RETURNS
        -------
        int
            Number of steps to travel tangential distance.
        """
       
        steps = int(dist / ((2.0 * radius * math.pi) / steps_pre_rev))
        logging.debug('steps: {}'.format(steps))
        return steps
    
    @staticmethod
    def calc_stepper_motor_sleep_time(velocity: float, radius: float, steps_pre_rev: float) -> float:
        """Calculate number of steps to move distance.

        PARAMETERS
        ----------
        velocity
            Tangential velocity to travel at.
        radius
            Radius of transport wheel.
        steps_pre_rev
            Steps per revolution.

        RETURNS
        -------
        float
            Stepper motor sleep time (between steps)
        """
        
        ts = (2 * radius * math.pi) / (steps_pre_rev * velocity)
        logging.debug('ts: {}'.format(ts))
        return ts
        
    def set_sheet_move(self, wheel_radius: float, trans: float = 1.0, dist: float = 0.0, vel: float = 0.1):
        """Set sheet movement.
        
        PARAMETERS
        ----------
        wheel_radius
            Radius of transport wheel.
        trans
            Transmission factor.
        dist
            Distance to travel.
        vel
            Linear velocity to travel at.
        """
        
        logging.debug('move_sheet {}'.format(dist))
        self.hwi.set_stepper_motor(
            self.calc_steps_for_distance(
                dist=dist, radius=wheel_radius / trans, steps_pre_rev=self.hwi.steps_per_rev
            ),
            self.calc_stepper_motor_sleep_time(
                velocity=vel, radius=wheel_radius / trans, steps_pre_rev=self.hwi.steps_per_rev)
        )

    def set_sheet_move_async(self, wheel_radius: float, dist: float = 0.0, vel: float = 1.0):
        """Set sheet movement asynchronously.

        PARAMETERS
        ----------
        wheel_radius
            Radius of transport wheel.
        dist
            Distance to travel.
        vel
            Linear velocity to travel at.
        """
    
        logging.debug('move_sheet_async {}'.format(dist))
        self.hwi.set_stepper_motor_async(
            self.calc_steps_for_distance(
                dist=dist, radius=wheel_radius, steps_pre_rev=self.hwi.steps_per_rev
            ),
            self.calc_stepper_motor_sleep_time(
                velocity=vel, radius=wheel_radius, steps_pre_rev=self.hwi.steps_per_rev)
        )

    def get_sheet_status(self) -> bool:
        """Get sheet movement status.
        
        RETURNS
        -------
        bool
            State of sheet: True (moving), False (not moving)
        """
        
        return self.hwi.get_stepper_motor_stat()[0]
