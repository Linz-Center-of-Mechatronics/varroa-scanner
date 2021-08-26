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
    
    steps_per_rev = 200
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
    pin_n_en
        GPIO pin (board numbering) to enable the motor (negated)
    pin_n_reset
        GPIO pin (board numbering) to reset driver (negated)
    pin_n_sleep
        GPIO pin (board numbering) to set motor driver into sleep mode (negated)
    pin_step
        GPIO pin (board numbering) to perform a motor step
    pin_dir
        GPIO pin (board numbering) to control the motor direction
    mot_steps_per_rev
        Motor steps per revolution
    """
    
    pulse_dur_min = 5.0e-6
    """Minimum pulse duration for stepper motor control
    """
    
    def __init__(self, pin_light: int, pin_n_en: int, pin_n_reset: int, pin_n_sleep: int, pin_step: int, pin_dir: int,
                 mot_steps_per_rev: int):
        from beeclient.drop_scanner.image import ImageProvider
        try:
            import RPi.GPIO as GPIO
            self._gpio = GPIO
        except RuntimeError:
            raise RuntimeError("Error importing RPi.GPIO! Have you added your user to the group gpio?")
        
        self._pin_light = pin_light
        self._pin_n_en = pin_n_en
        self._pin_n_slp = pin_n_sleep
        self._pin_n_rst = pin_n_reset
        self._pin_step = pin_step
        self._pin_dir = pin_dir
        self.steps_per_rev = mot_steps_per_rev
        self._gpio.setmode(self._gpio.BOARD)
        self._gpio.setwarnings(False)
        self._gpio.setup([self._pin_dir, self._pin_step, self._pin_light, self._pin_n_slp],
                         self._gpio.OUT, initial=self._gpio.LOW)
        self._gpio.setup([self._pin_n_en, self._pin_n_rst],
                         self._gpio.OUT, initial=self._gpio.HIGH)
        
        self.image_provider_class = ImageProvider

        super(HardwareInterfacePi, self).__init__()

    def set_light_gpio(self, value):
        if value:
            self._gpio.output(self._pin_light, self._gpio.HIGH)
        else:
            self._gpio.output(self._pin_light, self._gpio.LOW)

    def get_light_gpio(self):
        return self._gpio.input(self._pin_light)

    @staticmethod
    def tmc2209_velocity_scaling_compensation(v):
        return v / (-6.686362 * v**2 + 0.955385 * v + 0.000352)
    
    def set_stepper_motor(self, steps, ts):
        if ts < self.pulse_dur_min:
            ts = self.pulse_dur_min
        
        try:
            logging.debug('init pwm')
            
            # stepper pwm
            pwm = self._gpio.PWM(self._pin_step, int(1 / ts))

            # disable motor driver deep sleep
            logging.debug('n_slp: {} -> {}'.format(self._pin_n_slp, 'HIGH'))
            self._gpio.output(self._pin_n_slp, self._gpio.HIGH)
            time.sleep(0.005)
            
            # enable motor driver outputs
            logging.debug('n_en:  {} -> {}'.format(self._pin_n_en, 'LOW'))
            self._gpio.output(self._pin_n_en, self._gpio.LOW)
            time.sleep(0.005)
            
            # wait a bit
            time.sleep(0.1)

            # perform movement
            logging.debug('pwm:   {} -> {}'.format(self._pin_step, 'ON'))
            pwm.start(50)  # pwm with 50% duty cycle
            time.sleep(ts * steps)
            logging.debug('pwm:   {} -> {}'.format(self._pin_step, 'OFF'))
            pwm.stop()
                
        except KeyboardInterrupt:
            pass
        
        finally:
            # disable motor
            logging.debug('disable motor')
            logging.debug('n_en:  {} -> {}'.format(self._pin_n_en, 'HIGH'))
            self._gpio.output(self._pin_n_en, self._gpio.HIGH)
            # reset pin states
            logging.debug('n_slp: {} -> {}'.format(self._pin_n_slp, 'LOW'))
            self._gpio.output(self._pin_n_slp, self._gpio.LOW)
            logging.debug('stp:   {} -> {}'.format(self._pin_step, 'LOW'))
            self._gpio.output(self._pin_step, self._gpio.LOW)

    def get_stepper_motor_stat(self):
        return self._gpio.input(self._pin_n_en), self._gpio.input(self._pin_step), self._gpio.input(self._pin_dir)


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
            int(self.calc_steps_for_distance(
                dist=dist, radius=wheel_radius / trans, steps_pre_rev=self.hwi.steps_per_rev
            ) * HardwareInterfacePi.tmc2209_velocity_scaling_compensation(vel)),
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
