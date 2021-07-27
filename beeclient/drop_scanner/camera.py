"""Camera module camera.py

Interfacing and controlling cameras and image preview.
"""

import enum
from abc import ABC, abstractmethod
import os
import cv2
import time
import logging
from typing import Tuple, List, Dict
import subprocess


class VideoCaptureGetType(enum.Enum):
    READ = 0
    """Read frame from video capture
    """
    GRAB = 1
    """Grab next frame from video capture
    """
    RETRIEVE = 2
    """Retrieve last grabbed frame from video capture
    """


class CameraParameters:
    """Camera parameters configuration.
    
    Parameters
    ----------
    size
        Image size
    buffer_len
        Length of camera buffer
    fps
        Frame rate of camera
    fourcc
        Four character code defining image format
    brightness
        Camera brightness
    contrast
        Camera contrast
    saturation
        Camera color saturation
    hue
        Camera hue
    gain
        Camera gain
    sharpness
        Camera sharpness
    """
    
    def __init__(self,
                 size: Tuple[int, int] = None, buffer_len: int = 1, fps: int = None, fourcc: str = None,
                 camera_warm_up_time: float = None,
                 brightness: int = None, contrast: int = None, saturation: int = None, hue: int = None,
                 gain: int = None, sharpness: int = None, image_quality: int = None):
                 # auto_exposure: int = None, exposure: int = None,
                 # auto_wb: int = None, wb_blue_u: int = None, bw_red_v: int = None, wb_temperature: int = None):
        self.size = size
        self.buffer_len = buffer_len
        self.fps = fps
        self.fourcc = fourcc

        self.camera_warm_up_time = camera_warm_up_time
        
        self.brightness = brightness
        self.contrast = contrast
        self.saturation = saturation
        self.hue = hue
        self.gain = gain
        self.sharpness = sharpness
        self.image_quality = image_quality
        
        # self.auto_exposure = auto_exposure
        # self.exposure = exposure
        
        # self.auto_wb = auto_wb
        # self.wb_blue_u = wb_blue_u
        # self.wb_red_v = bw_red_v
        # self.wb_temperature = wb_temperature
        

class CameraInterface(ABC):
    """
    Abstract class prototype for interfacing with different camera types.
    
    Parameters
    ----------
    camera_parameters
        Camera parameter settings
    """
    
    def __init__(self, camera_parameters: CameraParameters):
        self._camera_parameters = camera_parameters

    @abstractmethod
    def get_cameras(self) -> List[Tuple[cv2.VideoCapture, Dict]]:
        """Get camera instances.
        
        Get camera instances to capture images from.
        
        RETURNS
        -------
        List[Tuple[cv2.VideoCapture, Dict]]
            List of camera instances and camera properties dictionaries.
        """
        
        pass

    @abstractmethod
    def capture_images(self, img_path: str = 'tmp.jpg', warm_up: bool = True, preview: bool = True, save: bool = True):
        """Capture images from cameras to file.

        Returns
        -------

        """

        pass

    @staticmethod
    def generate_file_path_str(base_path: str, prefix: str, date_fmt: str, postfix: str, file_ext: str):
        """Generate image file path

        Parameters
        ----------
        base_path
            Path to put file in
        prefix
            Filename prefix
        date_fmt
            Date format string
        postfix
            Filename postfix
        file_ext
            Filename extension

        Returns
        -------
        str
            The full file path
        """

        if prefix:
            prefix += '_'
        if postfix:
            postfix = '_' + postfix

        return os.path.join(
            base_path,
            '{}{}{}{}'.format(
                prefix,
                time.strftime(date_fmt),
                postfix,
                file_ext)
        )


class UvcCameraInterface(CameraInterface):
    """USB video class (UVC) compliant camera interface.
    
    An interface for UVC compliant camera modules using OpenCV.
    """
    
    def __init__(self, *args, **kwargs):
        super(UvcCameraInterface, self).__init__(*args, **kwargs)
        self._preview_window = 'preview'

    @staticmethod
    def list_ports() -> Tuple[List[int], List[int]]:
        """Test the ports and return a tuple with the available ports and the ones that are working.
        
        Returns
        -------
        Tuple[List[int], List[int]]
            List of working ports and list of available ports
        """
        
        working_ports = []
        available_ports = []
        devices = [dev for dev in os.listdir('/dev/') if 'video' in dev]
        for dev_port in range(len(devices)):
            camera = cv2.VideoCapture(dev_port)
            if camera.isOpened():
                is_reading, img = camera.read()
                if is_reading:
                    working_ports.append(dev_port)
                else:
                    available_ports.append(dev_port)
            camera.release()
    
        return working_ports, available_ports

    def get_cameras(self):
        video_ports, _ = self.list_ports()
        cams = list()
        for port_idx in video_ports:
            cam = cv2.VideoCapture(port_idx)

            if self._camera_parameters.fourcc is not None:
                cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self._camera_parameters.fourcc))

            if self._camera_parameters is not None:
                cam.set(cv2.CAP_PROP_FPS, self._camera_parameters.fps)
            if self._camera_parameters.buffer_len is not None:
                cam.set(cv2.CAP_PROP_BUFFERSIZE, self._camera_parameters.buffer_len)
            if self._camera_parameters.size is not None:
                cam.set(cv2.CAP_PROP_FRAME_WIDTH, self._camera_parameters.size[0])
                cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self._camera_parameters.size[1])
            
            if self._camera_parameters.brightness is not None:
                cam.set(cv2.CAP_PROP_BRIGHTNESS, self._camera_parameters.brightness)
            if self._camera_parameters.contrast is not None:
                cam.set(cv2.CAP_PROP_CONTRAST, self._camera_parameters.contrast)
            if self._camera_parameters.saturation is not None:
                cam.set(cv2.CAP_PROP_SATURATION, self._camera_parameters.saturation)
            if self._camera_parameters.hue is not None:
                cam.set(cv2.CAP_PROP_HUE, self._camera_parameters.hue)
            if self._camera_parameters.gain is not None:
                cam.set(cv2.CAP_PROP_GAIN, self._camera_parameters.gain)
            if self._camera_parameters.sharpness is not None:
                cam.set(cv2.CAP_PROP_SHARPNESS, self._camera_parameters.sharpness)
            
            cam_props = dict(
                # id=port_idx,
                size=(int(cam.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))),
                fps=int(cam.get(cv2.CAP_PROP_FPS)),
                fourcc="".join([chr((int(cam.get(cv2.CAP_PROP_FOURCC)) >> 8 * i) & 0xFF) for i in range(4)]),
                # buffer_len=int(cam.get(cv2.CAP_PROP_BUFFERSIZE)),
                brightness=int(cam.get(cv2.CAP_PROP_BRIGHTNESS)),
                contrast=int(cam.get(cv2.CAP_PROP_CONTRAST)),
                saturation=int(cam.get(cv2.CAP_PROP_SATURATION)),
                hue=int(cam.get(cv2.CAP_PROP_HUE)),
                gain=int(cam.get(cv2.CAP_PROP_GAIN)),
                sharpness=int(cam.get(cv2.CAP_PROP_SHARPNESS)),
                # mode=int(cam.get(cv2.CAP_PROP_MODE)),
                # auto_exposure=int(cam.get(cv2.CAP_PROP_AUTO_EXPOSURE)),
                # exposure=int(cam.get(cv2.CAP_PROP_EXPOSURE)),
                # autofocus=int(cam.get(cv2.CAP_PROP_AUTOFOCUS)),
                # focus=int(cam.get(cv2.CAP_PROP_FOCUS)),
                # auto_wb=int(cam.get(cv2.CAP_PROP_AUTO_WB)),
                # wb_blue_u=int(cam.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U)),
                # wb_red_v=int(cam.get(cv2.CAP_PROP_WHITE_BALANCE_RED_V)),
                # wb_temperature=int(cam.get(cv2.CAP_PROP_WB_TEMPERATURE))
            )
            
            cams.append((cam, cam_props))

        return cams

    def get_frames(self, cams, get_type: VideoCaptureGetType):
        frames = []
        for cam, _ in cams:
            
            if get_type == VideoCaptureGetType.READ:
                ret, frame = cam.read()
                if not ret:
                    logging.error('Error: Failed to get frame!')
                else:
                    frames.append(frame)
                    
            elif get_type == VideoCaptureGetType.RETRIEVE:
                ret, frame = cam.retrieve()
                if not ret:
                    logging.error('Error: Failed to retrieve frame!')
                else:
                    frames.append(frame)
                    
            elif get_type == VideoCaptureGetType.GRAB:
                ret = cam.grab()
                if not ret:
                    logging.error('Error: Failed to grab frame!')
            
            else:
                raise ValueError('Error: {} not implemented!'.format(get_type))
                
        return frames
    
    def release_cameras(self, cams):
        for cam, _ in cams:
            cam.release()

    def capture_images(self, img_path: str = 'tmp.jpg', warm_up: bool = True,
                       preview: bool = True, preview_size: Tuple[int, int] = (800, 480),
                       save: bool = True):
        """Capture images from cameras to file.

        Returns
        -------

        """

        cameras = self.get_cameras()
        if not cameras:
            return False

        if warm_up:
            ts_start = time.time()
            while time.time() - ts_start < self._camera_parameters.camera_warm_up_time:
                self.get_frames(cameras, VideoCaptureGetType.GRAB)

        frames = self.get_frames(cameras, VideoCaptureGetType.READ)

        if save:
            for idx, frame in enumerate(frames):
                cv2.imwrite(img_path.replace('.jpg', '_' + str(idx) + '.jpg'), frame)

        self.release_cameras(cameras)
        return frames


class PiCameraInterface(CameraInterface):

    def get_cameras(self):
        cams = list()
        if self.capture_images(warm_up=False):
            cams.append((0, dict))
            # cams.append((1, dict))
        return cams

    def capture_images(self, img_path: str = 'tmp.jpg', warm_up: bool = True,
                       preview: bool = True, preview_size: Tuple[int, int] = (800, 480),
                       save: bool = True):
        try:
            cmd = 'raspistill -v -dt '
            if not preview:
                cmd += '-n '
            else:
                cmd += '-p 0,0,{},{} '.format(preview_size[0], preview_size[1])
            cmd += '-q {} '.format(int(self._camera_parameters.image_quality))
            if not warm_up:
                cmd += '-t 1 '
            else:
                # cmd += '-t {}'.format(self._camera_parameters.camera_warm_up_time) # TODO seems to turn off time out
                pass
            cmd += '-o {}'.format(img_path if save else 'tmp.jpg')
            logging.debug(cmd)
            output = subprocess.check_output(cmd.split(' '), universal_newlines=True)
            logging.info(output)
            os.system('sync')
            return True
        
        except subprocess.CalledProcessError as err:
            logging.error('Error capturing image: code: {}, msg: {}'.format(err.returncode, err.output))
            return False


