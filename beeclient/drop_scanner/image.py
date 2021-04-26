"""Image module image.py

Capturing and providing images.
"""

import time
import os
import logging
from typing import Tuple, Type, List, Dict, Any
import numpy as np

from beeclient.drop_scanner.camera import CameraPreview, CameraInterface, VideoCaptureGetType, CameraParameters
from beeclient.drop_scanner.hw import HardwareAbstraction


class ImageProvider:
    """Class to abstract camera interfacing.
    
    Parameters
    ----------
    camera_interface
        Camera interface class
    camera_warmup
        Time to wait between trigger and executing the shutter
    image_preview
        Time to view taken image after shutter (in image preview only)
    camera_parameters
        Camera parameters
    destination_dir
        Image destination directory
    debug
        Enable debug mode
    camera_props_filename
        append camera parameters to filename
    """
    
    def __init__(self, camera_interface: Type[CameraInterface], camera_warmup: float, image_preview: float,
                 camera_parameters: CameraParameters, destination_dir: str = os.getcwd(), debug: bool = False,
                 camera_props_filename: bool = False):
        self._cam = camera_interface(load_cameras=False, camera_warmup=camera_warmup,
                                     camera_parameters=camera_parameters)
        self._camera_warmup = camera_warmup
        self._image_preview = image_preview
        self._destination_dir = destination_dir
        os.makedirs(self._destination_dir, exist_ok=True)
        self._img_file_ext = '.jpg'
        self._img_file_fmt_str = '%Y-%m-%d_%H-%M-%S'
        self._debug = debug
        self._camera_props_filename = camera_props_filename

        self._preview_fps = 1

        super(ImageProvider, self).__init__()
    
    @staticmethod
    def get_cam_props_str(cam_props: Dict) -> str:
        """Get camera properties string from properties dictionary
        
        Parameters
        ----------
        cam_props
            Camera properties dictionary
            
        Returns
        -------
        str
            Camera properties string
        """
        
        return str(cam_props).strip('{').strip('}')\
            .replace('\'', '').replace(' ', '').replace(':', '-').replace(',', '_')
    
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
       
    def capture_images_from_preview(self, frames: List[np.ndarray], destination_dir: str,
                                    prefix: str = '', postfix: Any = None) -> str:
        """Capture current image from preview.
        
        Parameters
        ----------
        frames
            Frames to save
        destination_dir
            Directory to store images to
        prefix
            prepend to filename
        postfix
            append to filename
        
            
        RETURNS
        -------
        str
            Destination directory name
        """
        
        if postfix:
            if type(postfix) == dict:
                postfix = self.get_cam_props_str(postfix)
            elif type(postfix) == str:
                pass
            else:
                raise ValueError('postfix value not implemented for {}'.format(type(postfix)))
        
        dst = self.generate_file_path_str(destination_dir, prefix, self._img_file_fmt_str, postfix, self._img_file_ext)
        logging.info('saving images to: {}'.format(dst))

        self._cam.capture_images(
            frames, dst, file_extension=self._img_file_ext
        )
        return dst

    def capture_images(self):
        """Capture images without preview.
        """
       
        self._cam.cams = self._cam.get_cameras()
        if not self._cam.cams:
            logging.error('No cameras detected!')
            return list()

        ts_start = time.time()
        while time.time() - ts_start < self._camera_warmup:
            self._cam.get_frames(VideoCaptureGetType.GRAB)
 
        postfix = ''
        if self._camera_props_filename:
            postfix = self.get_cam_props_str(self._cam.cams[0][1])
        dst = self.generate_file_path_str(self._destination_dir, '', self._img_file_fmt_str,
                                          postfix, self._img_file_ext)
        logging.info('saving images to: {}'.format(dst))

        self._cam.capture_images(
            self._cam.get_frames(VideoCaptureGetType.RETRIEVE),
            dst,
            file_extension=self._img_file_ext
        )
    
    def run_preview(self, size: Tuple[int, int] = None, hwa: HardwareAbstraction = None):
        """Open image preview window displaying all image frames.

        Parameters
        ----------
        size
            Size of image preview
        hwa
            Hardware interface instance
        """

        kw_args = dict()
        if size:
            kw_args.update(dict(size=size))

        preview = CameraPreview(
            cam_if=self._cam,
            destination_dir=self._destination_dir,
            camera_warmup=self._camera_warmup,
            image_preview=self._image_preview,
            fps=self._preview_fps,
            capture_img_fcn=self.capture_images_from_preview,
            light_ctrl_fcn=hwa.set_light,
            camera_props_filename=self._camera_props_filename,
            **kw_args
        )
        shutdown_flag = preview.show_preview()
        if shutdown_flag:
            self.shutdown()
    
    def shutdown(self):
        """Shut down image provider loop.
        """
        
        if not self._debug:
            os.system('sync')
            os.system('sudo poweroff')
