"""Camera module camera.py

Interfacing and controlling cameras and image preview.
"""

import enum
from abc import ABC, abstractmethod
import os
import threading
import cv2
import numpy as np
import time
import logging
from typing import Tuple, Callable, Any, List, Dict
import tkinter as tk
from PIL import Image, ImageTk


def check_within_rect(evt, x_min: int, x_max: int, y_min: int, y_max: int) -> bool:
    """Check if evt occurred with a given rectangle
    
    Parameters
    ----------
    evt
        The event
    x_min
        Minimal x coordinate of rectangle
    x_max
        Maximal x coordinate of rectangle
    y_min
        Minimal y coordinate of rectangle
    y_max
        Maximal y coordinate of rectangle
        
    Returns
    -------
    bool
        Truth value if event occurred within rectangle
    """
    
    if (evt.x > x_min) and (evt.x < x_max) and (evt.y > y_min) and (evt.y < y_max):
        return True
    return False


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
                 brightness: int = None, contrast: int = None, saturation: int = None, hue: int = None,
                 gain: int = None, sharpness: int = None):
                 # auto_exposure: int = None, exposure: int = None,
                 # auto_wb: int = None, wb_blue_u: int = None, bw_red_v: int = None, wb_temperature: int = None):
        self.size = size
        self.buffer_len = buffer_len
        self.fps = fps
        self.fourcc = fourcc
        
        self.brightness = brightness
        self.contrast = contrast
        self.saturation = saturation
        self.hue = hue
        self.gain = gain
        self.sharpness = sharpness
        
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
    camera_warmup
        Time between trigger and shutter, for camera adjustment
    camera_parameters
        Camera parameter settings
    preview_fps
        Preview frame rate
    load_cameras
        If false available cameras are not loaded at instantiation
    
    Attributes
    ----------
    cams
        List of camera instances.
    """
    
    def __init__(self, camera_warmup: float, camera_parameters: CameraParameters, preview_fps: int = 7, load_cameras: bool = True):
        if load_cameras:
            self.cams = self.get_cameras()
        else:
            self.cams = []
        self._camera_warmup = camera_warmup
        self._camera_parameters = camera_parameters
        self._frames = None
        self._show_preview = True
        self._preview_thread = None
        self._preview_fps = preview_fps
        self._preview = False
    
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
    def get_frames(self, get_type: VideoCaptureGetType) -> List[np.ndarray]:
        """Capture current frames from cameras.
        
        Parameters
        ----------
        get_type
            Function name to use to call on camera capture reference
            
        RETURNS
        -------
        List[np.ndarray]
            List of binary camera frames.
        """
        
        pass
    
    @abstractmethod
    def release_cameras(self):
        """Release all camera instances.
        """
        
        pass
    
    @staticmethod
    def _save_images(frames: List[np.ndarray], destination: str, file_extension: str = '.jpg'):
        """Save frames to disk.
        
        Save provided image frames to disk.
        
        PARAMETERS
        ----------
        frames
            List of image frames to save to disk.
        destination
            Destination file prefix to save images to.
        file_extension
            File extension to save images.
        """
        
        dest_str_split = destination.split(file_extension)
        for idx, frame in enumerate(frames):
            file_name = dest_str_split[0] + '_{:d}'.format(idx + 1) + file_extension
            logging.info(file_name)
            cv2.imwrite(file_name, frame)
    
    def capture_images(self, frames: List[np.ndarray], destination: str, file_extension: str = '.jpg'):
        """Capture images.
        
        Capture image frames from cameras.
        
        PARAMETERS
        ----------
        frames
            Frames to save
        destination
            Destination file name prefix to save images to.
        file_extension
            File extension to use for saving images.
        """
        
        if destination:
            self._save_images(frames, destination=destination, file_extension=file_extension)


class UvcCameraInterface(CameraInterface):
    """USB video class (UVC) compliant camera interface.
    
    An interface for UVC compliant camera modules using OpenCV.
    """
    
    def __init__(self, *args, **kwargs):
        super(UvcCameraInterface, self).__init__(*args, **kwargs)

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
    
    def get_frames(self, get_type: VideoCaptureGetType):
        frames = []
        for cam, _ in self.cams:
            
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
    
    def release_cameras(self):
        for cam, _ in self.cams:
            cam.release()


class PreviewState(enum.Enum):
    """Camera preview state enum values
    """

    STARTUP = 8
    """Startup state
    """
    NO_CAM = 6
    """No camera found
    """
    NO_STORAGE_PATH = 7
    """No storage path found
    """
    NO_IMG = 0
    """No retrieved image frames are displayed
    """
    PREVIEW_IMG = 1
    """Currently retrieved image frames are displayed
    """
    LIGHT_SWITCH_ON = 2
    """Light is turned on
    """
    LIGHTING = 3
    """Waiting state during the camera adjusts to the lighting
    """
    TAKE_PICTURE = 4
    """Pictures are taken and stored
    """
    AFTER_SHOT = 5
    """The frame from which the pictures were taken are displayed for a little while
    """


class CameraPreview:
    """Camera preview

    A Camera preview window built with tkinter. I previews the current camera images and provides control elements
    to take pictures, add id prefixes and shutdown the system.

    Parameters
    ----------
    cam_if
        Camera interface instance
    camera_warmup
        Time to warmup and adjust (e.g. lighting) the camera before actually taking a picture
    image_preview
        Time to show taken image after shutter
    fps
        Frame rate at which to refresh the camera preview
    size
        Size of the camera preview
    destination_dir
        Image storage destination directory
    capture_img_fcn
        Function reference to capture an image
    light_ctrl_fcn
        Function reference to control the camera light
    camera_props_filename
        Append camera properties to filename
    """

    def __init__(self, cam_if: CameraInterface, camera_warmup: float, image_preview: float, fps: int = 7, size: Tuple[int, int] = None,
                 destination_dir: str = '',
                 capture_img_fcn: Callable = None, light_ctrl_fcn: Callable = None,
                 camera_props_filename: bool = False):
        self._cam_if = cam_if
        self._camera_warmup = camera_warmup
        self._image_preview = image_preview
        self._fps = fps
        self._destination_dir = destination_dir
        self._capture_img_fcn = capture_img_fcn
        self._light_ctrl_fcn = light_ctrl_fcn
        self._camera_props_filename = camera_props_filename
        self._cam_props = dict()

        self._tk_root = tk.Tk()
        self._tk_root.lift()
        if size:
            self._size = size
            self._tk_root.geometry("{}x{}+{}+{}".format(self._size[0], self._size[1], 0, 0))
        else:
            w, h = (self._tk_root.winfo_screenwidth(), self._tk_root.winfo_screenheight())
            self._size = (w - 2, h - 26)
            self._tk_root.geometry("{}x{}+{}+{}".format(self._size[0], self._size[1], 0, 0))

        self._stopped_evt = threading.Event()
        self._stopped_evt.clear()
        self._trigger_evt = threading.Event()
        self._trigger_evt.clear()

        self._msg_no_camera = False
        self._msg_no_storage = False

        self._init_preview_window()

    def _init_preview_window(self):
        """Init preview window with all its components
        """

        self._tk_root.wm_title('Camera Preview')

        # register callbacks
        self._tk_root.bind('<ButtonPress>', self._click_handler)
        self._tk_root.protocol("WM_DELETE_WINDOW", self._shutdown)

        # create canvas
        self._tk_canvas = tk.Canvas(self._tk_root, width=self._size[0], height=self._size[1])
        self._tk_canvas.pack()
        tk.Canvas.create_circle = self._create_circle
        tk.Canvas.create_circle_arc = self._create_circle_arc

        # create id elements
        self._id_entry_bbox = [68, int(self._size[1] - self._size[1] / 11), 25*10, 20]  # [x_min, y_min, width_px, height_px]
        self._id_label = tk.Label(self._tk_root, text="Hive ID:", bg='white')
        self._id_label.pack()
        self._id_label.place(x=9, y=self._id_entry_bbox[1] + 1 - 10)
        self._id_entry = tk.Entry(self._tk_root,  bd=1, width=self._id_entry_bbox[2] // 11)
        self._id_entry.pack()
        self._id_entry.place(x=self._id_entry_bbox[0], y=self._id_entry_bbox[1] - 10)
        self._id_entry.config(validate="focusin", validatecommand=(self._id_entry.register(self.id_entry_callback), "%V"))

        # trigger canvas pictogram
        self._trigger_r = int(self._size[1] * 0.05)
        self._trigger_x = int(self._size[0] / 2)  #int(self._size[0] / 2)
        self._trigger_y = int(self._size[1] / 2)  #self._id_entry_bbox[1]

        # shutdown canvas pictogram
        size_rel = 0.03
        pad = self._size[0] * size_rel
        self._shutdown_r = int(self._size[1] * size_rel)
        self._shutdown_x = int(self._size[0] - pad)
        self._shutdown_y = int(0 + pad)

    @staticmethod
    def id_entry_callback(*args) -> bool:
        """Callback fcn to be called by focus on id entry field

        Returns
        -------
        bool
            True
        """

        os.system('florence show')
        return True

    @staticmethod
    def _create_circle(self, x: int, y: int, r: int, **kwargs) -> Any:
        """Create a tkinter canvas circle

        Parameters
        ----------
        x
            X coordinate of circle
        y
            Y coordinate of circle
        r
            Radius of circle

        Returns
        -------
        Any
            The circle reference
        """

        return self.create_oval(x - r, y - r, x + r, y + r, **kwargs)

    @staticmethod
    def _create_circle_arc(self, x: int, y: int, r: int, **kwargs) -> Any:
        """Create a tkinter canvas arc

        Parameters
        ----------
        x
            X coordinate of circle
        y
            Y coordinate of circle
        r
            Radius of circle

        Returns
        -------
        Any
            The circle reference
        """

        if "start" in kwargs and "end" in kwargs:
            kwargs["extent"] = kwargs["end"] - kwargs["start"]
            del kwargs["end"]
        return self.create_arc(x - r, y - r, x + r, y + r, **kwargs)

    def _draw_trigger(self, color: str = '#ffffff') -> List[Any]:
        """Create a tkinter canvas element representing a trigger button

        Parameters
        ----------
        color
            Color of the button

        Returns
        -------
        List[Any]
            The button references
        """

        return [
            self._tk_canvas.create_circle(
                self._trigger_x, self._trigger_y, self._trigger_r + 2, fill='black'),
            self._tk_canvas.create_circle(
                self._trigger_x, self._trigger_y, self._trigger_r, outline=color, width=2),
            self._tk_canvas.create_circle(
                self._trigger_x, self._trigger_y, self._trigger_r * 0.87, fill=color)
        ]

    def _draw_triggered_frame(self, color: str = '#ffffff') -> List[Any]:
        """Create a tkinter canvas element representing a rectangular frame around preview

        Parameters
        ----------
        color
            Color of the button

        Returns
        -------
        List[Any]
            The references
        """

        return [
            self._tk_canvas.create_rectangle(4, 4, self._size[0]-5, self._size[1]-5,  outline=color, width=10)
        ]

    def _draw_shutdown_pic(self, color: str = '#ffffff') -> List[Any]:
        """Create a tkinter canvas element representing a shutdown button

        Parameters
        ----------
        color
            Color of the button

        Returns
        -------
        List[Any]
            The button references
        """

        return [
            self._tk_canvas.create_circle(self._shutdown_x, self._shutdown_y, self._shutdown_r, fill='red'),
            self._tk_canvas.create_circle(self._shutdown_x, self._shutdown_y, self._shutdown_r - 4, outline=color, width=3),
            self._tk_canvas.create_line(self._shutdown_x, self._shutdown_y - self._shutdown_r,
                                        self._shutdown_x, self._shutdown_y, fill=color, width=3)
        ]

    def _draw_canvas_elements(self) -> List[Any]:
        """Create all tkinter canvas elements

        Returns
        -------
        List[Any]
            All the canvas element references
        """

        return self._draw_trigger() + self._draw_shutdown_pic()

    def _click_handler(self, event):
        """The mouse click handler callback function

        Parameters
        ----------
        event
            The click event
        """

        if event.num == 1:
            if check_within_rect(event, self._shutdown_x - self._shutdown_r, self._shutdown_x + self._shutdown_r,
                                 self._shutdown_y - self._shutdown_r, self._shutdown_y + self._shutdown_r):
                # inside shutdown pictogram
                logging.debug('click event: shutdown')
                self._shutdown()
            if check_within_rect(event, self._trigger_x - self._trigger_r, self._trigger_x + self._trigger_r,
                                 self._trigger_y - self._trigger_r, self._trigger_y + self._trigger_r):
                # inside trigger pictogram
                logging.debug('click event: trigger')
                self._trigger()
            if str(event.widget) != '.!entry' and not check_within_rect(
                    event,
                    self._id_entry_bbox[0], self._id_entry_bbox[0] + self._id_entry_bbox[2],
                    self._id_entry_bbox[1], self._id_entry_bbox[1] + self._id_entry_bbox[3]):
                # outside id_entry text input area:
                logging.debug('click event: entry field')
                try:
                    self._id_label.focus()
                except tk.TclError:
                    pass

    def _shutdown(self, set_stop_evt: bool = True):
        """Shutdown hook

        Parameters
        ----------
        set_stop_evt
            Flag weather to set the 'stopped event'
        """

        self._tk_root.quit()
        self._tk_root.destroy()
        if set_stop_evt:
            self._stopped_evt.set()

    def _trigger(self):
        """Trigger hook
        """

        self._trigger_evt.set()

    @staticmethod
    def _get_storage_path(media_path: str = '/media/pi') -> str:
        """Get image storage path

        Parameters
        ----------
        media_path
            Root directory for mounts on device

        Returns
        -------
        str
            The image storage path
        """

        if os.path.exists(media_path):
            # on RPi with user pi
            devices = os.listdir(media_path)
            if devices:
                return os.path.join(media_path, devices[0])
            else:
                return None
        return os.getcwd()

    @staticmethod
    def get_preview_state(state: PreviewState, action_ts: float, trigger_evt: threading.Event,
                          lighting: bool, camera_warmup: float, image_preview: float) -> PreviewState:
        """Get current preview state

        Parameters
        ----------
        state
            Last preview state
        preview_ts
            Last timestamp at which preview image was displayed
        fps
            Preview frame rate
        action_ts
            Last timestamp at which a action was performed
        trigger_evt
            The image trigger event reference
        lighting
            Flag to specify if lighting should occur before the image shutter
        camera_warmup
            Time to wait between trigger and shutter
        image_preview
            Time to view taken image after shutter

        Returns
        -------
        PreviewState
            The current state
        """
        
        now_ts = time.time()
        if (state == PreviewState.NO_CAM) or (state == PreviewState.NO_STORAGE_PATH):
            pass

        elif state == PreviewState.NO_IMG:
            state = PreviewState.PREVIEW_IMG

        elif state == PreviewState.PREVIEW_IMG:
            state = PreviewState.NO_IMG

        elif (state == PreviewState.AFTER_SHOT) and (now_ts - action_ts >= image_preview):
            state = PreviewState.NO_IMG

        if trigger_evt.isSet():
            if state == PreviewState.PREVIEW_IMG:
                state = PreviewState.LIGHT_SWITCH_ON

            elif state == PreviewState.LIGHT_SWITCH_ON:
                state = PreviewState.LIGHTING

            elif (state == PreviewState.LIGHTING) and ((now_ts - action_ts >= camera_warmup) or (not lighting)):
                state = PreviewState.TAKE_PICTURE

            elif state == PreviewState.TAKE_PICTURE:
                state = PreviewState.AFTER_SHOT

        return state

    @staticmethod
    def draw_preview_image(frames: List[np.ndarray], tk_root: tk.Tk, tk_canvas: tk.Canvas, size: Tuple[int, int]):
        """Show preview image

        Parameters
        ----------
        frames
            All image frames to display
        tk_root
            Preview window tkinter root reference
        tk_canvas
            Preview window tkinter canvas reference
        size
            The size of the preview window
        """

        if frames:
            # view image
            frame = cv2.hconcat(frames)
            frame = cv2.resize(frame, size)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            img_tk = ImageTk.PhotoImage(image=Image.fromarray(frame))
            tk_root.img_tk = img_tk  # Prevent image object to be garbage collected!
            tk_canvas.create_image(0, 0, image=img_tk, anchor='nw')

        else:
            # view black screen
            tk_canvas.create_rectangle(0, 0, size[0], size[1], fill='#000000')

    def tk_show(self, state: PreviewState = PreviewState.STARTUP,
                loop_time: int = int(1000 / 25), preview_ts: float = -1, action_ts: float = -1):
        """Build tkinter preview window

        Parameters
        ----------
        state
            The current preview state
        loop_time
            Wait period before looping this function
        preview_ts
            The last image preview timestamp
        action_ts
            The last action timestamp
        """

        try:
            self._cam_if.get_frames(VideoCaptureGetType.GRAB)

            state = self.get_preview_state(state, action_ts, self._trigger_evt, self._light_ctrl_fcn is not None,
                                           self._camera_warmup, self._image_preview)
            if state == PreviewState.STARTUP:
                loop_time = int(10)
                self._tk_canvas.delete('all')
                self.draw_preview_image([], self._tk_root, self._tk_canvas, self._size)
                self._draw_shutdown_pic()
                state = PreviewState.NO_CAM

            elif state == PreviewState.NO_STORAGE_PATH:
                loop_time = int(1000)
                if not self._destination_dir:
                    if not self._msg_no_storage:
                        logging.warning('no storage found')
                        self._msg_no_storage = True

                        self._tk_canvas.delete('all')
                        self.draw_preview_image([], self._tk_root, self._tk_canvas, self._size)
                        self._draw_shutdown_pic()
                        self._tk_canvas.create_text(
                            int(self._size[0] / 2), int(self._size[1] / 2),
                            text='DETECTING STORAGE DEVICES', fill='#ffffff'
                        )
                    else:
                        self._destination_dir = self._get_storage_path()
                else:
                    logging.info('storage path found: ' + self._destination_dir)
                    state = PreviewState.NO_IMG

            elif state == PreviewState.NO_CAM:
                loop_time = int(1000)
                if not self._cam_if.cams:
                    if not self._msg_no_camera:
                        logging.warning('no cameras connected')
                        self._msg_no_camera = True

                        self._tk_canvas.delete('all')
                        self.draw_preview_image([], self._tk_root, self._tk_canvas, self._size)
                        self._draw_shutdown_pic()
                        self._tk_canvas.create_text(
                            int(self._size[0] / 2), int(self._size[1] / 2),
                            text='DETECTING CAMERAS', fill='#ffffff'
                        )
                    else:
                        self._cam_if.cams = self._cam_if.get_cameras()
                        self._cam_props = self._cam_if.cams[0][1]
                else:
                    logging.info('camera connected: ' + str(self._cam_if.cams))
                    self._cam_if.get_frames(VideoCaptureGetType.GRAB)
                    state = PreviewState.NO_STORAGE_PATH

            elif state == PreviewState.NO_IMG:
                # no image is displayed
                pass

            elif state == PreviewState.PREVIEW_IMG:
                # preview

                self._tk_canvas.delete('all')
                self.draw_preview_image([], self._tk_root, self._tk_canvas, self._size)
                self._draw_shutdown_pic()
                self._draw_trigger()

            elif state == PreviewState.LIGHT_SWITCH_ON:
                # turn on light after trigger
                action_ts = time.time()
                if self._light_ctrl_fcn:
                    self._light_ctrl_fcn(True)

            elif state == PreviewState.LIGHTING:
                # lighting state to wait for camera auto adjust
                preview_ts = time.time()
                self._tk_canvas.delete('all')
                self.draw_preview_image([], self._tk_root, self._tk_canvas, self._size)
                self._tk_canvas.create_text(
                    int(self._size[0] / 2), int(self._size[1] / 2),
                    text='LIGHTING ON, CAMERA ADJUSTING', fill='#ffffff'
                )
                self._draw_triggered_frame()

            elif state == PreviewState.TAKE_PICTURE:
                # still frame and capture
                action_ts = time.time()

                frames = self._cam_if.get_frames(VideoCaptureGetType.RETRIEVE)
                self._tk_canvas.delete('all')
                self.draw_preview_image(frames, self._tk_root, self._tk_canvas, self._size)

                if self._capture_img_fcn:
                    logging.debug('dst_dir: {}'.format(self._destination_dir))
                    text_output = self._capture_img_fcn(
                        frames, self._destination_dir,
                        self._id_entry.get(),
                        self._cam_props if self._camera_props_filename else ''
                    )
                    if self._light_ctrl_fcn:
                        light_status = False
                        self._light_ctrl_fcn(light_status)

                    self._tk_canvas.create_rectangle(
                        0, 0, self._size[0], 35, fill='white', outline='white'
                    )
                    self._tk_canvas.create_text(
                        9, 10, text=text_output, anchor='nw'
                    )

            elif state == PreviewState.AFTER_SHOT:
                # still frame after capture
                self._trigger_evt.clear()

            # loop this function
            self._tk_root.after(loop_time, self.tk_show,
                                state, int(100 / self._fps), preview_ts, action_ts)
        except KeyboardInterrupt:
            pass

    def show_preview(self):
        """Show preview
        """

        try:
            self.tk_show()
            self._tk_root.mainloop()
        except KeyboardInterrupt:
            self._shutdown(set_stop_evt=False)

        return self._stopped_evt.is_set()
