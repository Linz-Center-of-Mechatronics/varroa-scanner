"""Image module image.py

Capturing and providing images.
"""

import time
import os
import logging
from typing import Type
import threading
import cv2
import enum
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk
from typing import Tuple, Callable, Any, List, Dict

from beeclient.drop_scanner.camera import CameraInterface, CameraParameters
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
    
    def __init__(self, camera_interface: Type[CameraInterface], camera_parameters: CameraParameters,
                 destination_dir: str = os.getcwd(), debug: bool = False,
                 append_camera_props_filename: bool = False):
        self._cam = camera_interface(camera_parameters=camera_parameters)
        self._destination_dir = destination_dir
        if self._destination_dir:
            os.makedirs(self._destination_dir, exist_ok=True)
        self._img_file_ext = '.jpg'
        self._img_file_fmt_str = '%Y-%m-%d_%H-%M-%S'
        self._debug = debug
        self._append_camera_props_filename = append_camera_props_filename

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
    def get_storage_path(media_path: str = '/media/pi') -> str:
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

    def capture_images(self, destination_dir: str, prefix: str = '', postfix: Any = '',
                       warm_up: bool = True, preview: bool = True, save: bool = True) -> str:
        """Capture current images.
        
        Parameters
        ----------
        destination_dir
            Directory to store images to
        prefix
            prepend to filename
        postfix
            append to filename
        warm_up
            perform camera warmup
            
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

        frames = self._cam.capture_images(dst, warm_up=warm_up, preview=preview, save=save)
        return dst, frames

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

        preview = ImagePreview(
            image_provider=self,
            destination_dir=self._destination_dir,
            camera_props_filename=self._append_camera_props_filename,
            capture_img_fcn=self.capture_images,
            fps=self._preview_fps,
            light_ctrl_fcn=hwa.set_light,
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


class ImagePreview:
    """Camera preview

    A Camera preview window built with tkinter. I previews the current camera images and provides control elements
    to take pictures, add id prefixes and shutdown the system.

    Parameters
    ----------
    image_provider
        Image provider instance
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

    def __init__(self, image_provider: ImageProvider,
                 destination_dir: str = '', camera_props_filename: bool = False,
                 capture_img_fcn: Callable = None,
                 fps: int = 7, size: Tuple[int, int] = None,
                 light_ctrl_fcn: Callable = None):
        self._image_provider = image_provider
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
        size_rel = 0.05
        self._trigger_r = int(self._size[1] * size_rel)
        self._trigger_x = int(self._size[0] / 2)  #int(self._size[0] / 2)
        self._trigger_y = int(self._size[1] / 2)  #self._id_entry_bbox[1]

        # shutdown canvas pictogram
        size_rel = 0.05
        pad = self._size[0] * size_rel
        self._shutdown_r = int(self._size[1] * size_rel)
        self._shutdown_x = int(self._size[0] - pad)
        self._shutdown_y = int(0 + pad)

        # info canvas pictorgram
        size_rel = 0.05
        pad = self._size[0] * size_rel
        self._info_x = int(self._size[0] - pad)
        self._info_y = int(self._size[1] - pad)
        self._info_r = int(pad * 2)

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

    @staticmethod
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

    def _draw_list_dir(self, color: str = '#ffffff') -> List[Any]:
        return [
            self._tk_canvas.create_text(self._info_x, self._info_y, text='?', anchor='se', fill=color, font='bold'),
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
            if self.check_within_rect(event, self._shutdown_x - self._shutdown_r, self._shutdown_x + self._shutdown_r,
                                      self._shutdown_y - self._shutdown_r, self._shutdown_y + self._shutdown_r):
                # inside shutdown pictogram
                logging.debug('click event: shutdown')
                self._shutdown()
            if self.check_within_rect(event, self._trigger_x - self._trigger_r, self._trigger_x + self._trigger_r,
                                      self._trigger_y - self._trigger_r, self._trigger_y + self._trigger_r):
                # inside trigger pictogram
                logging.debug('click event: trigger')
                self._trigger()
            if self.check_within_rect(event, self._info_x - self._info_r, self._info_x + self._info_r,
                                      self._info_y - self._info_r, self._info_y + self._info_r):
                logging.debug('open browser @ dir: {}'.format(self._destination_dir))
                os.system('pcmanfm {}'.format(self._destination_dir))
            if str(event.widget) != '.!entry' and not self.check_within_rect(
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
    def get_preview_state(state: PreviewState, action_ts: float, trigger_evt: threading.Event) -> PreviewState:
        """Get current preview state

        Parameters
        ----------
        state
            Last preview state
        action_ts
            Last timestamp at which a action was performed
        trigger_evt
            The image trigger event reference

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

        elif (state == PreviewState.AFTER_SHOT) and (now_ts - action_ts >= 2.0):
            state = PreviewState.NO_IMG

        if trigger_evt.isSet():
            if state == PreviewState.PREVIEW_IMG:
                state = PreviewState.LIGHT_SWITCH_ON

            elif state == PreviewState.LIGHT_SWITCH_ON:
                state = PreviewState.LIGHTING

            elif state == PreviewState.LIGHTING:
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
            state = self.get_preview_state(state, action_ts, self._trigger_evt)
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
                        self._destination_dir = self._image_provider.get_storage_path()
                else:
                    logging.info('storage path found: ' + self._destination_dir)
                    state = PreviewState.NO_IMG

            elif state == PreviewState.NO_CAM:
                loop_time = int(1000)
                if not self._image_provider.capture_images(warm_up=False, preview=False, destination_dir='.'):
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
                    logging.info('camera connected!')
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
                self._draw_list_dir()

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

                self._tk_canvas.delete('all')
                logging.debug('dst_dir: {}'.format(self._destination_dir))
                text_output, frames = self._image_provider.capture_images(
                    destination_dir=self._destination_dir,
                    prefix=self._id_entry.get(),
                    postfix=self._cam_props if self._camera_props_filename else ''
                )
                if type(frames) == list or type(frames) == np.ndarray:  # actual frames returned
                    self.draw_preview_image(frames, self._tk_root, self._tk_canvas, self._size)
                if self._light_ctrl_fcn:
                    light_status = False
                    self._light_ctrl_fcn(light_status)

                self._tk_canvas.create_rectangle(
                    0, 0, self._size[0], 35, fill='white', outline='white'
                )
                self._tk_canvas.create_text(
                    9, 10, text=text_output, anchor='nw'
                )
                action_ts = time.time()

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
