"""Main entrypoint of the drop_scanner package.

Main entrypoint with command line argument parser. See
    beebox.drop_scanner --help
for further information.
"""

import argparse
import os
import logging
import traceback
from logging.handlers import RotatingFileHandler
from distutils import util
from ast import literal_eval
from typing import Callable
from beeclient.drop_scanner.image import ImageProvider
from beeclient.drop_scanner.camera import UvcCameraInterface, CameraParameters, PiCameraInterface
from beeclient.drop_scanner.hw import HardwareAbstraction, HardwareInterfaceDummy, HardwareInterfacePi


TYPE = 'type'
TYPE_SCAN = 'scan'
TYPE_CAMERA = 'camera'
CMD = 'cmd'
CMD_DO = 'do'
CMD_GET = 'get'
ACTION = 'action'
ACTION_MOVE = 'move'
ACTION_LIGHT = 'light'
ACTION_IMAGE = 'image'


def arg_parse() -> argparse.ArgumentParser:
    """Command line argument parsing.
    
    A hierarchical argument parser:
    drop_scan_parser
      type_parser: {scan | camera}
        drop-scan: {do | get} [--dummy]
          do: {move | light | image}
            move: distance wheel_radius
            light: on
            image: light [-d, --destination]
          get: {light, sheet}
        camera: [-d, --destination]
        
    Returns
    -------
    argparse.ArgumentParse
        The argument parser
    """
    
    drop_scan_parser = argparse.ArgumentParser(prog='beeclient.drop_scanner')
    logging_group = drop_scan_parser.add_mutually_exclusive_group()
    logging_group.add_argument('--debug', action='store_true', default=False, help='enable debug logging to console')
    logging_group.add_argument('--info', action='store_true', default=False, help='enable info logging to console')
    drop_scan_parser.add_argument('--dummy', action='store_true', default=False,
                                  help='Run application with hardware dummy.')
    drop_scan_parser.add_argument('--file-log', dest='file_log', type=str, default='', metavar='string',
                                  help='enable logging to file')
    
    camera_preferences = drop_scan_parser.add_argument_group('camera preferences')
    camera_preferences.add_argument('-s', '--size', type=lambda x: tuple(literal_eval(x)), default=(4656, 3496),
                                    metavar='(int, int)',
                                    help='image size (default: %(default)s)')
    camera_preferences.add_argument('-f', '--fps', type=int, default=10, metavar='int',
                                    help='camera frame rate (default: %(default)s)')
    camera_preferences.add_argument('-o', '--fourcc', type=str, default='MJPG', metavar='string',
                                    help='camera four character code setting (default: %(default)s)')
    camera_preferences.add_argument('-b', '--brightness', type=int, default=64, metavar='int',
                                    help='set camera brightness (default: %(default)s)')
    camera_preferences.add_argument('-c', '--contrast', type=int, default=0, metavar='int',
                                    help='set camera contrast (default: %(default)s)')
    camera_preferences.add_argument('-t', '--saturation', type=int, default=64, metavar='int',
                                    help='set camera saturation (default: %(default)s)')
    camera_preferences.add_argument('-u', '--hue', type=int, default=0, metavar='int',
                                    help='set camera hue (default: %(default)s)')
    camera_preferences.add_argument('-g', '--gain', type=int, default=1, metavar='int',
                                    help='set camera gain (default: %(default)s)')
    camera_preferences.add_argument('-n', '--sharpness', type=int, default=2, metavar='int',
                                    help='set camera sharpness (default: %(default)s)')
    camera_preferences.add_argument('-w', '--warmup-time', dest='warmup_time', type=float, default=5.0, metavar='float',
                                    help='camera warmup/adjustment time (e.g. to lighting) before taking a picture  '
                                         '(default: %(default)s)')
    camera_preferences.add_argument('-p', '--preview-time', dest='preview_time', type=float, default=10.0, metavar='float',
                                    help='image preview time after image was taken  (default: %(default)s)')
    camera_preferences.add_argument('-a', '--append-properties', dest='append_properties', action='store_true',
                                    default=False, help='append camera properties to image file name')
    camera_preferences.add_argument('-x', '--camera-type', dest='camera_type', type=str, default='csi',
                                    choices=['csi', 'usb'],
                                    help='connected camera type (default: %(default)s)')
    camera_preferences.add_argument('-q', '--image-quality', dest='image_quality', type=int, default=95,
                                    help='image quality in %% (default: %(default)s)')
    
    io_preferences = drop_scan_parser.add_argument_group('i/o preferences')
    io_preferences.add_argument('--pin-light', dest='pin_light', default=40, metavar='int',
                                help='header pin nr.: light on/off (default: %(default)s)')
    io_preferences.add_argument('--pin-step', dest='pin_step', default=37, metavar='int',
                                help='header pin nr.: step motor (default: %(default)s)')
    io_preferences.add_argument('--pin-dir', dest='pin_dir', default=38, metavar='int',
                                help='header pin nr.: direction motor (default: %(default)s)')
    io_preferences.add_argument('--pin-n-en', dest='pin_n_en', default=36, metavar='int',
                                help='header pin nr.: not enable motor (default: %(default)s)')
    io_preferences.add_argument('--pin-n-sleep', dest='pin_n_sleep', default=33, metavar='int',
                                help='header pin nr.: not sleep (default: %(default)s)')
    io_preferences.add_argument('--pin-n-reset', dest='pin_n_reset', default=35, metavar='int',
                                help='header pin nr.: not reset (default: %(default)s)')
    io_preferences.add_argument('--motor-steps-per-rev', dest='motor_steps_per_rev', default=200,
                                metavar='int', type=int,
                                help='steps per revolution of stepper motor (default: %(default)s)')

    type_parsers = drop_scan_parser.add_subparsers(dest=TYPE)
    scan_parser = type_parsers.add_parser(TYPE_SCAN, help='droppings scanning utility')

    cmd_parsers = scan_parser.add_subparsers(dest=CMD)
    do_parser = cmd_parsers.add_parser(CMD_DO, help='hardware control actions')
    
    action_parsers = do_parser.add_subparsers(dest=ACTION)
    distance_parser = action_parsers.add_parser(ACTION_MOVE, help='move droppings sheet')
    distance_parser.add_argument('distance', type=float, metavar='float',
                                 help='distance to move the sheet module [mm]')
    distance_parser.add_argument('wheel_radius', type=float, metavar='float',
                                 help='transport wheel radius of the sheet module [mm]')
    distance_parser.add_argument('--velocity', type=float, default=0.05, metavar='float',
                                 help='velocity to move the sheet module at [m/s] (default: %(default)s m/s)')
    distance_parser.add_argument('--gear-ratio', dest='gear_ratio', type=float, default=6, metavar='float',
                                 help='gear ratio of motor to sheet (default: %(default)s)')

    light_parser = action_parsers.add_parser(ACTION_LIGHT, help='switch light on/off')
    light_parser.add_argument('on', type=lambda x: bool(util.strtobool(x)), default=False, metavar='bool',
                              help='set light on/off (default: %(default)s)')

    image_parser = action_parsers.add_parser(ACTION_IMAGE, help='record image set')
    image_parser.add_argument('light', type=lambda x: bool(util.strtobool(x)), default=True, metavar='bool',
                              help='set light on/off during image capture (default: %(default)s)')
    image_parser.add_argument('-d', '--destination', type=str, default=os.getcwd(), metavar='string',
                              help='destination directory for captured images (default: current work directory)')

    get_parser = cmd_parsers.add_parser(CMD_GET, help='hardware state queries, prints state dictionary')
    get_parser.add_argument('--light', action='store_true', default=False,
                            help='state of light.')
    get_parser.add_argument('--sheet', action='store_true', default=False,
                            help='state of drop sheet.')

    camera_parser = type_parsers.add_parser(TYPE_CAMERA, help='camera utility')
    camera_parser.add_argument('-d', '--destination', type=str, default='', metavar='string',
                               help='destination directory for captured images (default: no directory)')
    
    return drop_scan_parser


def profile(fcn: Callable):
    """Profiling function

    Parameters
    ----------
    fcn
        Function to profile
    """

    import cProfile
    import pstats

    pr = cProfile.Profile()
    pr.enable()

    ex = None
    try:
        fcn()
    except Exception as ex:
        logging.error("Error: {}".format(str(ex)))
        logging.error("Traceback: \n{}".format('\n'.join(traceback.extract_stack())))

    pr.disable()
    pr.dump_stats('profile.data')

    stats = pstats.Stats('profile.data')
    stats.sort_stats('tottime')  # 'cumulative', 'tottime'
    stats.print_stats(0.05)

    if ex:
        raise ex


def init_logging(args):
    """Set up logging.
    
    Parameters
    ----------
    args
        Command line arguments
    """
    
    log = logging.getLogger()
    log.setLevel(logging.INFO)
    log.propagate = 0
        
    log_formatter = logging.Formatter('%(module)s %(levelname)-8s %(message)s')
    console_logger = logging.StreamHandler()
    console_logger.setFormatter(log_formatter)
    if args.debug:
        console_logger.setLevel(logging.DEBUG)
    elif args.info:
        console_logger.setLevel(logging.INFO)
    else:
        console_logger.setLevel(logging.ERROR)
    logging.getLogger().addHandler(console_logger)

    if args.file_log:
        file_logger = RotatingFileHandler(args.file_log, mode='a', maxBytes=5*1024*1024, backupCount=1, delay=False)
        file_log_formatter = logging.Formatter('%(asctime)s %(module)s %(levelname)-8s %(message)s')
        file_logger.setFormatter(file_log_formatter)
        file_logger.setLevel(logging.DEBUG)
        logging.getLogger().addHandler(file_logger)
    
    
def main():
    """beebox.drop_scanner main function.

    Depending on the given command line argument the drop_scanner is started. See example for usage information.

    Examples
    --------
    Running the main function and get usage information.
    >>> python3 -m beebox.drop_scanner --help
    """
    
    args = arg_parse().parse_args()
    init_logging(args)

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    if args.dummy:
        hwi_class = HardwareInterfaceDummy
    else:
        hwi_class = HardwareInterfacePi
    hwa = HardwareAbstraction(
        hwi_class,
        pin_light=args.pin_light, pin_n_en=args.pin_n_en, pin_n_reset=args.pin_n_reset, pin_n_sleep=args.pin_n_sleep,
        pin_step=args.pin_step, pin_dir=args.pin_dir, mot_steps_per_rev=args.motor_steps_per_rev
    )
    camera_parameters = CameraParameters(
        size=args.size, fps=args.fps, fourcc=args.fourcc,
        camera_warm_up_time=args.warmup_time,
        brightness=args.brightness, contrast=args.contrast, saturation=args.saturation, hue=args.hue, gain=args.gain,
        sharpness=args.sharpness, image_quality=args.image_quality
    )
    
    if args.camera_type == 'csi':
        camera_interface = PiCameraInterface
        args.warmup_time = 0
    elif args.camera_type == 'usb':
        camera_interface = UvcCameraInterface
    else:
        raise ValueError('{} is not supported!'.format(args.camera_type))
    
    if args.type == TYPE_SCAN:
        
        if args.cmd == CMD_DO:
            if args.action == ACTION_MOVE:
                hwa.set_sheet_move(wheel_radius=args.wheel_radius / 1000, dist=args.distance / 1000,
                                   vel=args.velocity, trans=args.gear_ratio)
            if args.action == ACTION_LIGHT:
                hwa.set_light(on=args.on)
            if args.action == ACTION_IMAGE:
                image_provider = ImageProvider(camera_interface=camera_interface, camera_parameters=camera_parameters,
                                               destination_dir=args.destination,
                                               append_camera_props_filename=args.append_properties)
                hwa.set_light(on=args.light)
                image_provider.capture_images(destination_dir=args.destination, warm_up=True, preview=False, save=True)
                hwa.set_light(on=False)
            
        elif args.cmd == CMD_GET:
            states = dict()
            if args.light:
                states.update(dict(light=hwa.get_light_status()))
            if args.sheet:
                states.update(dict(sheet=hwa.get_sheet_status()))
            logging.info(states)

    elif args.type == TYPE_CAMERA:
        image_provider = ImageProvider(camera_interface=camera_interface, camera_parameters=camera_parameters,
                                       destination_dir=args.destination,
                                       append_camera_props_filename=args.append_properties,
                                       debug=args.debug)
        if args.debug:
            image_provider.run_preview(size=(640, 480), hwa=hwa)
        else:
            image_provider.run_preview(hwa=hwa)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logging.error("Error: {}".format(str(e)))
        logging.error("Traceback: \n{}".format('\n'.join(traceback.extract_stack())))
