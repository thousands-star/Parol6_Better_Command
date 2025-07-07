import os
import platform
import serial
import logging

def get_my_os():
    my_os = platform.system()
    if my_os == "Windows":
        logging.debug("Os is Windows")
    else:
        logging.debug("Os is Linux")
    return my_os

def get_image_path(subfolder="ImageGUI"):
    """
    Return the absolute path to the image folder relative to this script.
    """
    base_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    image_path = os.path.join(base_dir, subfolder)
    logging.debug(f"[init_tools] Image path resolved: {image_path}")
    return image_path

def init_serial(starting_port_win=3, starting_port_linux=0, baudrate=3000000):
    """
    Initialize and return serial.Serial object and the starting port number (int).
    """
    my_os = platform.system()
    starting_port = starting_port_win if my_os == "Windows" else starting_port_linux
    str_port = f"COM{starting_port}" if my_os == "Windows" else f"/dev/ttyACM{starting_port}"

    try:
        ser = serial.Serial(port=str_port, baudrate=baudrate, timeout=0)
        logging.info(f"[init_tools] Serial port opened: {str_port}")
    except Exception as e:
        logging.warning(f"[init_tools] Failed to open port {str_port}: {e}. Using dummy serial.")
        ser = serial.Serial()  # fallback

    return ser, starting_port

