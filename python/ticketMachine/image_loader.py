"""
image_loader.py

This module contains functions to display Images in the
background.

Functions:
load_cloud_image(application, x_position): load cloud image in certain position.
load_sun_image(application): load the sun Image.
load_clouds_background(application): load the clouds for the background.
load_kiosk_image(application): Load Kiosk Image.
load_background(application): loads all the background elements.
"""

import tkinter as tk
from PIL import Image, ImageTk
from pathlib import Path


def load_cloud_image(application, x_position):
    """
    loads the cloud image in certain position.

    Parameters:
    application (Tk): The main Tkinter application.
    x_position (int): Horizontal Position.

    Returns:
    None
    """

    # Loading the image's path
    route = Path("img/clouds.png")

    # Opening the path
    cloud_image = Image.open(route)

    # Loading image
    cloud_photo = ImageTk.PhotoImage(cloud_image)

    # Giving image characteristics
    cloud_label = tk.Label(application, image=cloud_photo, bg='#00FFFF')

    # Placing cloud
    cloud_label.place(x=x_position, y=0)

    # Keep a reference to the image to prevent garbage collection
    cloud_label.image = cloud_photo


def load_sun_image(application):
    """
    loads the sun image in certain position.

    Parameters:
    application (Tk): The main Tkinter application.

    Returns:
    None
    """

    # Loading the image's path
    route = Path("img/sun.png")

    # Opening the path
    sun_image = Image.open(route)

    # Loading image
    sun_photo = ImageTk.PhotoImage(sun_image)

    # Giving image characteristics
    sun_label = tk.Label(application, image=sun_photo, bg='#00FFFF')

    # Placing sun
    sun_label.place(x=850, y=-100)

    # Keep a reference to the image to prevent garbage collection
    sun_label.image = sun_photo


def load_kiosk_image(application):
    """
    loads the kiosk image in certain position.

    Parameters:
    application (Tk): The main Tkinter application.

    Returns:
    None
    """

    # Loading the image's path
    route = Path("img/kiosk.png")

    # Opening the path
    kiosk_image = Image.open(route)

    # Resizing image
    scale_image = kiosk_image.resize((880, 880), Image.LANCZOS)

    # Loading image
    kiosk_photo = ImageTk.PhotoImage(scale_image)

    # Giving image characteristics
    kiosk_label = tk.Label(application, image=kiosk_photo, bg='#00FFFF')

    # Placing the kiosk
    kiosk_label.place(x=50, y=115)

    # Keep a reference to the image to prevent garbage collection
    kiosk_label.image = kiosk_photo


def load_clouds_background(application):
    """
    loads all the clouds.

    Parameters:
    application (Tk): The main Tkinter application.

    Returns:
    None
    """

    counter = 0
    for element in range(0, 3):
        x_position = 300 * counter
        load_cloud_image(application, x_position)
        counter += 1


def load_background(application):
    """
    loads all the background Image.

    Parameters:
    application (Tk): The main Tkinter application.

    Returns:
    None
    """

    load_clouds_background(application)
    load_sun_image(application)
    load_kiosk_image(application)
