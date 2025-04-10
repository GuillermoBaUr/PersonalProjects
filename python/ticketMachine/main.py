"""
main.py

This module initializes the ticket machine application, sets up the main window,
loads background images, screen, and buttons, and starts the main loop.

Functions:
main(): Initializes and runs the ticket machine application.
"""

from tkinter import *
import image_loader
import screen
import buttons


def main():
    """
    Initializes and runs the ticket machine application.

    Sets up the main window with specified size and properties, loads background images,
    creates the screen and buttons, and starts the main loop.

    Parameters:
    None

    Returns:
    None
    """

    # Start tkinter
    application = Tk()

    # Window size
    application.geometry('1020x630+0+0')

    # avoid window change
    application.resizable(0, 0)

    # Screen title
    application.title('Ticket Machine')

    # Background color
    application.config(bg='#00FFFF')

    # Background Images
    image_loader.load_background(application)

    # Load Screen
    widget_screen = screen.create_screen(application)

    # Load Buttons
    buttons.create_buttons(application, widget_screen)

    # Start the main Loop
    application.mainloop()


if __name__ == "__main__":
    main()
