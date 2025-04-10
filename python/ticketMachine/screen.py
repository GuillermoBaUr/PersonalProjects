"""
screen.py

This module contains functions to create the screen and display
information.

Functions:
create_screen(application): Creates the screen.
print_ticket(screen, turn): prints the ticket in the screen.
"""

from tkinter import Frame, FLAT, END, NORMAL, DISABLED, Text
import datetime


def create_screen(application):
    """
    Creates the screen and display in the application.

    Parameters:
    application (Tk): The main Tkinter application.

    Returns:
    screen : the screen object
    """

    # Create The Frame
    screen_panel = Frame(application, bd=1, relief=FLAT, width=600, height=500)

    # Placing The Frame
    screen_panel.place(x=319, y=190)

    # Creating the screen
    screen = Text(screen_panel,
                  font=('Dosis', 16, 'bold'),
                  bd=1,
                  width=28,
                  height=15)

    # Disabling Modifications
    screen.config(state=DISABLED)

    # Placing the screen
    screen.grid(row=0,
                column=0)

    # Returning the Screen Object
    return screen


def print_ticket(screen, turn):
    """
    Prints the turn ticket in the screen

    Parameters:
    screen (Tk): The screen to display the information.
    turn (str): The person's Turn

    Returns:
    None
    """

    # Getting the date information
    date = datetime.datetime.now()

    # Formatting date
    turn_date = f'\t{date.day}/{date.month}/{date.year} - {date.hour}:{date.minute}\n\n'

    # Enabling changes
    screen.config(state=NORMAL)

    # Erasing previous data
    screen.delete(1.0, END)

    # Displaying new data
    screen.insert(END, turn_date)
    screen.insert(END, "***" * 14 + "\n")
    screen.insert(END, "\tYour Turn is: \n")
    screen.insert(END, f"\t       {turn}\n")
    screen.insert(END, "  Wait a moment to be attended\n")
    screen.insert(END, "***" * 14 + "\n")

    # Disabling changes
    screen.config(state=DISABLED)
