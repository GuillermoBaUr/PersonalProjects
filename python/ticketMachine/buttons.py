"""
buttons.py

This module contains functions to create buttons for the ticket machine application.

Functions:
create_buttons(application, widget_screen): Creates buttons
for the application and sets their commands.
"""

from tkinter import Frame, Button, FLAT
import turns


def create_buttons(application, widget_screen):
    """
    Creates buttons for the application and sets their commands.

    Parameters:
    application (Tk): The main Tkinter application.
    widget_screen (Text): The Text widget where the output will be displayed.

    Returns:
    None
    """

    # Creates the Frame for the buttons
    button_panel = Frame(application, bd=1, relief=FLAT)

    # Places the Frame
    button_panel.place(x=315, y=575)

    # List of the different button title
    buttons = ['Pharmacy', 'Perfume', 'Makeup']
    buttons_created = []
    columns = 0

    # For loop to create the buttons into the frame
    for button in buttons:
        button = Button(button_panel,
                        text=button.title(),
                        font=('Dosis', 14, 'bold'),
                        fg='white',
                        bg='azure4',
                        bd=1,
                        width=9)

        # Adding the buttons to a list
        buttons_created.append(button)

        button.grid(row=0,
                    column=columns)

        # Incrementing the counter to display the next button in another column
        columns += 1

    # Giving their function to each button
    buttons_created[0].config(command=lambda: turns.pharmacy_turn(widget_screen))
    buttons_created[1].config(command=lambda: turns.perfume_turn(widget_screen))
    buttons_created[2].config(command=lambda: turns.makeup_turn(widget_screen))
