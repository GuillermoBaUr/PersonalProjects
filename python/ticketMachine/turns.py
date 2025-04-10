"""
turns.py

This module contains functions to calculate the turn of
the new user.

Functions:
decorate_ticket(turn_function): Decorates the function
perfume_turn(): Creates the next perfume department turn.
pharmacy_turn(): Creates the next pharmacy department turn.
makeup_turn(): Creates the next makeup department turn.
"""

import screen


def decorate_ticket(turn_function):
    """
    It is a decorator to avoid repeating the same steps.

    Parameters:
    turn_function(function): the function to decorate

    Returns:
    function: The create ticket function
    """
    generator = turn_function()

    def create_ticket(widget_screen):
        screen.print_ticket(widget_screen, next(generator))
    return create_ticket


@decorate_ticket
def perfume_turn():
    """
    Generates the next perfume department turn.

    Parameters:
    None.

    Returns:
    str: The next turn
    """
    number = 0
    while True:
        number += 1
        yield f"Pe-{number}"


@decorate_ticket
def pharmacy_turn():
    """
    Generates the next pharmacy department turn.

    Parameters:
    None.

    Returns:
    str: The next turn
    """
    number = 0
    while True:
        number += 1
        yield f"Ph-{number}"


@decorate_ticket
def makeup_turn():
    """
    Generates the next makeup department turn.

    Parameters:
    None.

    Returns:
    str: The next turn
    """
    number = 0
    while True:
        number += 1
        yield f"Mk-{number}"
