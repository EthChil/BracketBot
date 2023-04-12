import curses

def main(stdscr):
    # turn off cursor blinking
    curses.curs_set(1)

    # define the 6 numbers to be displayed and updated
    numbers = ["", "", "", "", "", ""]

    # define the labels for each number
    labels = ["Position (x)", "Velocity (v)", "Pitch Angle (p)", "Pitch Rate (w)", "Yaw Angle (y)", "Yaw Rate (r)"]

    # create a loop that updates the numbers and displays them on the screen
    while True:
        # clear the screen
        stdscr.clear()

        # get the size of the terminal window
        height, width = stdscr.getmaxyx()

        # calculate the x-coordinate of the center of the window
        x = int((width - 11) / 2)

        # calculate the y-coordinate of the center of the window
        y = int((height - 6) / 2)

        # display the labels and numbers on the screen
        for i in range(6):
            label_width = len(labels[i]) + 2  # add 2 for the colon and space
            stdscr.addstr(y+i, x, f"{labels[i]}: ", curses.A_BOLD)
            stdscr.addstr(numbers[i], curses.A_UNDERLINE)

        # move the cursor to the current input position
        cursor_y = y
        for i in range(6):
            cursor_y += 1
            cursor_x = x + len(labels[i]) + 2 + len(numbers[i])
            stdscr.move(cursor_y, cursor_x)

        # refresh the screen to show the changes
        stdscr.refresh()

        # get a user input
        key = stdscr.getch()

        # update the corresponding number depending on the user input
        if key == ord('x'):
            curses.curs_set(1)
            current_number = 0
        elif key == ord('v'):
            curses.curs_set(1)
            current_number = 1
        elif key == ord('p'):
            curses.curs_set(1)
            current_number = 2
        elif key == ord('w'):
            curses.curs_set(1)
            current_number = 3
        elif key == ord('y'):
            curses.curs_set(1)
            current_number = 4
        elif key == ord('r'):
            curses.curs_set(1)
            current_number = 5

        elif key == 10:  # Enter key
            curses.curs_set(0)
            current_number = None

        elif key == curses.KEY_BACKSPACE or key == 127:  # Backspace key
            if current_number is not None:
                numbers[current_number] = numbers[current_number][:-1]

        elif current_number is not None:
            numbers[current_number] += chr(key)

        # exit the loop if the user presses the 'q' key
        elif key == ord('q'):
            break

curses.wrapper(main)
