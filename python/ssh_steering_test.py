import curses
import time

def main(stdscr):
    curses.cbreak()  # React instantly to keypresses
    stdscr.nodelay(True)  # Make `getch()` non-blocking
    stdscr.keypad(True)  # Capture special keys

    while True:
        key = stdscr.getch()
        if key == curses.KEY_UP:
            print("Move forward")
        elif key == curses.KEY_LEFT:
            print("Turn left")
        elif key == curses.KEY_DOWN:
            print("Move backward")
        elif key == curses.KEY_RIGHT:
            print("Turn right")
        elif key == ord('q'):
            break  # Quit on 'q' press
        time.sleep(0.05)  # Prevent excessive CPU usage

curses.wrapper(main)
