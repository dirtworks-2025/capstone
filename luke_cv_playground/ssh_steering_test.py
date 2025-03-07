import curses
import time

def main(stdscr):
    curses.cbreak()  # React instantly to keypresses
    stdscr.nodelay(True)  # Make `getch()` non-blocking
    stdscr.keypad(True)  # Capture special keys

    while True:
        key = stdscr.getch()
        if key == ord('w'):
            print("Move forward")
        elif key == ord('a'):
            print("Turn left")
        elif key == ord('s'):
            print("Move backward")
        elif key == ord('d'):
            print("Turn right")
        elif key == ord('q'):
            break  # Quit on 'q' press
        time.sleep(0.05)  # Prevent excessive CPU usage

curses.wrapper(main)
