import curses

rover = [
    "O|_____________|O",
    " |-------|=|---| ",
    " |_____________| ",
    "O|             |O",
]

def draw_line(stdscr, x1, y1, x2, y2, char="*"):
    """ Draws a line from (x1, y1) to (x2, y2) using Bresenham's algorithm. """
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        stdscr.addch(y1, x1, char)
        if x1 == x2 and y1 == y2:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy

def draw_rover(stdscr, x, y):
    for i, line in enumerate(rover):
        stdscr.addstr(y + i, x, line)  # Print each line of the rover at position (x, y)

def main(stdscr):
    curses.curs_set(0)  # Hide cursor
    stdscr.nodelay(True)  # Non-blocking input
    stdscr.timeout(50)  # Refresh rate (20 FPS)

    max_y, max_x = stdscr.getmaxyx()
    x, y = max_x // 2, max_y // 2  # Start at center of the screen

    while True:
        stdscr.clear()
        draw_rover(stdscr, x, y)
        # draw_line(stdscr, x + 1, y + 2, x + 10, y + 2)  # Draw a line from the front of the rover
        stdscr.refresh()

        key = stdscr.getch()
        if key == curses.KEY_UP:
            y = max(0, y - 1)
        elif key == curses.KEY_DOWN:
            y = min(max_y - len(rover), y + 1)
        elif key == curses.KEY_LEFT:
            x = max(0, x - 2)
        elif key == curses.KEY_RIGHT:
            x = min(max_x - len(rover[0]), x + 2)
        elif key == ord('q'):  # Quit
            break

curses.wrapper(main)
