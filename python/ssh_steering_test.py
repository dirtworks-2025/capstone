import curses
import math

rover = [
    "O|_____________|O",
    " |-------|=|---| ",
    " |_____________| ",
    "O|             |O",
]

BASE_GRID_SPACING = 15
DELTA_THETA_DEGREES = 5
DELTA_THETA_RADIANS = math.radians(DELTA_THETA_DEGREES)

def draw_rover(stdscr, x, y):
    for i, line in enumerate(rover):
        if 0 <= y + i < curses.LINES and 0 <= x < curses.COLS - len(line):  
            stdscr.addstr(y + i, x, line)

def rotate_and_translate(x, y, dx, dy, theta):
    """ Rotates and translates a point (x, y). """
    x_new = x * math.cos(theta) - y * math.sin(theta) + dx
    y_new = x * math.sin(theta) + y * math.cos(theta) + dy
    return int(x_new), int(y_new)

def draw_line(stdscr, x1, y1, x2, y2, char="+"):
    """ Draws a line using Bresenham's algorithm. """
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        if 0 < y1 < curses.LINES - 1 and 0 < x1 < curses.COLS - 1:
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

def get_scaled_spacing(theta):
    """ Adjusts grid spacing dynamically to maintain square proportions. """
    scale_factor = 0.4*math.cos(theta * 2) + 1

    scaled_hori = int(BASE_GRID_SPACING * scale_factor)
    scaled_vert = int(BASE_GRID_SPACING / scale_factor)

    return max(scaled_hori, 5), max(scaled_vert, 5)  # Prevent too-small grid gaps

def draw_grid(stdscr, dx, dy, theta):
    """ Draws a rotated and translated grid with dynamic spacing correction. """
    max_y, max_x = stdscr.getmaxyx()

    # Increase grid size to avoid clipping
    max_x *= 10
    max_y *= 10

    # Get dynamically adjusted spacing
    grid_spacing_hori, grid_spacing_vert = get_scaled_spacing(theta)

    # Draw rotated vertical lines
    for x in range(-max_x, max_x, grid_spacing_hori):
        x1, y1 = rotate_and_translate(x, -max_y, dx, dy, theta)
        x2, y2 = rotate_and_translate(x, max_y, dx, dy, theta)
        draw_line(stdscr, x1, y1, x2, y2)

    # Draw rotated horizontal lines
    for y in range(-max_y, max_y, grid_spacing_vert):
        x1, y1 = rotate_and_translate(-max_x, y, dx, dy, theta)
        x2, y2 = rotate_and_translate(max_x, y, dx, dy, theta)
        draw_line(stdscr, x1, y1, x2, y2)

def main(stdscr):
    curses.curs_set(0)  # Hide cursor
    stdscr.nodelay(True)  # Non-blocking input
    stdscr.timeout(100)  # Refresh rate (10 FPS)

    max_y, max_x = stdscr.getmaxyx()
    x, y = max_x // 2, max_y // 2  # Start at center
    dx, dy = 0, 0  # Grid displacement
    theta = 0  # Rotation angle (radians)

    while True:
        stdscr.clear()

        # Draw rotated & dynamically scaled grid
        draw_grid(stdscr, dx, dy, theta)

        # Draw rover
        draw_rover(stdscr, x, y)

        stdscr.refresh()

        key = stdscr.getch()
        if key == curses.KEY_UP:
            dy += 1  # Move grid downward (relative to rover)
        elif key == curses.KEY_DOWN:
            dy -= 1  # Move grid upward (relative to rover)
        elif key == curses.KEY_LEFT:
            theta += DELTA_THETA_RADIANS
        elif key == curses.KEY_RIGHT:
            theta -= DELTA_THETA_RADIANS
        elif key == ord('q'):  # Quit
            break

curses.wrapper(main)
