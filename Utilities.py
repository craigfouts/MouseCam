import ctypes

WINDOW_WIDTH = ctypes.windll.user32.GetSystemMetrics(78)
WINDOW_HEIGHT = ctypes.windll.user32.GetSystemMetrics(79)
WIDTH_BUFFER, HEIGHT_BUFFER = 0.5 * WINDOW_WIDTH, 2 * WINDOW_HEIGHT
WINDOW_WIDTH_RANGE = (WINDOW_WIDTH + WIDTH_BUFFER, 0)
WINDOW_HEIGHT_RANGE = (WINDOW_HEIGHT, -HEIGHT_BUFFER)
