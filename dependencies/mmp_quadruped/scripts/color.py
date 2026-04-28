import numpy as np

def get_constant_color():
    color = [1.0, 1.0, 1.0]  # Gray color
    return color

def get_random_colors():
    color = [0.0,
                0.50 + 0.50 * np.random.random_sample(),
                0.50 + 0.50 * np.random.random_sample()]
    return color