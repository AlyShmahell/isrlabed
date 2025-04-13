import cv2
import numpy as np

def cmap(*, colormap=cv2.COLORMAP_JET, num_colors=256):
    gradient = np.linspace(0, 255, num_colors).astype(np.uint8).reshape(-1, 1) 
    return [
        tuple(map(int, color)) 
        for color in cv2.applyColorMap(gradient, colormap)[:, 0, ::-1]
    ] 

# Example: Extract colors from JET colormap
jet_colors = cmap(cv2.COLORMAP_JET)
print(jet_colors[:10])  # Print the first 10 colors
