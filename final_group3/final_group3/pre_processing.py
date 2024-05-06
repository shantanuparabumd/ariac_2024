import cv2
import numpy as np

color_ranges = {
    'red': {
        'lower': (0, 155, 125),
        'upper': (20, 255, 175)
    },
    'blue': {
        'lower': (105, 165, 135),
        'upper': (125, 255, 190)
    },
    'green': {
        'lower': (50, 245, 85),
        'upper': (70, 255, 105)
    },
    'purple': {
        'lower': (125, 195, 195),
        'upper': (150, 255, 235)
    },
    'orange': {
        'lower': (7, 190, 190),
        'upper': (25, 255, 255)
    }
}




def compare_color(color):
    for key, value in color_ranges.items():
        lower = value['lower']
        upper = value['upper']
        if lower[0] <= color[0] <= upper[0] and lower[1] <= color[1] <= upper[1] and lower[2] <= color[2] <= upper[2]:
            return key
    return None

def canny_edge_detection(image):
    # Compute the median of the gradient magnitudes
    sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=5)
    sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=5)
    gradient_magnitude = np.sqrt(sobelx**2 + sobely**2)
    median_val = np.median(gradient_magnitude)

    # Set lower and upper thresholds based on the median
    sigma = 0.33
    lower = int(max(0, (1.0 - sigma) * median_val))
    upper = int(min(255, (1.0 + sigma) * median_val))
    # Check if image was successfully loaded
    if image is not None:
        # Convert the image to grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Canny edge detection
        edges = cv2.Canny(gray_image, lower, upper)
        edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
    return edges


def get_color(image, top, left, bottom, right, part_name):
    # Extract the bounding box region
    bbox = image[left:right, top:bottom]
    
    try:
        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(bbox, cv2.COLOR_RGB2HSV)
    except:
        print("Error converting image to HSV color space")
        return None
    # Get the center pixel coordinates
    center_x = bbox.shape[1] // 2
    center_y = bbox.shape[0] // 2
    
    # Get the pixel value of the center pixel
    center_pixel = hsv_image[center_y, center_x]
    
    # Return the dominant color in HSV format
    color = (center_pixel[0], center_pixel[1], center_pixel[2])
    color_name = compare_color(color)
    
    print(f"Detected Part Name: {part_name}, Color: {color_name}, Value: {color} at {top},{left}")
    return color_name