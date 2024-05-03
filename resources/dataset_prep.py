import cv2
import os
import numpy as np
def canny_edge_detection(input_folder, output_folder):
    # Check if output folder exists, if not, create it
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # List all files in the input folder
    files = [f for f in os.listdir(input_folder) if os.path.isfile(os.path.join(input_folder, f))]

    # Process each file
    for file_name in files:
        # Define the path for input and output
        input_path = os.path.join(input_folder, file_name)
        output_path = os.path.join(output_folder, file_name)

        # Read the image
        image = cv2.imread(input_path)
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

            # Save the processed image to the output folder
            cv2.imwrite(output_path, edges)
        else:
            print(f"Could not read image {file_name}")

# Set the input and output folders
input_folder = 'train'
output_folder = 'canny'

# Run the function
canny_edge_detection(input_folder, output_folder)
