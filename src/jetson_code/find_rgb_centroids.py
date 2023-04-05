import argparse
import re

def parse_rgb(rgb_str):
    """
    Parses a string of RGB values into a tuple of ints.
    """
    values = re.split(r'[, ]', rgb_str.strip())
    return tuple(map(int, values[:3]))

def calculate_centroid(rgb_list):
    """
    Calculates the centroid of a list of RGB values.
    """
    centroid = [0, 0, 0]

    # Calculate the sum of each RGB value
    for rgb in rgb_list:
        centroid[0] += rgb[0]
        centroid[1] += rgb[1]
        centroid[2] += rgb[2]
    
    # Divide each RGB value by the number of points
    num_points = len(rgb_list)
    centroid[0] /= num_points
    centroid[1] /= num_points
    centroid[2] /= num_points

    # Return centroid as a tuple
    return tuple(centroid)

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Calculate RGB centroids for n classes')
    parser.add_argument('-n', '--num_classes', type=int, required=True, help='Number of RGB centroids to calculate')
    parser.add_argument('-p', '--points_per_class', type=int, required=True, help='Number of points per class')

    # Parse arguments
    args = parser.parse_args()
    num_classes = args.num_classes
    points_per_class = args.points_per_class

    # Initialize list of lists to store RGB values
    class_rgb_values = [[] for _ in range(num_classes)]

    # Get RGB values from user
    for i in range(num_classes):
        for j in range(points_per_class):
            rgb_input = input(f'Enter RGB values for point {j+1} in class {i+1} (delimited by spaces or commas): ')
            class_rgb_values[i].append(parse_rgb(rgb_input))

    # Calculate centroids for each class
    centroids = [calculate_centroid(class_rgb) for class_rgb in class_rgb_values]

    # Print centroids
    print('\nRGB Centroids:')
    for i, centroid in enumerate(centroids):
        print(f'Class {i+1}: {centroid}')

if __name__ == '__main__':
    main()
