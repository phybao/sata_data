import csv
import matplotlib.pyplot as plt

# Function to read CSV files
def read_csv(file_path):
    x_data, y_data = [], []
    with open(file_path, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header
        for row in reader:
            x_data.append(float(row[0]))
            y_data.append(float(row[1]))
    return x_data, y_data

# Rotate the encoder data 90 degrees clockwise
def rotate_90_clockwise(x_data, y_data):
    rotated_x = [y for y in y_data]
    rotated_y = [-x for x in x_data]
    return rotated_x, rotated_y

# Transform encoder data by aligning to LiDAR reference
def align_to_lidar(lidar_x, lidar_y, encoder_x, encoder_y):
    # Rotate encoder data
    rotated_x, rotated_y = rotate_90_clockwise(encoder_x, encoder_y)
    
    # Compute centroids
    lidar_centroid = (sum(lidar_x) / len(lidar_x), sum(lidar_y) / len(lidar_y))
    encoder_centroid = (sum(rotated_x) / len(rotated_x), sum(rotated_y) / len(rotated_y))
    
    # Compute translation vector
    translation_vector = (lidar_centroid[0] - encoder_centroid[0], lidar_centroid[1] - encoder_centroid[1])
    
    # Apply translation
    transformed_x = [x + translation_vector[0] for x in rotated_x]
    transformed_y = [y + translation_vector[1] for y in rotated_y]
    
    return transformed_x, transformed_y

# Plot both datasets
def plot_data(lidar_x, lidar_y, encoder_x, encoder_y):

    # Ground truth trajectory points
    ground_truth_points = [
        (1.13, -0.37),
        (1.20, -1.44),
        (0.10, -1.41),
        (0.42, -0.36),
        (0.97, -0.39)
    ]
    ground_truth_x = [p[0] for p in ground_truth_points]
    ground_truth_y = [p[1] for p in ground_truth_points]
    
    plt.figure(figsize=(10, 8))
    plt.scatter(lidar_x, lidar_y, color='blue', label='LiDAR Data')
    plt.scatter(encoder_x, encoder_y, color='red', label='Encoder Data')
    plt.plot(ground_truth_x, ground_truth_y, color='green', marker='o', linestyle='-', label='Ground Truth')
    plt.title("Robot Trajactories estimated by LiDAR, Encoder, and Ground Truth")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()

# Main function
def main():
    lidar_file = "santa_v2.0.csv"
    encoder_file = "sat_encoder_xy_2.csv"
    
    # Read data
    lidar_x, lidar_y = read_csv(lidar_file)
    encoder_x, encoder_y = read_csv(encoder_file)
    
    # Align encoder data to LiDAR
    transformed_encoder_x, transformed_encoder_y = align_to_lidar(lidar_x, lidar_y, encoder_x, encoder_y)
    
    # Plot
    plot_data(lidar_x, lidar_y, transformed_encoder_x, transformed_encoder_y)

if __name__ == "__main__":
    main()

