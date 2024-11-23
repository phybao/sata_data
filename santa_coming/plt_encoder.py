import csv
import matplotlib.pyplot as plt

# CSV file name
csv_file = "sat_encoder_xy_0.csv"

def read_csv(file_name):
    """Read the x and y data from the CSV file."""
    x_data = []
    y_data = []
    try:
        with open(file_name, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header row
            for row in reader:
                x_data.append(float(row[0]))  # First column is x
                y_data.append(float(row[1]))  # Second column is y
    except FileNotFoundError:
        print(f"Error: File '{file_name}' not found.")
    except ValueError as e:
        print(f"Error reading data: {e}")
    return x_data, y_data

def plot_trajectory(x, y):
    """Plot the trajectory on the XY plane."""
    if not x or not y:
        print("No data to plot.")
        return

    plt.figure(figsize=(8, 6))
    plt.plot(x, y, marker='o', linestyle='-', color='b', label="Robot Path")
    plt.title("Robot Trajectory on XY Plane")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")  # Ensure equal scaling for X and Y axes
    plt.show()

def main():
    x_data, y_data = read_csv(csv_file)
    plot_trajectory(x_data, y_data)

if __name__ == "__main__":
    main()
