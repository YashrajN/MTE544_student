import matplotlib.pyplot as plt
from utilities import FileReader
import math




def plot_errors():
    
    headers, values=FileReader("robot_pose.csv").read_file()

    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)



    for i in range(0, len(headers) - 1):
        plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

    plt.legend()
    plt.grid()

    plt.show()

def plot_robot_and_planned_path():
    """
    Reads the robot pose CSV and the planned path CSV, then plots the x and y positions
    for both paths on the same graph.
    """
    # Read robot pose from CSV
    robot_headers, robot_values = FileReader("robotPose.csv").read_file()

    # Get kf_x and kf_y (third last and second last columns) for robot path
    kf_x = [val[-3] for val in robot_values]
    kf_y = [val[-2] for val in robot_values]

    # Read planned path from CSV (columns are x, y)
    planned_headers, planned_values = FileReader("planned_path.csv").read_file()
    planned_x = [val[0] for val in planned_values]
    planned_y = [val[1] for val in planned_values]

    # Calculate total length of the planned path
    total_length = 0
    for i in range(1, len(planned_x)):
        dx = planned_x[i] - planned_x[i-1]
        dy = planned_y[i] - planned_y[i-1]
        segment_length = math.sqrt(dx**2 + dy**2)
        total_length += segment_length

    print(f"Total length of the planned path: {total_length:.2f} m")

    # Plot robot path
    plt.figure(figsize=(10, 6))
    plt.plot(kf_x, kf_y, color='green', linestyle='-', label="Robot Path (kf_x, kf_y)")

    # Plot planned path
    plt.plot(planned_x, planned_y, color='orange', linestyle='-', marker='*', label="Planned Path (x, y)")

    # Highlight start and goal points
    plt.scatter(planned_x[0], planned_y[0], color='blue', edgecolor='black', s=100, label="Start Pose", marker='s')  # Boxed start point
    plt.scatter(planned_x[-1], planned_y[-1], color='red', edgecolor='black', s=100, label="Goal Pose", marker='o')  # Circled goal point
  
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Planned Path vs Robot Path")
    plt.legend()
    plt.grid(True)
    plt.show()
    
    

if __name__=="__main__":
    # plot_errors()
    plot_robot_and_planned_path()