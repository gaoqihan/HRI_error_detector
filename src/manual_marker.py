import tkinter as tk
from tkinter import messagebox, ttk
import rosbag
import rospy
import os
from sensor_msgs.msg import CompressedImage
import csv
import json
import os


# Define the rosbag file path
rosbag_file_path = "/media/qihan/CORSAIR/Test/Data/User_2/2_1.bag"

# Initialize the ROS node
rospy.init_node('manual_marker')

# Define the start and end time variables
start_time = None
end_time = None
time = None

def start_clicked():
    global start_time, indicator_label
    start_time = time
    print("Start button clicked!")
    print(start_time)
    indicator_label.config(bg="red")  # Set indicator color to red

def end_clicked():
    global end_time, indicator_label
    end_time = time
    print("End button clicked!")
    error_type = show_error_selection_popup()
    if error_type is not None:
        create_rosbag(error_type)
        print("Saved")
        indicator_label.config(bg="green")  # Set indicator color to green
        window.after(500, lambda: indicator_label.config(bg="blue"))  # Set indicator color to blue after 0.5 seconds
    else:
        print("Error type not selected.")
    print(end_time)



def show_error_selection_popup():
    error_types = ["No Error",
                   "Delayed response",
                   "Moving too slow",
                   "Inappropriate placement",
                   "Not release",
                   "Stutter motion",
                   "Non-optimal motion path",
                   "Unintended error"]
    selected_error_type = None

    def confirm_clicked():
        nonlocal selected_error_type
        selected_error_type = error_type_combobox.get()
        if selected_error_type == "Unintended error":
            open_error_details_entry()
        else:
            popup_window.destroy()

    def open_error_details_entry():
        nonlocal selected_error_type,error_details_window
        error_details_window = tk.Toplevel(window)
        error_details_window.title("Error Details")
        error_details_window.geometry("300x150")

        error_details_label = tk.Label(error_details_window, text="Enter Error Details:")
        error_details_label.pack()

        error_details_entry = tk.Entry(error_details_window)
        error_details_entry.pack()

        confirm_button = tk.Button(error_details_window, text="Confirm", command=lambda: confirm_error_details(error_details_entry.get()))
        confirm_button.pack()

    def confirm_error_details(details):
        nonlocal selected_error_type, error_details_window
        selected_error_type = details
        if error_details_window is not None:
            error_details_window.destroy()  # Close the error details window
        popup_window.destroy()

    popup_window = tk.Toplevel(window)
    error_details_window = None  # Define the error_details_window variable
    popup_window.title("Error Selection")
    popup_window.geometry("300x100")

    error_type_label = tk.Label(popup_window, text="Select Error Type:")
    error_type_label.pack()

    error_type_combobox = ttk.Combobox(popup_window, values=error_types)
    error_type_combobox.pack()

    confirm_button = tk.Button(popup_window, text="Confirm", command=confirm_clicked)
    confirm_button.pack()

    popup_window.wait_window()  # Wait for the popup window to be closed
    return selected_error_type

def image_callback(msg):
    global time
    time = msg.header.stamp

rospy.Subscriber("/usb_cam1/image_raw/compressed", CompressedImage, image_callback)

def create_rosbag(error_type):
    global start_time, end_time
    if start_time is not None and end_time is not None:
        # Create a new rosbag file
        new_rosbag_file_path = get_new_rosbag_file_path()
        with rosbag.Bag(new_rosbag_file_path, 'w') as new_rosbag:
            # Read the messages from rosbag_1 and write them to the new rosbag file
            with rosbag.Bag(rosbag_file_path, 'r') as rosbag_1:
                for topic, msg, t in rosbag_1.read_messages(start_time=start_time, end_time=end_time):
                    new_rosbag.write(topic, msg, t)
        print(f"New rosbag {new_rosbag_file_path} created!")
        index, sub_rosbag_paths = create_subfolders(new_rosbag_file_path)
        add_to_catalog(error_type, start_time, end_time, index, sub_rosbag_paths)
        # Delete the new rosbag file
    else:
        print("Please click the start and end buttons first.")

def get_new_rosbag_file_path():
    # Get the directory path of the original rosbag file
    rosbag_dir = os.path.dirname(rosbag_file_path)
    # Create the "rosbag_clip_full" folder if it doesn't exist
    rosbag_clip_full_dir = os.path.join(rosbag_dir, "rosbag_clip_full")
    if not os.path.exists(rosbag_clip_full_dir):
        os.makedirs(rosbag_clip_full_dir)
    # Get the base name of the original rosbag file
    rosbag_base_name = os.path.basename(rosbag_file_path)
    # Remove the file extension from the base name
    rosbag_base_name_without_ext = os.path.splitext(rosbag_base_name)[0]
    # Find the maximum index for the new rosbag file
    index = 0
    while True:
        new_rosbag_base_name = f"{rosbag_base_name_without_ext}_{index}.bag"
        new_rosbag_file_path = os.path.join(rosbag_clip_full_dir, new_rosbag_base_name)
        if not os.path.exists(new_rosbag_file_path):
            break
        index += 1
    return new_rosbag_file_path

def create_subfolders(rosbag_file_path):
    # Get the directory path of the rosbag file
    rosbag_dir = os.path.dirname(rosbag_file_path)+"/../"
    # Define the topics and corresponding names
    topics = [
        ("/usb_cam1/image_raw/compressed", "video_1"),
        ("/usb_cam2/image_raw/compressed", "video_2"),
        ("/camera/color/image_raw/compressed", "video_3"),
        ("/audio/audio", "audio"),
        ("/ee_pose_publisher", "ee_pose"),
        ("/error_log", "error_log"),
        ("/franka_state_controller/franka_states", "robot_state")
    ]
    # Create subfolders for each topic
    index = int(rosbag_file_path.split("_")[-1].split(".")[0])
    subfolder_paths = {}
    for topic, name in topics:
        subfolder_path = os.path.join(rosbag_dir, name)
        if not os.path.exists(subfolder_path):
            os.makedirs(subfolder_path)
        # Store the rosbag clip containing the corresponding topic only
        clip_file_path = os.path.join(subfolder_path, f"{os.path.basename(rosbag_file_path).replace('.bag', '')}_{name}.bag")
        with rosbag.Bag(clip_file_path, 'w') as clip_rosbag:
            with rosbag.Bag(rosbag_file_path, 'r') as original_rosbag:
                for msg in original_rosbag.read_messages(topics=[topic]):
                    clip_rosbag.write(topic, msg.message, msg.timestamp)
        subfolder_paths[name] = clip_file_path
    print(index)
    return index,subfolder_paths

def add_to_catalog(error_type, start_time, end_time, index,sub_rosbag_paths):
    # Convert the Time objects to floats
    start_time = start_time.to_sec() if start_time else None
    end_time = end_time.to_sec() if end_time else None

    catalog_entry = {
        "index": index,
        "error_type": error_type,
        "start_time": start_time,
        "end_time": end_time,
        "subfolder_paths": {
            "video_1": sub_rosbag_paths["video_1"],
            "video_2": sub_rosbag_paths["video_2"],
            "video_3": sub_rosbag_paths["video_3"],
            "audio": sub_rosbag_paths["audio"],
            "ee_pose": sub_rosbag_paths["ee_pose"],
            "error_log": sub_rosbag_paths["error_log"],
            "robot_state": sub_rosbag_paths["robot_state"]
        }
    }

    rosbag_dir = os.path.dirname(rosbag_file_path)
    csv_file_path = os.path.join(rosbag_dir, 'catalog.csv')
    with open(csv_file_path, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([json.dumps(catalog_entry)])

def get_subfolder_paths(rosbag_file_path):
    subfolder_paths = {}
    rosbag_dir = os.path.dirname(rosbag_file_path)
    topics = [
        ("/usb_cam1/image_raw/compressed", "video_1"),
        ("/usb_cam2/image_raw/compressed", "video_2"),
        ("/camera/color/image_raw/compressed", "video_3"),
        ("/audio/audio", "audio"),
        ("/ee_pose_publisher", "ee_pose"),
        ("/error_log", "error_log"),
        ("/franka_state_controller/franka_states", "robot_state")
    ]
    for topic, name in topics:
        subfolder_path = os.path.join(rosbag_dir, name)
        subfolder_paths[name] = subfolder_path
    return subfolder_paths

def delete_latest_batch():
    # Get the directory path of the rosbag file
    rosbag_dir = os.path.dirname(rosbag_file_path) 
    # Get the subfolder paths
    subfolder_paths = get_subfolder_paths(rosbag_file_path)
    
    # Delete the latest batch of rosbag files from each subfolder
    for name, subfolder_path in subfolder_paths.items():
        # Get the list of rosbag files in the subfolder
        rosbag_files = sorted(os.listdir(subfolder_path), reverse=True)
        if rosbag_files:
            # Delete the latest rosbag file
            latest_rosbag_file = os.path.join(subfolder_path, rosbag_files[0])
            os.remove(latest_rosbag_file)
    # Delete the latest rosbag_clip_full file
    rosbag_clip_full = os.path.join(rosbag_dir, 'rosbag_clip_full')
    rosbag_files = sorted(os.listdir(rosbag_clip_full), reverse=True)
    if rosbag_files:
        # Delete the latest rosbag file
        latest_rosbag_file = os.path.join(rosbag_clip_full, rosbag_files[0])
        os.remove(latest_rosbag_file)
    # Delete the latest entry in the csv file
    csv_file_path = os.path.join(rosbag_dir, 'catalog.csv')
    with open(csv_file_path, 'r') as f:
        lines = f.readlines()
    with open(csv_file_path, 'w') as f:
        f.writelines(lines[:-1])




# Create the main window
window = tk.Tk()
# Create the indicator label
indicator_label = tk.Label(window, width=10, bg="blue")
indicator_label.pack(side="right")
# Create the buttons
start_button = tk.Button(window, text="Start", command=start_clicked)
end_button = tk.Button(window, text="End", command=end_clicked)
# Create the delete button
delete_button = tk.Button(window, text="Delete Latest Batch", command=delete_latest_batch)
# Add the delete button to the window
delete_button.pack()
# Add the buttons to the window
start_button.pack()
end_button.pack()

# Start the main event loop
window.mainloop()
