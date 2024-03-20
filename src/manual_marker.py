import tkinter as tk
from tkinter import messagebox, ttk
import rosbag
import rospy
import os
from sensor_msgs.msg import CompressedImage
import csv
import json
import os
import subprocess


# Define the rosbag file path
user_number=3
task_number=3
rosbag_file_path = f"/media/qihan/CORSAIR/Test/Data/User_3/{str(user_number)}_{str(task_number)}.bag"


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
                   "Move too fast",
                   "Move too slow",
                   "Wrong placement",
                   "Wrong orientation",
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
    rosbag_dir = os.path.dirname(rosbag_file_path) + "/../"
    # Define the topics and corresponding names
    topics = [
        ("/usb_cam1/image_raw/compressed", "video_1"),
        ("/usb_cam2/image_raw/compressed", "video_2"),
        ("/camera/color/image_raw/compressed", "video_3"),
        ("/audio/audio", "audio"),
        ("/ee_pose_publisher", "ee_pose"),
        ("/error_log", "error_log"),
        ("/franka_state_controller/franka_states", "robot_state"),
        ("/franka_gripper/joint_states", "gripper_state")
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
        if name in ["video_1", "video_2", "video_3"]:
            # Convert the rosbag clip into a video
            output_file = os.path.join(subfolder_path, f"{os.path.basename(clip_file_path).replace('.bag', '')}.mp4")
            command = f"python3 /home/qihan/catkin_ws/src/multicam/src/rosbag2video.py --fps 30 --rate 1 -o {output_file} -t {topic} {clip_file_path}"
            subprocess.run(command, shell=True)
            # Delete the rosbag file
            os.remove(clip_file_path)
        if name in ["audio"]:
            # Convert the rosbag clip into a WAV file
            output_file = os.path.join(subfolder_path, f"{os.path.basename(clip_file_path).replace('.bag', '')}.wav")
            command = f"rosrun audio_convert bag2wav --input={clip_file_path} --output={output_file} --input-audio-topic={topic}"
            subprocess.run(command, shell=True)
            # Delete the rosbag file
            os.remove(clip_file_path)

        if name in ["ee_pose"]:
            # Convert the StampedPose messages to a CSV file
            csv_file_path = os.path.join(subfolder_path, f"{os.path.basename(rosbag_file_path).replace('.bag', '')}_{name}.csv")
            with open(csv_file_path, 'w') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(["timestamp", "x", "y", "z", "roll", "pitch", "yaw"])
                for topic, msg, t in rosbag.Bag(rosbag_file_path).read_messages(topics=[topic]):
                    csv_writer.writerow([msg.header.stamp.to_sec(), msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            os.remove(clip_file_path)
        
        if name in ["robot_state"]:
            # Convert the StampedPose messages to a CSV file
            csv_file_path = os.path.join(subfolder_path, f"{os.path.basename(rosbag_file_path).replace('.bag', '')}_{name}.csv")
            with open(csv_file_path, 'w') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(["timestamp", "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q_d0", "q_d1", "q_d2", "q_d3", "q_d4", "q_d5", "q_d6", "dq0", "dq1", "dq2", "dq3", "dq4", "dq5", "dq6", "dq_d0", "dq_d1", "dq_d2", "dq_d3", "dq_d4", "dq_d5", "dq_d6", "ddq_d0", "ddq_d1", "ddq_d2", "ddq_d3", "ddq_d4", "ddq_d5", "ddq_d6", "theta0", "theta1", "theta2", "theta3", "theta4", "theta5", "theta6", "dtheta0", "dtheta1", "dtheta2", "dtheta3", "dtheta4", "dtheta5", "dtheta6", "tau_J0", "tau_J1", "tau_J2", "tau_J3", "tau_J4", "tau_J5", "tau_J6", "dtau_J0", "dtau_J1", "dtau_J2", "dtau_J3", "dtau_J4", "dtau_J5", "dtau_J6", "tau_J_d0", "tau_J_d1", "tau_J_d2", "tau_J_d3", "tau_J_d4", "tau_J_d5", "tau_J_d6", "K_F_ext_hat_K0", "K_F_ext_hat_K1", "K_F_ext_hat_K2", "K_F_ext_hat_K3", "K_F_ext_hat_K4", "K_F_ext_hat_K5"])
                for topic, msg, t in rosbag.Bag(rosbag_file_path).read_messages(topics=[topic]):
                    csv_writer.writerow([
                        msg.header.stamp.to_sec(),
                        msg.q[0], msg.q[1], msg.q[2], msg.q[3], msg.q[4], msg.q[5], msg.q[6],
                        msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3], msg.q_d[4], msg.q_d[5], msg.q_d[6],
                        msg.dq[0], msg.dq[1], msg.dq[2], msg.dq[3], msg.dq[4], msg.dq[5], msg.dq[6],
                        msg.dq_d[0], msg.dq_d[1], msg.dq_d[2], msg.dq_d[3], msg.dq_d[4], msg.dq_d[5], msg.dq_d[6],
                        msg.ddq_d[0], msg.ddq_d[1], msg.ddq_d[2], msg.ddq_d[3], msg.ddq_d[4], msg.ddq_d[5], msg.ddq_d[6],
                        msg.theta[0], msg.theta[1], msg.theta[2], msg.theta[3], msg.theta[4], msg.theta[5], msg.theta[6],
                        msg.dtheta[0], msg.dtheta[1], msg.dtheta[2], msg.dtheta[3], msg.dtheta[4], msg.dtheta[5], msg.dtheta[6],
                        msg.tau_J[0], msg.tau_J[1], msg.tau_J[2], msg.tau_J[3], msg.tau_J[4], msg.tau_J[5], msg.tau_J[6],
                        msg.dtau_J[0], msg.dtau_J[1], msg.dtau_J[2], msg.dtau_J[3], msg.dtau_J[4], msg.dtau_J[5], msg.dtau_J[6],
                        msg.tau_J_d[0], msg.tau_J_d[1], msg.tau_J_d[2], msg.tau_J_d[3], msg.tau_J_d[4], msg.tau_J_d[5], msg.tau_J_d[6],
                        msg.K_F_ext_hat_K[0], msg.K_F_ext_hat_K[1], msg.K_F_ext_hat_K[2], msg.K_F_ext_hat_K[3], msg.K_F_ext_hat_K[4], msg.K_F_ext_hat_K[5]
                    ])
            os.remove(clip_file_path)
        
        if name in ["gripper_state"]:
            # Convert the StampedPose messages to a CSV file
            csv_file_path = os.path.join(subfolder_path, f"{os.path.basename(rosbag_file_path).replace('.bag', '')}_{name}.csv")
            with open(csv_file_path, 'w') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(["position", "velocity", "effort"])
                for topic, msg, t in rosbag.Bag(rosbag_file_path).read_messages(topics=[topic]):
                    csv_writer.writerow([
                        msg.header.stamp.to_sec(),
                        msg.position, msg.velocity, msg.effort
                    ])
            os.remove(clip_file_path)

    print(index)
    return index, subfolder_paths

def add_to_catalog(error_type, start_time, end_time, index,sub_rosbag_paths):
    # Convert the Time objects to floats
    start_time = start_time.to_sec() if start_time else None
    end_time = end_time.to_sec() if end_time else None

    catalog_entry = {
        "index": f"{str(user_number)}_{str(task_number)}_{str(index)}",
        "error_type": error_type,
        "start_time": start_time,
        "end_time": end_time,
        "subfolder_paths": {
            "video_1": "./video_1",
            "video_2": "./video_2",
            "video_3": "./video_3",
            "audio": "./audio",
            "ee_pose": "./ee_pose",
            "error_log": "./error_log",
            "robot_state": "./robot_state",
            "gripper_state": "./gripper_state"

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
        ("/franka_state_controller/franka_states", "robot_state"),
        ("/franka_gripper/joint_states", "gripper_state")

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

# Create a frame for the buttons
button_frame = tk.Frame(window)
button_frame.pack()

# Create the start button
start_button = tk.Button(button_frame, text="Start", command=start_clicked)
start_button.pack(side="left", padx=10)

# Create the end button
end_button = tk.Button(button_frame, text="End", command=end_clicked)
end_button.pack(side="left", padx=10)

# Create the delete button
delete_button = tk.Button(window, text="Delete Latest Batch", command=delete_latest_batch)
delete_button.pack(side="bottom", padx=10,pady=30)


# Start the main event loop
window.mainloop()
