import wave
from PIL import Image
import os
import os
import cv2
from tqdm import tqdm

def extract_frames_from_video(video_file, output_folder, frame_rate=30):
    video = cv2.VideoCapture(video_file)
    num_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    downsample_factor = int(video.get(cv2.CAP_PROP_FPS) / frame_rate)
    folder_name = f'{output_folder}_framerate_{frame_rate}'
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
    else:
        return

    progress_bar = tqdm(total=num_frames, desc='Extracting frames', unit='frame')

    for frame in range(0, num_frames, downsample_factor):
        video.set(cv2.CAP_PROP_POS_FRAMES, frame)
        ret, frame_data = video.read()
        if ret:
            image = Image.fromarray(cv2.cvtColor(frame_data, cv2.COLOR_BGR2RGB))
            image.save(f'{folder_name}/frame_{frame}.png')
        progress_bar.update(1)

    progress_bar.close()
    video.release()

def convert_videos_to_images(input_folder, output_folder, frame_rate=30):
    video_files = [file for file in os.listdir(input_folder) if file.endswith('.mp4')]

    for video_file in video_files:
        video_name = os.path.splitext(video_file)[0]
        video_output_folder = os.path.join(output_folder, video_name)
        os.makedirs(video_output_folder, exist_ok=True)
        extract_frames_from_video(os.path.join(input_folder, video_file), video_output_folder, frame_rate)
        os.rmdir(video_output_folder)  # Delete the video_output_folder after extracting frames
        

convert_videos_to_images('/media/qihan/CORSAIR/Test/Data/User_3/video_2', '/media/qihan/CORSAIR/Test/Data/User_3/video_2', 3)
