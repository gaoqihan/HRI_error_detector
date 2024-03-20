from face_crop_plus import Cropper
import os
import shutil
import torch

print(torch.cuda.is_available())
print(torch.cuda.current_device())
cropper = Cropper(face_factor=0.7, strategy="largest",device="cuda")

input_dir = "/media/qihan/CORSAIR/Test/Data_clean/User_3/video_2/"
for folder in os.listdir(input_dir):
    if "face" in folder:
        continue
    
    folder_path = os.path.join(input_dir, folder)
    if os.path.isdir(folder_path): 
        cropper.process_dir(input_dir=folder_path) 
        shutil.rmtree(folder_path)

