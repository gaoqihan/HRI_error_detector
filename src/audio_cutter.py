from pydub import AudioSegment
import os
def clip_audio(file_path, output_path, cut_time=10):
    audio = AudioSegment.from_wav(file_path)
    clipped_audio = audio[:cut_time * 1000]  # Clip the specified number of seconds
    output_filename = os.path.splitext(os.path.basename(output_path))[0]
    output_filename += f"_cut{cut_time}s.wav"
    output_path_with_time = os.path.join(os.path.dirname(output_path), output_filename)
    clipped_audio.export(output_path_with_time, format='wav', tags={'length': str(cut_time)})

# Usage example
input_folder = '/media/qihan/CORSAIR/Test/Data/User_3/audio'
output_folder = '/media/qihan/CORSAIR/Test/Data/User_3/audio'
cut_time = 10  # Specify the desired cut time in seconds
for filename in os.listdir(input_folder):
    if filename.endswith('.wav'):
        input_file = os.path.join(input_folder, filename)
        output_file = os.path.join(output_folder, filename)
        clip_audio(input_file, output_file, cut_time)
