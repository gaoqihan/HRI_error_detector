import wave
import numpy as np
import matplotlib.pyplot as plt

# Open the WAV file
with wave.open('/media/qihan/CORSAIR/Test/Data/User_2/2_1.wav', 'rb') as wav_file:
    # Get the audio data
    audio_data = wav_file.readframes(-1)
    audio_data = np.frombuffer(audio_data, dtype=np.int16)

    # Get the sample rate and duration
    sample_rate = wav_file.getframerate()
    duration = len(audio_data) / sample_rate

    # Create the time axis
    time = np.linspace(0., duration, len(audio_data))

    # Plot the amplitude
    plt.plot(time, audio_data)
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.title('Amplitude Visualization')
    plt.show()