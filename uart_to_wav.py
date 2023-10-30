import serial
import wave
import struct

# UART configuration
ser = serial.Serial('COM29', 921600)  # Replace 'COMx' with your UART port and baud rate

# WAV file configuration
output_file = 'output.wav'
sample_width = 2  # 16-bit audio
channels = 1  # Mono
sample_rate = 16000  # Sample rate (you can adjust this)

# Create a WAV file for writing
with wave.open(output_file, 'wb') as wav_file:
    wav_file.setnchannels(channels)
    wav_file.setsampwidth(sample_width)
    wav_file.setframerate(sample_rate)

    try:
        while True:
            data = ser.read(1)  # Read 2 bytes (adjust for your UART data format)
            wav_file.writeframes(data)
    except KeyboardInterrupt:
        print("Recording stopped.")

ser.close()
