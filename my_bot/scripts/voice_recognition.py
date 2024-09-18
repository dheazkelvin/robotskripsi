#!/usr/bin/env python3

import os
import pyaudio
import wave
import speech_recognition as sr
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QMessageBox, QTextEdit
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtCore import Qt, QThread, pyqtSignal

import rclpy
from rclpy.node import Node

# Initialize recognizer
r = sr.Recognizer()

class RecordThread(QThread):
    recording_finished = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_recording = False
        self.frames = []

    def run(self):
        self.is_recording = True
        self.frames = []
        self.record_audio()

    def stop(self):
        self.is_recording = False

    def record_audio(self, filename='testing.wav'):
        # Redirect stderr to /dev/null to suppress ALSA warnings
        devnull = os.open(os.devnull, os.O_WRONLY)
        old_stderr = os.dup(2)
        os.dup2(devnull, 2)

        # Set chunk size, sample format, channels, and rate
        chunk = 1024  # Record in chunks of 1024 samples
        sample_format = pyaudio.paInt16  # 16 bits per sample
        channels = 1
        fs = 44100  # Record at 44100 samples per second

        p = pyaudio.PyAudio()  # Create an interface to PortAudio

        print('Recording')

        stream = p.open(format=sample_format,
                        channels=channels,
                        rate=fs,
                        frames_per_buffer=chunk,
                        input=True)

        # Store data in chunks for the duration of record_seconds
        while self.is_recording:
            data = stream.read(chunk)
            self.frames.append(data)

        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        # Terminate the PortAudio interface
        p.terminate()

        print('Finished recording')

        # Save the recorded data as a WAV file
        wf = wave.open(filename, 'wb')
        wf.setnchannels(channels)
        wf.setsampwidth(p.get_sample_size(sample_format))
        wf.setframerate(fs)
        wf.writeframes(b''.join(self.frames))
        wf.close()

        # Restore stderr
        os.dup2(old_stderr, 2)
        os.close(devnull)

        self.recording_finished.emit()

def recognize_and_respond(audio_file, recognizer, language="id-ID"):
    try:
        with sr.AudioFile(audio_file) as source:
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.record(source)
        
        text = recognizer.recognize_google(audio, language=language)
        print("Recognized Text:", text)
        if "kelas" in text.lower():
            return text, "A"
        elif "tangga" in text.lower():
            return text, "B"
        elif "toilet" in text.lower():
            return text, "C"
        elif "kantor" in text.lower():
            return text, "D"
        elif "gudang" in text.lower():
            return text, "E"
        else:
            return text, "Tujuan tidak terdefinisi, rekaman ulang."
    except sr.UnknownValueError:
        return "", "Tidak dapat mengenali audio, rekaman ulang."
    except sr.RequestError as e:
        return "", f"Error: {e}"

class VoiceRecognitionApp(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Voice Recognition')
        self.setGeometry(100, 100, 400, 300)

        layout = QVBoxLayout()

        # Load microphone icon
        try:
            icon_path = os.path.join(os.path.dirname(__file__), 'mic_icon.png')
            mic_pixmap = QPixmap(icon_path)
            if mic_pixmap.isNull():
                raise FileNotFoundError(f"File {icon_path} not found or could not be loaded.")
            mic_pixmap = mic_pixmap.scaled(100, 100, Qt.KeepAspectRatio)
            self.mic_icon = QIcon(mic_pixmap)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error loading microphone icon: {e}")
            self.close()
            return

        # Create a button with the microphone icon
        self.record_button = QPushButton(self)
        self.record_button.setIcon(self.mic_icon)
        self.record_button.setIconSize(mic_pixmap.rect().size())
        self.record_button.clicked.connect(self.toggle_recording)
        layout.addWidget(self.record_button)

        # Label to display the result
        self.result_label = QLabel('Output:', self)
        self.result_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.result_label)

        # Text edit to display the recognized text
        self.recognized_text = QTextEdit(self)
        self.recognized_text.setReadOnly(True)
        layout.addWidget(self.recognized_text)

        self.setLayout(layout)

        # Thread for recording
        self.record_thread = RecordThread()
        self.record_thread.recording_finished.connect(self.finish_recording)

    def toggle_recording(self):
        if self.record_thread.is_recording:
            self.record_thread.stop()
            self.record_button.setStyleSheet("")
        else:
            self.record_button.setStyleSheet("background-color: green")
            self.record_thread.start()

    def finish_recording(self):
        text, result = recognize_and_respond('testing.wav', r)
        self.recognized_text.setPlainText(text)
        self.result_label.setText(f"Output: {result}")
        self.record_button.setStyleSheet("")

        if result in ['A', 'B', 'C', 'D', 'E']:
            launch_file_map = {
                'A': 'go_to_pose_launch.py',
                'B': 'go_to_pose2_launch.py',
                'C': 'go_to_pose3_launch.py',
                'D': 'go_to_pose4_launch.py',
                'E': 'go_to_pose5_launch.py'
            }
            launch_file = launch_file_map[result]
            use_sim_time = 'false'  
            os.system(f'ros2 launch my_bot {launch_file} use_sim_time:={use_sim_time}')

def main(args=None):
    rclpy.init(args=args)
    node = Node('voice_recognition_node')
    
    app = QApplication([])
    voice_recognition_app = VoiceRecognitionApp(node)
    voice_recognition_app.show()

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
