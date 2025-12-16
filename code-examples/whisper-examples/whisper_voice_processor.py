#!/usr/bin/env python3

"""
Whisper Voice Processor for Robotics

This script implements a real-time voice processing system using OpenAI Whisper
for robotic applications. It captures audio from a microphone and transcribes
speech to text for further processing by robotic systems.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import pyaudio
import numpy as np
import torch
import whisper
from dataclasses import dataclass
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from typing import Optional, List
import threading
import time
import queue
import webrtcvad
from scipy import signal


@dataclass
class VoiceCommand:
    """Data class for voice commands"""
    text: str
    confidence: float
    timestamp: float
    command_type: str = "general"


class VoiceActivityDetector:
    """Voice Activity Detection using WebRTC VAD"""
    def __init__(self, sample_rate: int = 16000, vad_mode: int = 3):
        self.sample_rate = sample_rate
        self.vad = webrtcvad.Vad(vad_mode)
        self.frame_duration = 30  # ms
        self.frame_size = int(sample_rate * self.frame_duration / 1000)

    def is_speech(self, audio_data: bytes) -> bool:
        """Detect if audio frame contains speech"""
        try:
            return self.vad.is_speech(audio_data, self.sample_rate)
        except:
            return False


class WhisperProcessor:
    """Whisper model processor with caching and optimization"""
    def __init__(self, model_size: str = "small", device: str = None):
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"

        self.model = whisper.load_model(model_size, device=device)

    def transcribe(self, audio: np.ndarray) -> str:
        """Transcribe audio using Whisper"""
        # Convert audio to appropriate format
        if len(audio.shape) > 1:
            audio = audio.mean(axis=1)  # Convert to mono if needed

        # Transcribe using the model's transcribe method
        result = self.model.transcribe(audio)
        return result["text"]


class WhisperVoiceProcessorNode(Node):
    """
    ROS 2 Node for Whisper-based voice processing
    """
    def __init__(self):
        super().__init__('whisper_voice_processor')

        # Declare parameters
        self.declare_parameter('model_size', 'small')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('buffer_duration', 5.0)  # seconds
        self.declare_parameter('min_voice_duration', 0.5)  # seconds
        self.declare_parameter('max_silence_duration', 1.0)  # seconds
        self.declare_parameter('vad_enabled', True)

        # Get parameter values
        self.model_size = self.get_parameter('model_size').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.buffer_duration = self.get_parameter('buffer_duration').value
        self.min_voice_duration = self.get_parameter('min_voice_duration').value
        self.max_silence_duration = self.get_parameter('max_silence_duration').value
        self.vad_enabled = self.get_parameter('vad_enabled').value

        # Initialize Whisper processor
        self.get_logger().info(f'Loading Whisper model ({self.model_size})...')
        self.whisper_processor = WhisperProcessor(self.model_size)
        self.get_logger().info('Whisper model loaded successfully')

        # Initialize audio parameters
        self.format = pyaudio.paInt16
        self.channels = 1
        self.audio_buffer_size = int(self.sample_rate * self.buffer_duration)

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Initialize VAD if enabled
        if self.vad_enabled:
            self.vad = VoiceActivityDetector(self.sample_rate)
            self.get_logger().info('Voice Activity Detection enabled')

        # Initialize buffers and state
        self.audio_buffer = np.array([], dtype=np.float32)
        self.voice_buffer = np.array([], dtype=np.float32)
        self.is_listening = False
        self.silence_counter = 0
        self.min_voice_samples = int(self.sample_rate * self.min_voice_duration)
        self.max_silence_samples = int(self.sample_rate * self.max_silence_duration)

        # Initialize queues for thread communication
        self.audio_queue = queue.Queue(maxsize=100)
        self.transcription_queue = queue.Queue(maxsize=10)

        # Create publishers
        self.transcript_pub = self.create_publisher(
            String,
            'voice_transcript',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.command_pub = self.create_publisher(
            String,
            'voice_command',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        # Start audio stream
        try:
            self.stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )
        except Exception as e:
            self.get_logger().error(f'Failed to open audio stream: {e}')
            raise

        # Start processing threads
        self.audio_thread = threading.Thread(target=self.audio_capture_thread, daemon=True)
        self.processing_thread = threading.Thread(target=self.processing_thread, daemon=True)

        self.audio_thread.start()
        self.processing_thread.start()

        self.get_logger().info('Whisper Voice Processor initialized')

    def audio_capture_thread(self):
        """Thread for capturing audio from microphone"""
        while rclpy.ok():
            try:
                # Read audio data
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)

                if self.vad_enabled:
                    # Process with VAD
                    is_speech = self.vad.is_speech(data)

                    if is_speech:
                        # Convert to float32 and add to voice buffer
                        audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
                        self.voice_buffer = np.concatenate([self.voice_buffer, audio_data])
                        self.silence_counter = 0
                        self.is_listening = True
                    elif self.is_listening:
                        # Add to silence counter
                        self.silence_counter += self.chunk_size

                        # If enough silence, process the collected voice
                        if (self.silence_counter >= self.max_silence_samples or
                            len(self.voice_buffer) >= self.audio_buffer_size):
                            if len(self.voice_buffer) >= self.min_voice_samples:
                                # Add to processing queue
                                try:
                                    self.audio_queue.put_nowait(self.voice_buffer.copy())
                                except queue.Full:
                                    self.get_logger().warning('Audio queue full, dropping frame')

                            # Reset for next voice segment
                            self.voice_buffer = np.array([], dtype=np.float32)
                            self.is_listening = False
                            self.silence_counter = 0
                else:
                    # Process without VAD - accumulate in main buffer
                    audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
                    self.audio_buffer = np.concatenate([self.audio_buffer, audio_data])

                    # Process when buffer reaches threshold
                    if len(self.audio_buffer) >= self.audio_buffer_size:
                        try:
                            self.audio_queue.put_nowait(self.audio_buffer.copy())
                        except queue.Full:
                            self.get_logger().warning('Audio queue full, dropping frame')

                        # Keep last 1 second of audio to maintain context
                        keep_samples = int(self.sample_rate)
                        if len(self.audio_buffer) > keep_samples:
                            self.audio_buffer = self.audio_buffer[-keep_samples:]

            except Exception as e:
                self.get_logger().error(f'Error in audio capture: {e}')
                time.sleep(0.01)  # Brief pause to avoid busy loop on error

    def processing_thread(self):
        """Thread for processing audio with Whisper"""
        while rclpy.ok():
            try:
                # Get audio data from queue
                audio_data = self.audio_queue.get(timeout=1.0)

                # Transcribe using Whisper
                transcript = self.whisper_processor.transcribe(audio_data)

                if transcript.strip():  # Only process non-empty transcripts
                    # Create voice command
                    command = VoiceCommand(
                        text=transcript,
                        confidence=0.8,  # Placeholder confidence
                        timestamp=time.time()
                    )

                    # Add to transcription queue
                    try:
                        self.transcription_queue.put_nowait(command)
                    except queue.Full:
                        self.get_logger().warning('Transcription queue full, dropping result')

            except queue.Empty:
                continue  # Timeout is normal, continue loop
            except Exception as e:
                self.get_logger().error(f'Error in processing thread: {e}')

    def publish_transcription(self, command: VoiceCommand):
        """Publish transcription to ROS topics"""
        # Publish raw transcript
        transcript_msg = String()
        transcript_msg.data = command.text
        self.transcript_pub.publish(transcript_msg)

        # Extract and publish commands
        extracted_command = self.extract_command(command.text)
        if extracted_command:
            command_msg = String()
            command_msg.data = extracted_command
            self.command_pub.publish(command_msg)

        self.get_logger().info(f'Transcribed: "{command.text}"')

    def extract_command(self, transcript: str) -> Optional[str]:
        """Simple command extraction from transcript"""
        transcript_lower = transcript.lower().strip()

        # Define command patterns
        command_patterns = [
            (r"move\s+(forward|backward|left|right)", "move_\\1"),
            (r"go\s+(forward|backward|left|right)", "move_\\1"),
            (r"turn\s+(left|right)", "turn_\\1"),
            (r"stop", "stop"),
            (r"come\s+here", "come_here"),
            (r"follow\s+me", "follow"),
            (r"wave", "wave"),
            (r"dance", "dance"),
            (r"hello", "greet"),
            (r"goodbye", "goodbye"),
        ]

        for pattern, replacement in command_patterns:
            import re
            match = re.search(pattern, transcript_lower)
            if match:
                return replacement.replace("\\1", match.group(1))

        # If no specific command found, return the full transcript
        return transcript if transcript.strip() else None

    def spin_once(self):
        """Process any pending transcriptions"""
        try:
            while True:  # Process all available transcriptions
                command = self.transcription_queue.get_nowait()
                self.publish_transcription(command)
        except queue.Empty:
            pass  # No more transcriptions to process

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.get_logger().info('Cleaning up Whisper Voice Processor...')

        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()

        if hasattr(self, 'audio'):
            self.audio.terminate()

        super().destroy_node()


def main(args=None):
    """Main function to run the Whisper Voice Processor node"""
    rclpy.init(args=args)

    node = WhisperVoiceProcessorNode()

    try:
        while rclpy.ok():
            # Process any pending transcriptions
            node.spin_once()
            # Brief sleep to prevent busy loop
            time.sleep(0.01)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Whisper Voice Processor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()