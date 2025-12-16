---
sidebar_label: 'Chapter 7: Voice Processing with Whisper'
sidebar_position: 1
---

# Chapter 7: Voice Processing with Whisper

## Overview
This chapter covers the implementation of voice processing systems using OpenAI's Whisper for speech-to-text conversion in humanoid robotics applications. Students will learn to integrate Whisper with ROS 2, process voice commands in real-time, and handle various acoustic conditions that may be encountered in real-world robotic environments.

## Learning Objectives
After completing this chapter, students will be able to:
- Install and configure OpenAI Whisper for robotic applications
- Integrate Whisper with ROS 2 for real-time voice processing
- Process voice commands and convert them to text
- Handle various acoustic conditions and noise environments
- Implement voice activity detection and command parsing
- Validate voice processing accuracy and latency

## 7.1 Introduction to Whisper for Robotics

### What is Whisper?
Whisper is a general-purpose speech recognition model developed by OpenAI. It is designed to be robust across various domains, languages, and acoustic conditions. For robotics applications, Whisper provides:
- High accuracy speech-to-text conversion
- Support for multiple languages
- Robustness to background noise
- Real-time processing capabilities

### Key Features for Robotics
- **Multi-language Support**: Supports 99 languages for international applications
- **Robustness**: Performs well in noisy environments
- **Timestamps**: Provides word-level timing information
- **Punctuation**: Automatically adds punctuation to transcriptions
- **Speaker Identification**: Can distinguish between different speakers

### Whisper Model Variants
Whisper comes in several sizes optimized for different performance requirements:

| Model | Parameters | Required VRAM | Relative Speed |
|-------|------------|---------------|----------------|
| tiny  | 39 M       | ~1 GB         | 32x            |
| base  | 74 M       | ~1 GB         | 16x            |
| small | 244 M      | ~2 GB         | 6x             |
| medium| 769 M      | ~5 GB         | 2x             |
| large | 1550 M     | ~10 GB        | 1x             |

For robotics applications, the "small" or "medium" models typically provide the best balance of accuracy and computational requirements.

## 7.2 Installing and Setting Up Whisper

### System Requirements
- Python 3.9 or higher
- PyTorch 1.10 or higher
- At least 2GB RAM (for small model) or 5GB (for medium model)
- CUDA-compatible GPU recommended for real-time performance

### Installation Process
```bash
# Install Whisper and its dependencies
pip install openai-whisper

# For GPU acceleration (if available)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Additional dependencies for audio processing
pip install pyaudio soundfile librosa
```

### Basic Whisper Usage Example
```python
import whisper

# Load model (downloads automatically on first use)
model = whisper.load_model("small")

# Transcribe audio file
result = model.transcribe("audio_file.wav")

# Print the transcribed text
print(result["text"])
```

## 7.3 Whisper Integration with ROS 2

### Audio Input from Microphone
For real-time voice processing, we need to capture audio from a microphone and process it in chunks:

```python
import rclpy
from rclpy.node import Node
import pyaudio
import numpy as np
import whisper
import threading
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Initialize Whisper model
        self.model = whisper.load_model("small", device="cuda" if torch.cuda.is_available() else "cpu")

        # Audio parameters
        self.rate = 16000  # Sample rate
        self.chunk = 1024  # Buffer size
        self.format = pyaudio.paInt16
        self.channels = 1

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # ROS 2 publishers
        self.transcript_pub = self.create_publisher(String, 'voice_transcript', 10)
        self.command_pub = self.create_publisher(String, 'voice_command', 10)

        # Start audio stream
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_audio)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info('Whisper node initialized')

    def process_audio(self):
        """Process audio in chunks for real-time transcription"""
        buffer = np.array([], dtype=np.float32)

        while rclpy.ok():
            # Read audio data
            data = self.stream.read(self.chunk, exception_on_overflow=False)
            audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

            # Add to buffer
            buffer = np.concatenate([buffer, audio_data])

            # Process when buffer reaches 5 seconds (80,000 samples at 16kHz)
            if len(buffer) >= self.rate * 5:
                # Process audio with Whisper
                transcript = self.transcribe_audio(buffer)

                if transcript.strip():  # Only publish if there's actual text
                    self.publish_transcript(transcript)

                # Keep last 1 second of audio to maintain context
                buffer = buffer[-self.rate:]

    def transcribe_audio(self, audio_buffer):
        """Transcribe audio buffer using Whisper"""
        try:
            # Convert to the format expected by Whisper
            audio_tensor = torch.from_numpy(audio_buffer).to(self.model.device)

            # Transcribe
            result = self.model.transcribe(audio_tensor.cpu().numpy())
            return result["text"]
        except Exception as e:
            self.get_logger().error(f'Error during transcription: {e}')
            return ""

    def publish_transcript(self, transcript):
        """Publish transcript to ROS topics"""
        # Publish raw transcript
        transcript_msg = String()
        transcript_msg.data = transcript
        self.transcript_pub.publish(transcript_msg)

        # Extract and publish commands (simplified - in practice you'd use NLP)
        command = self.extract_command(transcript)
        if command:
            command_msg = String()
            command_msg.data = command
            self.command_pub.publish(command_msg)

    def extract_command(self, transcript):
        """Simple command extraction (in practice, use more sophisticated NLP)"""
        # Convert to lowercase for easier matching
        text = transcript.lower().strip()

        # Define simple command patterns
        commands = [
            "move forward", "move backward", "turn left", "turn right",
            "stop", "sit down", "stand up", "wave", "dance", "hello"
        ]

        for cmd in commands:
            if cmd in text:
                return cmd

        return None

def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperNode()

    try:
        rclpy.spin(whisper_node)
    except KeyboardInterrupt:
        pass
    finally:
        whisper_node.stream.stop_stream()
        whisper_node.stream.close()
        whisper_node.audio.terminate()
        whisper_node.destroy_node()
        rclpy.shutdown()
```

## 7.4 Voice Activity Detection

### Importance in Robotics
Voice activity detection (VAD) is crucial for robotics applications to:
- Reduce computational load by only processing when speech is detected
- Improve accuracy by avoiding background noise processing
- Enable more responsive voice interaction

### Implementation with Silero VAD
```python
import torch
import torchaudio
from scipy import signal

class VoiceActivityDetector:
    def __init__(self, threshold=0.5):
        # Load Silero VAD model
        self.model, _ = torch.hub.load(
            repo_or_dir='snakers4/silero-vad',
            model='silero_vad',
            force_reload=False
        )
        self.threshold = threshold
        self.sample_rate = 16000

    def is_speech(self, audio_chunk):
        """Detect if audio chunk contains speech"""
        # Ensure audio is in the right format
        if len(audio_chunk.shape) == 1:
            audio_chunk = audio_chunk.unsqueeze(0)

        # Get VAD probability
        vad_prob = self.model(audio_chunk, self.sample_rate).item()

        return vad_prob > self.threshold
```

### Integration with Whisper Processing
```python
class WhisperWithVAD(WhisperNode):
    def __init__(self):
        super().__init__()

        # Initialize VAD
        self.vad = VoiceActivityDetector()

        # Voice activity state
        self.is_listening = False
        self.voice_buffer = np.array([], dtype=np.float32)
        self.silence_counter = 0
        self.min_voice_duration = self.rate * 0.5  # Minimum 0.5 seconds of voice
        self.max_silence_duration = self.rate * 1.0  # Maximum 1 second of silence

    def process_audio(self):
        """Process audio with VAD for more efficient processing"""
        while rclpy.ok():
            # Read audio data
            data = self.stream.read(self.chunk, exception_on_overflow=False)
            audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

            # Check for voice activity
            is_speech = self.vad.is_speech(torch.from_numpy(audio_data))

            if is_speech:
                # Add to voice buffer
                self.voice_buffer = np.concatenate([self.voice_buffer, audio_data])
                self.silence_counter = 0
                self.is_listening = True
            elif self.is_listening:
                # We were listening but now there's silence
                self.silence_counter += len(audio_data)

                # If enough silence, process the collected voice
                if self.silence_counter >= self.max_silence_duration or len(self.voice_buffer) >= self.rate * 5:
                    if len(self.voice_buffer) >= self.min_voice_duration:
                        # Process collected voice segment
                        transcript = self.transcribe_audio(self.voice_buffer)
                        if transcript.strip():
                            self.publish_transcript(transcript)

                    # Reset for next voice segment
                    self.voice_buffer = np.array([], dtype=np.float32)
                    self.is_listening = False
                    self.silence_counter = 0
```

## 7.5 Real-time Processing Optimization

### Batch Processing
For improved efficiency, process multiple audio chunks together:

```python
class OptimizedWhisperNode(WhisperNode):
    def __init__(self):
        super().__init__()

        # Processing parameters
        self.processing_interval = 2.0  # Process every 2 seconds
        self.buffer_size = int(self.rate * self.processing_interval)
        self.audio_buffer = np.array([], dtype=np.float32)

        # Timer for periodic processing
        self.process_timer = self.create_timer(
            self.processing_interval,
            self.process_buffered_audio
        )

    def process_buffered_audio(self):
        """Process accumulated audio buffer"""
        if len(self.audio_buffer) > 0:
            # Process the accumulated buffer
            transcript = self.transcribe_audio(self.audio_buffer)

            if transcript.strip():
                self.publish_transcript(transcript)

            # Clear buffer
            self.audio_buffer = np.array([], dtype=np.float32)

    def process_audio(self):
        """Accumulate audio for batch processing"""
        while rclpy.ok():
            data = self.stream.read(self.chunk, exception_on_overflow=False)
            audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

            # Add to buffer
            self.audio_buffer = np.concatenate([self.audio_buffer, audio_data])

            # Keep buffer size reasonable
            if len(self.audio_buffer) > self.buffer_size * 2:
                self.audio_buffer = self.audio_buffer[-self.buffer_size:]
```

## 7.6 Handling Acoustic Conditions

### Noise Reduction
```python
from scipy import signal
import webrtcvad

class RobustWhisperNode(WhisperNode):
    def __init__(self):
        super().__init__()

        # Initialize noise reduction
        self.setup_noise_reduction()

    def setup_noise_reduction(self):
        """Set up noise reduction filters"""
        # Create a simple low-pass filter to remove high-frequency noise
        nyquist = self.rate / 2
        cutoff = 8000  # Hz
        order = 6
        b, a = signal.butter(order, cutoff / nyquist, btype='low')
        self.filter_b = b
        self.filter_a = a

    def preprocess_audio(self, audio_data):
        """Apply noise reduction and preprocessing"""
        # Apply low-pass filter
        filtered_audio = signal.filtfilt(self.filter_b, self.filter_a, audio_data)

        # Normalize audio
        max_val = np.max(np.abs(filtered_audio))
        if max_val > 0:
            filtered_audio = filtered_audio / max_val

        return filtered_audio

    def process_audio(self):
        """Process audio with preprocessing"""
        buffer = np.array([], dtype=np.float32)

        while rclpy.ok():
            data = self.stream.read(self.chunk, exception_on_overflow=False)
            audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

            # Apply preprocessing
            processed_audio = self.preprocess_audio(audio_data)

            # Add to buffer
            buffer = np.concatenate([buffer, processed_audio])

            # Process when buffer reaches 5 seconds
            if len(buffer) >= self.rate * 5:
                transcript = self.transcribe_audio(buffer)

                if transcript.strip():
                    self.publish_transcript(transcript)

                # Keep last 1 second
                buffer = buffer[-self.rate:]
```

## 7.7 Voice Command Parsing

### Simple Command Recognition
```python
import re
from dataclasses import dataclass
from typing import Optional

@dataclass
class VoiceCommand:
    action: str
    parameters: dict
    confidence: float

class VoiceCommandParser:
    def __init__(self):
        # Define command patterns
        self.command_patterns = [
            {
                "pattern": r"go\s+(forward|backward|left|right|up|down)",
                "action": "move",
                "extractor": lambda match: {"direction": match.group(1)}
            },
            {
                "pattern": r"move\s+(forward|backward|left|right|up|down)",
                "action": "move",
                "extractor": lambda match: {"direction": match.group(1)}
            },
            {
                "pattern": r"turn\s+(left|right)",
                "action": "turn",
                "extractor": lambda match: {"direction": match.group(1)}
            },
            {
                "pattern": r"go\s+to\s+(.+)",
                "action": "navigate",
                "extractor": lambda match: {"location": match.group(1).strip()}
            },
            {
                "pattern": r"pick\s+up\s+(.+)",
                "action": "pick",
                "extractor": lambda match: {"object": match.group(1).strip()}
            },
            {
                "pattern": r"put\s+down|place",
                "action": "place",
                "extractor": lambda match: {}
            },
            {
                "pattern": r"stop|halt|freeze",
                "action": "stop",
                "extractor": lambda match: {}
            }
        ]

    def parse_command(self, transcript: str) -> Optional[VoiceCommand]:
        """Parse voice transcript into structured command"""
        transcript_lower = transcript.lower().strip()

        for pattern_info in self.command_patterns:
            match = re.search(pattern_info["pattern"], transcript_lower)
            if match:
                try:
                    parameters = pattern_info["extractor"](match)
                    return VoiceCommand(
                        action=pattern_info["action"],
                        parameters=parameters,
                        confidence=0.8  # Simple confidence for now
                    )
                except Exception:
                    continue

        # If no specific pattern matched, return a general command
        return VoiceCommand(
            action="unknown",
            parameters={"text": transcript},
            confidence=0.5
        )
```

## 7.8 Practical Exercise: Voice Processing System

### Exercise Objective
Create a complete voice processing system that integrates Whisper with ROS 2 for robotic command recognition.

### Steps
1. Set up Whisper model with appropriate size for your hardware
2. Implement real-time audio capture and processing
3. Add voice activity detection to optimize processing
4. Implement command parsing for robotic actions
5. Test the system with various voice commands
6. Validate accuracy and response time

### Expected Results
- Real-time voice processing with &lt;2 second latency
- Accurate transcription (>80% accuracy in quiet conditions)
- Proper command recognition and parsing
- Integration with ROS 2 messaging system

## Summary
Voice processing with Whisper provides a robust foundation for natural human-robot interaction. By properly integrating Whisper with ROS 2 and implementing voice activity detection, we can create responsive voice interfaces for humanoid robots. The key is balancing accuracy, latency, and computational efficiency for real-world robotic applications.

## Key Terms
- **Whisper**: OpenAI's automatic speech recognition system
- **Voice Activity Detection (VAD)**: System to detect when speech is present
- **Real-time Processing**: Processing audio as it's captured
- **Acoustic Conditions**: Environmental factors affecting audio quality
- **Command Parsing**: Converting natural language to structured commands

## References
- [OpenAI Whisper GitHub](https://github.com/openai/whisper)
- [Silero VAD](https://github.com/snakers4/silero-vad)
- [PyAudio Documentation](https://pyaudio.readthedocs.io/)
- [Speech Recognition in Robotics](https://arxiv.org/abs/2104.02744)