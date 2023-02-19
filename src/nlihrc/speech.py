"""Speech Recognition Module"""
import vosk
from pathlib import Path
import numpy as np
import onnxruntime
import json
import time
from word2number import w2n
from nlihrc.misc import CLIPORT_CMDS


class OnnxWrapper():
    """OnnxWrapper for Voice Activity Detector"""
    def __init__(self, path):
        self.session = onnxruntime.InferenceSession(path)
        self.session.intra_op_num_threads = 1
        self.session.inter_op_num_threads = 1

        self.reset_states()

    def reset_states(self):
        self._h = np.zeros((2, 1, 64)).astype('float32')
        self._c = np.zeros((2, 1, 64)).astype('float32')

    def __call__(self, x, sr: int):
        if x.ndim == 1:
            x = x[np.newaxis, ...]
        if x.ndim > 2:
            raise ValueError(f"Too many dimensions for input audio chunk {x.dim}")

        if sr != 16000 and (sr % 16000 == 0):
            step = sr // 16000
            x = x[::step]
            sr = 16000

        if x.shape[0] > 1:
            raise ValueError("Onnx model does not support batching")

        if sr not in [16000]:
            raise ValueError(f"Supported sample rates: {[16000]}")

        if sr / x.shape[1] > 31.25:
            raise ValueError("Input audio chunk is too short")

        ort_inputs = {'input': x, 'h0': self._h, 'c0': self._c}
        ort_outs = self.session.run(None, ort_inputs)
        out, self._h, self._c = ort_outs

        # out = torch.tensor(out).squeeze(2)[:, 1]  # make output type match JIT analog

        return out.reshape((1,-1))[:, 1][0]


# Provided by Alexander Veysov
def int2float(sound):
    abs_max = np.abs(sound).max()
    sound = sound.astype('float32')
    if abs_max > 0:
        sound *= 1/abs_max
    sound = sound.squeeze()  # depends on the use case
    return sound

class SpeechRecognizer:
    """Handles vosk and vad speech to text"""
    def __init__(self, model_path, sample_rate):
        """Class Constructor"""
        self.vad = OnnxWrapper(str(Path(model_path, 'silero_vad.onnx')))
        self.audio_chunks = []
        self.speech_start_idx = 0
        self.start_speech = False
        self.time_since_last_speech = 0.0
        self.speech_end_interval = 0.5
        self.chunk_offset = 3
        self.rate = sample_rate

        self.numbers = ["one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "zero",
        "eleven", "twelve", "thirteen", "fourteen", "fifteen", "sixteen", "seventeen", "eighteen", "nineteen", "twenty",
        "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety", "hundred", "thousand",]
        
        model = vosk.Model(model_path)
        self.rec = vosk.KaldiRecognizer(model, self.rate, json.dumps([
        # System commands
        "start", "stop", "robot", "execution", "move", "go", "set mode", "continuous", "model", "step", "size", "tool", "open", "close", "rotate",
        "save", "home", "position", "load", "the"
        # Directions
        "up", "upward", "down", "left", "right", "forward", "backward", "front", "back",
        # numbers
        *self.numbers,
        "minus", "negative",
        # cliport
        *CLIPORT_CMDS,
        # unknown
        "[unk]"]))
        
    
    def speech_to_text(self, data):
        """Convert speech to text using speech model recognizer"""
        words = []
        number = None
        # Detect Speech
        self.audio_chunks.append(data)
        audio_int16 = np.frombuffer(data, np.int16)
        audio_float32 = int2float(audio_int16)
        output = self.vad(audio_float32, self.rate)
        if output > 0.5:
            #print("Speec Detected")
            if not self.start_speech:
                self.speech_start_idx = len(self.audio_chunks) - 1
            self.start_speech = True
            self.time_since_last_speech = time.time()
        else:
            if self.start_speech and time.time() - self.time_since_last_speech > self.speech_end_interval:
                #print("Speech Ended")
                start_idx = self.speech_start_idx - self.chunk_offset
                #print(f"Speech Index Interval: [{start_idx}, {end_idx}]")
                self.start_speech = False
                self.speech_start_idx = 0
                speech = b''.join(self.audio_chunks[start_idx:])
                self.audio_chunks = []
                # Convert speech to text
                self.rec.AcceptWaveform(speech)
                words = json.loads(self.rec.FinalResult())["text"].split(' ')
                # Find number in word sequence (ONLY works for single numeric sequence)
                num_str = ""
                is_positive = True
                for word in words:
                    if word in self.numbers:
                        num_str = num_str + f" {word}"
                    if word in ['minus', 'negative']:
                        is_positive = False
                if num_str != "":
                    try:
                        number = (2*is_positive - 1)*w2n.word_to_num(num_str)
                    except Exception:
                        number = None
                
        return words, number
