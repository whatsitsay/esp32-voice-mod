# esp32-voice-mod
Voice modulator based around the ESP32-A1S Audio Kit from AI Thinker

In its current iteration, uses the phase vocoder technique to pitch-shift microphone input in real-time before driving speakers or headphones.

Capable of switching between four modes by pressing key 1:
1. *Chorus Mode*: Original sound shifted multiple times and mixed to produce an approximate minor 7th
2. *Low-pitch Mode*: Original sound shifted down to the lower 5th
3. *High-pitch Mode*: Original sound shifted up to the upper 5th
4. *Passthrough Mode*: Original sound unmodified
