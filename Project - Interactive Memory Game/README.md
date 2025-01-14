The project implemented an **interactive memory game** that utilizes an accelerometer to detect physical movements and a speaker to provide auditory feedback. The game generates sequences of tones corresponding to up or down movements, which the player must replicate by moving the device. High scores are stored in flash memory, and game instructions and results are communicated via UART and audio recordings. Audio compression is employed to optimize flash memory usage for voice instructions.

---

#### **Key Functionalities**

1. **Accelerometer-Based Movement Detection:**
   - Reads and calibrates acceleration values to detect up and down movements.
   - Uses defined thresholds to ensure accurate detection and avoid false positives.
   - **Below** shows accelerometer readings for a sequence of up-down-down-up movements, where all movements were correctly detected using activation and deactivation thresholds.
   

2. **Audio Feedback with Compression:**
   - Uses a DAC and speaker to play tones and voice instructions.
   - Implements a custom compression algorithm to reduce the storage size of voice recordings.
   - **Below** shows the output values after compression and decompression relative to input values, showing accurate recovery of compressed audio data.
   

   - **Beloow** shows the absolute error introduced by the compression algorithm, which remains minimal and does not degrade audio quality perceptibly.
   

3. **Game Logic and User Interaction:**
   - Increases difficulty (longer tone sequences) upon successful user inputs and resets the game on errors.
   - Updates and stores high scores in flash memory, allowing persistence across game sessions.
   - Provides feedback and instructions via UART and pre-recorded audio messages.

4. **Velocity Estimation Challenges:**
   - Estimates velocity along the z-axis by integrating accelerometer readings.
   - **Below** shows the velocity drift at rest and after movements, attributed to calibration inaccuracies and gain differences between accelerometer axes.
   

---

#### **Evaluation and Results**

- The game operates as intended, successfully detecting movements and providing clear feedback through audio and visual outputs.
- Audio compression reduced flash memory usage by nearly 50%, from 850 KB to 450 KB, while maintaining good sound quality.
- Challenges in velocity estimation due to calibration errors and axis gain inconsistencies were identified, suggesting the need for more robust calibration methods.

---
