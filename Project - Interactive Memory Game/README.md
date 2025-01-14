The project implemented an **interactive memory game** that utilizes an accelerometer to detect physical movements and a speaker to provide auditory feedback. The game generates sequences of tones corresponding to up or down movements, which the player must replicate by moving the device. High scores are stored in flash memory, and game instructions and results are communicated via UART and audio recordings. Audio compression is employed to optimize flash memory usage for voice instructions.
<img width="621" alt="Fig 1" src="https://github.com/user-attachments/assets/70f750fb-dd2d-4490-b742-88542186c545" />

---

#### **Key Functionalities**

1. **Accelerometer-Based Movement Detection:**
   - Reads and calibrates acceleration values to detect up and down movements.
   - Uses defined thresholds to ensure accurate detection and avoid false positives.
   - **Below** shows accelerometer readings for a sequence of up-down-down-up movements, where all movements were correctly detected using activation and deactivation thresholds.

<img width="499" alt="Screenshot 2025-01-14 at 11 39 57 AM" src="https://github.com/user-attachments/assets/b9469a91-ad05-41a0-bb19-44d4604e23d6" />


2. **Audio Feedback with Compression:**
   - Uses a DAC and speaker to play tones and voice instructions.
   - Implements a custom compression algorithm to reduce the storage size of voice recordings.
   - **Below** shows the output values after compression and decompression relative to input values, showing accurate recovery of compressed audio data.
   
<img width="506" alt="Fig 4" src="https://github.com/user-attachments/assets/3a27a454-7bb4-477e-967f-360d4a6d2417" />

   - **Below** shows the absolute error introduced by the compression algorithm, which remains minimal and does not degrade audio quality perceptibly.
   
<img width="474" alt="Fig 5" src="https://github.com/user-attachments/assets/d7504eef-a587-476a-970c-3bcc9f730a0d" />

3. **Game Logic and User Interaction:**
   - Increases difficulty (longer tone sequences) upon successful user inputs and resets the game on errors.
   - Updates and stores high scores in flash memory, allowing persistence across game sessions.
   - Provides feedback and instructions via UART and pre-recorded audio messages.

4. **Velocity Estimation Challenges:**
   - Estimates velocity along the z-axis by integrating accelerometer readings.
   - **Below** shows the velocity drift at rest and after movements, attributed to calibration inaccuracies and gain differences between accelerometer axes.
   
<img width="519" alt="Fig 6" src="https://github.com/user-attachments/assets/e69ee1d7-8df7-4f2f-b6a2-ea389e7dd601" />

---

#### **Evaluation and Results**

- The game operates as intended, successfully detecting movements and providing clear feedback through audio and visual outputs.
- Audio compression reduced flash memory usage by nearly 50%, from 850 KB to 450 KB, while maintaining good sound quality.
- Challenges in velocity estimation due to calibration errors and axis gain inconsistencies were identified, suggesting the need for more robust calibration methods.

---
