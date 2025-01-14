### **Learning Objectives**
- **GPIO Input and Output:** Understand and implement basic GPIO functionalities for buttons and LEDs to control and respond to digital signals.  
- **DAC and ADC Operations:** Learn to configure and use the digital-to-analog converter (DAC) for waveform generation and the analog-to-digital converter (ADC) for sensor data acquisition.  
- **Signal Processing and Debugging:** Gain experience in generating periodic signals, scaling ADC outputs, and debugging variable traces using the ITM and SWV tools.

---

### **Microprocessor Concepts Covered**
- **GPIO Configuration:** Configure GPIO pins for digital and analog modes, using HAL functions to manage button and LED states.  
- **Waveform Generation:** Generate triangle, saw, and sine waves using DAC with CMSIS-DSP functions for real-time audio output.  
- **Analog-to-Digital Conversion:** Configure ADCs to measure internal voltages and temperature, scale the output to meaningful physical values, and handle calibration adjustments.  
- **Application Integration:** Combine multiple peripherals into a cohesive application that adapts functionality dynamically based on inputs.  

---

### **Summary of the Problem and Deliverables**
#### **Problem:** Build a system that integrates GPIO, DAC, and ADC functionalities to generate and manipulate signals based on user input and sensor data.  

---

### **Steps and Deliverables**
1. **GPIO Button and LED Control:**  
   - Write code to light up an LED when a button is pressed using GPIO polling.  

2. **Waveform Generation with DAC:**  
   - Generate triangle, saw, and sine wave signals using the DAC.  
   - Assign each signal to a different DAC channel and ensure audible output using a speaker.  

3. **Temperature Measurement with ADC:**  
   - Configure the ADC to read internal temperature and voltage reference sensors.  
   - Scale ADC outputs to calculate the CPU temperature in degrees Celsius.  

4. **Integrated Application:**  
   - Create an application that:  
     - By default, plays a fixed waveform (triangle, saw, or sine).  
     - On button press, changes the waveform to one based on CPU temperature and turns on the LED.  
     - On a subsequent button press, switches back to a fixed waveform and turns off the LED.  

5. **Debugging and Visualization:**  
   - Use the SWV timeline graph to monitor signal outputs and validate waveforms with an oscilloscope.  

---