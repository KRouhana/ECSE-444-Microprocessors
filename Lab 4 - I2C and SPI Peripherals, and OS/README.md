### **Learning Objectives**
- **I2C and SPI Peripherals:** Understand and configure I2C and SPI protocols for communication with sensors and flash memory.  
- **Embedded RTOS Programming:** Learn to use FreeRTOS for multitasking, enabling task scheduling, inter-task communication, and efficient resource management.  
- **Data Acquisition and Management:** Implement a system to acquire, transmit, and store sensor data using I2C, UART, and QSPI flash memory.  

---

### **Microprocessor Concepts Covered**
- **I2C Communication:** Configure and use I2C to interface with multiple onboard sensors, such as temperature, humidity, and motion sensors.  
- **UART Communication:** Transmit sensor data over a UART interface to a terminal for real-time monitoring.  
- **RTOS Task Scheduling:** Utilize FreeRTOS to create and manage tasks, ensuring periodic sensor data acquisition, display, and logging.  
- **QSPI Flash Memory:** Store sensor data in onboard flash memory and perform read/write operations for long-term data storage.  
- **Data Analysis:** Calculate summary statistics, including mean and variance, for logged sensor data stored in flash memory.  

---

### **Summary of the Problem and Deliverables**
#### **Problem:** Develop a multitasking embedded application using FreeRTOS to acquire, process, and store sensor data, while enabling real-time monitoring via UART.  

---

### **Steps and Deliverables**
1. **I2C Sensor Initialization and Data Acquisition:**  
   - Configure and initialize four different I2C sensors (e.g., temperature, humidity, accelerometer, and barometer).  
   - Acquire data from each sensor at a rate of 10 Hz using board support package (BSP) drivers.  

2. **UART Communication:**  
   - Transmit real-time sensor data to a terminal using the UART interface.  
   - Display data from a specific sensor, changing the sensor output based on push-button presses.  

3. **RTOS Implementation:**  
   - Use FreeRTOS to divide the application into three tasks:  
     - Task 1: Detect and respond to push-button presses to cycle through sensors.  
     - Task 2: Read sensor data periodically.  
     - Task 3: Transmit sensor data over UART.  

4. **QSPI Flash Integration:**  
   - Use QSPI flash memory to store sensor data for long-term logging.  
   - Implement functions to write, read, and erase flash memory blocks.  

5. **Data Analysis and Summary:**  
   - Compute and display summary statistics (sample count, mean, and variance) for logged data stored in flash memory.  
   - Cycle through summary statistics and real-time sensor data based on push-button presses.  

6. **Integrated Application:**  
   - Develop a final application that integrates I2C, UART, RTOS, and QSPI functionalities.  
   - Implement state management to switch between real-time data display, logged data analysis, and sensor cycling modes.  

---