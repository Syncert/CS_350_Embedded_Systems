# Thermostat Project: Emerging Systems Architectures and Technologies  

## Project Overview  
This project demonstrates the development of a software-controlled thermostat system leveraging **emerging systems architectures and technologies**. The project showcases my ability to:  
- Write interface software to control hardware components such as temperature sensors, LEDs, and buttons.  
- Analyze hardware architecture considerations and their impact on application performance.  
- Recommend systems architectures and technologies to solve specific business requirements.  

The thermostat system:  
- Reads ambient temperature via an **I2C temperature sensor**.  
- Displays heating status via **GPIO-driven LEDs** (ON/OFF).  
- Adjusts the set-point temperature using **GPIO interrupts for button inputs**.  
- Transmits system state to a cloud server simulation using **UART**.  
- Executes tasks at specified intervals using a **task scheduler and timer interrupts**.  

---

## Project Artifacts  
The following artifacts reflect my talents and the project's goals:  
1. **Source Code:** A modular and maintainable thermostat system written in C for an ARM Cortex-M4 processor.  
    - Implements I2C communication, GPIO management, UART transmission, and task scheduling.  
    - Modular design enables easy debugging, maintenance, and scalability.  

2. **Task Scheduler Flow Diagram:**  
    - Visualizes the execution of tasks such as reading temperature, updating heater state, and sending system status to the cloud.  
    - Highlights the periodic task execution (200 ms, 300 ms, and 500 ms intervals).  
![ProjectOne_TaskScheduler drawio](https://github.com/user-attachments/assets/ddb64d7e-c1fd-42da-8cdf-f7ca3d00ecc0)

---

## Reflection  

### **Summarize the Project and Problem**  
The thermostat project solves the challenge of integrating multiple hardware components to simulate a fully functional embedded system. By reading ambient temperature, adjusting set-point, and updating system status, the project explores hardware control, task scheduling, and cloud connectivity. It highlights the interplay between **I2C sensors, GPIO inputs/outputs, UART communication**, and the **ARM Cortex-M4** architecture.  

### **What Did I Do Particularly Well?**  
I excelled at **digging through documentation** to find solutions. My consistency in troubleshooting problems, verifying functionality during the testing phase, and thoroughly logging results helped ensure a robust implementation.  

### **Where Could I Improve?**  
My knowledge of **embedded systems** remains a growth area. I learned how vast and specialized this field is and aim to build a stronger foundation in embedded programming and architecture design.  

### **What Tools and Resources Am I Adding to My Support Network?**  
I am developing a **curiosity-driven mindset** that focuses on asking better questions. This approach leads to deeper learning and helps identify the right resources for solving problems.  

### **What Transferable Skills Did I Develop?**  
The most transferable skill I honed is the ability to **identify knowledge gaps**. Instead of immediately seeking quick answers, I focused on understanding what I did not know. This skill is crucial in debugging, problem-solving, and working with incomplete or ambiguous information.  

### **How Did I Make This Project Maintainable, Readable, and Adaptable?**  
I emphasized **modularity** in the code structure. By breaking down functionality into separate, reusable modules (e.g., I2C driver, UART communication, and task scheduler), I ensured that the project is:  
- **Maintainable:** Bugs and updates are easy to isolate and address.  
- **Readable:** Code sections are logically organized with clear comments and documentation.  
- **Adaptable:** Future enhancements, such as integrating new sensors or changing task schedules, require minimal effort.

---

## Tools and Technologies  
- **Programming Language:** C  
- **Microcontroller:** TI CC3220 (ARM Cortex-M4 Core)  
- **Technologies:** I2C, GPIO, UART, Timer Interrupts  
- **Tools:** Code Composer Studio, Datasheets, Flow Diagrams  

---

## Future Enhancements  
- Add support for real cloud storage (e.g., MQTT communication with an IoT platform).  
- Implement error-handling mechanisms for sensor failures or communication timeouts.  
- Expand the system to include additional sensors (e.g., humidity) for a smart thermostat upgrade.  

---
