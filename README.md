# Self-sustaining Environmental Patrol Robot – ECE 528 Final Project

This project combines energy harvesting and autonomous robotics.  
A small DC motor is used as a generator to create 5 V power, which runs an MSP432-based robot that patrols its environment and monitors humidity.

---

## Author

**Juan Zendejas**  
Instructor: Professor Aaron Nanas  
Course: ECE 528 – Embedded System Design  
Semester: Fall 2025  

---

## Introduction

This project implements a **self-sustaining environmental patrol robot**.  
The system has two main parts:

1. **Energy Harvesting Front-End**  
   - A DC motor is driven externally (e.g., by a drill) and used as a generator.  
   - Each motor lead is fed into its own full-bridge rectifier built from Schottky diodes.  
   - The rectified outputs are filtered using capacitors and then regulated by an LM2596 buck converter, adjusted to **5 V**.  
   - The 5 V output powers the MSP432 LaunchPad through a USB-A connector, demonstrating how harvested energy can run the controller.

2. **Environmental Patrol Robot**  
   - An MSP432P401R LaunchPad on a TI-RSLK chassis uses three Sharp GP2Y0A21YK0F distance sensors to navigate a hallway.  
   - The robot follows walls using a proportional controller and slows or redirects when objects are too close.  
   - A DHT22 sensor monitors ambient temperature and humidity.  
   - A PWM-driven LED and UART terminal messages form a simple **irrigation state machine** that classifies the environment as STARTUP, OK, DRY, or VERY DRY.

Together, these subsystems show how embedded systems can tie together **energy harvesting, sensing, and autonomous motion** in a compact platform.

---

## Results and Video Demonstration

The final prototype successfully:

- Harvests energy from the DC motor and regulates it to 5 V using the LM2596 buck converter.
- Powers the MSP432 LaunchPad entirely from the harvested 5 V supply.
- Uses three IR distance sensors for wall following and object detection.
- Reads temperature and humidity from the DHT22 and maps humidity to an irrigation state machine:
  - **OK** – acceptable moisture  
  - **DRY** – early warning  
  - **VERY DRY** – irrigation strongly recommended
- Reports sensor values and state over the UART terminal.
- Uses a PWM LED to visually indicate the current irrigation state.

**Demo Videos :**

- Energy harvesting and power chain:  
  `https://youtube.com/shorts/bpxP3ixZ84w?feature=share`
- Patrol behavior and wall following:  
  `https://youtube.com/shorts/vx2ME-NlFOU?feature=share`
- DHT22 readings + state machine output over UART:  
  `https://youtu.be/DQGt3XiO_hc`
