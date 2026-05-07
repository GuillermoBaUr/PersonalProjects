# Personal Projects Portfolio

Welcome to my Personal Projects repository, a hub showcasing the embedded systems, firmware, and software projects I’ve built.  
Each project links to its own repository with **source code**, **documentation**, and in some cases **demo videos**.

# Featured Projects

## 🔧 Embedded Systems / C Project

### **1. CRJ‑200 Autopilot Panel – STM32 Firmware**
**C | STM32F4 | Interrupts | UART | Encoders | LEDs | Data Packing**

<p align="center">
  <img src="Img_project/CRJ-200.jpg" width="520" />
</p

A custom firmware solution that interfaces a physical CRJ‑200 autopilot panel with the X‑Plane simulator.

Key features:
- Encoder direction detection via EXTI interrupts  
- Button debouncing and state‑change logic  
- Linked‑list event queue for reliable transmission  
- Bit‑packed 8‑bit command protocol  
- LED output parsing with delta‑update logic  

🔗 **Repository:** [Link to Repository](https://github.com/GuillermoBaUr/CRJ-200_Autopilot_Panel_STM32)

---


### **2. FreeRTOS Whack‑A‑Mole – STM32 Real‑Time Game**
**C | STM32F4 | FreeRTOS | Queues | Semaphores | Timers | OLED (SSD1331)**

<p align="center">
  <img src="Img_project/Whack-A-Mole.jpeg" width="520" />
</p

A real‑time embedded system implemented on an STM32 microcontroller using FreeRTOS.  
The project recreates a Whack‑A‑Mole style game using LEDs, input buttons, and an SPI OLED display.

Key features include:

- Multiple FreeRTOS tasks (display, game loop, timer, scoring)
- Message queues for ISR‑to‑task communication
- Semaphores to synchronize OLED updates and turn progression
- Software timer callback for randomized LED selection
- EXTI interrupts for button input with debouncing
- SPI‑driven SSD1331 OLED to display score and countdown
- Deterministic timing and non‑blocking task behavior

🔗 **Repository:**  [Link to Repository](https://github.com/GuillermoBaUr/whackamole-rtos)

---

## 🐍 Python Projects

### **1. Metroliner III Pedestal Simulation – Touch Display Framework**
**Python | Raspberry Pi | Multithreading | UDP | Tkinter GUI**

<p align="center">
  <img src="Img_project/Pedestal.png" width="520" />
</p

A fully animated touchscreen simulation of the Metroliner III pedestal for aviation training systems.

Highlights:
- Multithreaded event‑driven architecture  
- Real‑time UDP link with X‑Plane  
- Image‑based control widgets and animations  
- UI callback framework for system integration  

🔗 **Repository:** [Link to Repository](https://github.com/GuillermoBaUr/Metroliner-III-Pedestal-Display)

---

### **2. SGI Data Extractor – Academic Metrics Automation**
**Python | Tkinter | Multithreading | Regex | MS Word COM | Excel Automation**

<p align="center">
  <img src="Img_project/SGI-Extractor.png" width="520" />
</p>

An automation tool designed to extract and consolidate academic metrics from institutional SGI documents into professional Excel reports.

Highlights:
- **Multithreaded Architecture**: Background processing ensures a responsive UI during heavy I/O operations.
- **Automated Document Conversion**: Interfaces with Microsoft Word COM API to convert legacy `.doc` files to `.docx` automatically.
- **Advanced Regex Parsing**: Robust extraction of teacher data, subjects, and tracking tables, even across complex paragraph splits.
- **Recursive Processing**: Scans entire directories and deep-reads inside `.zip` archives to locate relevant documentation.
- **Formatted Data Export**: Generates styled Excel workbooks with frozen panes, alternating row colors, and auto-adjusted layouts.

🔗 **Repository:** [Link to Repository](https://github.com/GuillermoBaUr/data-extraction-project)

--- 

 👤 About Me

I’m a Mechatronics Engineer with experience across electronics, control systems, robotics, and embedded programming :

- STM32 firmware (C/C++)  
- FreeRTOS and real‑time design  
- Python simulation frameworks  
- Hardware‑software integration  
- Communication protocols (UART, SPI, I2C, UDP)

I focus on building systems that combine **clear architecture**, **robust communication**, and **deterministic real‑time behavior**, from both my **Mechatronics Engineering foundation** and my experience in **software development, testing, and embedded systems**.

---

# 📬 Contact

If you'd like to collaborate or have any questions:

📧 **Email:** [badillouribeguillermoca@gmail.com](mailto:badillouribeguillermoca@gmail.com)  
🔗 **LinkedIn:** https://www.linkedin.com/in/guillermo-badillo-uribe-382301228/

---

# 📝 License

This repository is licensed under the MIT License.  
See the `LICENSE` file for more details.
