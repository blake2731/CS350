**Thermostat Project**
Project Overview
This project involves developing a simple thermostat system using the TI CC3220SF LaunchPad, along with peripherals such as GPIO, I2C, UART, and timers. The thermostat's functionality includes reading room temperature from a sensor, adjusting the temperature set-point using buttons, controlling a heater, and simulating data transmission to a server via UART.

The primary problem this project addresses is managing and controlling room temperature by comparing the current room temperature with a user-defined set-point. Based on this comparison, the thermostat can turn a heater on or off. This project emulates the behavior of a real-world thermostat, while showcasing embedded system programming techniques.

**Achievements**
I did particularly well in handling the integration of multiple peripherals, including I2C communication for reading the temperature sensor, GPIO for handling the buttons and LEDs, and UART for transmitting data to simulate server communication. I also successfully implemented a task scheduler, which drives the main loop and controls the timing of operations like temperature readings, button presses, and UART output.

Additionally, managing interrupt-based inputs for button presses and using the timer to control system actions at defined intervals were challenging but ultimately successful elements of the project.

**Areas for Improvement**
One area for improvement would be optimizing the efficiency of the code, especially in managing the task scheduler and interrupts. Currently, the timing of operations like temperature reading and updating the heater status could be further refined to achieve smoother transitions and better response times. There is also room to enhance the error-handling mechanisms for I2C and UART communication, especially in case of sensor failures or communication interruptions.

**Tools and Resources**
Throughout this project, I leveraged tools such as TI Code Composer Studio (CCS) for code development and debugging, Uniflash for programming the device, and draw.io for documenting the task scheduler. I am adding the official TI SimpleLink SDK as a key resource in my support network, along with online communities like TI E2E Community and Stack Overflow. These resources will be invaluable for troubleshooting and improving embedded systems knowledge.

T**ransferable Skills**
This project developed several skills that will be highly transferable to other coursework and future projects, including:
Embedded Systems Programming: Working with peripherals like GPIO, I2C, UART, and timers is foundational to embedded systems work. These skills will apply to future projects involving microcontrollers and sensor integration.
Task Scheduling: The task scheduler implementation enhanced my ability to manage timed events and interrupts in embedded systems, which is useful for real-time systems development.
Peripheral Integration: Managing multiple peripherals and understanding how they interact with each other is a critical skill in embedded software development.
Problem-Solving and Debugging: Tackling issues related to communication protocols and timing will aid in solving complex challenges in future projects.

**Code Maintainability and Adaptability**
I made this project maintainable by keeping the code modular and well-commented. Each peripheral was initialized and controlled through separate functions, making it easy to modify or update specific parts of the code without affecting others. The project also follows clear naming conventions for variables and functions, which improves readability and makes the codebase easier for others to follow or extend.

I ensured adaptability by organizing the code in a way that allows easy changes to system configurations, such as adjusting the timer intervals or adding more functionality. By documenting key decisions and commenting on complex sections of the code, this project is now easier to maintain and adapt to future needs.
