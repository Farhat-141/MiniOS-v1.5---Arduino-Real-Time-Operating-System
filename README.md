# MiniOS v1.5 - Arduino Real-Time Operating System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Arduino](https://img.shields.io/badge/Arduino-UNO-blue.svg)](https://www.arduino.cc/)
[![Version](https://img.shields.io/badge/Version-1.5-green.svg)](https://github.com/Farhat-141/OS-)

## 🚀 Overview

MiniOS v1.5 is a sophisticated real-time operating system simulation designed for Arduino Uno. It demonstrates advanced embedded systems concepts including multi-tasking, interrupt handling, system monitoring, and human-machine interface design. This project showcases how to implement OS-like functionality on a microcontroller with limited resources.

## ✨ Key Features

### 🔧 Hardware Components
- **3 Status LEDs**: Visual indicators for different system tasks
  - **LED1 (Red)**: Task A indicator - blinks every 1 second
  - **LED2 (Green)**: Task B indicator - blinks every 2 seconds  
  - **LED3 (Yellow)**: Emergency alert indicator
  - **Built-in LED**: CPU load indicator with breathing effect
- **2 Push Buttons**: User interaction controls
  - **Emergency Button**: Triggers priority interrupt handling
  - **Menu Button**: Navigates through information displays
- **2 I2C LCD Displays**: Real-time system information
  - **Main LCD (0x27)**: System status and current command
  - **Menu LCD (0x26)**: Multi-page detailed system information

### 💻 Software Architecture

#### Multi-Threading Simulation
```cpp
// Task scheduling with different priorities
Task A: LED1 control (1000ms interval)
Task B: LED2 control (2000ms interval)  
Task C: System monitoring (100ms interval)
Emergency: Highest priority interrupt-driven task
```

#### Real-Time Features
- **Cooperative Multi-tasking**: Non-blocking task execution
- **Interrupt Handling**: Hardware interrupt for emergency responses
- **System Monitoring**: CPU usage calculation and performance metrics
- **Memory Management**: Efficient variable and state management
- **Watchdog Simulation**: System health monitoring

## 🔌 Hardware Setup

### Pin Configuration
```cpp
#define LED1_PIN 2        // Red LED - Task A
#define LED2_PIN 3        // Green LED - Task B  
#define LED3_PIN 6        // Yellow LED - Emergency
#define EMERGENCY_BTN 7   // Emergency button (INPUT_PULLUP)
#define MENU_BTN 8        // Menu button (INPUT_PULLUP)
#define CPU_LED 13        // Built-in LED - CPU load indicator
```

### I2C Connections
```cpp
A4 (SDA) -> Both LCD SDA pins
A5 (SCL) -> Both LCD SCL pins
Main LCD: 0x27
Menu LCD: 0x26
```

### Circuit Diagram Components
- Arduino Uno R3
- 3x LEDs (Red, Green, Yellow)
- 3x 220Ω resistors for LEDs
- 2x Push buttons
- 2x 16x2 I2C LCD displays
- Breadboard and jumper wires

## 📦 Required Libraries

```cpp
#include <Wire.h>                    // I2C communication
#include <LiquidCrystal_I2C.h>      // LCD control
```

Install via Arduino IDE Library Manager:
1. Go to **Sketch → Include Library → Manage Libraries**
2. Search for "LiquidCrystal I2C" by Frank de Brabander
3. Click **Install**

## 🚀 Installation & Setup

1. **Clone the Repository**
   ```bash
   git clone https://github.com/Farhat-141/OS-
   cd OS-/MiniOS
   ```

2. **Hardware Assembly**
   - Connect components according to pin configuration
   - Ensure I2C LCD addresses are correct (use I2C scanner if needed)
   - Test all connections before powering on

3. **Upload Code**
   - Open `MiniOS_v1_5.ino` in Arduino IDE
   - Select **Board: Arduino Uno**
   - Select correct **Port**
   - Click **Upload**

4. **Verification**
   - Open Serial Monitor (9600 baud)
   - You should see: "MiniOS v1.5 Starting..."
   - LEDs should begin blinking patterns
   - LCDs should display system information

## 🎮 Usage & Operation

### Serial Commands
Connect via Serial Monitor (9600 baud) and use these commands:

| Command | Function | Example |
|---------|----------|---------|
| `status` | Display complete system status | `status` |
| `ledon` | Turn LED1 on manually | `ledon` |
| `ledoff` | Turn LED1 off manually | `ledoff` |
| `pause` | Toggle system pause/resume | `pause` |
| `auto` | Toggle auto/manual mode | `auto` |
| `bright N` | Set LED brightness (0-255) | `bright 128` |
| `reset` | Reset all counters | `reset` |
| `help` | Show command list | `help` |

### Hardware Controls

#### Emergency Button (Red)
- **Function**: Triggers emergency response
- **Action**: Stops all tasks, blinks yellow LED 5 times
- **Priority**: Highest (interrupt-driven)
- **Debouncing**: 200ms hardware debounce implemented

#### Menu Button (Blue)  
- **Function**: Cycles through LCD menu pages
- **Pages**: 5 different information screens
- **Debouncing**: 300ms software debounce

### LCD Display Information

#### Main LCD (0x27)
```
Line 1: LED1:ON  LED2:OFF
Line 2: Cmd:status       !
```

#### Menu LCD (0x26) - 5 Pages
1. **Page 1**: System info and uptime
2. **Page 2**: Task intervals and operation mode
3. **Page 3**: Task execution counters
4. **Page 4**: Performance metrics (CPU usage, brightness)
5. **Page 5**: Current status summary

## 🔧 System Architecture

### Task Scheduling
```cpp
void loop() {
    unsigned long currentTime = millis();
    
    // Emergency handling (highest priority)
    if (emergencyTriggered) {
        emergencyTask();
    }
    
    // Task A (1Hz frequency)
    if (currentTime - lastTaskA >= TASK_A_INTERVAL) {
        taskA();
        lastTaskA = currentTime;
    }
    
    // Task B (0.5Hz frequency)  
    if (currentTime - lastTaskB >= TASK_B_INTERVAL) {
        taskB();
        lastTaskB = currentTime;
    }
    
    // Task C (10Hz frequency - system monitoring)
    if (currentTime - lastTaskC >= TASK_C_INTERVAL) {
        taskC();
        lastTaskC = currentTime;
    }
}
```

### Memory Usage
- **Program Storage**: ~85% of Arduino Uno capacity
- **Dynamic Memory**: ~45% of available RAM
- **EEPROM**: Not used (available for future features)

### Performance Characteristics
- **Task A Frequency**: 1 Hz (1000ms interval)
- **Task B Frequency**: 0.5 Hz (2000ms interval)
- **System Monitor**: 10 Hz (100ms interval)
- **LCD Update Rate**: 4 Hz (250ms interval)
- **Maximum CPU Usage**: ~15% under normal load

## 🔍 Advanced Features

### CPU Usage Monitoring
```cpp
// Real-time CPU usage calculation
float cpuUsage = (activeTime / totalTime) * 100;
```

### Interrupt Service Routine (ISR)
```cpp
void emergencyISR() {
    static unsigned long lastInterrupt = 0;
    unsigned long currentTime = millis();
    
    // Debouncing in ISR
    if (currentTime - lastInterrupt > 200) {
        emergencyTriggered = true;
        emergencyTriggerTime = currentTime;
        lastInterrupt = currentTime;
    }
}
```

### Breathing LED Effect
```cpp
// CPU idle indicator with sinusoidal breathing
int breath = (sin((currentTime / 1000.0) * 2 * PI) + 1) * 127;
analogWrite(CPU_LED, breath);
```

## 🐛 Troubleshooting

### Common Issues

1. **LCD Not Displaying**
   - Check I2C connections (SDA, SCL)
   - Verify I2C addresses using I2C scanner
   - Ensure 5V power supply to LCDs

2. **LEDs Not Blinking**
   - Verify pin connections
   - Check resistor values (220Ω recommended)
   - Test with simple blink sketch

3. **Buttons Not Responding**
   - Confirm INPUT_PULLUP configuration
   - Check for proper grounding
   - Verify debounce timing

4. **Serial Communication Issues**
   - Set baud rate to 9600
   - Check USB cable connection
   - Verify correct COM port selection

### Memory Optimization
If running low on memory:
```cpp
// Use F() macro for string literals
Serial.println(F("String stored in flash memory"));

// Reduce LCD update frequency
const unsigned long LCD_UPDATE_INTERVAL = 500; // Instead of 250ms
```

## 🔮 Future Enhancements

### Planned Features
- [ ] **EEPROM Settings Storage**: Persistent configuration
- [ ] **Wireless Communication**: ESP32 integration
- [ ] **Sensor Integration**: Temperature, humidity monitoring
- [ ] **Data Logging**: SD card storage capability
- [ ] **Web Interface**: HTTP server for remote monitoring
- [ ] **Real-Time Clock**: RTC module integration
- [ ] **Watchdog Timer**: Hardware watchdog implementation

### Hardware Expansions
- [ ] **Additional Sensors**: PIR, ultrasonic, light sensors
- [ ] **Motor Control**: Stepper/servo motor integration
- [ ] **Audio Feedback**: Buzzer/speaker alerts
- [ ] **Power Management**: Battery monitoring and sleep modes

## 📄 Code Structure

```
MiniOS_v1_5/
├── MiniOS_v1_5.ino          # Main Arduino sketch
├── README.md                # This documentation
├── circuit_diagram.png      # Hardware connection diagram
├── wokwi_project.json      # Wokwi simulation file
└── examples/
    ├── basic_test.ino       # Basic functionality test
    └── lcd_scanner.ino      # I2C address scanner
```

## 🤝 Contributing

We welcome contributions! Please follow these steps:

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/AmazingFeature`)
3. **Commit** your changes (`git commit -m 'Add some AmazingFeature'`)
4. **Push** to the branch (`git push origin feature/AmazingFeature`)
5. **Open** a Pull Request

### Contribution Guidelines
- Follow Arduino coding standards
- Add comments for complex functions
- Test on actual hardware before submitting
- Update documentation for new features

## 👨‍💻 Author

**Farhat-141**
- GitHub: [@Farhat-141](https://github.com/Farhat-141)
- Project Link: [https://github.com/Farhat-141/OS-](https://github.com/Farhat-141/OS-)

## 🙏 Acknowledgments

- Arduino Community for excellent documentation
- LiquidCrystal_I2C library contributors
- Wokwi for online simulation platform
- Open source embedded systems community

## 📊 Technical Specifications

| Specification | Value |
|---------------|-------|
| **Microcontroller** | ATmega328P (Arduino Uno) |
| **Clock Speed** | 16 MHz |
| **Flash Memory** | 32 KB (2 KB bootloader) |
| **SRAM** | 2 KB |
| **EEPROM** | 1 KB |
| **Digital I/O Pins** | 14 (6 PWM capable) |
| **Analog Input Pins** | 6 |
| **Operating Voltage** | 5V |
| **Input Voltage** | 7-12V (recommended) |

---

**MiniOS v1.5** - Bringing real-time operating system concepts to Arduino! 🚀

*Last updated: June 2025*
