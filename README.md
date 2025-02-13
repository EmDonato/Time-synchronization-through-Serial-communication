# Timestamp Synchronization via Serial Communication

This program synchronizes timestamps between a computer and an ESP32 microcontroller using a serial connection.

## Features
- Reads the current timestamp (seconds and nanoseconds) from the computer.
- Sends the timestamp to the ESP32 via a serial connection.
- Waits for a start signal from the ESP32 before initiating synchronization.
- Continuously sends timestamps every 10 seconds.

## Requirements
- A Linux system (tested on Ubuntu) with a functional serial port.
- An ESP32 microcontroller configured to receive timestamps.
- A serial connection (e.g., USB to UART adapter).

## Compilation and Execution
### Compilation
Use `g++` to compile the program:
```sh
 g++ -o timestamp_sync timestamp_sync.cpp
```

### Execution
Run the program with:
```sh
 ./timestamp_sync
```
Ensure the correct serial port (`/dev/ttyUSB0`) is accessible and replace it if necessary.

## Code Overview
### 1. Obtaining the Current Time
The `get_current_time()` function retrieves the system's current time in seconds and nanoseconds.
```cpp
void get_current_time(uint32_t &sec, uint32_t &nanosec);
```

### 2. Configuring the Serial Port
The `configure_serial_port()` function initializes the serial port with 115200 baud rate and standard settings.
```cpp
int configure_serial_port(const char *port);
```

### 3. Sending Timestamps to ESP32
The `send_timestamp_to_esp32()` function sends the current timestamp over the serial port.
```cpp
void send_timestamp_to_esp32(int serial_fd);
```

### 4. Receiving Start Signal from ESP32
The `receive_start_signal_from_esp32()` function waits for a start signal (`1` byte) from the ESP32 before beginning synchronization.
```cpp
void receive_start_signal_from_esp32(int serial_fd);
```

### 5. Main Loop
The program:
1. Opens the serial connection.
2. Waits for a start signal from the ESP32.
3. Sends timestamps every 10 seconds.
4. Runs indefinitely until manually stopped.

## Debugging and Logs
The program includes debug messages for tracking execution:
- `[INFO]` indicates normal operation.
- `[DEBUG]` provides additional details.
- `[ERROR]` highlights issues.

## Troubleshooting
- **Permission issues on serial port:** Run the program with `sudo` or add your user to the `dialout` group:
  ```sh
  sudo usermod -a -G dialout $USER
  ```
  Then restart your system.

- **Incorrect serial port:** Check available ports using:
  ```sh
  ls /dev/ttyUSB*
  ```
  Update the code accordingly.

- **ESP32 not receiving timestamps:** Verify baud rate and correct wiring.

## License
This project is open-source and can be modified as needed.

