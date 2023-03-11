# N3-Camera

This code is designed to stream video from an AI-Thinker ESP32-CAM module to a WebSocket server using the Arduino framework.

## Dependencies

1. Arduino.h
2. esp_camera.h
3. WiFi.h
4. ArduinoWebsockets.h

## Code Structure

### setup()

The setup() function initializes the serial communication with the ESP32-CAM module and the camera's configuration. It sets up the camera configuration to use JPEG format and configure the pins that the camera module uses.

### startStream()

The startStream() function initializes the running average filter and calls the handleStream() function to start the video streaming process.

### handleStream()

The handleStream() function captures frames from the camera and sends them to the WebSocket server in binary format. If the frame format is not JPEG, it converts the frame to JPEG format before sending. It also calculates the frame rate and sends it to the Serial Monitor.

### ra_filter_init()

This function initializes the running average filter by creating an array with the specified number of samples and setting the size parameter of the filter.

### ra_filter_run()

This function runs the running average filter on the specified value and returns the filtered value.

## Variables

**ssid**: The name of the Wi-Fi network that the ESP32-CAM module should connect to.
**password**: The password for the Wi-Fi network.
**websockets_server_host**: The IP address of the WebSocket server that the video stream should be sent to.
**websockets_server_port**: The port number of the WebSocket server.
**ra_filter**: A struct that holds the parameters for the running average filter.
**_jpg_buf_len**: The length of the JPEG buffer.
**_jpg_buf**: A pointer to the JPEG buffer.

## Libraries

### esp_camera.h

The esp_camera.h library provides the functions necessary for initializing the camera and capturing frames.

### WiFi.h

The WiFi.h library provides the functions necessary for connecting to a Wi-Fi network.

### ArduinoWebsockets.h

The ArduinoWebsockets.h library provides the functions necessary for creating a WebSocket client and sending data over the WebSocket connection.
