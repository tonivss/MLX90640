# MLX90640 Driver

This is a driver for the MLX90640 thermal camera. It is based on the driver provided by [Melexis](https://www.melexis.com/en/product/MLX90640/Far-Infrared-Thermal-Sensor-Array) and the implementation [REDar](https://github.com/linkineo/REDar) to the ESP32 by [Patrick Wieder](https://github.com/linkineo).

## How to Run

1\. Connect the MLX90640 sensor to your ESP device using serial data (SDA) and serial clock (SCL).\
2\. Open `menuconfig` and configure I2C pins and the I2C address of the sensor. Also you can configure refresh rate, resolution and if you wish to enable interleave mode or use the default chess mode.\
2\. Compile and flash the code to your ESP device.\
3\. Once the device boots up, it will initialize the sensor, capture a thermal frame, and display the pixel temperatures on the console.

## License
This example code is in the Public Domain (or CC0 licensed, at your option.) It is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

## Acknowledgements
Thanks to Melexis for providing the MLX90640 sensor specifications and I2C protocol details. Also thanks to Patrick Wieder for providing the example implementation to the ESP32.