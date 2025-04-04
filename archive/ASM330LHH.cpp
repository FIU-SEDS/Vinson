// Includes
#include <ASM330LHHSensor.h>

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

#define INT_1 A5

// Components
ASM330LHHSensor AccGyr(&DEV_I2C, ASM330LHH_I2C_ADD_L);

void setup()
{
  // Initialize serial for output.
  SerialPort.begin(9600);

  // Initialize I2C bus.
  DEV_I2C.begin();

  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Enable_G();
}

void loop()
{
  // Read accelerometer and gyroscope.
  int32_t accelerometer[3] = {};
  int32_t gyroscope[3] = {};
  AccGyr.Get_X_Axes(accelerometer);
  AccGyr.Get_G_Axes(gyroscope);

  // Output data.
  SerialPort.print("ASM330LHH: | Acc[mg]: ");
  SerialPort.print(accelerometer[0]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer[1]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer[2]);
  SerialPort.print(" | Gyr[mdps]: ");
  SerialPort.print(gyroscope[0]);
  SerialPort.print(" ");
  SerialPort.print(gyroscope[1]);
  SerialPort.print(" ");
  SerialPort.print(gyroscope[2]);
  SerialPort.println(" |");
  delay(250);
}
