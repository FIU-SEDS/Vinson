/**
 ******************************************************************************
 * @file    ASM330LHH_I2C_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Arduino test application for the STMicrolectronics STEVAL-MKI193V1
 *          adapter board via I2C.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
// In order to test this sketch you need to plug a STEVAL-MKI193V1 in the DIL24 adapter of the X-NUCLEO-IKS01A3

// Includes
#include <ASM330LHHSensor.h>
#include <Adafruit_I2CDevice.h>

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
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Force INT1 of ASM330LHH low in order to enable I2C
  pinMode(INT_1, OUTPUT);

  digitalWrite(INT_1, LOW);

  delay(200);

  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize I2C bus.
  DEV_I2C.begin();

  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Enable_G();
}

void loop()
{
  // Led blinking.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

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
}
