#include <stdint.h> // Needed for uint8_t
#include <stdio.h>

void value_binary(uint8_t var);

int main()
{
  // PWR Control Register
  uint8_t pwr = 0b00000000;
  pwr |= (0b1);
  pwr |= (0b1 << 1);
  pwr |= (0b11 << 4);
  value_binary(pwr);
  printf("PWR_CTRL Value in decimal: %d\n", pwr); // Decimal output

  // OSR Register
  uint8_t osr = 0b00000000; // Start with all bits cleared
  osr |= (0b011);           // Set OSR_P (bits 2..0) to 011
  osr |= (0b001 << 3);      // Set OSR_T (bits 5..3) to 001
  value_binary(osr);
  printf("OSR Value in decimal: %d\n", osr); // Decimal output

  // IIR Filter Coeff Register
  uint8_t iir = 0b00000000;
  iir |= (0b001 << 1);
  value_binary(iir);
  printf("ODR Value in decimal: %d\n", iir); // Decimal output

  printf("\n\nSensor settings:\nPower Mode: Normal\nTemp_en: Enabled\nPress_en: enabled\nosr_p: x8\nosr_t: x2\nIIR filter: 1\nODR: 50Hz");

  return 0;
}

void value_binary(uint8_t var)
{
  printf("Value in binary: %c%c%c%c%c%c%c%c\n",
         (var & 0x80 ? '1' : '0'),
         (var & 0x40 ? '1' : '0'),
         (var & 0x20 ? '1' : '0'),
         (var & 0x10 ? '1' : '0'),
         (var & 0x08 ? '1' : '0'),
         (var & 0x04 ? '1' : '0'),
         (var & 0x02 ? '1' : '0'),
         (var & 0x01 ? '1' : '0'));
}