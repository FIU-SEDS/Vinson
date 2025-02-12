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
    uint8_t osr = 0b00000000;              // Start with all bits cleared
    osr |= (0b010);                        // Set OSR_P (bits 2..0) to 010
    osr |= (0b011 << 3);                   // Set OSR_T (bits 5..3) to 011
    value_binary(osr);
    printf("OSR Value in decimal: %d\n", osr); // Decimal output

  // ODR Register
  uint8_t odr = 0b00000000;
  odr |= (0b001 << 1);
  value_binary(odr);
  printf("ODR Value in decimal: %d\n", odr); // Decimal output

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