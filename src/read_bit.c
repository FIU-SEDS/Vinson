#include <stdint.h> // Needed for uint8_t
#include <stdio.h>

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

int main(void)
{
  uint8_t test = 0b11001100;
  uint8_t mask = (0b0001 << 3);
  test ^= mask;
  value_binary(test);

  return 0;
}