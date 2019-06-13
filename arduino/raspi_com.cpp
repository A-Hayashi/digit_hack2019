#include <bluefruit.h>
#include "raspi_com.h"

#define CHARBUFFERSIZE 100

static char Buffer[100];
static boolean readSerial();
static int strToInt(char *str);

static boolean readSerial()
{
  static byte index = 0; // Buffer array index counter

  while (Serial.available() > 0)
  {                          // Does serial1 have characters available?
    char rc = Serial.read(); // Read a character

    if (rc == '\n')
    {                    // Is it newline?
      Buffer[index] = 0; // Yes, so terminate char array with 0
      index = 0;         // Reset array index
      return true;       // Return true (we have a full line)
    }
    if (true)
    {
      // if (rc >= '0') {                    // Is value between 32 and 127?
      Buffer[index++] = rc; // Yes, so store it and increment index
      if (index > CHARBUFFERSIZE)
      {          // Have we filled the buffer?
        index--; // Yes, so decrement index
      }
    }
  }
  return false; // Return false (we don't have a full line)
}

void ExecUartCmd()
{
  int motor;
  boolean ret = readSerial();
  if (ret == true)
  {
    if (Buffer[0] == 'm')
    {
      uint8_t motor = strToInt(&Buffer[1]);
      if (Buffer[2] == ':')
      {
        int speed = strToInt(&Buffer[3]);
        ExecUartCmdCallback('m', motor, speed);
      }
    }

    if (Buffer[0] == 's')
    {
      uint8_t servo = strToInt(&Buffer[1]);
      if (Buffer[2] == ':')
      {
        int angle = strToInt(&Buffer[3]);
        ExecUartCmdCallback('s', servo, angle);
      }
    }
  }
}

static int strToInt(char *str)
{
  char c;
  int result = 0;
  int sign = 1; // default to no minus.
  // Check for minus
  if (str[0] == '-')
  {
    str++;     // Skip over the minus.
    sign = -1; // It's a negative number.
  }

  while ((c = *str)) // Stop the loop if we are at the end of the string.
  {
    // See if it's valid digit
    if (c >= '0' && c <= '9')
    {
      result *= 10;
      result += c - '0';
    }
    else // Not a valid digit
    {
      break; // Stop looking for more digits.
    }
    str++; // Go to next digit.
  }
  return result * sign;
}