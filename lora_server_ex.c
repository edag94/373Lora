//SERVER TRANSACTION

#include "lora.h"

void server_loop()
{
	uint8_t avail = available();
  if (avail != FALSE)
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (recv(buf, &len))
    {
      printf("got request: ");
      printf((char*)buf);
      printf("\r\n");

      // Send a reply
      uint8_t data[] = "And hello back to you";
      send(data, sizeof(data));
      wait_packet_sent(0);
      printf("Sent a reply\r\n");

    }
    else
    {
      printf("recv failed\r\n");
    }
  }
}
