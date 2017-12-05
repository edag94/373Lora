//CLIENT transaction

#include "lora.h"

void client_transaction(void){

  printf("Sending message to server.\r\n");
  uint8_t data[] = "Hi!";
  send(data, sizeof(data));

  wait_packet_sent(0);
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (wait_available_timeout(10000))
  {
    // Should be a reply message for us now
    if (recv(buf, &len))
   {
      printf("got reply: ");
      printf((char*)buf);
      printf("\r\n");
    }
    else
    {
      printf("recv failed\r\n");
    }
  }
  else
  {
    printf("No reply, is server running?\r\n");
  }
  //delay;
  volatile int i;
  for (i = 0; i < 10000000; ++i);
}
