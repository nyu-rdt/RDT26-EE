// #include <Arduino.h>
// #include <Wire.h>
// #include <FlexCAN_T4.h>
// FlexCAN_T4<CAN1,RX_SIZE_256,TX_SIZE_16>c;CAN_message_t m;volatile uint8_t d;volatile uint32_t t;
// void R(int){d=Wire.read();t=millis();}
// void setup(){m.flags.extended=1;m.len=4;Wire.begin(8);Wire.onReceive(R);c.begin();c.setBaudRate(5e5);}
// void loop(){uint8_t g=d>>4,p=d&15;if(g||millis()-t>200){int32_t l=0,r=0;if(g>1&&g<6){int32_t x=((p>3?3:p)+1)*8250;l=(g==3||g==4)?-x:x;r=(g==3||g==5)?-x:x;}m.id=120;*(int32_t*)m.buf=__builtin_bswap32(l);c.write(m);m.id=22;*(int32_t*)m.buf=__builtin_bswap32(r);c.write(m);d=0;t=millis();}}