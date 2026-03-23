// #include <Arduino.h>
// #include <Wire.h>
// #include <FlexCAN_T4.h>
// #define G(c) (((c) >> 4) & 15)
// #define P(c) ((c) & 15)
// #define S(i) ((float[]){.25, .5, .75, 1}[(i) > 3 ? 3 : (i)] * .33)
// #define W(m, v) ({int32_t x=(v)*1e5;(m).buf[0]=x>>24;(m).buf[1]=x>>16;(m).buf[2]=x>>8;(m).buf[3]=x;bus.write(m); })
// #define M(i, v) ({CAN_message_t m;m.flags.extended=1;m.id=i;m.len=4;W(m,v); })
// #define L(l, r) (M(0x78, l), M(0x16, r))
// #define H(n, ...) \
//     static void h##n(uint8_t p) { __VA_ARGS__; }
// static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> bus;
// static volatile struct
// {
//     uint8_t d;
//     bool f;
//     unsigned long t;
// } I;
// static void R(int)
// {
//     if (Wire.available())
//     {
//         I.d = Wire.read();
//         I.f = 1;
//     }
// }
// H(0, if (p == 1) L(0, 0))
// H(1, L(0, 0))
// H(2, float s = S(p); L(s, s))
// H(3, float s = S(p); L(-s, -s))
// H(4, float s = S(p); L(-s, s))
// H(5, float s = S(p); L(s, -s))
// H(6, ;)
// H(7, ;)
// H(8, ;)
// static void (*T[])(uint8_t) = {h0, h1, h2, h3, h4, h5, h6, h7, h8};
// void setup()
// {
//     Serial.begin(115200);
//     Wire.begin(8);
//     Wire.onReceive(R);
//     bus.begin();
//     bus.setBaudRate(5e5);
//     Serial.println("OK");
// }
// void loop()
// {
//     if (I.f)
//     {
//         I.f = 0;
//         I.t = millis();
//         if (T[G(I.d)])
//             T[G(I.d)](P(I.d));
//     }
//     if (millis() - I.t > 200)
//     {
//         T[1](0);
//         I.t = millis();
//     }
// }
