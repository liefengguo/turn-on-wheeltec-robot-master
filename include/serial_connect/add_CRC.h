#include <cstdint>
// #include <vector>

#include <iostream>
using namespace std;
 
uint16_t usMBCRC16(uint8_t * pucFrame, uint16_t usLen,uint8_t &pucCRCHi,uint8_t &pucCRCLo);

uint16_t usMBCRC16(uint8_t * pucFrame, uint16_t usLen);
