//*****************************************************************************
//* base64.c                                                                  *
//*                                                                           *
//* knihovna kodovani a dekodovani base64                                     *
//*                                                                           *
//*****************************************************************************
//* History:                                                                  *
//* 07.04.2019  JD  prvni verze                                               *
//*                                                                           *
//*****************************************************************************


#include <string.h>
#include "pico/stdlib.h"


// tabulka pro vypocet base64
static unsigned char bBase64[] = {'A','B','C','D','E','F','G','H','I','J',
                                  'K','L','M','N','O','P','Q','R','S','T',
                                  'U','V','W','X','Y','Z','a','b','c','d',
                                  'e','f','g','h','i','j','k','l','m','n',
                                  'o','p','q','r','s','t','u','v','w','x',
                                  'y','z','0','1','2','3','4','5','6','7',
                                  '8','9','+','/','=',0x00};


//*****************************************************************************
//* encodeBase64(uint8_t *bStr, uint16_t bLen, uint8_t *chBase64)             *
//*                                                                           *
//* kodovani do base64                                                        *
//*                                                                           *
//*  *bStr ....... vstupni data                                               *
//*  bLen ........ delka vstupnich dat                                        *
//*  *chBase64 ... vystupni base64 string                                     *
//*                                                                           *
//*****************************************************************************
void encodeBase64(uint8_t *bStr, uint16_t bLen, uint8_t *chBase64) {

  int i = 0;
  unsigned char b1, b2, b3;
  unsigned char enc1, enc2, enc3, enc4;
  int ahead;

  while (i < bLen) {

    b1 = *bStr++;
    b2 = *bStr++;
    b3 = *bStr++;

    i += 3;
    ahead = i - bLen;
    if (ahead > 0) {
      b3 = 0;
      if (ahead == 2) b2 = 0;
    }

    enc1 = b1 >> 2;
    enc2 = ((b1 & 0x03) << 4) | (b2 >> 4);
    enc3 = ((b2 & 0x0f) << 2) | (b3 >> 6);
    enc4 = b3 & 0x3f;

    if (ahead > 0) {
      enc4 = 64;
      if (ahead == 2) enc3 = 64;
    }

    *chBase64++ = *(bBase64 + enc1);
    *chBase64++ = *(bBase64 + enc2);
    *chBase64++ = *(bBase64 + enc3);
    *chBase64++ = *(bBase64 + enc4);
  }

  *chBase64 = 0;
}


//*****************************************************************************
//* decodeBase64(uint8_t *chBase64, uint8_t *chStr)                           *
//*                                                                           *
//* dekodovani z base64                                                       *
//*                                                                           *
//*  *chBase64 ... vstupni base64 string                                      *
//*  *chStr ...... vystupni dekodovany string                                 *
//*                                                                           *
//*****************************************************************************
void decodeBase64(uint8_t *chBase64, uint8_t *chStr) {

  unsigned char *bTmp1;
  int i, x, shifter;
  int cnt = -1;

  for (i = 0; i < strlen((char*)chBase64); i++) {

    bTmp1 = (unsigned char*)strchr((char*)bBase64, *(chBase64+i));
    if (!bTmp1) {
      continue;
    } else {
      if (*bTmp1 == '=') {
        continue;
      } else {
        x = (unsigned char)(bTmp1 - bBase64);
      }
    }

    cnt++;

    switch (cnt % 4) {

      case 0:
        shifter = x;
        continue;
      case 1:
        *chStr++ = (shifter << 2) | (x >> 4);
        shifter = x & 0x0f;
        break;
      case 2:
        *chStr++ = (shifter << 4) | (x >> 2);
        shifter = x & 0x03;
        break;
      case 3:
        *chStr++ = (shifter << 6) | (x >> 0);
        shifter = x & 0x00;
        break;
    }
  }

  *chStr = 0;
}

