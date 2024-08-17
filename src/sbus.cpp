#include "sbus.hpp"
#include <Arduino.h>
#include <stdint.h>

//ローカル変数の宣言
uint16_t chars_rxed = 0;
uint16_t data_num=0;
uint8_t sbus_data[25];
uint8_t ch;
uint16_t Chdata[18];

//S.BUS Imitilize
void sbus_init(void) {
    /// シリアル通信の設定
    Serial1.begin(SBUS_BAUDRATE,SERIAL_8E2, 2, 1, true);
    memset(&sbus_data, 0, sizeof(sbus_data));
}

//S.BUS Decoder
uint8_t  sbus_loop(void) {
    uint8_t state = 1;
    while (Serial1.available()) {
        ch = Serial1.read();
        if(ch==0x0f&&chars_rxed==0){
            sbus_data[chars_rxed]=ch;
            chars_rxed++;
        }
        else if(chars_rxed>0){
            sbus_data[chars_rxed]=ch;
            chars_rxed++;            
        }
        
        switch(chars_rxed){
            case 3:
                Chdata[0]=(sbus_data[1]|(sbus_data[2]<<8)&0x07ff);
                break;
            case 4:
                Chdata[1]=(sbus_data[3]<<5|sbus_data[2]>>3)&0x07ff;
                break;
            case 6:
                Chdata[2]=(sbus_data[3]>>6|sbus_data[4]<<2|sbus_data[5]<<10)&0x07ff;
                break;
            case 7:
                Chdata[3]=(sbus_data[6]<<7|sbus_data[5]>>1)&0x07ff;
                break;
            case 8:
                Chdata[4]=(sbus_data[7]<<4|sbus_data[6]>>4)&0x07ff;
                break;
            case 10:
                Chdata[5]=(sbus_data[7]>>7|sbus_data[8]<<1|sbus_data[9]<<9)&0x07ff;
                break;
            case 11:
                Chdata[6]  = ((sbus_data[9]>>2|sbus_data[10]<<6) & 0x07FF);
                break;
            case 12:
                Chdata[7]  = ((sbus_data[10]>>5|sbus_data[11]<<3) & 0x07FF);
                break;
            case 14:
                Chdata[8]  = ((sbus_data[12]|sbus_data[13]<< 8) & 0x07FF);
                break;
            case 15:
                Chdata[9]  = ((sbus_data[13]>>3|sbus_data[14]<<5) & 0x07FF);
                break;
            case 16:
                Chdata[10] = ((sbus_data[14]>>6|sbus_data[15]<<2|sbus_data[16]<<10) & 0x07FF);
                break;
            case 17:
                Chdata[11] = ((sbus_data[16]>>1|sbus_data[17]<<7) & 0x07FF);
                break;
            case 19:
                Chdata[12] = ((sbus_data[17]>>4|sbus_data[18]<<4) & 0x07FF);
                break;
            case 21:
                Chdata[13] = ((sbus_data[18]>>7|sbus_data[19]<<1|sbus_data[20]<<9) & 0x07FF);
                break;
            case 22:
                Chdata[14] = ((sbus_data[20]>>2|sbus_data[21]<<6) & 0x07FF);
                break;
            case 23:
                Chdata[15] = ((sbus_data[21]>>5|sbus_data[22]<<3) & 0x07FF);
                break;
            case 24:
                Chdata[16] = sbus_data[23];
                break;
        }

        if(chars_rxed==25){
            Chdata[17]=sbus_data[24];
            chars_rxed=0;
        }
    }
    if (Chdata[16]==0)state = 0;
    
    return state;
}

uint16_t sbus_getChannel(uint8_t ch){
    return Chdata[ch - 1];
}

