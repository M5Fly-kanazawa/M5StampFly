/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "rc.hpp"
#include "sbus.hpp"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "flight_control.hpp"


// Telemetry相手のMAC ADDRESS 
// 4C:75:25:AD:B6:6C
// ATOM Lite (C): 4C:75:25:AE:27:FC
// 赤水玉テープ　ATOM lite: 4C:75:25:AD:8B:20 
uint8_t TelemAddr[6] = {0,0,0,0,0,0};
volatile uint16_t Connect_flag = 0;
volatile uint8_t Rc_err_flag  = 0;
volatile float Stick[18];
volatile uint8_t Recv_MAC[3];
volatile uint8_t MyMacAddr[6];
volatile uint8_t peer_command[4] = {0xaa, 0x55, 0x16, 0x88};

esp_now_peer_info_t peerInfo;

void on_esp_now_sent(const uint8_t *mac_addr, esp_now_send_status_t status);
void packetChannels(void);

void rc_loop(void) {
    Connect_flag = sbus_loop();
    packetChannels();
}

// ESP-NOW受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len) {
    //Connect_flag = 0;

    uint8_t *d_int;
    // int16_t d_short;
    float d_float;

    if (!TelemAddr[0] && !TelemAddr[1] && !TelemAddr[2] && !TelemAddr[3] && !TelemAddr[4] && !TelemAddr[5]) {
        memcpy(TelemAddr, mac_addr, 6);
        memcpy(peerInfo.peer_addr, TelemAddr, 6);
        peerInfo.channel = CHANNEL;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            USBSerial.println("Failed to add peer2");
            memset(TelemAddr, 0, 6);
        } else {
            esp_now_register_send_cb(on_esp_now_sent);
        }
    }
}

// 送信コールバック
uint8_t esp_now_send_status;
void on_esp_now_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    esp_now_send_status = status;
}

float dead_band(float x, float min, float max) {
    if ( (min < x) && (x < max)  ) x = 0.0;
    return x;
}

void packetChannels(void)
{
#if 0
    USBSerial.printf("%04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d  %04d %04d \n\r", 
        sbus_getChannel(1),//ch1
        sbus_getChannel(2),//ch2
        sbus_getChannel(3),//ch3
        sbus_getChannel(4),//ch4
        sbus_getChannel(5),//SF
        sbus_getChannel(6),//SB
        sbus_getChannel(7),//SC
        sbus_getChannel(8),//SG
        sbus_getChannel(9),//SE
        sbus_getChannel(10),//SA
        sbus_getChannel(11),//SD
        sbus_getChannel(12),//SH
        sbus_getChannel(13),//not installed
        sbus_getChannel(14),//not installed
        sbus_getChannel(15),//not installed
        sbus_getChannel(16),//not installed
        sbus_getChannel(17),//not installed
        sbus_getChannel(18));//not installed
#endif


    Stick[AILERON]=  2.0 * (float)(sbus_getChannel(4) - AILERON_MID)/(float)(AILERON_MAX - AILERON_MIN);
    Stick[ELEVATOR]= 2.0 * (float)(sbus_getChannel(2) - ELEVATOR_MID)/(float)(ELEVATOR_MAX - ELEVATOR_MIN);
    Stick[THROTTLE]= 2.0 * (float)(sbus_getChannel(3) - THROTTLE_MID)/(float)(THROTTLE_MAX - THROTTLE_MIN);
    Stick[RUDDER]=   2.0 * (float)(sbus_getChannel(1) - RUDDER_MID)/(float)(RUDDER_MAX - RUDDER_MIN);
    Stick[CONTROLMODE] = 0;
    Stick[BUTTON_ARM] = (uint8_t)(sbus_getChannel(5)>1100);//auto_up_down_status    
    Stick[ALTCONTROLMODE] = 1;//高度制御
    Stick[BUTTON_FLIP] = 0;
    
    Stick[THROTTLE] = 0.8 * dead_band(Stick[THROTTLE], -0.15, 0.15);
#if 0
    USBSerial.printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f  %6.3f\n\r", 
                                                Stick[THROTTLE],
                                                Stick[AILERON],
                                                Stick[ELEVATOR],
                                                Stick[RUDDER],
                                                Stick[BUTTON_ARM],
                                                Stick[BUTTON_FLIP],
                                                Stick[CONTROLMODE],
                                                Stick[ALTCONTROLMODE],
                                                Stick[LOG]);
#endif

}

void rc_init(void) {
    //
    //Initialize S.BUS
    sbus_init();
    for (uint8_t i = 0; i < 18; i++) Stick[i] = 0.0;
    
    //
    //Initialize ESP-NOW Telemetry
    // ESP-NOW初期化
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    WiFi.macAddress((uint8_t *)MyMacAddr);
    USBSerial.printf("MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X\r\n", MyMacAddr[0], MyMacAddr[1], MyMacAddr[2],
                     MyMacAddr[3], MyMacAddr[4], MyMacAddr[5]);

    if (esp_now_init() == ESP_OK) {
        USBSerial.println("ESPNow Init Success");
    } else {
        USBSerial.println("ESPNow Init Failed");
        ESP.restart();
    }

    // MACアドレスブロードキャスト
    uint8_t addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(peerInfo.peer_addr, addr, 6);
    peerInfo.channel = CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        USBSerial.println("Failed to add peer");
        return;
    }
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Send my MAC address
    for (uint16_t i = 0; i < 50; i++) {
        send_peer_info();
        delay(50);
        USBSerial.printf("%d\n", i);
    }

    // ESP-NOW再初期化
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
        USBSerial.println("ESPNow Init Success2");
    } else {
        USBSerial.println("ESPNow Init Failed2");
        ESP.restart();
    }

    // ESP-NOWコールバック登録
    esp_now_register_recv_cb(OnDataRecv);
    USBSerial.println("ESP-NOW Ready.");    
}

void send_peer_info(void) {
    uint8_t data[11];
    data[0] = CHANNEL;
    memcpy(&data[1], (uint8_t *)MyMacAddr, 6);
    memcpy(&data[1 + 6], (uint8_t *)peer_command, 4);
    esp_now_send(peerInfo.peer_addr, data, 11);
}

uint8_t telemetry_send(uint8_t *data, uint16_t datalen) {
    static uint32_t cnt       = 0;
    static uint8_t error_flag = 0;
    static uint8_t state      = 0;

    esp_err_t result;

    if ((error_flag == 0) && (state == 0)) {
        result = esp_now_send(peerInfo.peer_addr, data, datalen);
        cnt    = 0;
    } else
        cnt++;

    if (esp_now_send_status == 0) {
        error_flag = 0;
        // state = 0;
    } else {
        error_flag = 1;
        // state = 1;
    }
    // 一度送信エラーを検知してもしばらくしたら復帰する
    if (cnt > 500) {
        error_flag = 0;
        cnt        = 0;
    }
    cnt++;
    // USBSerial.printf("%6d %d %d\r\n", cnt, error_flag, esp_now_send_status);

    return error_flag;
}

uint8_t rc_isconnected(void) {
    bool status;
    Connect_flag++;
    if (Connect_flag < 40)
        status = 1;
    else
        status = 0;
    // USBSerial.printf("%d \n\r", Connect_flag);
    return status;
}