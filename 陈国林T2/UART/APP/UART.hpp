#ifndef __UART_HPP__
#define __UART_HPP__

#include "main.h"
#include "usart.h"
#include <stdint.h>

#ifdef __cplusplus

typedef void (*UART_Callback)(uint8_t* data, uint16_t length);

class SerialPort {
public:
    SerialPort(UART_HandleTypeDef* huart, uint8_t* rxBuffer, uint16_t bufferSize);
    void startReceive();
    bool send(const uint8_t* data, uint16_t length);
    void setCallback(UART_Callback callback);
    bool hasError() const;
    void clearError();
    void onReceiveComplete(uint16_t length);

private:
    UART_HandleTypeDef* huart;
    uint8_t* rxBuffer;
    uint16_t bufferSize;
    UART_Callback callback;
    bool error;
};

extern SerialPort uart1;
extern SerialPort uart2;
extern SerialPort uart3;

#endif

#ifdef __cplusplus
extern "C" {
#endif

void UART_OnReceiveComplete(uint8_t uart_num, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif