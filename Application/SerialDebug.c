#include "SerialDebug.h"

uint8_t DebugBuffer[100] = {0};
uint8_t position = 0;

uint8_t Debug_Count = 0;
uint8_t Debug_Period = 10;

void Serial_Debug(UART_HandleTypeDef *huart, uint16_t debug_period, float a, float b, float c, float d, float e, float f)
{
    if (debug_period == 0)
        return;
    if (Debug_Count >= debug_period)
    {
        Debug_Count = 0;
        Debug_Buf_Generate(huart, a, b, c, d, e, f, (uint8_t *)DebugBuffer);
    }
    Debug_Count++;
}

void Debug_Buf_Send(UART_HandleTypeDef *huart, uint8_t *data, uint8_t length)
{

}

static int _float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal >= 0.5f)
        integer++;
    return integer;
}

void Debug_Buf_Generate(UART_HandleTypeDef *huart, float a, float b, float c, float d, float e, float f, uint8_t *data)
{
    uint8_t position = 0;
    float buffer[6] = {a, b, c, d, e, f};
    for (uint8_t cnt = 0; cnt < 6; cnt++)
    {
        if (buffer[cnt] > -0.01f && buffer[cnt] < 0.01f)
        {
            data[position++] = cnt + 'a';
            data[position++] = '=';
            data[position++] = '0';
            data[position++] = ',';
            continue;
        }
        if (buffer[cnt] > 0.0f && buffer[cnt] < 0.1f)
        {
            data[position++] = cnt + 'a';
            data[position++] = '=';
            data[position++] = '0';
            data[position++] = '.';
            data[position++] = '0';
            data[position++] = _float_rounding(buffer[cnt] * 100) % 10 + '0';
            data[position++] = ',';
            continue;
        }
        if (buffer[cnt] > -0.1f && buffer[cnt] < 0.0f)
        {
            data[position++] = cnt + 'a';
            data[position++] = '=';
            data[position++] = '-';
            data[position++] = '0';
            data[position++] = '.';
            data[position++] = '0';
            data[position++] = _float_rounding(-buffer[cnt] * 100) % 10 + '0';
            data[position++] = ',';
            continue;
        }
        float2char(buffer[cnt], data, cnt, &position);
    }
    data[position - 1] = '\n';
    Debug_Buf_Send(huart, (uint8_t *)DebugBuffer, position);
}

void float2char(float floatdata, uint8_t *buffer, uint8_t n, uint8_t *position) 
{
    int32_t slope;
    int32_t temp;
    int8_t i, j;
    slope = _float_rounding(floatdata * 100);
    buffer[*position] = n + 'a';
    *position += 1;
    buffer[*position] = '=';
    *position += 1;
    if (slope < 0) 
    {
        buffer[*position] = '-';
        slope = -slope;
        *position += 1;
    }
    temp = slope;              
    for (i = 0; temp != 0; i++)
        temp /= 10;
    temp = slope;
    for (j = i; j >= 0; j--)
    {
        buffer[*position + j] = temp % 10 + '0';
        temp /= 10;
        if ((i - j) == 1)
        {
            buffer[*position + j - 1] = '.'; 
            j -= 1;
        }
    }
    *position += i + 1;
    buffer[*position] = ',';
    *position += 1;
}