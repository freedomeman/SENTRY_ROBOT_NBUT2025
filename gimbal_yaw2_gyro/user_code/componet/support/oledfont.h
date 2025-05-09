/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       oledfont.h
 * @brief      character lib of oled
 * @note
 * @Version    V1.0.0
 * @Date       Oct-7-2019      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __OLED__FONT__H
#define __OLED__FONT__H
#include "struct_typedef.h"
//PC2LCD2002ȡģ��ʽ���ã�����+����ʽ+˳��+C51��ʽ
typedef struct
{
    uint16_t length;
    uint16_t width;
    uint8_t *data;
}picture_t;



/* RM logo */
static const unsigned char RM_LOGO_BMP[] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCF,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCF,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF9,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE7,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC3,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC3,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC3,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC7,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7C,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDF,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,
    0x80,0x00,0x00,0x00,0x00,0x3C,0x00,0x1C,0xC0,0x00,0x00,0x00,0x01,0xFC,0x00,0x7E,
    0xF0,0x00,0x00,0x00,0x1F,0xFC,0x00,0xFF,0xF8,0x00,0x00,0x01,0xFF,0xFC,0x00,0xE7,
    0xFE,0x00,0x00,0x0F,0xFF,0xFC,0x00,0xC3,0xFF,0x00,0x00,0xFF,0xFF,0xFC,0x00,0xC3,
    0xFF,0xC0,0x07,0xFF,0xFF,0xFC,0x00,0xC3,0xFF,0xE0,0x1F,0xFF,0xFF,0xFC,0x00,0xC7,
    0xFF,0xF0,0x1F,0xFF,0xFF,0xFC,0x00,0xFE,0xFF,0xF8,0x1F,0xFF,0xFF,0xFC,0x00,0x7C,
    0xFF,0xFC,0x1F,0xFF,0xFF,0xFC,0x00,0x39,0xFF,0xFE,0x1F,0xFF,0xFF,0xFC,0x00,0x07,
    0xFF,0xFF,0x1F,0xFF,0xFF,0xFC,0x00,0x1F,0xFF,0xFF,0x9F,0xFF,0xFF,0xE0,0x00,0xFC,
    0xFF,0xFF,0x9F,0xFF,0xFF,0x00,0x00,0xF0,0xFF,0xFF,0x9F,0xFF,0xF0,0x00,0x00,0xFC,
    0xFF,0xFF,0x9F,0xFF,0x80,0x00,0x00,0x7F,0xFF,0xFF,0x9F,0xF8,0x00,0x00,0x00,0x0F,
    0xFF,0xFF,0x9F,0xF8,0x00,0x00,0x00,0x0F,0xFF,0xFF,0x9F,0xF8,0x00,0x00,0x00,0x1C,
    0xFF,0xFF,0x9F,0xF8,0x00,0x00,0x00,0x38,0xFF,0xFF,0x9F,0xF8,0x00,0x00,0x00,0xF0,
    0xFF,0xDF,0x9F,0xF8,0x00,0x00,0x00,0xFF,0xFF,0xCF,0x9F,0xF8,0x00,0x80,0x00,0xFF,
    0xFF,0xC7,0x9F,0xF8,0x00,0x80,0x00,0x07,0xFF,0xC3,0x9F,0xF8,0x00,0xC0,0x00,0x00,
    0xFF,0xC0,0x9F,0xFB,0x00,0xC0,0x00,0x01,0xFF,0xC0,0x1F,0xFB,0x80,0xE0,0x00,0x03,
    0xFF,0xC0,0x1F,0xFB,0xC0,0xE0,0x00,0x07,0xFF,0xC0,0x1F,0xFB,0xE0,0xF0,0x00,0x1E,
    0xFF,0xC0,0x1F,0xFB,0xF0,0xF0,0x00,0x3D,0xFF,0xC0,0x1F,0xFB,0xF8,0xF8,0x00,0x73,
    0xFF,0xC0,0x1F,0xFB,0xFC,0xF8,0x00,0xE3,0xFF,0xC0,0x1F,0xFB,0xFF,0xFC,0x00,0xE3,
    0xFF,0xC0,0x1F,0xFB,0xFF,0xFC,0x00,0xFF,0xFF,0xC0,0x1F,0xFB,0xFF,0xFE,0x00,0x7F,
    0xFF,0xC0,0x1F,0xFB,0xFF,0xFE,0x00,0x1F,0xFF,0xE0,0x1F,0xFB,0xFF,0xFF,0x00,0x07,
    0xFF,0xE0,0x3F,0xFB,0xFF,0xFF,0x00,0x01,0xFF,0xF0,0x7F,0xFB,0xFF,0xFF,0x80,0x00,
    0x7F,0xF8,0xFF,0xF1,0xFF,0xFF,0x80,0x01,0x7F,0xFF,0xFF,0xF0,0xFF,0xEF,0xC0,0x73,
    0x7F,0xFF,0xFF,0xF0,0x7F,0xE7,0xC0,0xFB,0x3F,0xFF,0xFF,0xE0,0x3F,0xE3,0xE0,0xFB,
    0x3F,0xFF,0xFF,0xE0,0x1F,0xE1,0xE0,0xDB,0x1F,0xFF,0xFF,0xC0,0x0F,0xE0,0xF0,0xDB,
    0x0F,0xFF,0xFF,0x80,0x07,0xE0,0x70,0xDB,0x07,0xFF,0xFF,0x00,0x03,0xE0,0x38,0xDB,
    0x03,0xFF,0xFE,0x00,0x01,0xE0,0x18,0xDF,0x00,0xFF,0xF8,0x00,0x00,0xE0,0x0C,0xDE,
    0x00,0x3F,0xE0,0x00,0x00,0x60,0x04,0x8E,0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x40,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0B,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCF,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCF,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFB,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x71,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    /* (128 X 64 ) */
};
static const picture_t rm_logo = {128, 64, (uint8_t *)RM_LOGO_BMP};

static const uint8_t RM_LOGO_BMP_TRANS[] =
{
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x06,0x0E,0X1E,0X3E,0X7E,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFE,0XFC,0XFC,0XF8,0XF8,0XF0,0XE0,0XC0,0X80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x07,0x0F,0X1F,0X3F,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XE7,0XC7,0X87,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x0F,0X1F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0XC0,0XFC,0XFC,0XFC,0XFC,0XFC,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFC,0XFC,0XFC,0XFC,0XFC,0XFC,0XFC,0XFC,0XFC,0XFC,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X7F,0X3F,0x0F,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0X80,0XF0,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X1F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0X1F,0X3F,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0XCF,0X87,0x07,0x03,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0XE0,0XFC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X3F,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0X40,0X40,0X40,0XC0,0XC0,0XC0,0XC0,0XC1,0XC3,0XC7,0XCF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFC,0XF8,0XF0,0XE0,0XC0,0X80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x0C,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x03,0x03,0x07,0x07,0x0F,0x0F,0X1F,0X1F,0X3F,0X3F,0X7F,0X7F,0XFF,0XF3,0XE3,0XC3,0X83,0x03,0x03,0x03,0x03,0x03,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x03,0x03,0x06,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x78,0x7B,0x1B,0x1B,0x1B,0x7B,0x7F,0x6F,0x4E,0x00,0x3E,0x7F,0x63,0x63,0x63,0x7F,0x3E,0x00,0x78,0x7B,0x6B,0x6B,0x6B,0x6B,0x7B,0x7F,0x36,0x00,0x3E,0x7F,0x63,0x63,0x63,0x7F,0x3E,0x40,0x60,0x78,0x3D,0x07,0x1F,0x7C,0x70,0x1D,0x07,0x7F,0x78,0x40,0x00,0x60,0x70,0x38,0x5D,0x6F,0x67,0x6F,0x7C,0x70,0x40,0x00,0x40,0x66,0x6F,0x6B,0x6B,0x6B,0x6B,0x7B,0x31,0x02,0x03,0x03,0x7F,0x7F,0x03,0x03,0x61,0x68,0x6B,0x6B,0x6B,0x6B,0x6B,0x0B,0x03,0x60,0x78,0x7B,0x1B,0x1B,0x1B,0x7B,0x7B,0x6F,0x4E,0x00,0x46,0x6F,0x6F,0x6B,0x6B,0x6B,0x7B,0x31,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};


/* ���ʿ��� */
static const unsigned char dinosaur_data[] = {
    0x00,0x00,0x00,0x00,0x40,0x80,0x00,0x00,0x00,0x00,0x40,0x80,0x00,0x00,0x00,0x00,
    0x40,0x80,0x00,0x00,0x00,0x00,0x40,0x80,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,
    0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x44,0x00,0x00,0x00,0x00,0x00,0x44,0x00,
    0x00,0x00,0x00,0x00,0x44,0x00,0x00,0x00,0x00,0x00,0x44,0x00,0x00,0x03,0xFF,0xC0,
    0x40,0x00,0x00,0x03,0xFF,0xC0,0x40,0x00,0x00,0x00,0x3F,0xE0,0x40,0x00,0x00,0x00,
    0x3F,0xF0,0x40,0x00,0x00,0x00,0x0F,0xF8,0x40,0x80,0x00,0x00,0x0F,0xFC,0x40,0x80,
    0x00,0x00,0x03,0xFE,0x00,0x00,0x00,0x00,0x03,0xFF,0x00,0x00,0x00,0x00,0x03,0xFF,
    0x80,0x00,0x00,0x00,0x07,0xFF,0xC0,0x00,0x00,0x00,0x0F,0xFF,0xFF,0x80,0x00,0x00,
    0x3F,0xFF,0xF9,0x80,0x00,0x00,0x3F,0xFF,0xF9,0x80,0x00,0x00,0x3F,0xFF,0xE0,0x00,
    0x00,0x00,0xFF,0xFF,0xE0,0x00,0x00,0x00,0xFF,0xFF,0x80,0x00,0x00,0x00,0xFF,0xFF,
    0x80,0x00,0x00,0x03,0xFF,0xFF,0xE0,0x00,0x00,0x03,0xFF,0xFF,0xE0,0x00,0x3F,0xFF,
    0xFF,0xFF,0xFF,0x80,0x3F,0xFF,0xFF,0xFF,0xFF,0x80,0xFF,0xFF,0xFF,0xFE,0x01,0x80,
    0xFF,0xFF,0xFF,0xFE,0x01,0x80,0xCF,0xFF,0xFF,0xF8,0x00,0x00,0xCF,0xFF,0xFF,0xF8,
    0x00,0x00,0xFF,0xFF,0xFF,0xC0,0x40,0x00,0xFF,0xFF,0xFF,0xE0,0x40,0x00,0xFF,0xFC,
    0x30,0x00,0x40,0x00,0xFF,0xFC,0x30,0x00,0x40,0x00,0xFF,0xCC,0x3C,0x00,0x40,0x00,
    0xFF,0xCC,0x3C,0x00,0x41,0x00,0xFF,0xCC,0x00,0x00,0x40,0x00,0xFF,0xCC,0x00,0x00,
    0x40,0x00,0xFF,0xCC,0x00,0x00,0x40,0x00,0xFF,0xCC,0x00,0x00,0x40,0x00,0xFF,0xC0,
    0x00,0x00,0x40,0x00,0xFF,0xC0,0x00,0x00,0x40,0x00,0x3F,0xC0,0x00,0x00,0x40,0x80,
    0x3F,0xC0,0x00,0x00,0x40,0x80,0x00,0x00,0x00,0x00,0x40,0x80,0x00,0x00,0x00,0x00,
    0x40,0x80,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,
    0x00,0x00,0x48,0x00,0x00,0x00,0x00,0x00,0x48,0x00,0x00,0x00,0x00,0x00,0x48,0x00,
    0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,
    0x40,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,
    0x00,0x00,0x40,0x00
    /* (62 X 41 ) */
};
static const picture_t dinosaur = {62, 41, (uint8_t *)dinosaur_data};


/* �ܲ��� */
static const unsigned char luobojundata[] = {
0x00,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,
0x00,0x01,0x9C,0x80,0x00,0x00,0x00,0x1F,0xFF,0x8F,0x80,0x00,0x00,0x00,0x7F,0xFF,
0xE3,0x00,0x00,0x00,0x00,0xF0,0x00,0xFE,0x00,0x00,0x00,0x01,0xC0,0x00,0x1F,0x00,
0x00,0x00,0x01,0x80,0x00,0x07,0xC0,0x00,0x07,0x03,0x00,0x00,0x00,0xF0,0x00,0x07,
0xE3,0x06,0x00,0x00,0x7C,0x00,0x0C,0x7B,0x06,0x00,0x00,0x1E,0x00,0x7C,0x0F,0x00,
0x00,0x00,0x07,0x80,0xFF,0xE6,0x00,0x00,0x00,0x01,0xC0,0xC0,0xFE,0x00,0x20,0x00,
0x00,0xE0,0xC0,0x0E,0x00,0x3C,0x00,0x00,0x70,0xC0,0x02,0x00,0x26,0x00,0x00,0x30,
0xC0,0x02,0x00,0x3E,0x00,0x00,0x30,0xC0,0x3E,0x00,0x26,0x00,0x00,0x30,0xFF,0xF2,
0x00,0x3C,0x00,0x00,0x70,0x7E,0x06,0x00,0x20,0x00,0x00,0xE0,0x0C,0x1F,0x00,0x00,
0x00,0x03,0xC0,0x0C,0xFB,0x06,0x00,0x00,0x07,0x80,0x07,0xE3,0x06,0x00,0x00,0x1E,
0x00,0x07,0x03,0x00,0x00,0x00,0x78,0x00,0x00,0x01,0x80,0x00,0x03,0xE0,0x00,0x00,
0x01,0xC0,0x00,0x0F,0x80,0x00,0x00,0x00,0xF0,0x00,0xFF,0x00,0x00,0x00,0x00,0x7F,
0xFF,0xFF,0x00,0x00,0x00,0x00,0x1F,0xFF,0x0D,0x80,0x00,0x00,0x00,0x00,0x03,0xB8,
0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x00,0x00,
/* (32 X 52 ) */
};
static const picture_t luobojun = {32, 52, (uint8_t *)luobojundata};



/* СR */
static const unsigned char r_data[] = {
0x00,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x03,0x80,0xC0,0x00,0x00,0x1F,0x80,0xF0,
0x00,0x03,0xFF,0x80,0xFC,0x00,0x3F,0xFF,0x80,0xFE,0x01,0xFF,0xFF,0x80,0xFF,0x07,
0xFF,0xFF,0x80,0xFF,0x87,0xFF,0xFF,0x80,0xFF,0xC7,0xFF,0xFF,0x80,0xFF,0xE7,0xFF,
0xFF,0x80,0xFF,0xE7,0xFF,0xFC,0x00,0xFF,0xE7,0xFF,0x80,0x00,0xFF,0xE7,0xF8,0x00,
0x00,0xFF,0xE7,0xF8,0x00,0x00,0xFF,0xE7,0xF8,0x00,0x00,0xFF,0xE7,0xF8,0x00,0x00,
0xFE,0xE7,0xF8,0x00,0x00,0xFE,0x67,0xF8,0x00,0x00,0xFE,0x27,0xFA,0x00,0x00,0xFE,
0x07,0xFB,0x04,0x00,0xFE,0x07,0xFB,0x86,0x00,0xFE,0x07,0xFB,0xC6,0x00,0xFE,0x07,
0xFB,0xE7,0x00,0xFE,0x07,0xFB,0xF7,0x00,0xFE,0x07,0xFB,0xFF,0x80,0xFE,0x07,0xFB,
0xFF,0x80,0xFE,0x07,0xFB,0xFF,0xC0,0xFF,0x07,0xFB,0xFF,0xE0,0xFF,0x9F,0xFB,0xFF,
0xE0,0xFF,0xFF,0xF9,0xFF,0xF0,0x7F,0xFF,0xF0,0xFE,0xF0,0x7F,0xFF,0xF0,0x7E,0x78,
0x3F,0xFF,0xE0,0x3E,0x38,0x3F,0xFF,0xC0,0x1E,0x1C,0x1F,0xFF,0x80,0x0E,0x0C,0x0F,
0xFF,0x00,0x06,0x04,0x07,0xFC,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
/* (38 X 39 ) */
};
static const picture_t logo_s = {38, 39, (uint8_t *)r_data};






/* ���߷��� С�� */
static const unsigned char offline_icon_data[] = {
    0x82,0x44,0x28,0x10,0x28,0x44,0x82,0x00,
    /* (8 X 8 ) */
};
static const picture_t offline_icon = {8, 8, (uint8_t *)offline_icon_data};

/* ���߷��� ��̾�� */
static const unsigned char offline_icon_data1[] = {
0x00,0x00,0x78,0x00,0xFE,0xC0,0xFE,0xC0,0x78,0x00,0x00,0x00,
/* (6 X 12 ) */
};
static const picture_t offline_icon1 = {6, 12, (uint8_t *)offline_icon_data1};


/* �����ź�ǿ�ȣ�0-5�� */
static const unsigned char signal0_data[] = {
    0x40,0x00,0x70,0x00,0x78,0x00,0x4F,0xE0,0x4F,0xE0,0x78,0x00,0x70,0x00,0x40,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
    /* (24 X 12 ) */
};
static const unsigned char signal1_data[] = {
    0x40,0x00,0x70,0x00,0x78,0x00,0x4F,0xE0,0x4F,0xE0,0x78,0x00,0x70,0x60,0x40,0x60,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
    /* (24 X 12 ) */
};
static const unsigned char signal2_data[] = {
    0x40,0x00,0x70,0x00,0x78,0x00,0x4F,0xE0,0x4F,0xE0,0x78,0x00,0x70,0x60,0x40,0x60,
    0x00,0x00,0x00,0x00,0x01,0xE0,0x01,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
    /* (24 X 12 ) */
};
static const unsigned char signal3_data[] = {
    0x40,0x00,0x70,0x00,0x78,0x00,0x4F,0xE0,0x4F,0xE0,0x78,0x00,0x70,0x60,0x40,0x60,
    0x00,0x00,0x00,0x00,0x01,0xE0,0x01,0xE0,0x00,0x00,0x00,0x00,0x07,0xE0,0x07,0xE0,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
    /* (24 X 12 ) */
};
static const unsigned char signal4_data[] = {
    0x40,0x00,0x70,0x00,0x78,0x00,0x4F,0xE0,0x4F,0xE0,0x78,0x00,0x70,0x60,0x40,0x60,
    0x00,0x00,0x00,0x00,0x01,0xE0,0x01,0xE0,0x00,0x00,0x00,0x00,0x07,0xE0,0x07,0xE0,
    0x00,0x00,0x00,0x00,0x1F,0xE0,0x1F,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
    /* (24 X 12 ) */
};
static const unsigned char signal5_data[] = {
    0x40,0x00,0x70,0x00,0x78,0x00,0x4F,0xE0,0x4F,0xE0,0x78,0x00,0x70,0x60,0x40,0x60,
    0x00,0x00,0x00,0x00,0x01,0xE0,0x01,0xE0,0x00,0x00,0x00,0x00,0x07,0xE0,0x07,0xE0,
    0x00,0x00,0x00,0x00,0x1F,0xE0,0x1F,0xE0,0x00,0x00,0x00,0x00,0x7F,0xE0,0x7F,0xE0
    /* (24 X 12 ) */
};
#define SIGNAL_MAX_LEVEL    5
static const picture_t wifi_signal[SIGNAL_MAX_LEVEL + 1] = {
    {24, 12, (uint8_t *)signal0_data},
    {24, 12, (uint8_t *)signal1_data},
    {24, 12, (uint8_t *)signal2_data},
    {24, 12, (uint8_t *)signal3_data},
    {24, 12, (uint8_t *)signal4_data},
    {24, 12, (uint8_t *)signal5_data},
};



/* ��ѡ��ѡ�� */
static const unsigned char check_box_y_data[] = {
    0x00,0x00,0x7F,0xE0,0x40,0x20,0x43,0x20,0x41,0xA0,0x40,0xE0,
    0x43,0xA0,0x4F,0x20,0x5C,0x20,0x70,0x20,0x7F,0xE0,0x00,0x00,
    /* (12 X 12 ) */
};
/* ��ѡ��δѡ�� */
static const unsigned char check_box_n_data[] = {
    0x00,0x00,0x7F,0xE0,0x40,0x20,0x40,0x20,0x40,0x20,0x40,0x20,
    0x40,0x20,0x40,0x20,0x40,0x20,0x40,0x20,0x7F,0xE0,0x00,0x00,
    /* (12 X 12 )*/
};
static const picture_t check_box[2] = {
    {12, 12, (uint8_t *)check_box_y_data},
    {12, 12, (uint8_t *)check_box_n_data},
};

// battery box
static const unsigned char battery_box_data[] = {
0xFF,0xFC,0x80,0x04,0x80,0x04,0x80,0x04,0x80,0x04,0x80,0x04,0x80,0x04,0x80,0x04,
0x80,0x04,0x80,0x04,0x80,0x04,0x80,0x04,0x80,0x04,0x80,0x04,0x80,0x04,0x80,0x04,
0x80,0x04,0x80,0x04,0x80,0x04,0x80,0x04,0x80,0x04,0x80,0x04,0xFF,0xFC,0x07,0x80,/*"?��?��?????t",0*/
};
static const picture_t battery_box = {24,14, (uint8_t *)battery_box_data};


//the common ascii character
static const unsigned char asc2_1206[95][12]={
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*" ",0*/
{0x00,0x00,0x00,0x00,0x3F,0x40,0x00,0x00,0x00,0x00,0x00,0x00},/*"!",1*/
{0x00,0x00,0x30,0x00,0x40,0x00,0x30,0x00,0x40,0x00,0x00,0x00},/*""",2*/
{0x09,0x00,0x0B,0xC0,0x3D,0x00,0x0B,0xC0,0x3D,0x00,0x09,0x00},/*"#",3*/
{0x18,0xC0,0x24,0x40,0x7F,0xE0,0x22,0x40,0x31,0x80,0x00,0x00},/*"$",4*/
{0x18,0x00,0x24,0xC0,0x1B,0x00,0x0D,0x80,0x32,0x40,0x01,0x80},/*"%",5*/
{0x03,0x80,0x1C,0x40,0x27,0x40,0x1C,0x80,0x07,0x40,0x00,0x40},/*"&",6*/
{0x10,0x00,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"'",7*/
{0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x80,0x20,0x40,0x40,0x20},/*"(",8*/
{0x00,0x00,0x40,0x20,0x20,0x40,0x1F,0x80,0x00,0x00,0x00,0x00},/*")",9*/
{0x09,0x00,0x06,0x00,0x1F,0x80,0x06,0x00,0x09,0x00,0x00,0x00},/*"*",10*/
{0x04,0x00,0x04,0x00,0x3F,0x80,0x04,0x00,0x04,0x00,0x00,0x00},/*"+",11*/
{0x00,0x10,0x00,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*",",12*/
{0x04,0x00,0x04,0x00,0x04,0x00,0x04,0x00,0x04,0x00,0x00,0x00},/*"-",13*/
{0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*".",14*/
{0x00,0x20,0x01,0xC0,0x06,0x00,0x38,0x00,0x40,0x00,0x00,0x00},/*"/",15*/
{0x1F,0x80,0x20,0x40,0x20,0x40,0x20,0x40,0x1F,0x80,0x00,0x00},/*"0",16*/
{0x00,0x00,0x10,0x40,0x3F,0xC0,0x00,0x40,0x00,0x00,0x00,0x00},/*"1",17*/
{0x18,0xC0,0x21,0x40,0x22,0x40,0x24,0x40,0x18,0x40,0x00,0x00},/*"2",18*/
{0x10,0x80,0x20,0x40,0x24,0x40,0x24,0x40,0x1B,0x80,0x00,0x00},/*"3",19*/
{0x02,0x00,0x0D,0x00,0x11,0x00,0x3F,0xC0,0x01,0x40,0x00,0x00},/*"4",20*/
{0x3C,0x80,0x24,0x40,0x24,0x40,0x24,0x40,0x23,0x80,0x00,0x00},/*"5",21*/
{0x1F,0x80,0x24,0x40,0x24,0x40,0x34,0x40,0x03,0x80,0x00,0x00},/*"6",22*/
{0x30,0x00,0x20,0x00,0x27,0xC0,0x38,0x00,0x20,0x00,0x00,0x00},/*"7",23*/
{0x1B,0x80,0x24,0x40,0x24,0x40,0x24,0x40,0x1B,0x80,0x00,0x00},/*"8",24*/
{0x1C,0x00,0x22,0xC0,0x22,0x40,0x22,0x40,0x1F,0x80,0x00,0x00},/*"9",25*/
{0x00,0x00,0x00,0x00,0x08,0x40,0x00,0x00,0x00,0x00,0x00,0x00},/*":",26*/
{0x00,0x00,0x00,0x00,0x04,0x60,0x00,0x00,0x00,0x00,0x00,0x00},/*";",27*/
{0x00,0x00,0x04,0x00,0x0A,0x00,0x11,0x00,0x20,0x80,0x40,0x40},/*"<",28*/
{0x09,0x00,0x09,0x00,0x09,0x00,0x09,0x00,0x09,0x00,0x00,0x00},/*"=",29*/
{0x00,0x00,0x40,0x40,0x20,0x80,0x11,0x00,0x0A,0x00,0x04,0x00},/*">",30*/
{0x18,0x00,0x20,0x00,0x23,0x40,0x24,0x00,0x18,0x00,0x00,0x00},/*"?",31*/
{0x1F,0x80,0x20,0x40,0x27,0x40,0x29,0x40,0x1F,0x40,0x00,0x00},/*"@",32*/
{0x00,0x40,0x07,0xC0,0x39,0x00,0x0F,0x00,0x01,0xC0,0x00,0x40},/*"A",33*/
{0x20,0x40,0x3F,0xC0,0x24,0x40,0x24,0x40,0x1B,0x80,0x00,0x00},/*"B",34*/
{0x1F,0x80,0x20,0x40,0x20,0x40,0x20,0x40,0x30,0x80,0x00,0x00},/*"C",35*/
{0x20,0x40,0x3F,0xC0,0x20,0x40,0x20,0x40,0x1F,0x80,0x00,0x00},/*"D",36*/
{0x20,0x40,0x3F,0xC0,0x24,0x40,0x2E,0x40,0x30,0xC0,0x00,0x00},/*"E",37*/
{0x20,0x40,0x3F,0xC0,0x24,0x40,0x2E,0x00,0x30,0x00,0x00,0x00},/*"F",38*/
{0x0F,0x00,0x10,0x80,0x20,0x40,0x22,0x40,0x33,0x80,0x02,0x00},/*"G",39*/
{0x20,0x40,0x3F,0xC0,0x04,0x00,0x04,0x00,0x3F,0xC0,0x20,0x40},/*"H",40*/
{0x20,0x40,0x20,0x40,0x3F,0xC0,0x20,0x40,0x20,0x40,0x00,0x00},/*"I",41*/
{0x00,0x60,0x20,0x20,0x20,0x20,0x3F,0xC0,0x20,0x00,0x20,0x00},/*"J",42*/
{0x20,0x40,0x3F,0xC0,0x24,0x40,0x0B,0x00,0x30,0xC0,0x20,0x40},/*"K",43*/
{0x20,0x40,0x3F,0xC0,0x20,0x40,0x00,0x40,0x00,0x40,0x00,0xC0},/*"L",44*/
{0x3F,0xC0,0x3C,0x00,0x03,0xC0,0x3C,0x00,0x3F,0xC0,0x00,0x00},/*"M",45*/
{0x20,0x40,0x3F,0xC0,0x0C,0x40,0x23,0x00,0x3F,0xC0,0x20,0x00},/*"N",46*/
{0x1F,0x80,0x20,0x40,0x20,0x40,0x20,0x40,0x1F,0x80,0x00,0x00},/*"O",47*/
{0x20,0x40,0x3F,0xC0,0x24,0x40,0x24,0x00,0x18,0x00,0x00,0x00},/*"P",48*/
{0x1F,0x80,0x21,0x40,0x21,0x40,0x20,0xE0,0x1F,0xA0,0x00,0x00},/*"Q",49*/
{0x20,0x40,0x3F,0xC0,0x24,0x40,0x26,0x00,0x19,0xC0,0x00,0x40},/*"R",50*/
{0x18,0xC0,0x24,0x40,0x24,0x40,0x22,0x40,0x31,0x80,0x00,0x00},/*"S",51*/
{0x30,0x00,0x20,0x40,0x3F,0xC0,0x20,0x40,0x30,0x00,0x00,0x00},/*"T",52*/
{0x20,0x00,0x3F,0x80,0x00,0x40,0x00,0x40,0x3F,0x80,0x20,0x00},/*"U",53*/
{0x20,0x00,0x3E,0x00,0x01,0xC0,0x07,0x00,0x38,0x00,0x20,0x00},/*"V",54*/
{0x38,0x00,0x07,0xC0,0x3C,0x00,0x07,0xC0,0x38,0x00,0x00,0x00},/*"W",55*/
{0x20,0x40,0x39,0xC0,0x06,0x00,0x39,0xC0,0x20,0x40,0x00,0x00},/*"X",56*/
{0x20,0x00,0x38,0x40,0x07,0xC0,0x38,0x40,0x20,0x00,0x00,0x00},/*"Y",57*/
{0x30,0x40,0x21,0xC0,0x26,0x40,0x38,0x40,0x20,0xC0,0x00,0x00},/*"Z",58*/
{0x00,0x00,0x00,0x00,0x7F,0xE0,0x40,0x20,0x40,0x20,0x00,0x00},/*"[",59*/
{0x00,0x00,0x70,0x00,0x0C,0x00,0x03,0x80,0x00,0x40,0x00,0x00},/*"\",60*/
{0x00,0x00,0x40,0x20,0x40,0x20,0x7F,0xE0,0x00,0x00,0x00,0x00},/*"]",61*/
{0x00,0x00,0x20,0x00,0x40,0x00,0x20,0x00,0x00,0x00,0x00,0x00},/*"^",62*/
{0x00,0x10,0x00,0x10,0x00,0x10,0x00,0x10,0x00,0x10,0x00,0x10},/*"_",63*/
{0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"`",64*/
{0x00,0x00,0x02,0x80,0x05,0x40,0x05,0x40,0x03,0xC0,0x00,0x40},/*"a",65*/
{0x20,0x00,0x3F,0xC0,0x04,0x40,0x04,0x40,0x03,0x80,0x00,0x00},/*"b",66*/
{0x00,0x00,0x03,0x80,0x04,0x40,0x04,0x40,0x06,0x40,0x00,0x00},/*"c",67*/
{0x00,0x00,0x03,0x80,0x04,0x40,0x24,0x40,0x3F,0xC0,0x00,0x40},/*"d",68*/
{0x00,0x00,0x03,0x80,0x05,0x40,0x05,0x40,0x03,0x40,0x00,0x00},/*"e",69*/
{0x00,0x00,0x04,0x40,0x1F,0xC0,0x24,0x40,0x24,0x40,0x20,0x00},/*"f",70*/
{0x00,0x00,0x02,0xE0,0x05,0x50,0x05,0x50,0x06,0x50,0x04,0x20},/*"g",71*/
{0x20,0x40,0x3F,0xC0,0x04,0x40,0x04,0x00,0x03,0xC0,0x00,0x40},/*"h",72*/
{0x00,0x00,0x04,0x40,0x27,0xC0,0x00,0x40,0x00,0x00,0x00,0x00},/*"i",73*/
{0x00,0x10,0x00,0x10,0x04,0x10,0x27,0xE0,0x00,0x00,0x00,0x00},/*"j",74*/
{0x20,0x40,0x3F,0xC0,0x01,0x40,0x07,0x00,0x04,0xC0,0x04,0x40},/*"k",75*/
{0x20,0x40,0x20,0x40,0x3F,0xC0,0x00,0x40,0x00,0x40,0x00,0x00},/*"l",76*/
{0x07,0xC0,0x04,0x00,0x07,0xC0,0x04,0x00,0x03,0xC0,0x00,0x00},/*"m",77*/
{0x04,0x40,0x07,0xC0,0x04,0x40,0x04,0x00,0x03,0xC0,0x00,0x40},/*"n",78*/
{0x00,0x00,0x03,0x80,0x04,0x40,0x04,0x40,0x03,0x80,0x00,0x00},/*"o",79*/
{0x04,0x10,0x07,0xF0,0x04,0x50,0x04,0x40,0x03,0x80,0x00,0x00},/*"p",80*/
{0x00,0x00,0x03,0x80,0x04,0x40,0x04,0x50,0x07,0xF0,0x00,0x10},/*"q",81*/
{0x04,0x40,0x07,0xC0,0x02,0x40,0x04,0x00,0x04,0x00,0x00,0x00},/*"r",82*/
{0x00,0x00,0x06,0x40,0x05,0x40,0x05,0x40,0x04,0xC0,0x00,0x00},/*"s",83*/
{0x00,0x00,0x04,0x00,0x1F,0x80,0x04,0x40,0x00,0x40,0x00,0x00},/*"t",84*/
{0x04,0x00,0x07,0x80,0x00,0x40,0x04,0x40,0x07,0xC0,0x00,0x40},/*"u",85*/
{0x04,0x00,0x07,0x00,0x04,0xC0,0x01,0x80,0x06,0x00,0x04,0x00},/*"v",86*/
{0x06,0x00,0x01,0xC0,0x07,0x00,0x01,0xC0,0x06,0x00,0x00,0x00},/*"w",87*/
{0x04,0x40,0x06,0xC0,0x01,0x00,0x06,0xC0,0x04,0x40,0x00,0x00},/*"x",88*/
{0x04,0x10,0x07,0x10,0x04,0xE0,0x01,0x80,0x06,0x00,0x04,0x00},/*"y",89*/
{0x00,0x00,0x04,0x40,0x05,0xC0,0x06,0x40,0x04,0x40,0x00,0x00},/*"z",90*/
{0x00,0x00,0x00,0x00,0x04,0x00,0x7B,0xE0,0x40,0x20,0x00,0x00},/*"{",91*/
{0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xF0,0x00,0x00,0x00,0x00},/*"|",92*/
{0x00,0x00,0x40,0x20,0x7B,0xE0,0x04,0x00,0x00,0x00,0x00,0x00},/*"}",93*/
{0x40,0x00,0x80,0x00,0x40,0x00,0x20,0x00,0x20,0x00,0x40,0x00},/*"~",94*/
};


#endif
