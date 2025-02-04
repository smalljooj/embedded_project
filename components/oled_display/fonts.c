#include "fonts.h"

// The Display font 

uint8_t font_8x8[950] = {
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // space
    0x02, 0xBE, 0xBE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // !
    0x05, 0x06, 0x06, 0x00, 0x06, 0x06, 0x00, 0x00, 0x00, // "
    0x08, 0x28, 0x28, 0xFE, 0x28, 0x28, 0xFE, 0x28, 0x28, // #
    0x08, 0x48, 0x54, 0x54, 0xFE, 0xFE, 0x54, 0x54, 0x24, // $
    0x06, 0x46, 0x66, 0x30, 0x18, 0xCC, 0xC4, 0x00, 0x00, // %
    0x06, 0x6C, 0x92, 0x92, 0xAA, 0x44, 0xA0, 0x00, 0x00, // &
    0x02, 0x0E, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // '
    0x04, 0x38, 0x7C, 0xC6, 0x82, 0x00, 0x00, 0x00, 0x00, // (
    0x04, 0x82, 0xC6, 0x7C, 0x38, 0x00, 0x00, 0x00, 0x00, // )
    0x05, 0x14, 0x08, 0x3E, 0x08, 0x14, 0x00, 0x00, 0x00, // *
    0x06, 0x10, 0x10, 0x7C, 0x7C, 0x10, 0x10, 0x00, 0x00, // +
    0x03, 0x80, 0xE0, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, // ,
    0x06, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, // -
    0x02, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // .
    0x08, 0x80, 0xC0, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x02, // /
    0x06, 0x7C, 0xFE, 0xA2, 0x92, 0xFE, 0x7C, 0x00, 0x00, // 0
    0x04, 0x84, 0xFE, 0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, // 1
    0x06, 0xE2, 0xF2, 0x92, 0x92, 0x9E, 0x8C, 0x00, 0x00, // 2
    0x06, 0x82, 0x92, 0x92, 0x92, 0xFE, 0x6C, 0x00, 0x00, // 3
    0x06, 0x30, 0x38, 0x2C, 0x26, 0xFE, 0xFE, 0x00, 0x00, // 4
    0x06, 0x8E, 0x8E, 0x8A, 0x8A, 0xFA, 0x72, 0x00, 0x00, // 5
    0x06, 0x78, 0xFC, 0x96, 0x92, 0xF2, 0x60, 0x00, 0x00, // 6
    0x06, 0x02, 0x02, 0xF2, 0xFA, 0x0E, 0x06, 0x00, 0x00, // 7
    0x06, 0x6C, 0xFE, 0x92, 0x92, 0xFE, 0x6C, 0x00, 0x00, // 8
    0x06, 0x0C, 0x9E, 0x92, 0xD2, 0x7E, 0x3C, 0x00, 0x00, // 9
    0x02, 0xCC, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // :
    0x03, 0x80, 0xCC, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x00, // ;
    0x04, 0x10, 0x38, 0x6C, 0x44, 0x00, 0x00, 0x00, 0x00, // <
    0x06, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00, // =
    0x04, 0x44, 0x6C, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, // >
    0x06, 0x04, 0x02, 0xB2, 0xBA, 0x0E, 0x04, 0x00, 0x00, // ?
    0x06, 0x7C, 0x82, 0xBA, 0xAE, 0xBE, 0x3C, 0x00, 0x00, // @
    0x06, 0xFC, 0xFE, 0x22, 0x22, 0xFE, 0xFC, 0x00, 0x00, // A
    0x06, 0xFE, 0xFE, 0x92, 0x92, 0xFE, 0x6C, 0x00, 0x00, // B
    0x06, 0x7C, 0xFE, 0x82, 0x82, 0x82, 0x44, 0x00, 0x00, // C
    0x06, 0xFE, 0xFE, 0x82, 0x82, 0xFE, 0x7C, 0x00, 0x00, // D
    0x06, 0xFE, 0xFE, 0x92, 0x92, 0x92, 0x92, 0x00, 0x00, // E
    0x06, 0xFE, 0xFE, 0x12, 0x12, 0x12, 0x12, 0x00, 0x00, // F
    0x06, 0x7C, 0xFE, 0x82, 0xA2, 0xE6, 0x64, 0x00, 0x00, // G
    0x06, 0xFE, 0xFE, 0x10, 0x10, 0xFE, 0xFE, 0x00, 0x00, // H
    0x04, 0x82, 0xFE, 0xFE, 0x82, 0x00, 0x00, 0x00, 0x00, // I
    0x04, 0xC0, 0x82, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, // J
    0x06, 0xFE, 0xFE, 0x10, 0x38, 0x6C, 0xC6, 0x00, 0x00, // K
    0x04, 0xFE, 0xFE, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, // L
    0x08, 0xFE, 0xFE, 0x0C, 0x18, 0x18, 0x0C, 0xFE, 0xFE, // M
    0x08, 0xFE, 0xFE, 0x0C, 0x18, 0x30, 0x60, 0xFE, 0xFE, // N
    0x08, 0x7C, 0xFE, 0x82, 0x82, 0x82, 0x82, 0xFE, 0x7C, // O
    0x06, 0xFE, 0xFE, 0x12, 0x12, 0x1E, 0x1E, 0x00, 0x00, // P
    0x08, 0x7C, 0xFE, 0x82, 0xA2, 0xA2, 0x42, 0xBE, 0xBC, // Q
    0x06, 0xFE, 0xFE, 0x22, 0x62, 0xFE, 0x9C, 0x00, 0x00, // R
    0x06, 0x8C, 0x92, 0x92, 0x92, 0x92, 0x62, 0x00, 0x00, // S
    0x06, 0x02, 0x02, 0xFE, 0xFE, 0x02, 0x02, 0x00, 0x00, // T
    0x06, 0xFE, 0xFE, 0x80, 0x80, 0xFE, 0xFE, 0x00, 0x00, // U
    0x06, 0x3E, 0x7E, 0x80, 0x80, 0x7E, 0x3E, 0x00, 0x00, // V
    0x08, 0xFE, 0xFE, 0x60, 0x30, 0x30, 0x60, 0xFE, 0xFE, // W
    0x06, 0xC6, 0xEE, 0x38, 0x38, 0xEE, 0xC6, 0x00, 0x00, // X
    0x06, 0x0E, 0x1E, 0xF0, 0xF0, 0x1E, 0x0E, 0x00, 0x00, // Y
    0x06, 0xC2, 0xE2, 0xB2, 0x9A, 0x8E, 0x86, 0x00, 0x00, // Z
    0x04, 0xFE, 0xFE, 0x82, 0x82, 0x00, 0x00, 0x00, 0x00, // [
    0x08, 0x02, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0x80, /* \  */
    0x04, 0x82, 0x82, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, // ]
    0x06, 0x08, 0x0C, 0x06, 0x06, 0x0C, 0x08, 0x00, 0x00, // ^
    0x08, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, // _
    0x04, 0x02, 0x06, 0x0C, 0x08, 0x00, 0x00, 0x00, 0x00, // `
    0x06, 0x70, 0xF8, 0x88, 0x08, 0xF8, 0xF8, 0x00, 0x00, // a
    0x06, 0xFE, 0xFE, 0x88, 0x88, 0xF8, 0x70, 0x00, 0x00, // b
    0x06, 0x70, 0xF8, 0x88, 0x88, 0x88, 0x90, 0x00, 0x00, // c
    0x06, 0x70, 0xF8, 0x88, 0x88, 0xFE, 0xFE, 0x00, 0x00, // d
    0x06, 0x70, 0xF8, 0xA8, 0xA8, 0xB8, 0x30, 0x00, 0x00, // e
    0x06, 0x88, 0xFC, 0x7E, 0x0A, 0x0A, 0x00, 0x00, 0x00, // f
    0x06, 0x1C, 0x3E, 0xA2, 0xA2, 0xFE, 0x7E, 0x00, 0x00, // g
    0x06, 0xFE, 0xFE, 0x08, 0x08, 0xF8, 0xF0, 0x00, 0x00, // h
    0x02, 0xF4, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // i
    0x04, 0x80, 0x80, 0xFA, 0x7A, 0x00, 0x00, 0x00, 0x00, // j
    0x06, 0xFE, 0xFE, 0x20, 0x70, 0xD8, 0x88, 0x00, 0x00, // k
    0x02, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // l
    0x08, 0xF8, 0xF8, 0x08, 0xF8, 0xF8, 0x08, 0xF8, 0xF0, // m
    0x06, 0xF8, 0xF8, 0x08, 0x08, 0xF8, 0xF0, 0x00, 0x00, // n
    0x06, 0x70, 0xF8, 0x88, 0x88, 0xF8, 0x70, 0x00, 0x00, // o
    0x06, 0xFC, 0xFC, 0x24, 0x24, 0x3C, 0x18, 0x00, 0x00, // p
    0x06, 0x18, 0x3C, 0x24, 0x24, 0xFC, 0xFC, 0x00, 0x00, // q
    0x04, 0xF8, 0xF0, 0x18, 0x08, 0x00, 0x00, 0x00, 0x00, // r
    0x06, 0x10, 0xB8, 0xA8, 0xA8, 0xE8, 0x40, 0x00, 0x00, // s
    0x04, 0x7C, 0xFC, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, // t
    0x06, 0xF8, 0xF8, 0x80, 0x80, 0xF8, 0x78, 0x00, 0x00, // u
    0x06, 0x38, 0x78, 0x80, 0x80, 0x78, 0x38, 0x00, 0x00, // v
    0x08, 0xF8, 0xF8, 0x80, 0xF8, 0xF8, 0x80, 0xF8, 0x78, // w
    0x06, 0x88, 0xD8, 0x70, 0x70, 0xD8, 0x88, 0x00, 0x00, // x
    0x06, 0x18, 0x38, 0x20, 0xA0, 0xF8, 0x78, 0x00, 0x00, // y
    0x06, 0xC8, 0xC8, 0xA8, 0xA8, 0x98, 0x98, 0x00, 0x00, // z
    0x03, 0x38, 0xEE, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, // {
    0x02, 0xFE, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, // |
    0x03, 0x82, 0xEE, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, // }
    0x06, 0x04, 0x06, 0x02, 0x04, 0x04, 0x06, 0x00, 0x00, // ~
};