//
// Created by david on 10/15/2020.
//

#ifndef GEO_00001_USB_IO_H
#define GEO_00001_USB_IO_H

#include <memory.h>

char get_usb_char();
void write_usb(const void *buf, unsigned buf_len);
void init_usb_io();
static inline void print_usb_string(const void* str) {
  write_usb(str, strlen(str));
}
static inline void put_usb_char(char ch) {
  write_usb(&ch, 1);
}

#define CRLF "\r\n"

#endif //GEO_00001_USB_IO_H
