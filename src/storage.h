/*************************************************************************
**  AVR J1850VPW VPW Interface
**  by Stepan Matafonov
**
**  Released under Microsoft Public License
**
**  contact: matafonoff@gmail.com
**  homepage: xelb.ru
**************************************************************************/
#pragma once
#include "common.h"

#ifndef STORAGE_SIZE
#define STORAGE_SIZE 60
#endif

class StorageItem
{
public:
    uint8_t content[BS];
    uint8_t size;
};

class Storage
{
    volatile int8_t _first;
    volatile int8_t _last;
    StorageItem _items[STORAGE_SIZE];

public:
    Storage();

    void push(uint8_t *buff, uint8_t size);
    uint8_t tryPopItem(uint8_t *pBuff);
};