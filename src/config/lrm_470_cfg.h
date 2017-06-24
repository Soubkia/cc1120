#pragma once
#include "config.h"
#include "../enum/register.h"
#include "../enum/ext_register.h"

extern const Config LRM_470_CFG = {
    {
        {Register::IOCFG3,            0xB0},
        {Register::IOCFG2,            0x06},
        {Register::IOCFG1,            0xB0},
        {Register::IOCFG0,            0x40},
        {Register::SYNC_CFG1,         0x08},
        {Register::DEVIATION_M,       0x89},
        {Register::MODCFG_DEV_E,      0x09},
        {Register::DCFILT_CFG,        0x1C},
        {Register::IQIC,              0xC6},
        {Register::CHAN_BW,           0x50},
        {Register::SYMBOL_RATE2,      0x33},
        {Register::AGC_REF,           0x20},
        {Register::AGC_CS_THR,        0x19},
        {Register::AGC_CFG1,          0xA9},
        {Register::AGC_CFG0,          0xCF},
        {Register::FIFO_CFG,          0x00},
        {Register::SETTLING_CFG,      0x03},
        {Register::FS_CFG,            0x14},
        {Register::PKT_CFG0,          0x20},
        {Register::PA_CFG0,           0x7E},
        {Register::PKT_LEN,           0xFF},
    },
    {
        {ExtRegister::IF_MIX_CFG,        0x00},
        {ExtRegister::FREQOFF_CFG,       0x30},
        {ExtRegister::FREQ2,             0x75},
        {ExtRegister::FREQ1,             0x80},
        {ExtRegister::FS_DIG1,           0x00},
        {ExtRegister::FS_DIG0,           0x5F},
        {ExtRegister::FS_CAL1,           0x40},
        {ExtRegister::FS_CAL0,           0x0E},
        {ExtRegister::FS_DIVTWO,         0x03},
        {ExtRegister::FS_DSM0,           0x33},
        {ExtRegister::FS_DVC0,           0x17},
        {ExtRegister::FS_PFD,            0x50},
        {ExtRegister::FS_PRE,            0x6E},
        {ExtRegister::FS_REG_DIV_CML,    0x14},
        {ExtRegister::FS_SPARE,          0xAC},
        {ExtRegister::FS_VCO0,           0xB4},
        {ExtRegister::LNA,               0x03},
        {ExtRegister::XOSC5,             0x0E},
        {ExtRegister::XOSC1,             0x03},
    }
};
