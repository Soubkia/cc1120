#pragma once
#include <stdint.h>

#include "config/config.h"
#include "enum/ext_register.h"
#include "enum/register.h"
#include "enum/strobe.h"

class cc1120 {
  public:
    cc1120();
    cc1120(Config cfg);
    uint8_t read_register(Register addr);
    uint8_t read_register(ExtRegister addr);
    uint8_t write_register(Register addr, uint8_t val);
    uint8_t write_register(Register addr, uint8_t *vals);
    uint8_t write_register(ExtRegister addr, uint8_t val);
    // TODO: Decide what I'm doing with burst access.
    //uint8_t* write_register(ExtRegister addr, uint8_t *vals);
    uint8_t strobe_command(Strobe cmd);
    void configure(Config cfg);
    void calibrate();
};
