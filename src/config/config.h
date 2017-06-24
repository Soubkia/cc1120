#pragma once
#include <map>

#include "../enum/register.h"
#include "../enum/ext_register.h"

class Config {
  public:
    std::map<Register, uint8_t> register_settings_;
    std::map<ExtRegister, uint8_t> ext_register_settings_;
};
