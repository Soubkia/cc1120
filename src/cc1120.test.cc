#include <bitset>
#include <iostream>

#include "bcm2835.h"
#include "cc1120.h"
#include "enum/ext_register.h"
#include "enum/register.h"
#include "enum/strobe.h"

int main()
{
    cc1120 *driver = new cc1120();
    while (true) {
        std::cout << "STATUS: " << std::bitset<8>(driver->strobe_command(Strobe::SNOP)) << std::endl;
        std::cout << "PARTNUMBER: " << std::bitset<8>(driver->read_register(ExtRegister::PARTNUMBER)) << std::endl;
        std::cout << "FS_CFG: " << std::bitset<8>(driver->read_register(Register::FS_CFG)) << std::endl;
        delay(1000);
    }
}
