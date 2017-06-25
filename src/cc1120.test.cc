#include <bitset>
#include <iostream>

#include "bcm2835.h"
#include "cc1120.h"
#include "enum/ext_register.h"
#include "enum/register.h"
#include "enum/strobe.h"

#include "gtest/gtest.h"

TEST(read_register, PARTNUMBER) {
    cc1120 driver;
    EXPECT_EQ(0x48, driver.read_register(ExtRegister::PARTNUMBER));
}

TEST(strobe_command, SNOP) {
    cc1120 driver;
    EXPECT_EQ(driver.strobe_command(Strobe::SNOP), 0x0F);
}
