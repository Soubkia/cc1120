#include <iostream>
//#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
//#include "bcm2835.h"
#include "cc1120.h"
#include "config/lrm_470_cfg.h"

int main()
{
    cc1120 driver;
    std::cout << "Initialized Driver" << std::endl;
    driver.configure(LRM_470_CFG);
    std::cout << "Calibrated using provided config" << std::endl;
    //if (!bcm2835_init())
        //return 1;
}
