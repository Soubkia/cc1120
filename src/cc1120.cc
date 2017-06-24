#include <cstring>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include "bcm2835.h"
#include "cc1120.h"

const uint8_t READ      = 0b10000000;
const uint8_t WRITE     = 0b00000000;
const uint8_t BURST_ON  = 0b01000000;
const uint8_t BURST_OFF = 0b00000000;

cc1120::cc1120()
{
    if (!bcm2835_init()) {
        fprintf(stderr, "Unable to initialize bcm2835 device driver: %s\n",
                strerror(errno));
        exit(1);
    }
    const uint8_t reset_pin = 22; // TODO: Expose this to the user.
    bcm2835_gpio_set_pud(reset_pin, BCM2835_GPIO_PUD_OFF);
    bcm2835_gpio_fsel(reset_pin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(reset_pin, HIGH);

    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // default
}

uint8_t cc1120::read_register(Register address)
{
    uint8_t addr = address;
    addr |= READ;
    addr |= BURST_OFF;
    uint8_t buf[2] = {addr, 0x00};
    bcm2835_spi_transfern((char *)buf, sizeof(buf));
    return buf[1];
}

uint8_t cc1120::read_register(ExtRegister address)
{
    uint8_t addr = address;
    uint8_t header = Register::EXTENDED_ADDRESS;
    header |= READ;
    header |= BURST_OFF;
    uint8_t buf[3] = {header, addr, 0x00};
    bcm2835_spi_transfern((char *)buf, sizeof(buf));
    return buf[2];
}

uint8_t cc1120::write_register(Register address, uint8_t val)
{
    uint8_t addr = address;
    addr |= WRITE;
    addr |= BURST_OFF;
    uint8_t buf[3] = {addr, val, 0x00};
    bcm2835_spi_transfern((char *)buf, sizeof(buf));
    return buf[2];
}

uint8_t cc1120::write_register(ExtRegister address, uint8_t val)
{
    uint8_t addr = address;
    uint8_t header = Register::EXTENDED_ADDRESS;
    header |= WRITE;
    header |= BURST_OFF;
    uint8_t buf[4] = {header, addr, val, 0x00};
    bcm2835_spi_transfern((char *)buf, sizeof(buf));
    return buf[3];
}

uint8_t cc1120::strobe_command(Strobe command)
{
    uint8_t cmd = command;
    uint8_t buf[2] = {cmd, 0x00};
    bcm2835_spi_transfern((char *)buf, sizeof(buf));
    return buf[1];
}

void cc1120::configure(Config cfg)
{
    strobe_command(Strobe::SRES);
    for (auto& pair : cfg.register_settings_) {
        write_register(pair.first, pair.second);
    }
    for (auto& pair: cfg.ext_register_settings_) {
        write_register(pair.first, pair.second);
    }
}

void cc1120::calibrate()
{
    // This function calibrates the radio according to the CC112x errata.
    // 1) Set VCO (Voltage Controlled Oscillator) cap-array to 0.
    write_register(ExtRegister::FS_VCO2, 0x00);
    // 2) Start with high VCDAC (original VCDAC_START + 2).
    uint8_t fs_cal2 = read_register(ExtRegister::FS_CAL2) + 2;
    write_register(ExtRegister::FS_CAL2, fs_cal2);
    // 3) Calibrate and wait for calibration to be done.
    //    (radio back in IDLE state)
    strobe_command(Strobe::SCAL);
    while (true) {
        // TODO: Enumify MARC states... 0x41 is IDLE.
        if (read_register(ExtRegister::MARCSTATE) == 0x41) break;
    }
    // 4) Read FS_VCO2, FSVO4 and FS_CHP registers obtained with
    //    high VCDAC_START value.

}
