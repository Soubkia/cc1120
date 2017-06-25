#include <iostream>
#include <unistd.h>
#include <vector>
#include "cc1120.h"
#include "config/lrm_470_cfg.h"
/**
 * This program is based off ref/.../cc1120_lrm_rx.c. It runs the long range
 * mode algorithm described here::
 *     http://processors.wiki.ti.com/index.php/Cc112x_cc120x_lrm
 *
 * TODO::
 * - The reference doesn't use frequency compensation for 470MHz 
 *   but I don't understand enough about this technique to know why.
 * - Read the received signal strength indicator (RSSI) at each
 *   iteration.
 */

int main()
{
    std::cout << "Initializing Driver." << std::endl;
    cc1120 driver(LRM_470_CFG);

    std::cout << "Forcing fixed packet length in RX." << std::endl;
    driver.write_register(
        Register::PKT_CFG0,
        (driver.read_register(Register::PKT_CFG0) & 0x9F) | 0x00
    );

    std::cout << "Calibrating according to the CC112x errata." << std::endl;
    driver.calibrate();

    // The transmitter sends with a peroid of 322ms.
    static const uint16_t timeout = 322;
    static const uint8_t pkt_len = 0x03;
    static uint16_t packets_received = 0;
    static uint16_t packets_lost = 0;

    while (true) {
        // Set a 3 byte fixed packet length.
        driver.write_register(Register::PKT_LEN, pkt_len);
        // Write the second sync word.
        driver.write_register(Register::SYNC3, 0x93);
        driver.write_register(Register::SYNC2, 0x0B);
        driver.write_register(Register::SYNC1, 0x51);
        driver.write_register(Register::SYNC0, 0xDE);
        // Set RX filter bandwidth to 7.8kHz.
        driver.write_register(Register::CHAN_BW, 0x50);
        // Strobe RX to read the packet.
        driver.strobe_command(Strobe::SRX);
        // Wait for MARCSTATE to idle.
        // 0x6D -> 01101(RX) 10(IDLE) 1(NOT USED).
        while (driver.read_register(ExtRegister::MARCSTATE) != 0x6D);
        // Read from the RX FIFO.
        std::cout << "Checking the RX FIFO..." << std::endl;
        std::vector<uint8_t> pkt = driver.read_register(Register::FIFO,
                                                        pkt_len + 2);
        if (pkt[pkt_len + 1] & 0x80) {
            // CRC Ok -- We caught one!
            packets_received++;
            std::cout << "  packet: [ ";
        }
        else {
            // Lost to the ether... R.I.P. buddy.
            packets_lost++;
            // TODO: This isn't actually true. It might show up sometime
            //     soon. We should set a timer and increment this after some
            //     timeout is exceeded.

            // Exit RX mode, turn off the frequency synthesizer, and exit eWOR
            // mode if applicable.
            driver.strobe_command(Strobe::SIDLE);
            // Flush the RX FIFO.
            driver.strobe_command(Strobe::SFRX);
            std::cout << "  junk: [ ";
        }
        for (uint8_t byte : pkt) {
            std::cout << std::hex << (uint16_t)byte << " ";
        }
        std::cout << "]" << std::endl;
        // TODO: Interrupt handler.
        sleep(1);
    }
}
