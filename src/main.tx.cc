#include <bitset>
#include <iostream>
#include <unistd.h>
#include <vector>
#include "cc1120.h"
#include "config/lrm_470_cfg.h"
/**
 * This program is based off ref/.../cc1120_lrm_tx.c. It just configures
 * the cc1120 in long range mode and sends dummy packets.
 *
 * TODO::
 *   - Set up some logging (Boost logger?).
 */

int main()
{
    std::cout << "Initializing Driver." << std::endl;
    cc1120 driver(LRM_470_CFG);

    std::cout << "Forcing fixed packet length in TX." << std::endl;
    driver.write_register(
        Register::PKT_CFG0,
        (driver.read_register(Register::PKT_CFG0) & 0x9F) | 0x00
    );

    std::cout << "Calibrating according to the CC112x errata." << std::endl;
    driver.calibrate();

    uint8_t seqnum = 0;
    while (true) {
        // TODO: Packet class that abstracts this stuff?
        std::vector<uint8_t> pkt = {
            ++seqnum,  // Sequence number.
            0x55,      // Dummy byte.
        };
        std::cout << "Sending packet " << (uint16_t)seqnum << "..." << std::endl;
        // Write both packets to TX FIFO.
        driver.write_register(Register::FIFO, pkt);
        // Configure a packet size of 2.
        driver.write_register(Register::PKT_LEN, 0x02);
        // Write the sync word.
        driver.write_register(Register::SYNC3, 0x26);
        driver.write_register(Register::SYNC2, 0x33);
        driver.write_register(Register::SYNC1, 0xD9);
        driver.write_register(Register::SYNC0, 0xCC);
        // Strobe TX to send packet 1.
        // ---------------------------------------------------------------
        // | 0xAA 0xAA 0xAA |  0x26 0x33 0xD9 0xCC | Seq. 0x55 | CRC CRC |
        // ---------------------------------------------------------------
        std::cout << "  FIFO write status: "
                  << std::bitset<8>(driver.strobe_command(Strobe::STX))
                  << std::endl;

        // TODO: It should really be waiting for the chip to say it's done
        // sending but I don't know how to do that!
        sleep(1);
    }
}
