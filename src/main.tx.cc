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
 * - Set up some logging (Boost logger?).
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

    // TODO: Should stuff like this go into the config?
    static const uint8_t pkt_len = 0x03;
    uint8_t seqnum = 0;
    while (true) {
        // So this is a little weird. They send two packets. The first is used
        // for frequency compensation and has no payload and the second is
        // the "real" packet. It looks like their receiver code does nothing
        // with the first packet for 470MHz so I have excluded it here (meaning
        // we will only send the payload packet). Hopefully that doesn't break
        // anything.
        //
        // TODO: Packet class that abstracts this stuff?
        std::vector<uint8_t> pkt = {
            ++seqnum,  // Sequence number.
            seqnum,    // Sequence number.
            0x55,      // Dummy byte.
        };
        std::cout << "Sending packet " << (uint16_t)seqnum << "..." << std::endl;
        // Write out packet to the TX FIFO.
        driver.write_register(Register::FIFO, pkt);
        // Configure a packet size of 3.
        driver.write_register(Register::PKT_LEN, pkt_len);
        // Write the second sync word.
        driver.write_register(Register::SYNC3, 0x93);
        driver.write_register(Register::SYNC2, 0x0B);
        driver.write_register(Register::SYNC1, 0x51);
        driver.write_register(Register::SYNC0, 0xDE);
        // Strobe TX to send packet.
        // -------------------------------------------------------------------
        // | 0xAA 0xAA 0xAA | 0x93 0x0B 0x51 0xDE | Seq. Seq. 0x55 | CRC CRC |
        // -------------------------------------------------------------------
        std::cout << "  payload: [ ";
        for (uint8_t byte : pkt) {
            std::cout << std::hex << (uint16_t)byte << " ";
        }
        std::cout << "]" << std::endl;
        std::cout << "  FIFO write status: "
                  << std::bitset<8>(driver.strobe_command(Strobe::STX))
                  << std::endl;

        // TODO: It should really be waiting for the chip to say it's done
        // sending but I don't know how to do that!
        //
        // I think I am supposed to connect a GPIO of the cc1120 to some
        // pin on the MCU and set up an interrupt handler that I can wait
        // for to tell me when the packet has been sent.
        sleep(1);
    }
}
