#pragma once

enum Strobe {
    // Reset chip
    SRES = 0x30,
    // Enable and calibrate frequency synthesizer (if SETTLING_CFG.FS_AUTOCAL = 1).
    // If in RX and PKT_CFG2.CCA_MODE ≠ 0: Go to a wait state where only the
    // synthesizer is running (for quick RX/TX turnaround).
    SFSTXON = 0x31,
    // Enter XOFF state when CSn is de-asserted
    SXOFF = 0x32,
    // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from
    // IDLE mode without setting manual calibration mode (SETTLING_CFG.FS_AUTOCAL = 0)
    SCAL = 0x33,
    // Enable RX. Perform calibration first if coming from IDLE and
    // SETTLING_CFG.FS_AUTOCAL = 1
    SRX = 0x34,
    // In IDLE state: Enable TX. Perform calibration first if
    // SETTLING_CFG.FS_AUTOCAL = 1. If in RX state and PKT_CFG2.CCA_MODE ≠ 0: Only
    // go to TX if channel is clear
    STX = 0x35,
    // Exit RX/TX, turn off frequency synthesizer and exit eWOR mode if applicable
    SIDLE = 0x36,
    // Automatic Frequency Compensation
    SAFC = 0x37,
    // Start automatic RX polling sequence (eWOR) as described in Section 9.6
    // if WOR_CFG0.RC_PD = 0
    SWOR = 0x38,
    // Enter SLEEP mode when CSn is de-asserted
    SPWD = 0x39,
    // Flush the RX FIFO. Only issue SFRX in IDLE or RX_FIFO_ERR states
    SFRX = 0x3A,
    // Flush the TX FIFO. Only issue SFTX in IDLE or TX_FIFO_ERR states
    SFTX = 0x3B,
    // Reset the eWOR timer to the Event1 value
    SWORRST = 0x3C,
    // No operation. May be used to get access to the chip status byte
    SNOP = 0x3D
};
