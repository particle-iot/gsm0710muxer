/*
 * Copyright 2018 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GSM0710_MUXER_DEF_H
#define GSM0710_MUXER_DEF_H

#include <cstddef>

namespace gsm0710 {

#ifndef GSM0710_ERROR_OFFSET
#define GSM0710_ERROR_OFFSET (-60000)
#endif // GSM0710_ERROR_OFFSET

#define GSM0710_ERROR(idx, name) GSM0710_ERROR_ ## name = (GSM0710_ERROR_OFFSET - idx)

enum {
    GSM0710_ERROR_NONE = 0,
    GSM0710_ERROR(0, UNKNOWN),
    GSM0710_ERROR(1, INVALID_STATE),
    GSM0710_ERROR(2, TIMEOUT),
    GSM0710_ERROR(3, INVALID_ARGUMENT),
    GSM0710_ERROR(4, NO_MEMORY),
    GSM0710_ERROR(5, NOT_FOUND),
    GSM0710_ERROR(6, BAD_DATA),
    GSM0710_ERROR(7, TRY_AGAIN),
    GSM0710_ERROR(8, FLOW_CONTROL)
};

namespace proto {

enum Address {
    CR = 0x02  // C/R: Command/Response bit
};

enum Extension {
    EA = 0x01, // EA: Extension bit
};

enum FrameType {
    // Table 2: Coding of Control Field
    SABM = 0x2f, // Set Asynchronous Balanced Mode
    UA   = 0x63, // Unnumbered Acknowledgement
    DM   = 0x04, // Disconnected Mode
    DISC = 0x43, // Disconnect
    UIH  = 0xef, // Unnumbered Information with Header check
    UI   = 0x03, // Unnumbered Information
    PF   = 0x10  // P/F: Poll/Final bit
};

enum ControlChannelCommand {
    // 5.4.6.3 Message Type and Actions
    PN    = 0x80, // 5.4.6.3.1  DLC parameter negotiation (PN)
    PSC   = 0x40, // 5.4.6.3.2  Power Saving Control (PSC)
    CLD   = 0xc0, // 5.4.6.3.3  Multiplexer close down (CLD)
    TEST  = 0x20, // 5.4.6.3.4  Test Command (Test)
    FCON  = 0xa0, // 5.4.6.3.5  Flow Control On Command (FCon)
    FCOFF = 0x60, // 5.4.6.3.6  Flow Control Off Command (FCoff)
    MSC   = 0xe0, // 5.4.6.3.7  Modem Status Command (MSC)
    NSC   = 0x10, // 5.4.6.3.8  Non Supported Command Response (NSC)
    RPN   = 0x90, // 5.4.6.3.9  Remote Port Negotiation Command (RPN)
    RLS   = 0x50, // 5.4.6.3.10 Remote Line Status Command (RLS)
    SNC   = 0xd0  // 5.4.6.3.11 Service Negotiation Command (SNC)
};

enum Frame {
    BASIC_FLAG      = 0xf9, // 5.2.6   Basic Option
    ADVANCED_FLAG   = 0x7e, // 5.2.7.4 Frame Structure
    CONTROL_ESCAPE  = 0x7d, // 5.2.7.1 Control-octet transparency
    CONTROL_RESTORE = 0x20
};

enum V24Signals {
    // Figure 10: Format of control signal octet
    // NOTE: >> 1
    FC  = 0x01, // Flow Control (FC)
    RTC = 0x02, // Ready To Communicate (RTC)
    RTR = 0x04, // Ready To Receive (RTR)
    IC  = 0x20, // Incoming call indicator (IC)
    DV  = 0x40  // Data Valid (DV)
};

enum SoftwareFlowControl {
    XON  = 0x11,
    XOFF = 0x13
};

enum ConvergenceLayer {
    UNSTRUCTURED                   = 1, // 5.5.1 Type 1 - Unstructured Octet Stream
    UNSTRUCTURED_WITH_FLOW_CONTROL = 2, // 5.5.2 Type 2 - Unstructured Octet Stream with flow control,
                                        // break signal handling and transmission of V.24 signal states
    UNINTERRUPTIBLE_FRAMED         = 3, // 5.5.3 Type 3 - Uninterruptible Framed Data
    INTERRUPTIBLE_FRAMED           = 4, // 5.5.4 Type 4 - Interruptible Framed Data
};

const unsigned int DEFAULT_T1 = 100; // 5.7.1 Acknowledgement Timer (T1), milliseconds
const unsigned int DEFAULT_T2 = 300; // 5.7.5 Response Timer for multiplexer control channel (T2), milliseconds
const unsigned int DEFAULT_N1 = 64;  // 5.7.2 Maximum Frame Size (N1)
const unsigned int DEFAULT_N2 = 3;   // 5.7.3 Maximum number of retransmissions (N2)

const size_t MAX_CONTROL_REQUEST_DATA = 16;
const size_t MAX_CHANNELS = 5;

} // proto

} // gsm0710

#endif // GSM0710_MUXER_DEF_H
