// Copyright 2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define W6100_SYSR_CHPL_LOCK	(1<<7)
#define W6100_SYSR_CHPL_ULOCK	(0<<7)


#define W6100_SYSR_NETL_LOCK	(1<<6)
#define W6100_SYSR_NETL_ULOCK	(0<<6)


#define W6100_SYSR_PHYL_LOCK	(1<<5)
#define W6100_SYSR_PHYL_ULOCK	(0<<5)





#define W6100_CHPLCKR_UNLOCK	0xCE
#define W6100_CHPLCKR_LOCK  	0xFF

#define W6100_NETLCKR_UNLOCK	0x3A
#define W6100_NETLCKR_LOCK	    0xC5

#define W6100_PHYLCKR_UNLOCK	0x53
#define W6100_PHYLCKR_LOCK	    0xFF





#define W5500_ADDR_OFFSET (16) // Address length
#define W5500_BSB_OFFSET  (3)  // Block Select Bits offset
#define W5500_RWB_OFFSET  (2)  // Read Write Bits offset

#define W5500_BSB_COM_REG        (0x00)    // Common Register
#define W5500_BSB_SOCK_REG(s)    ((s)*4+1) // Socket Register
#define W5500_BSB_SOCK_TX_BUF(s) ((s)*4+2) // Socket TX Buffer
#define W5500_BSB_SOCK_RX_BUF(s) ((s)*4+3) // Socket RX Buffer

#define W5500_ACCESS_MODE_READ  (0) // Read Mode
#define W5500_ACCESS_MODE_WRITE (1) // Write Mode

#define W5500_SPI_OP_MODE_VDM   (0x00) // Variable Data Length Mode (SPI frame is controlled by CS line)
#define W5500_SPI_OP_MODE_FDM_1 (0x01) // Fixed Data Length Mode, 1 Byte Length
#define W5500_SPI_OP_MODE_FDM_2 (0x02) // Fixed Data Length Mode, 2 Bytes Length
#define W5500_SPI_OP_MODE_FDM_4 (0x03) // Fixed Data Length Mode, 4 Bytes Length

#define W5500_MAKE_MAP(offset, bsb) ((offset) << W5500_ADDR_OFFSET | (bsb) << W5500_BSB_OFFSET)

//
// // Common REGISTER BLOCK side 29
//https://cdn.sparkfun.com/datasheets/Dev/Arduino/Shields/W5500_datasheet_v1.0.2_1.pdf

#define W6100_REG_4MR        W5500_MAKE_MAP(0x4000, W5500_BSB_COM_REG) // 
#define W6100_REG_6MR        W5500_MAKE_MAP(0x4004, W5500_BSB_COM_REG) // 
#define W6100_REG_NETMR        W5500_MAKE_MAP(0x4008, W5500_BSB_COM_REG) // 

#define W6100_CHPLCKR           W5500_MAKE_MAP(0x41F4, W5500_BSB_COM_REG)
#define W6100_NETLCKR           W5500_MAKE_MAP(0x41F5, W5500_BSB_COM_REG)
#define W6100_PHYLCKR           W5500_MAKE_MAP(0x41F6, W5500_BSB_COM_REG)
#define SYSR_W6100              W5500_MAKE_MAP(0x2000, W5500_BSB_COM_REG)
#define W6100_SYCR0         W5500_MAKE_MAP(0x2004, W5500_BSB_COM_REG) // Mode
#define W6100_SYCR1         W5500_MAKE_MAP(0x2005, W5500_BSB_COM_REG) // Mode
#define W6100_REG_MAC       W5500_MAKE_MAP(0x4120, W5500_BSB_COM_REG) // MAC Address




#define W6100_REG_SIMR      W5500_MAKE_MAP(0x2114, W5500_BSB_COM_REG) // Socket Interrupt Mask
#define W6100_REG_PHYSR   W5500_MAKE_MAP(0x3000, W5500_BSB_COM_REG) // PHY STATUS register
#define W6100_REG_PHYCR0   W5500_MAKE_MAP(0x301C, W5500_BSB_COM_REG) // PHY Control register 0
#define W6100_REG_PHYCR1   W5500_MAKE_MAP(0x301D, W5500_BSB_COM_REG) // PHY Control register 0




#define W6100_REG_VERSIONR  W5500_MAKE_MAP(0x0000, W5500_BSB_COM_REG) // Chip version               // måske alternativ 0x0002



//
// SOCKET REGISTER BLOCK side 30
// https://cdn.sparkfun.com/datasheets/Dev/Arduino/Shields/W5500_datasheet_v1.0.2_1.pdf

#define W6100_REG_SOCK_MR(s)         W5500_MAKE_MAP(0x0000, W5500_BSB_SOCK_REG(s)) // Socket Mode
#define W6100_REG_SOCK_CR(s)         W5500_MAKE_MAP(0x0010, W5500_BSB_SOCK_REG(s)) // Socket Command    // Måske fel i (s)
#define W6100_REG_SOCK_IR(s)         W5500_MAKE_MAP(0x0020, W5500_BSB_SOCK_REG(s)) // Socket Interrupt

#define W6100_REG_SOCK_IRCLR(s)         W5500_MAKE_MAP(0x0028, W5500_BSB_SOCK_REG(s)) // Socket Interrupt


#define W6100_REG_SOCK_MR2(s)         W5500_MAKE_MAP(0x0144, W5500_BSB_SOCK_REG(s)) // Socket Status

#define W6100_REG_SOCK_RXBUF_SIZE(s) W5500_MAKE_MAP(0x0220, W5500_BSB_SOCK_REG(s)) // Socket Receive Buffer Size
#define W6100_REG_SOCK_TXBUF_SIZE(s) W5500_MAKE_MAP(0x0200, W5500_BSB_SOCK_REG(s)) // Socket Transmit Buffer Size





#define W6100_REG_SOCK_TX_FSR(s)     W5500_MAKE_MAP(0x0204, W5500_BSB_SOCK_REG(s)) // Socket TX Free Size   // Måske fel i (s)
#define W6100_REG_SOCK_TX_WR(s)      W5500_MAKE_MAP(0x020c, W5500_BSB_SOCK_REG(s)) // Socket TX Write Pointer
#define W6100_REG_SOCK_RX_RSR(s)     W5500_MAKE_MAP(0x0224, W5500_BSB_SOCK_REG(s)) // Socket RX Received Size //  Måske fel i (s)
#define W6100_REG_SOCK_RX_RD(s)      W5500_MAKE_MAP(0x0228, W5500_BSB_SOCK_REG(s)) // Socket RX Read Pointer
#define W6100_REG_SOCK_IMR(s)        W5500_MAKE_MAP(0x0024, W5500_BSB_SOCK_REG(s)) // Socket Interrupt Mask

#define W6100_MEM_SOCK_TX(s,addr) W5500_MAKE_MAP(addr, W5500_BSB_SOCK_TX_BUF(s)) // Socket TX buffer address
#define W5500_MEM_SOCK_RX(s,addr) W5500_MAKE_MAP(addr, W5500_BSB_SOCK_RX_BUF(s)) // Socket RX buffer address

#define W6100_REG_SIPR0   W5500_MAKE_MAP(0x4138, W5500_BSB_COM_REG) // PHY Control register 0


//
// REGISTER DEscription  BLOCK side 32
// https://cdn.sparkfun.com/datasheets/Dev/Arduino/Shields/W5500_datasheet_v1.0.2_1.pdf

#define W6100_MR_PB  (1<<0) // (1<<4) // Ping block (block the response to a ping request)   0001 0000      16


#define W6100_MR_RST 0// Software reset                                      1000 0000     128



#define W6100_SIMR_SOCK0 (1<<0) // Socket 0 interrupt



// Socket Registers side 44-45
// https://cdn.sparkfun.com/datasheets/Dev/Arduino/Shields/W5500_datasheet_v1.0.2_1.pdf


#define W6100_SMR_MAC_RAW   0x07    // (1<<2) // MAC RAW mode
#define W6100_SMR_MAC_FILTER (1<<7) // MAC filter
// Sn_CR (Socket n Command Register) [R/W] [0x0001] [0x00]  side 46-48

#define W6100_SCR_OPEN  (0x01) // Open command
#define W6100_SCR_CLOSE (0x10) // Close command
#define W6100_SCR_SEND  (0x20) // Send command
#define W6100_SCR_RECV  (0x40) // Recv command



#define W6100_SIR_RECV (1<<2)  // Receive done
#define W6100_SIR_SEND (1<<4)  // Send done
