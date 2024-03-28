/*
 * sx_1278.c
 *
 *  Created on: Sep 21, 2023
 *      Author: Khoa dong
 */

#include "sx_1278.h"
#include "sx_1278_hw.h"
#include "main.h"

/**
 * The function SX_1278_SPIWrite writes a value to a specified address using SPI communication.
 * 
 * @param module The "module" parameter is a pointer to an instance of the SX1278_t structure. This
 * structure contains the necessary information and configurations for the SX1278 module.
 * @param addr The addr parameter is the address of the register in the SX1278 module that you want to
 * write to.
 * @param value The "value" parameter in the SX_1278_SPIWrite function is the data value that you want
 * to write to the specified address in the SX1278 module.
 */
static void SX_1278_SPIWrite(SX1278_t *module, uint8_t addr, uint8_t value)
{
    SX_1278_HW_Set_NSS(module->hw, GPIO_PIN_RESET);
    SX_1278_HW_Command(module->hw, addr | 0x80, WRITE_OPTION);
    SX_1278_HW_Command(module->hw, value, WRITE_OPTION);
    SX_1278_HW_Set_NSS(module->hw, GPIO_PIN_SET);
}

/**
 * The function SX_1278_SPIRead reads a value from a specified address using SPI communication.
 * 
 * @param module A pointer to the SX1278_t structure, which contains the hardware configuration and
 * settings for the SX1278 module.
 * @param addr The addr parameter is the address of the register that you want to read from in the
 * SX1278 module.
 * 
 * @return a uint8_t value, which is an unsigned 8-bit integer.
 */
static uint8_t SX_1278_SPIRead(SX1278_t *module, uint8_t addr)
{
    uint8_t value = 0;
    SX_1278_HW_Set_NSS(module->hw, GPIO_PIN_RESET);
    SX_1278_HW_Command(module->hw, addr, WRITE_OPTION);
    value = SX_1278_HW_Command(module->hw, addr, READ_OPTION);
    SX_1278_HW_Set_NSS(module->hw, GPIO_PIN_SET);
    return value;
}

/**
 * The function SX_1278_MODE_Lora sets the mode of the SX1278 module to Lora mode.
 * 
 * @param module The parameter "module" is a pointer to a structure of type SX1278_t. This structure
 * likely contains various configuration parameters and status variables related to the SX1278 LoRa
 * module.
 */
static void SX_1278_MODE_Lora(SX1278_t *module)
{
    SX_1278_SPIWrite(module, LR_RegOpMode, 0x88);
}

/**
 * The function initializes the SX1278 module with the specified parameters.
 * 
 * @param module A pointer to an instance of the SX1278_t structure, which contains the configuration
 * parameters for the SX1278 module.
 * @param frequency The frequency parameter is the desired operating frequency of the SX1278 module. It
 * is typically specified in Hz (e.g., 433000000 for 433 MHz).
 * @param power The power parameter is used to set the transmission power of the SX1278 module. It
 * determines the output power level in dBm (decibels milliwatt) that the module will use for
 * transmitting data.
 * @param LoRa_SF LoRa_SF stands for LoRa Spreading Factor. It determines the rate at which data is
 * spread over the available bandwidth. Higher spreading factors provide longer range but lower data
 * rates. The valid values for LoRa_SF are typically between 6 and 12.
 * @param LoRa_BW LoRa_BW stands for LoRa bandwidth. It determines the bandwidth of the LoRa signal.
 * The available options for LoRa_BW are typically 125 kHz, 250 kHz, 500 kHz, etc.
 * @param LoRa_CR LoRa_CR stands for LoRa coding rate. It is a parameter used in LoRa modulation to
 * control the error correction capability of the transmitted signal. It determines the ratio of the
 * number of coding bits to the number of information bits in each transmitted symbol. The available
 * options for LoRa_CR are typically
 * @param LoRa_CRC_sum The parameter "LoRa_CRC_sum" in the function "SX_1278_Init" is used to specify
 * whether to enable or disable CRC (Cyclic Redundancy Check) in LoRa packets. CRC is a method used to
 * detect errors in data transmission. If "LoRa_CRC_sum"
 * @param packetLength The parameter "packetLength" in the function SX_1278_Init is used to specify the
 * length of the packets that will be transmitted or received by the SX1278 module. It is an 8-bit
 * unsigned integer that represents the number of bytes in a packet.
 * @param sx_1278_hw The parameter "sx_1278_hw" is a pointer to a structure of type "SX1278_hw_t". This
 * structure contains the hardware configuration for the SX1278 module. It likely includes information
 * such as the SPI interface pins, reset pin, and other necessary pins for communication with the
 * module
 */
void SX_1278_Init(SX1278_t *module, uint64_t frequency, uint8_t power, uint8_t LoRa_SF,
                  uint8_t LoRa_BW, uint8_t LoRa_CR, uint8_t LoRa_CRC_sum, uint8_t packetLength, SX1278_hw_t *sx_1278_hw)
{
    module->frequency = frequency;
    module->power = power;
    module->LoRa_SF = LoRa_SF;
    module->LoRa_BW = LoRa_BW;
    module->LoRa_CR = LoRa_CR;
    module->LoRa_CRC_sum = LoRa_CRC_sum;
    module->packetLength = packetLength;
    module->hw = sx_1278_hw;

    SX_1278_HW_Init(module->hw);
    SX_1278_Config(module);
}

/**
 * The function SX_1278_Config configures the SX1278 module for LoRa communication by setting various
 * parameters such as frequency, bandwidth, coding rate, and sync word.
 * 
 * @param module A pointer to the SX1278_t structure, which contains the configuration parameters for
 * the SX1278 module.
 */
void SX_1278_Config(SX1278_t *module)
{
    SX_1278_SLEEP(module);
    HAL_Delay(15);
    SX_1278_MODE_Lora(module);

    // setting  frequency parameter
    uint64_t freq = ((uint64_t)module->frequency << 19) / 32000000;

    SX_1278_SPIWrite(module, LR_RegFrMsb, (uint8_t)(freq << 16));
    SX_1278_SPIWrite(module, LR_RegFrMid, (uint8_t)(freq << 8));
    SX_1278_SPIWrite(module, LR_RegFrLsb, (uint8_t)(freq << 0));

    SX_1278_SPIWrite(module, RegSyncWord, 0x34);

    // setting base parameter

    if (module->LoRa_SF == SX1278_LORA_SF_6)
    {
        SX_1278_SPIWrite(module,
                         LR_RegModemConfig1,
                         ((module->LoRa_BW << 4) + (module->LoRa_CR << 1) + 0x01)); // Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

        SX_1278_SPIWrite(module,
                         LR_RegModemConfig2,
                         ((module->LoRa_SF << 4) + (module->LoRa_CRC_sum << 2) + 0x03)); // normal mode
        SX_1278_SPIWrite(module, LR_RegDetectOptimize, 0x05);
        SX_1278_SPIWrite(module, LR_RegDetectionThreshold, 0x0C);
    }
    else
    {
        SX_1278_SPIWrite(module,
                         LR_RegModemConfig1,
                         ((module->LoRa_BW << 4) + (module->LoRa_CR << 1) + 0x00)); // Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

        SX_1278_SPIWrite(module,
                         LR_RegModemConfig2,
                         ((module->LoRa_SF << 4) + (module->LoRa_CRC_sum << 2) + 0x03)); // normal mode
        SX_1278_SPIWrite(module, LR_RegDetectOptimize, 0x03);
        SX_1278_SPIWrite(module, LR_RegDetectionThreshold, 0x0A);
    }
    uint8_t tmp = (((uint8_t)((1 << module->LoRa_SF) / SX1278_LoRaBandwidth[module->LoRa_BW]) & 0xF0) >= 16) ? 0x80 : 0;
    SX_1278_SPIWrite(module, LR_RegModemConfig3, 0x04 | tmp);
    SX_1278_SPIWrite(module, LR_RegSymbTimeoutLsb, 0x08); // RegSymbTimeoutLsb Timeout = 0x3FF(Max)
    SX_1278_SPIWrite(module, LR_RegPreambleMsb, 0x00);    // RegPreambleMsb
    SX_1278_SPIWrite(module, LR_RegPreambleLsb, 8);       // RegPreambleLsb 8+4=12byte Preamble
    SX_1278_SPIWrite(module, REG_LR_DIOMAPPING2, 0x01);   // RegDioMapping2 DIO5=00, DIO4=01

    module->readBytes = 0;
    SX_1278_STANDBY(module);
}

/**
 * The function SX_1278_STANDBY sets the SX1278 module to standby mode by writing a specific value to
 * the OpMode register.
 * 
 * @param module The parameter "module" is a pointer to a structure of type SX1278_t. This structure
 * likely contains various variables and settings related to the SX1278 module, such as the SPI
 * interface, register addresses, and current status.
 */
void SX_1278_STANDBY(SX1278_t *module)
{
    SX_1278_SPIWrite(module, LR_RegOpMode, 0x09);
    module->status = STANDBY;
}

/**
 * The SX_1278_SLEEP function puts the SX1278 module into sleep mode by writing a specific value to the
 * OpMode register.
 * 
 * @param module The parameter "module" is a pointer to a structure of type SX1278_t. This structure
 * contains information about the SX1278 module, such as its SPI interface and current status.
 */
void SX_1278_SLEEP(SX1278_t *module)
{
    SX_1278_SPIWrite(module, LR_RegOpMode, 0x08);
    module->status = SLEEP;
}

/**
 * The function initializes the SX1278 module for transmission with specified parameters.
 * 
 * @param module A pointer to the SX1278_t structure, which contains the configuration and status
 * information for the SX1278 module.
 * @param length The length parameter is the length of the packet to be transmitted. It specifies the
 * number of bytes in the payload of the packet.
 * @param timeout The timeout parameter is the maximum amount of time (in milliseconds) that the
 * function will wait for the payload length to be set to the specified length. If the timeout is
 * reached and the payload length is still not set, the function will return 0.
 * 
 * @return a uint8_t value. If the condition "temp == length" is true, it returns 1. If the timeout is
 * reached (HAL_GetTick() - time >= timeout), it returns 0.
 */
uint8_t SX_1278_TX_Init(SX1278_t *module, uint8_t length, uint32_t timeout)
{
    uint8_t addr;
    uint8_t temp;
    uint32_t time = HAL_GetTick();
    module->packetLength = length;
    SX_1278_SPIWrite(module, LR_RegPaConfig, module->power); // Setting output power parameter
    SX_1278_SPIWrite(module, LR_RegOcp, 0x1B);               // OCP = 1

    SX_1278_SPIWrite(module, REG_LR_PADAC, 0x87);       // Tx for 20dBm
    SX_1278_SPIWrite(module, LR_RegHopPeriod, 0x00);    // RegHopPeriod NO FHSS
    SX_1278_SPIWrite(module, REG_LR_DIOMAPPING1, 0x41); // DIO0=01, DIO1=00,DIO2=00, DIO3=01
    SX_1278_ClearIRQ(module);
    SX_1278_SPIWrite(module, LR_RegIrqFlagsMask, 0xF7);    // Open TxDone interrupt
    SX_1278_SPIWrite(module, LR_RegPayloadLength, length); // RegPayloadLength 21byte
    addr = SX_1278_SPIRead(module, LR_RegFifoTxBaseAddr);  // RegFiFoTxBaseAddr
    SX_1278_SPIWrite(module, LR_RegFifoAddrPtr, addr);     // RegFifoAddrPtr

    while (1)
    {
        temp = SX_1278_SPIRead(module, LR_RegPayloadLength);
        if (temp == length)
        {
            module->status = TX;
            return 1;
        }

        if (HAL_GetTick() - time >= timeout)
        {
            return 0;
        }
    }
}

uint8_t SX_1278_TX_Packet(SX1278_t *module, uint8_t *txBuffer,
                          uint8_t length, uint32_t timeout)
{
}

/**
 * The function initializes the SX1278 module for receiving data with specified length and timeout.
 * 
 * @param module A pointer to the SX1278_t structure, which contains the configuration and status
 * information for the SX1278 module.
 * @param length The length parameter is the length of the packet that the SX1278 module is expected to
 * receive.
 * @param timeout The timeout parameter is the maximum amount of time (in milliseconds) that the
 * function will wait for the SX1278 module to enter the RX mode. If the module does not enter the RX
 * mode within this time, the function will return 0 to indicate a timeout.
 * 
 * @return a uint8_t value. If the condition `(SX_1278_SPIRead(module, LR_RegModemStat) & 0x04) ==
 * 0x04` is true, it returns 1. If the timeout is reached (`HAL_GetTick() - time >= timeout`), it
 * returns 0.
 */
uint8_t SX_1278_RX_Init(SX1278_t *module, uint8_t length, uint32_t timeout)
{
    uint8_t addr;
    uint32_t time = HAL_GetTick();

    module->packetLength = length;

    SX_1278_SPIWrite(module, LR_RegLna, 0x23);          // 0b00100011 boost on, maximum gain
    SX_1278_SPIWrite(module, REG_LR_PADAC, 0x84);       // Normal and RX
    SX_1278_SPIWrite(module, LR_RegHopPeriod, 0xFF);    // No FHSS
    SX_1278_SPIWrite(module, REG_LR_DIOMAPPING1, 0x01); // DIO=00,DIO1=00,DIO2=00, DIO3=01
    SX_1278_SPIWrite(module, LR_RegIrqFlagsMask, 0x3F); // Open RxDone interrupt & Timeout
    SX_1278_ClearIRQ(module);
    SX_1278_SPIWrite(module, LR_RegPayloadLength, length); // Payload Length 21byte(this register must difine when the data long of one byte in SF is 6)
    addr = SX_1278_SPIRead(module, LR_RegFifoRxBaseAddr);  // Read RxBaseAddr
    SX_1278_SPIWrite(module, LR_RegFifoAddrPtr, addr);     // RxBaseAddr->FiFoAddrPtr
    SX_1278_SPIWrite(module, LR_RegOpMode, 0x8d);          // Mode//Low Frequency Mode
    // SX_1278_SPIWrite(module, LR_RegOpMode,0x05);	//Continuous Rx Mode //High Frequency Mode
    module->readBytes = 0;

    while (1)
    {
        if ((SX_1278_SPIRead(module, LR_RegModemStat) & 0x04) == 0x04)
        { // Rx-on going RegModemStat
            module->status = RX;
            return 1;
        }

        if (HAL_GetTick() - time >= timeout)
        {
            return 0;
        }
    }
}
uint8_t SX_1278_RX_Packet(SX1278_t *module, uint32_t timeout)
{
}

/**
 * The function SX_1278_ClearIRQ clears the interrupt flags of the SX1278 module.
 * 
 * @param module The parameter "module" is a pointer to a structure of type SX1278_t. This structure
 * likely contains information about the SX1278 module, such as its SPI interface and other
 * configuration settings.
 */
void SX_1278_ClearIRQ(SX1278_t *module)
{
    SX_1278_SPIWrite(module, LR_RegIrqFlags, 0xFF);
}