#ifndef L3GD20_H
#define L3GD20_H

#include <stdint.h>

/* L3GD20 Register Addresses */
#define L3GD20_WHO_AM_I 0x0F      // Device identification register (read-only)
#define L3GD20_CTRL_REG1 0x20     // Control register 1: Configures data rate, bandwidth, power mode, and axis enable
#define L3GD20_CTRL_REG2 0x21     // Control register 2: Configures high-pass filter mode and cutoff frequency
#define L3GD20_CTRL_REG3 0x22     // Control register 3: Configures interrupt enables and output modes for INT1 and INT2
#define L3GD20_CTRL_REG4 0x23     // Control register 4: Configures block data update, endianness, full scale, and SPI mode
#define L3GD20_CTRL_REG5 0x24     // Control register 5: Configures boot, FIFO, high-pass filter, and output selection
#define L3GD20_REFERENCE 0x25     // Reference register: Sets reference value for interrupt generation
#define L3GD20_OUT_TEMP 0x26      // Temperature data register: Provides 8-bit temperature data in two's complement
#define L3GD20_STATUS_REG 0x27    // Status register: Indicates data availability and overrun conditions
#define L3GD20_OUT_X_L 0x28       // X-axis angular rate low byte (two's complement)
#define L3GD20_OUT_X_H 0x29       // X-axis angular rate high byte (two's complement)
#define L3GD20_OUT_Y_L 0x2A       // Y-axis angular rate low byte (two's complement)
#define L3GD20_OUT_Y_H 0x2B       // Y-axis angular rate high byte (two's complement)
#define L3GD20_OUT_Z_L 0x2C       // Z-axis angular rate low byte (two's complement)
#define L3GD20_OUT_Z_H 0x2D       // Z-axis angular rate high byte (two's complement)
#define L3GD20_FIFO_CTRL_REG 0x2E // FIFO control register: Sets FIFO mode and watermark threshold
#define L3GD20_FIFO_SRC_REG 0x2F  // FIFO source register: Indicates FIFO status (watermark, overrun, empty)
#define L3GD20_INT1_CFG 0x30      // Interrupt 1 configuration register: Configures interrupt events and latching
#define L3GD20_INT1_SRC 0x31      // Interrupt 1 source register: Indicates interrupt event sources (read-only)
#define L3GD20_INT1_THS_XH 0x32   // X-axis high threshold for interrupt 1
#define L3GD20_INT1_THS_XL 0x33   // X-axis low threshold for interrupt 1
#define L3GD20_INT1_THS_YH 0x34   // Y-axis high threshold for interrupt 1
#define L3GD20_INT1_THS_YL 0x35   // Y-axis low threshold for interrupt 1
#define L3GD20_INT1_THS_ZH 0x36   // Z-axis high threshold for interrupt 1
#define L3GD20_INT1_THS_ZL 0x37   // Z-axis low threshold for interrupt 1
#define L3GD20_INT1_DURATION 0x38 // Interrupt 1 duration register: Sets minimum duration for interrupt recognition

/* CTRL_REG1 Bit Definitions */
#define CTRL_REG1_DR1 (1 << 7) // Data rate selection bit 1
#define CTRL_REG1_DR0 (1 << 6) // Data rate selection bit 0
#define CTRL_REG1_BW1 (1 << 5) // Bandwidth selection bit 1
#define CTRL_REG1_BW0 (1 << 4) // Bandwidth selection bit 0
#define CTRL_REG1_PD (1 << 3)  // Power down mode (0: power down, 1: normal/sleep)
#define CTRL_REG1_ZEN (1 << 2) // Z-axis enable (0: disabled, 1: enabled)
#define CTRL_REG1_YEN (1 << 1) // Y-axis enable (0: disabled, 1: enabled)
#define CTRL_REG1_XEN (1 << 0) // X-axis enable (0: disabled, 1: enabled)

/* CTRL_REG2 Bit Definitions */
#define CTRL_REG2_HPM1 (1 << 5)  // High-pass filter mode bit 1
#define CTRL_REG2_HPM0 (1 << 4)  // High-pass filter mode bit 0
#define CTRL_REG2_HPCF3 (1 << 3) // High-pass filter cutoff frequency bit 3
#define CTRL_REG2_HPCF2 (1 << 2) // High-pass filter cutoff frequency bit 2
#define CTRL_REG2_HPCF1 (1 << 1) // High-pass filter cutoff frequency bit 1
#define CTRL_REG2_HPCF0 (1 << 0) // High-pass filter cutoff frequency bit 0

/* CTRL_REG3 Bit Definitions */
#define CTRL_REG3_I1_INT1 (1 << 7)   // Interrupt 1 enable on INT1 pin (0: disable, 1: enable)
#define CTRL_REG3_I1_BOOT (1 << 6)   // Boot status on INT1 (0: disable, 1: enable)
#define CTRL_REG3_H_LACTIVE (1 << 5) // Interrupt active level on INT1 (0: high, 1: low)
#define CTRL_REG3_PP_OD (1 << 4)     // INT1 pin mode (0: push-pull, 1: open drain)
#define CTRL_REG3_I2_DRDY (1 << 3)   // Data ready on DRDY/INT2 (0: disable, 1: enable)
#define CTRL_REG3_I2_WTM (1 << 2)    // FIFO watermark interrupt on DRDY/INT2 (0: disable, 1: enable)
#define CTRL_REG3_I2_ORUN (1 << 1)   // FIFO overrun interrupt on DRDY/INT2 (0: disable, 1: enable)
#define CTRL_REG3_I2_EMPTY (1 << 0)  // FIFO empty interrupt on DRDY/INT2 (0: disable, 1: enable)

/* CTRL_REG4 Bit Definitions */
#define CTRL_REG4_BDU (1 << 7) // Block data update (0: continuous, 1: wait for MSB/LSB read)
#define CTRL_REG4_BLE (1 << 6) // Big/little endian selection (0: LSB low address, 1: MSB low address)
#define CTRL_REG4_FS1 (1 << 5) // Full scale selection bit 1
#define CTRL_REG4_FS0 (1 << 4) // Full scale selection bit 0
#define CTRL_REG4_SIM (1 << 0) // SPI interface mode (0: 4-wire, 1: 3-wire)

/* CTRL_REG5 Bit Definitions */
#define CTRL_REG5_BOOT (1 << 7)      // Reboot memory content (0: normal, 1: reboot)
#define CTRL_REG5_FIFO_EN (1 << 6)   // FIFO enable (0: disable, 1: enable)
#define CTRL_REG5_HPEN (1 << 4)      // High-pass filter enable (0: disable, 1: enable)
#define CTRL_REG5_INT1_SEL1 (1 << 3) // INT1 selection configuration bit 1
#define CTRL_REG5_INT1_SEL0 (1 << 2) // INT1 selection configuration bit 0
#define CTRL_REG5_OUT_SEL1 (1 << 1)  // Output selection configuration bit 1
#define CTRL_REG5_OUT_SEL0 (1 << 0)  // Output selection configuration bit 0

/* STATUS_REG Bit Definitions */
#define STATUS_REG_ZYXOR (1 << 7) // X, Y, Z axis data overrun (0: no overrun, 1: overrun)
#define STATUS_REG_ZOR (1 << 6)   // Z-axis data overrun (0: no overrun, 1: overrun)
#define STATUS_REG_YOR (1 << 5)   // Y-axis data overrun (0: no overrun, 1: overrun)
#define STATUS_REG_XOR (1 << 4)   // X-axis data overrun (0: no overrun, 1: overrun)
#define STATUS_REG_ZYXDA (1 << 3) // X, Y, Z axis new data available (0: not available, 1: available)
#define STATUS_REG_ZDA (1 << 2)   // Z-axis new data available (0: not available, 1: available)
#define STATUS_REG_YDA (1 << 1)   // Y-axis new data available (0: not available, 1: available)
#define STATUS_REG_XDA (1 << 0)   // X-axis new data available (0: not available, 1: available)

/* FIFO_CTRL_REG Bit Definitions */
#define FIFO_CTRL_FM2 (1 << 7)  // FIFO mode selection bit 2
#define FIFO_CTRL_FM1 (1 << 6)  // FIFO mode selection bit 1
#define FIFO_CTRL_FM0 (1 << 5)  // FIFO mode selection bit 0
#define FIFO_CTRL_WTM4 (1 << 4) // FIFO watermark threshold bit 4
#define FIFO_CTRL_WTM3 (1 << 3) // FIFO watermark threshold bit 3
#define FIFO_CTRL_WTM2 (1 << 2) // FIFO watermark threshold bit 2
#define FIFO_CTRL_WTM1 (1 << 1) // FIFO watermark threshold bit 1
#define FIFO_CTRL_WTM0 (1 << 0) // FIFO watermark threshold bit 0

/* FIFO_SRC_REG Bit Definitions */
#define FIFO_SRC_WTM (1 << 7)   // FIFO watermark status (0: below threshold, 1: at or above)
#define FIFO_SRC_OVRN (1 << 6)  // FIFO overrun status (0: not full, 1: full)
#define FIFO_SRC_EMPTY (1 << 5) // FIFO empty status (0: not empty, 1: empty)

/* INT1_CFG Bit Definitions */
#define INT1_CFG_AND_OR (1 << 7) // Interrupt combination mode (0: OR, 1: AND)
#define INT1_CFG_LIR (1 << 6)    // Latch interrupt request (0: not latched, 1: latched)
#define INT1_CFG_ZHIE (1 << 5)   // Z high interrupt enable (0: disable, 1: enable)
#define INT1_CFG_ZLIE (1 << 4)   // Z low interrupt enable (0: disable, 1: enable)
#define INT1_CFG_YHIE (1 << 3)   // Y high interrupt enable (0: disable, 1: enable)
#define INT1_CFG_YLIE (1 << 2)   // Y low interrupt enable (0: disable, 1: enable)
#define INT1_CFG_XHIE (1 << 1)   // X high interrupt enable (0: disable, 1: enable)
#define INT1_CFG_XLIE (1 << 0)   // X low interrupt enable (0: disable, 1: enable)

/* INT1_SRC Bit Definitions */
#define INT1_SRC_IA (1 << 6) // Interrupt active (0: no interrupt, 1: interrupt generated)
#define INT1_SRC_ZH (1 << 5) // Z high event (0: no event, 1: event occurred)
#define INT1_SRC_ZL (1 << 4) // Z low event (0: no event, 1: event occurred)
#define INT1_SRC_YH (1 << 3) // Y high event (0: no event, 1: event occurred)
#define INT1_SRC_YL (1 << 2) // Y low event (0: no event, 1: event occurred)
#define INT1_SRC_XH (1 << 1) // X high event (0: no event, 1: event occurred)
#define INT1_SRC_XL (1 << 0) // X low event (0: no event, 1: event occurred)

/* L3GD20 Data Rate Selection => MAL SOLO SON 2 BITS, NO 8 */
typedef enum
{
    L3GD20_DR_95HZ = 0x00,  // 95 Hz output data rate
    L3GD20_DR_190HZ = 0x40, // 190 Hz output data rate
    L3GD20_DR_380HZ = 0x80, // 380 Hz output data rate
    L3GD20_DR_760HZ = 0xC0  // 760 Hz output data rate
} L3GD20_DataRate_t;

/* L3GD20 Bandwidth Selection => MAL SOLO SON 2 BITS, NO 8  */
typedef enum
{
    L3GD20_BW_LOW = 0x00,     // Low bandwidth
    L3GD20_BW_MEDLOW = 0x10,  // Medium-low bandwidth
    L3GD20_BW_MEDHIGH = 0x20, // Medium-high bandwidth
    L3GD20_BW_HIGH = 0x30     // High bandwidth
} L3GD20_Bandwidth_t;

/* L3GD20 Full Scale Selection */
typedef enum
{
    L3GD20_FS_250DPS = 0x00, // ±250 degrees per second
    L3GD20_FS_500DPS = 0x10, // ±500 degrees per second
    L3GD20_FS_2000DPS = 0x20 // ±2000 degrees per second
} L3GD20_FullScale_t;

/* L3GD20 FIFO Mode Selection */
typedef enum
{
    L3GD20_FIFO_BYPASS = 0x00,          // Bypass mode
    L3GD20_FIFO_MODE = 0x20,            // FIFO mode
    L3GD20_FIFO_STREAM = 0x40,          // Stream mode
    L3GD20_FIFO_STREAM_TO_FIFO = 0x60,  // Stream-to-FIFO mode
    L3GD20_FIFO_BYPASS_TO_STREAM = 0x80 // Bypass-to-Stream mode
} L3GD20_FIFOMode_t;

/* L3GD20 High-Pass Filter Mode */
typedef enum
{
    L3GD20_HPM_NORMAL_RESET = 0x00, // Normal mode with reset
    L3GD20_HPM_REFERENCE = 0x10,    // Reference signal for filtering
    L3GD20_HPM_NORMAL = 0x20,       // Normal mode
    L3GD20_HPM_AUTORESET = 0x30     // Autoreset on interrupt event
} L3GD20_HPM_t;

/* L3GD20 Configuration Structure */
typedef struct
{
    L3GD20_DataRate_t dataRate;   // Output data rate selection
    L3GD20_Bandwidth_t bandwidth; // Bandwidth selection
    L3GD20_FullScale_t fullScale; // Full scale selection
    L3GD20_FIFOMode_t fifoMode;   // FIFO mode selection
    L3GD20_HPM_t highPassMode;    // High-pass filter mode
    uint8_t highPassCutoff;       // High-pass filter cutoff (HPCF3-0)
    uint8_t enableAxes;           // Enable X, Y, Z axes (bitmask: Z=4, Y=2, X=1)
    uint8_t powerMode;            // Power mode: 0=power down, 1=normal, 2=sleep
    uint8_t interrupt1Enable;     // Enable INT1 interrupts
    uint8_t interrupt2Enable;     // Enable INT2 interrupts (DRDY, WTM, ORUN, EMPTY)
    uint8_t blockDataUpdate;      // BDU: 0=continuous, 1=wait for MSB/LSB read
    uint8_t bigLittleEndian;      // BLE: 0=LSB low addr, 1=MSB low addr
    uint8_t spiMode;              // SIM: 0=4-wire, 1=3-wire
} L3GD20_Config_t;

/* L3GD20 Data Structure */
typedef struct
{
    int16_t x;          // X-axis angular rate (dps)
    int16_t y;          // Y-axis angular rate (dps)
    int16_t z;          // Z-axis angular rate (dps)
    int8_t temperature; // Temperature data
} L3GD20_Data_t;

/* Function Prototypes */
void L3GD20_Init(L3GD20_Config_t *config);                                                // Initialize sensor with configuration
void L3GD20_ReadData(L3GD20_Data_t *data);                                                // Read angular rate and temperature data
uint8_t L3GD20_ReadWhoAmI(void);                                                          // Read WHO_AM_I register
void L3GD20_SetInterruptThresholds(uint16_t xThresh, uint16_t yThresh, uint16_t zThresh); // Set interrupt thresholds
void L3GD20_SetInterruptConfig(uint8_t config, uint8_t duration);                         // Configure INT1 interrupts

#endif /* L3GD20_H */