
//From DeepSeek AI
/* Register addresses */
#define BMI160_CHIP_ID_ADDR          0x00
#define BMI160_ERR_REG_ADDR          0x02
#define BMI160_PMU_STATUS_ADDR       0x03
#define BMI160_DATA_ADDR             0x04  // 20 bytes (0x04-0x17)
#define BMI160_SENSORTIME_ADDR       0x18  // 3 bytes (0x18-0x1A)
#define BMI160_STATUS_ADDR           0x1B
#define BMI160_INT_STATUS_ADDR       0x1C  // 4 bytes (0x1C-0x1F)
#define BMI160_TEMPERATURE_ADDR      0x20  // 2 bytes (0x20-0x21)
#define BMI160_FIFO_LENGTH_ADDR      0x22  // 2 bytes (0x22-0x23)
#define BMI160_FIFO_DATA_ADDR        0x24
#define BMI160_ACC_CONF_ADDR         0x40
#define BMI160_ACC_RANGE_ADDR        0x41
#define BMI160_GYR_CONF_ADDR         0x42
#define BMI160_GYR_RANGE_ADDR        0x43
#define BMI160_MAG_CONF_ADDR         0x44
#define BMI160_FIFO_DOWNS_ADDR       0x45
#define BMI160_FIFO_CONFIG_ADDR      0x46  // 2 bytes (0x46-0x47)
#define BMI160_MAG_IF_ADDR           0x4B  // 5 bytes (0x4B-0x4F)
#define BMI160_INT_EN_ADDR           0x50  // 3 bytes (0x50-0x52)
#define BMI160_INT_OUT_CTRL_ADDR     0x53
#define BMI160_INT_LATCH_ADDR        0x54
#define BMI160_INT_MAP_ADDR          0x55  // 3 bytes (0x55-0x57)
#define BMI160_INT_DATA_ADDR         0x58  // 2 bytes (0x58-0x59)
#define BMI160_INT_LOWHIGH_ADDR      0x5A  // 5 bytes (0x5A-0x5E)
#define BMI160_INT_MOTION_ADDR       0x5F  // 4 bytes (0x5F-0x62)
#define BMI160_INT_TAP_ADDR          0x63  // 2 bytes (0x63-0x64)
#define BMI160_INT_ORIENT_ADDR       0x65  // 2 bytes (0x65-0x66)
#define BMI160_INT_FLAT_ADDR         0x67  // 2 bytes (0x67-0x68)
#define BMI160_FOC_CONF_ADDR         0x69
#define BMI160_CONF_ADDR             0x6A
#define BMI160_IF_CONF_ADDR          0x6B
#define BMI160_PMU_TRIGGER_ADDR      0x6C
#define BMI160_SELF_TEST_ADDR        0x6D
#define BMI160_NV_CONF_ADDR          0x70
#define BMI160_OFFSET_ADDR           0x71  // 7 bytes (0x71-0x77)
#define BMI160_STEP_CNT_ADDR         0x78  // 2 bytes (0x78-0x79)
#define BMI160_STEP_CONF_ADDR        0x7A  // 2 bytes (0x7A-0x7B)
#define BMI160_CMD_ADDR              0x7E

/* Register bit definitions */

// CHIP_ID (0x00)
#define BMI160_CHIP_ID               0xD1

// ERR_REG (0x02)
#define BMI160_DROP_CMD_ERR          (1 << 6)
#define BMI160_FATAL_ERR             (1 << 0)

// PMU_STATUS (0x03)
#define BMI160_ACC_PMU_STATUS_MASK   0x03
#define BMI160_GYR_PMU_STATUS_MASK   0x0C
#define BMI160_MAG_PMU_STATUS_MASK   0x30

// STATUS (0x1B)
#define BMI160_DRDY_ACC              (1 << 7)
#define BMI160_DRDY_GYR              (1 << 6)
#define BMI160_DRDY_MAG              (1 << 5)
#define BMI160_NVM_RDY               (1 << 4)
#define BMI160_FOC_RDY               (1 << 3)
#define BMI160_MAG_MAN_OP            (1 << 2)
#define BMI160_GYR_SELF_TEST_OK      (1 << 1)

// ACC_CONF (0x40)
#define BMI160_ACC_US                (1 << 7)
#define BMI160_ACC_BWP_MASK          0x70
#define BMI160_ACC_ODR_MASK          0x0F

// ACC_RANGE (0x41)
#define BMI160_ACC_RANGE_MASK        0x0F
#define BMI160_ACC_RANGE_2G          0x03
#define BMI160_ACC_RANGE_4G          0x05
#define BMI160_ACC_RANGE_8G          0x08
#define BMI160_ACC_RANGE_16G         0x0C

// GYR_CONF (0x42)
#define BMI160_GYR_BWP_MASK          0x70
#define BMI160_GYR_ODR_MASK          0x0F

// GYR_RANGE (0x43)
#define BMI160_GYR_RANGE_MASK        0x07
#define BMI160_GYR_RANGE_2000DPS     0x00
#define BMI160_GYR_RANGE_1000DPS     0x01
#define BMI160_GYR_RANGE_500DPS      0x02
#define BMI160_GYR_RANGE_250DPS      0x03
#define BMI160_GYR_RANGE_125DPS      0x04

// FIFO_CONFIG (0x46-0x47)
#define BMI160_FIFO_HEADER_EN        (1 << 4)
#define BMI160_FIFO_MAG_EN           (1 << 5)
#define BMI160_FIFO_ACC_EN           (1 << 6)
#define BMI160_FIFO_GYR_EN           (1 << 7)

// INT_EN (0x50-0x52)
#define BMI160_INT_FLAT_EN           (1 << 7)
#define BMI160_INT_ORIENT_EN         (1 << 6)
#define BMI160_INT_S_TAP_EN          (1 << 5)
#define BMI160_INT_D_TAP_EN          (1 << 4)
#define BMI160_INT_ANYMO_EN          (1 << 2)
#define BMI160_INT_FWM_EN            (1 << 6)
#define BMI160_INT_FFULL_EN          (1 << 5)
#define BMI160_INT_DRDY_EN           (1 << 4)
#define BMI160_INT_STEP_DET_EN       (1 << 3)

// INT_OUT_CTRL (0x53)
#define BMI160_INT2_OUTPUT_EN        (1 << 7)
#define BMI160_INT2_OD               (1 << 6)
#define BMI160_INT2_LVL              (1 << 5)
#define BMI160_INT2_EDGE_CTRL        (1 << 4)
#define BMI160_INT1_OUTPUT_EN        (1 << 3)
#define BMI160_INT1_OD               (1 << 2)
#define BMI160_INT1_LVL              (1 << 1)
#define BMI160_INT1_EDGE_CTRL        (1 << 0)

// CMD (0x7E) commands
#define BMI160_CMD_SOFT_RESET        0xB6
#define BMI160_CMD_ACC_PMU_SUSPEND   0x10
#define BMI160_CMD_ACC_PMU_NORMAL    0x11
#define BMI160_CMD_ACC_PMU_LOWPOWER  0x12
#define BMI160_CMD_GYR_PMU_SUSPEND   0x14
#define BMI160_CMD_GYR_PMU_NORMAL    0x15
#define BMI160_CMD_GYR_PMU_FASTSTART 0x17
#define BMI160_CMD_FIFO_FLUSH        0xB0
#define BMI160_CMD_INT_RESET         0xB1
#define BMI160_CMD_STEP_CNT_CLR      0xB2
#define BMI160_CMD_PROG_NVM          0xA0
#define BMI160_CMD_FOC               0x03