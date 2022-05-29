#include <stdint.h>
#include <unistd.h>

#include "common.h"
#include "devmem.h"
#include "fpga_manager.h"

/* Constants. */
#define IO_BASE 0xFF000000
#define IO_SIZE 0x01000000

/* Register offsets */
#define FPGMGR_BASE                 0x00706000
#define FPGMGR_STAT                 (FPGMGR_BASE + 0x0)
#define FPGMGR_CTL                  (FPGMGR_BASE + 0x4)
#define FPGMGR_DCLKCNT              (FPGMGR_BASE + 0x8)
#define FPGMGR_DCLKSTAT             (FPGMGR_BASE + 0xC)
#define FPGMGR_GPIO_INTEN           (FPGMGR_BASE + 0x830)
#define FPGMGR_GPIO_INTMSK          (FPGMGR_BASE + 0x834)
#define FPGMGR_GPIO_INTTYPE_LEVEL   (FPGMGR_BASE + 0x838)
#define FPGMGR_GPIO_INT_POL         (FPGMGR_BASE + 0x83C)
#define FPGMGR_GPIO_INTSTAT         (FPGMGR_BASE + 0x840)
#define FPGMGR_GPIO_RAW_INTSTAT     (FPGMGR_BASE + 0x844)
#define FPGMGR_GPIO_PORTA_EOI       (FPGMGR_BASE + 0x84C)
#define FPGMGR_GPIO_EXT_PORTA       (FPGMGR_BASE + 0x850)

#define FPGDATA                     0x00B90000

/* FPGMGR_STAT register mode field values. */
#define FPGMGR_STAT_POWER_OFF       0x0
#define FPGMGR_STAT_RESET           0x1
#define FPGMGR_STAT_CFG             0x2
#define FPGMGR_STAT_INIT            0x3
#define FPGMGR_STAT_USER_MODE       0x4
#define FPGMGR_STAT_UNKNOWN         0x5
#define FPGMGR_STAT_STATE_MASK      0x7

#define FPGMGR_STAT_MSEL_MASK       0x000000F8
#define FPGMGR_STAT_MSEL_SHIFT      3

/* FPGMGR_GPIO_* registers share the same bit positions,
 * so can re-use the FPGA_MANAGER_IRQ_* constants.
 */
#define FPGMGR_MON_NSTATUS          FPGA_MANAGER_IRQ_NSTATUS
#define FPGMGR_MON_CONF_DONE        FPGA_MANAGER_IRQ_CONF_DONE
#define FPGMGR_MON_INIT_DONE        FPGA_MANAGER_IRQ_INIT_DONE
#define FPGMGR_MON_CRC_ERROR        FPGA_MANAGER_IRQ_CRC_ERROR
#define FPGMGR_MON_CVP_CONF_DONE    FPGA_MANAGER_IRQ_CVP_CONF_DONE
#define FPGMGR_MON_PR_READY         FPGA_MANAGER_IRQ_PR_READY
#define FPGMGR_MON_PR_ERROR         FPGA_MANAGER_IRQ_PR_ERROR
#define FPGMGR_MON_PR_DONE          FPGA_MANAGER_IRQ_PR_DONE
#define FPGMGR_MON_NCONFIG_PIN      FPGA_MANAGER_IRQ_NCONFIG_PIN
#define FPGMGR_MON_NSTATUS_PIN      FPGA_MANAGER_IRQ_NSTATUS_PIN
#define FPGMGR_MON_CONF_DONE_PIN    FPGA_MANAGER_IRQ_CONF_DONE_PIN
#define FPGMGR_MON_FPGA_POWER_ON    FPGA_MANAGER_IRQ_FPGA_POWER_ON
#define FPGMGR_MON_STATUS_MASK      0x00000FFF

/* SOCFPGMGR_CTL register */
#define FPGMGR_CTL_EN               0x00000001
#define FPGMGR_CTL_NCE              0x00000002
#define FPGMGR_CTL_NCFGPULL         0x00000004

#define CDRATIO_X1                  0x00000000
#define CDRATIO_X2                  0x00000040
#define CDRATIO_X4                  0x00000080
#define CDRATIO_X8                  0x000000C0
#define FPGMGR_CTL_CDRATIO_MASK     0x000000C0

#define FPGMGR_CTL_AXICFGEN         0x00000100

#define CFGWDTH_16                  0x00000000
#define CFGWDTH_32                  0x00000200
#define FPGMGR_CTL_CFGWDTH_MASK     0x00000200

/* FPGMGR_DCLKSTAT register */
#define FPGMGR_DCLKSTAT_DCNTDONE    0x1

/* Tables for configuration. */
struct cfgmgr_mode {
    /* flag that this table entry is a valid mode */
    int valid;

    /* Values to set in the CTRL register */
    uint32_t ctrl;

    /* compression feature */
    int compression;

    /* supports partial reconfiguration */
    int partial_reconf;

    /* POR delay */
    int por_delay_fast;
};

/* For FPGMGR_STAT_MSEL field */
static
struct cfgmgr_mode cfgmgr_modes[] = {
    { TRUE,  CFGWDTH_16 | CDRATIO_X1, FALSE, TRUE,  TRUE  }, /* 0b00000 */
    { TRUE,  CFGWDTH_16 | CDRATIO_X2, FALSE, TRUE,  TRUE  }, /* 0b00001 */
    { TRUE,  CFGWDTH_16 | CDRATIO_X4, TRUE,  TRUE,  TRUE  }, /* 0b00010 */
    { FALSE, 0,                       FALSE, FALSE, FALSE }, /* 0b00011 */
    { TRUE,  CFGWDTH_16 | CDRATIO_X1, FALSE, TRUE,  FALSE }, /* 0b00100 */
    { TRUE,  CFGWDTH_16 | CDRATIO_X2, FALSE, TRUE,  FALSE }, /* 0b00101 */
    { TRUE,  CFGWDTH_16 | CDRATIO_X4, TRUE,  TRUE,  FALSE }, /* 0b00110 */
    { FALSE, 0,                       FALSE, FALSE, FALSE }, /* 0b00111 */
    { TRUE,  CFGWDTH_32 | CDRATIO_X1, FALSE, FALSE, TRUE  }, /* 0b01000 */
    { TRUE,  CFGWDTH_32 | CDRATIO_X4, FALSE, FALSE, TRUE  }, /* 0b01001 */
    { TRUE,  CFGWDTH_32 | CDRATIO_X8, TRUE,  FALSE, TRUE  }, /* 0b01010 */
    { FALSE, 0,                       FALSE, FALSE, FALSE }, /* 0b01011 */
    { TRUE,  CFGWDTH_32 | CDRATIO_X1, FALSE, FALSE, FALSE }, /* 0b01100 */
    { TRUE,  CFGWDTH_32 | CDRATIO_X4, FALSE, FALSE, FALSE }, /* 0b01101 */
    { TRUE,  CFGWDTH_32 | CDRATIO_X8, TRUE,  FALSE, FALSE }, /* 0b01110 */
    { FALSE, 0,                       FALSE, FALSE, FALSE }, /* 0b01111 */
};


void fpga_manager_initvar(struct fpga_manager *mgr)
{
    mgr->map_base = NULL;
}

void fpga_manager_destroy(struct fpga_manager *mgr)
{
    if (mgr->map_base) {
        devmem_unmap(mgr->map_base, IO_SIZE);
        mgr->map_base = NULL;
    }
}

int fpga_manager_create(struct fpga_manager *mgr)
{
    fpga_manager_initvar(mgr);

    mgr->map_base = devmem_map((off_t) IO_BASE, IO_SIZE);
    if (!mgr->map_base) {
        error("could not map FPGA I/O");
        return FALSE;
    }

    return TRUE;
}

/* Auxiliary functions. */

/* Read from the register at `reg_offset`. */
static
uint32_t readl(struct fpga_manager *mgr, uint32_t reg_offset)
{
    volatile uint32_t *addr;

    addr = (volatile uint32_t *)(&((char *) mgr->map_base)[reg_offset]);
    return addr[0];
}

/* Write `value` to the register at `reg_offset`. */
static
void writel(struct fpga_manager *mgr, uint32_t reg_offset, uint32_t value)
{
    volatile uint32_t *addr;

    addr = (volatile uint32_t *)(&((char *) mgr->map_base)[reg_offset]);
    addr[0] = value;
}

/* Set the bits `bits` of the register at `reg_offset`. */
static
void set_bitsl(struct fpga_manager *mgr, uint32_t reg_offset, uint32_t bits)
{
    uint32_t val;

    val = readl(mgr, reg_offset);
    val |= bits;
    writel(mgr, reg_offset, val);
}

/* Clear the bits `bits` of the register at `reg_offset`. */
static
void clr_bitsl(struct fpga_manager *mgr, uint32_t reg_offset, uint32_t bits)
{
    uint32_t val;

    val = readl(mgr, reg_offset);
    val &= ~bits;
    writel(mgr, reg_offset, val);
}

/* Wait for a specific state (or one of multiple states).
 * The `state` variable is the logic OR of the states that
 * this function should wait for.
 * Return TRUE on success (FALSE on timeout).
 */
static
int wait_for_state(struct fpga_manager *mgr, uint32_t state)
{
    int timeout = 2;

    /* Poll the state until one of the states is reached. */
    do {
        if ((fpga_manager_get_state(mgr) & state) != 0)
            return TRUE;
        usleep(20000);
    } while (timeout--);

    error("timeout on wait_for_state");
    return FALSE;
}

/* Clear the DONE status of the DCLK counter (DCLKCNT). */
static
void clear_done_status(struct fpga_manager *mgr)
{
    writel(mgr, FPGMGR_DCLKSTAT, FPGMGR_DCLKSTAT_DCNTDONE);
}

/*
 * Set the DCLKCNT, wait for DCLKSTAT to report the count completed,
 * and clear the complete status. This will pulse the DCLK `count`
 * times.
 * Return TRUE on success
 */
static
int dclk_set_and_wait(struct fpga_manager *mgr, uint32_t count)
{
    int timeout = 2;
    uint32_t done;

    /* Clear any existing DONE status. */
    if (readl(mgr, FPGMGR_DCLKSTAT))
        clear_done_status(mgr);

    /* Issue the DCLK count. */
    writel(mgr, FPGMGR_DCLKCNT, count);

    /* Poll DCLKSTAT to see if it completed in the timeout period. */
    do {
        done = readl(mgr, FPGMGR_DCLKSTAT);
        if (done == FPGMGR_DCLKSTAT_DCNTDONE) {
            clear_done_status(mgr);
            return TRUE;
        }
        usleep(1);
    } while (timeout--);

    error("timout on dclk_set_and_wait");
    return FALSE;
}

/* Set the FPGA manager into configuration mode.
 * Return TRUE on success.
 */
static
int cfg_mode_set(struct fpga_manager *mgr)
{
    uint32_t ctrl_reg;
    int msel;

    /* get value from MSEL pins */
    msel = fpga_manager_get_msel(mgr);
    if (msel < 0) {
        error("could not get msel for configuration mode");
        return FALSE;
    }

    /* Adjust CTRL for the CDRATIO */
    ctrl_reg = readl(mgr, FPGMGR_CTL);
    ctrl_reg &= (uint32_t) (~FPGMGR_CTL_CDRATIO_MASK);
    ctrl_reg &= (uint32_t) (~FPGMGR_CTL_CFGWDTH_MASK);
    ctrl_reg |= cfgmgr_modes[msel].ctrl;

    /* Set NCE to 0. */
    ctrl_reg &= (uint32_t) (~FPGMGR_CTL_NCE);
    writel(mgr, FPGMGR_CTL, ctrl_reg);

    return TRUE;
}


/* Reset the FPGA.
 * Return TRUE on success.
 */
static
int fpga_reset(struct fpga_manager *mgr)
{
    uint32_t ctrl_reg;
    int ret;

    /*
     * Step 1:
     *  - Set CTRL.CFGWDTH, CTRL.CDRATIO to match cfg mode
     *  - Set CTRL.NCE to 0
     */
    ret = cfg_mode_set(mgr);
    if (!ret) {
        error("could not set the configuration mode");
        return FALSE;
    }

    /* Step 2: Set CTRL.EN to 1 */
    set_bitsl(mgr, FPGMGR_CTL, FPGMGR_CTL_EN);

    /* Step 3: Set CTRL.NCONFIGPULL to 1 to put FPGA in reset */
    ctrl_reg = readl(mgr, FPGMGR_CTL);
    ctrl_reg |= FPGMGR_CTL_NCFGPULL;
    writel(mgr, FPGMGR_CTL, ctrl_reg);

    /* Step 4: Wait for STATUS.MODE to report FPGA is in reset phase */
    ret = wait_for_state(mgr, FPGMGR_STAT_RESET);

    /* Step 5: Set CONTROL.NCONFIGPULL to 0 to release FPGA from reset */
    ctrl_reg &= (uint32_t) (~FPGMGR_CTL_NCFGPULL);
    writel(mgr, FPGMGR_CTL, ctrl_reg);

    /* Timeout waiting for reset. */
    if (!ret) {
        error("could not reset fpga");
        return FALSE;
    }

    return TRUE;
}

/*
 * Prepare the FPGA to receive the configuration data.
 * Return TRUE on success.
 */
static
int configure_init(struct fpga_manager *mgr)
{
    int ret;

    /* Steps 1 - 5: Reset the FPGA */
    ret = fpga_reset(mgr);
    if (!ret) {
        error("could not reset FPGA for configuration");
        return FALSE;
    }

    /* Step 6: Wait for FPGA to enter configuration phase */
    if (!wait_for_state(mgr, FPGMGR_STAT_CFG)) {
        error("FPGA did not enter configuration phase");
        return FALSE;
    }

    /* Step 7: Clear nSTATUS interrupt */
    writel(mgr, FPGMGR_GPIO_PORTA_EOI, FPGMGR_MON_NSTATUS);

    /* Step 8: Set CTRL.AXICFGEN to 1 to enable transfer of config data */
    set_bitsl(mgr, FPGMGR_CTL, FPGMGR_CTL_AXICFGEN);

    return TRUE;
}

/* Write the configuration data to the FPGA data register.
 * The configuration data is in `conf_data` and the size of the
 * configuration in `size`.
 * Return TRUE on success.
 */
static
int configure_write(struct fpga_manager *mgr,
                    const void *conf_data, size_t size)
{
    uint32_t *data32 = (uint32_t *) conf_data;
    size_t i = 0;

    if (size <= 0) {
        error("size <= 0 for write");
        return FALSE;
    }

    /*
     * Step 9: write data to the FPGA data register
     */

    /* Write out the complete 32-bit chunks. */
    while (size >= sizeof(uint32_t)) {
        writel(mgr, FPGDATA, data32[i++]);
        size -= sizeof(uint32_t);
    }

    /* Write out remaining non 32-bit chunks. */
    switch (size) {
    case 3:
        writel(mgr, FPGDATA, data32[i++] & 0x00FFFFFF);
        break;
    case 2:
        writel(mgr, FPGDATA, data32[i++] & 0x0000FFFF);
        break;
    case 1:
        writel(mgr, FPGDATA, data32[i++] & 0x000000FF);
        break;
    case 0:
        break;
    default:
        /* This will never happen. */
        error("size should never be bigger than 3");
        return FALSE;
    }

    return TRUE;
}

/* Wait for the configuration to be done.
 * Return TRUE on success.
 */
static
int wait_for_config_done(struct fpga_manager *mgr)
{
    int timeout = 2;
    uint32_t mask;

    mask = FPGMGR_MON_NSTATUS | FPGMGR_MON_CONF_DONE;

    do {
        uint32_t mon_status;
        mon_status = fpga_manager_get_mon_status(mgr);

        /* Some error has occurred. */
        if (!(mon_status & mask)) {
            error("config not done: strange error");
            return FALSE;
        }

        if ((mon_status & mask) == mask)
            return TRUE;

        usleep(20000);
    } while (timeout--);

    error("wait_for_config_done: timeout");
    return FALSE;
}


/* Wait for the FPGA manager to complete the configuration.
 * Return TRUE on success.
 */
static
int configure_complete(struct fpga_manager *mgr)
{
    int ret;

    /*
     * Step 10:
     *  - Observe CONF_DONE and nSTATUS (active low)
     *  - if CONF_DONE = 1 and nSTATUS = 1, configuration was successful
     *  - if CONF_DONE = 0 and nSTATUS = 0, configuration failed
     */
    ret = wait_for_config_done(mgr);
    if (!ret) {
        error("error while waiting for configuration to be done");
        return FALSE;
    }

    /* Step 11: Clear CTRL.AXICFGEN to disable transfer of config data */
    clr_bitsl(mgr, FPGMGR_CTL, FPGMGR_CTL_AXICFGEN);

    /*
     * Step 12:
     *  - Write 4 to DCLKCNT
     *  - Wait for STATUS.DCNTDONE = 1
     *  - Clear bit in STATUS.DCNTDONE
     */
    if (dclk_set_and_wait(mgr, 4)) {
        return FALSE;
    }

    /* Step 13: Wait for STATUS.MODE to report USER MODE or INIT PHASE */
    if (wait_for_state(mgr, FPGMGR_STAT_USER_MODE | FPGMGR_STAT_INIT)) {
        error("error waiting for USER_MODE or INIT phase");
        return FALSE;
    }

    /* Step 14: Additional clocks for the CB to exit initialization phase */
    if (dclk_set_and_wait(mgr, 0x5000)) {
        return FALSE;
    }

    /* Step 15: Wait for STATUS.MODE to report USER MODE */
    if (wait_for_state(mgr, FPGMGR_STAT_USER_MODE)) {
        error("error waiting for USER_MODE");
        return FALSE;
    }

    /* Step 16: Set CTRL.EN to 0 */
    clr_bitsl(mgr, FPGMGR_CTL, FPGMGR_CTL_EN);

    return TRUE;
}


uint32_t fpga_manager_get_mon_status(struct fpga_manager *mgr)
{
    return readl(mgr, FPGMGR_GPIO_EXT_PORTA) &
        FPGMGR_MON_STATUS_MASK;
}

uint32_t fpga_manager_get_state(struct fpga_manager *mgr)
{
    uint32_t status = fpga_manager_get_mon_status(mgr);

    if ((status & FPGMGR_MON_FPGA_POWER_ON) == 0)
        return FPGMGR_STAT_POWER_OFF;

    return readl(mgr, FPGMGR_STAT) & FPGMGR_STAT_STATE_MASK;
}

int fpga_manager_get_msel(struct fpga_manager *mgr)
{
    int msel;

    msel = (int) readl(mgr, FPGMGR_STAT);
    msel &= FPGMGR_STAT_MSEL_MASK;
    msel >>= FPGMGR_STAT_MSEL_SHIFT;

    /* Check that this MSEL setting is supported */
    if (msel >= (int) ((sizeof(cfgmgr_modes) / sizeof(cfgmgr_modes[0])))) {
        error("invalid msel mode: %d", msel);
        return -1;
    }

    if (!cfgmgr_modes[msel].valid) {
        error("invalid msel mode: %d", msel);
        return -1;
    }

    return msel;
}

void fpga_manager_enable_irqs(struct fpga_manager *mgr, uint32_t irqs)
{
    /* set irqs to level sensitive */
    writel(mgr, FPGMGR_GPIO_INTTYPE_LEVEL, 0);

    /* set interrupt polarity */
    writel(mgr, FPGMGR_GPIO_INT_POL, irqs);

    /* clear irqs */
    writel(mgr, FPGMGR_GPIO_PORTA_EOI, irqs);

    /* unmask interrupts */
    writel(mgr, FPGMGR_GPIO_INTMSK, 0);

    /* enable interrupts */
    writel(mgr, FPGMGR_GPIO_INTEN, irqs);
}

void fpga_manager_disable_irqs(struct fpga_manager *mgr)
{
    writel(mgr, FPGMGR_GPIO_INTEN, 0);
}

int fpga_manager_configure(struct fpga_manager *mgr,
                           void *conf_data, size_t size)
{
    if (size <= 0) {
        error("size <= 0");
        return FALSE;
    }

    if (!configure_init(mgr))
        return FALSE;

    if (!configure_write(mgr, conf_data, size))
        return FALSE;

    return configure_complete(mgr);
}
