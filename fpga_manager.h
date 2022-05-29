#ifndef __FPGA_MANAGER_H
#define __FPGA_MANAGER_H

#include <stdint.h>

/* Constants. */
/* Possible IRQs to enable/disable. */
#define FPGA_MANAGER_IRQ_NSTATUS       0x00000001
#define FPGA_MANAGER_IRQ_CONF_DONE     0x00000002
#define FPGA_MANAGER_IRQ_INIT_DONE     0x00000004
#define FPGA_MANAGER_IRQ_CRC_ERROR     0x00000008
#define FPGA_MANAGER_IRQ_CVP_CONF_DONE 0x00000010
#define FPGA_MANAGER_IRQ_PR_READY      0x00000020
#define FPGA_MANAGER_IRQ_PR_ERROR      0x00000040
#define FPGA_MANAGER_IRQ_PR_DONE       0x00000080
#define FPGA_MANAGER_IRQ_NCONFIG_PIN   0x00000100
#define FPGA_MANAGER_IRQ_NSTATUS_PIN   0x00000200
#define FPGA_MANAGER_IRQ_CONF_DONE_PIN 0x00000400
#define FPGA_MANAGER_IRQ_FPGA_POWER_ON 0x00000800

/* Structures and definitions. */
struct fpga_manager {
    void *map_base;
};

/* Exported functions. */

/* Initialize the fpga_manager structure. */
void fpga_manager_initvar(struct fpga_manager *mgr);

/* Destroy a fpga_manager object. */
void fpga_manager_destroy(struct fpga_manager *mgr);

/* Create a fpga_manager object.
 * Return TRUE on success.
 */
int fpga_manager_create(struct fpga_manager *mgr);

/* Get the status of the configuration monitor. */
uint32_t fpga_manager_get_mon_status(struct fpga_manager *mgr);

/* Get the state of the FPGA manager. */
uint32_t fpga_manager_get_state(struct fpga_manager *mgr);

/* Get the value of MSEL mode.
 * Return -1 if invalid MSEL mode.
 */
int fpga_manager_get_msel(struct fpga_manager *mgr);

/* Enable interrupts.
 * The set of enable interrups is given by `irqs`
 * (see the FPGA_MANAGER_IRQ constants above).
 */
void fpga_manager_enable_irqs(struct fpga_manager *mgr, uint32_t irqs);

/* Disable all interrupts. */
void fpga_manager_disable_irqs(struct fpga_manager *mgr);

/* Configure the FPGA.
 * The configuration data is in `conf_data` and the size of the
 * configuration data in `size`.
 * Return TRUE on success.
 */
int fpga_manager_configure(struct fpga_manager *mgr,
                           void *conf_data, size_t size);

#endif /* __FPGA_MANAGER_H */
