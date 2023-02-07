#include "../defs.h"
#include "../gpio_config/gpio_config_io.c"

// --------------------------------------------------------
// Firmware routines
// --------------------------------------------------------

void set_registers() {
    reg_mprj_io_0 =  GPIO_MODE_MGMT_STD_INPUT_NOPULL;
    reg_mprj_io_1 =  GPIO_MODE_MGMT_STD_INPUT_NOPULL;
    reg_mprj_io_2 =  GPIO_MODE_MGMT_STD_INPUT_NOPULL;
    reg_mprj_io_3 =  GPIO_MODE_MGMT_STD_INPUT_NOPULL;
    reg_mprj_io_4 =  GPIO_MODE_MGMT_STD_INPUT_NOPULL;
    reg_mprj_io_5 =  GPIO_MODE_MGMT_STD_INPUT_NOPULL;
    reg_mprj_io_6 =  GPIO_MODE_MGMT_STD_OUTPUT;
}

void main()
{
    reg_gpio_mode1 = 1;
    reg_gpio_mode0 = 0;
    reg_gpio_ien = 1;
    reg_gpio_oe = 1;

    set_registers();
    reg_mprj_datah = 0;
    reg_mprj_datal = 0;
    gpio_config_io();

    reg_gpio_out = 1; // OFF

	while(1) {
        uint32_t xor = 0;
        for (unsigned i = 0; i < 6; i++)
            xor ^= (reg_mprj_datal >> i) & 0x1;
        reg_mprj_datal = (xor & 0x1) << 6;
        reg_mprj_datah = 0;
        reg_gpio_out = xor;
	}

}
