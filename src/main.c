/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

/* Select 1 for nRF21540EK connected; select 0 for no external FEM. */
#define NRF21540_USE_FEM 1

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/devicetree.h>
#if NRF21540_USE_FEM
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#if NRF21540_USE_FEM
#define NRF21540_MAX_TX_POWER_DBM 20
#else
/* nRF54L15 maximum is +8 dBm for CSP or +7 dBm for QFN. */
#define NRF21540_MAX_TX_POWER_DBM 8
#endif
//this is has been chaanges in this fffff ----------------

#if NRF21540_USE_FEM
#define FEM_SPI_NODE DT_NODELABEL(nrf_radio_fem_spi)
#define NRF21540_REG_PARTNUMBER 0x14
#define NRF21540_PARTNUMBER_VALUE 0x0C
#define NRF21540_READ_COMMAND 0x80
#define NRF21540_MAX_REGISTER_ADDRESS 0x3F

/*
 * Antenna selection:
 *   0 = ANT1 selected/enabled, ANT2 not selected/disabled
 *   1 = ANT2 selected/enabled, ANT1 not selected/disabled
 *
 * Change this value and rebuild to select the other antenna.
 */
#define NRF21540_SELECTED_ANTENNA 0

#if (NRF21540_SELECTED_ANTENNA != 0) && (NRF21540_SELECTED_ANTENNA != 1)
#error "NRF21540_SELECTED_ANTENNA must be 0 (ANT1) or 1 (ANT2)"
#endif

#define NRF21540_PRECHECK_INIT_PRIORITY 51
#endif /* NRF21540_USE_FEM */

#if NRF21540_USE_FEM
#if !DT_NODE_HAS_STATUS(FEM_SPI_NODE, okay)
#error "The target board must provide an enabled nRF21540 FEM SPI node"
#endif

BUILD_ASSERT(IS_ENABLED(CONFIG_BT_HAS_HCI_VS),
	     "This application requires Bluetooth HCI vendor extensions");
BUILD_ASSERT(CONFIG_MPSL_FEM_INIT_PRIORITY >
		     NRF21540_PRECHECK_INIT_PRIORITY,
	     "The FEM precheck must run before MPSL claims the FEM SPI bus");
BUILD_ASSERT(CONFIG_SPI_INIT_PRIORITY <
		     NRF21540_PRECHECK_INIT_PRIORITY,
	     "The FEM SPI device must initialize before the register precheck");
#endif /* NRF21540_USE_FEM */

#if NRF21540_USE_FEM
static const struct gpio_dt_spec fem_tx_en =
	GPIO_DT_SPEC_GET(DT_NODELABEL(nrf_radio_fem), tx_en_gpios);
static const struct gpio_dt_spec fem_rx_en =
	GPIO_DT_SPEC_GET(DT_NODELABEL(nrf_radio_fem), rx_en_gpios);
static const struct gpio_dt_spec fem_pdn =
	GPIO_DT_SPEC_GET(DT_NODELABEL(nrf_radio_fem), pdn_gpios);
static const struct gpio_dt_spec fem_ant_sel =
	GPIO_DT_SPEC_GET(DT_NODELABEL(nrf_radio_fem), ant_sel_gpios);
static const struct spi_dt_spec fem_spi =
	SPI_DT_SPEC_GET(FEM_SPI_NODE,
			SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
				SPI_TRANSFER_MSB,
			0);

struct register_info {
	uint8_t address;
	const char *name;
};

static const struct register_info readable_registers[] = {
	{ 0x00, "CONFREG0" },
	{ 0x01, "CONFREG1" },
	{ 0x14, "PARTNUMBER" },
	{ 0x15, "HW_REVISION" },
	{ 0x16, "HW_ID0" },
	{ 0x17, "HW_ID1" },
};

struct register_result {
	uint8_t rx_frame[2];
	int error;
	bool attempted;
};

static struct register_result startup_register_results[
	ARRAY_SIZE(readable_registers)];
static int startup_verification_result = -EINPROGRESS;
static uint8_t startup_antenna_selected;

#endif /* NRF21540_USE_FEM */

static const struct bt_data advertising_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_le_adv_param *const advertising_parameters =
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN,
			BT_GAP_MS_TO_ADV_INTERVAL(100),
			BT_GAP_MS_TO_ADV_INTERVAL(100), NULL);

#if NRF21540_USE_FEM
static int nrf21540_read_register(uint8_t address, uint8_t rx_frame[2])
{
	uint8_t tx_frame[2] = {
		(uint8_t)(NRF21540_READ_COMMAND |
			  (address & NRF21540_MAX_REGISTER_ADDRESS)),
		0x00,
	};
	rx_frame[0] = 0;
	rx_frame[1] = 0;
	const struct spi_buf tx_buf = {
		.buf = tx_frame,
		.len = sizeof(tx_frame),
	};
	const struct spi_buf rx_buf = {
		.buf = rx_frame,
		.len = 2,
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf_set rx_set = {
		.buffers = &rx_buf,
		.count = 1,
	};
	int err;

	err = spi_transceive_dt(&fem_spi, &tx_set, &rx_set);
	return err;
}

static int initialize_fem_for_precheck(void)
{
	int err;

	if (!gpio_is_ready_dt(&fem_tx_en) ||
	    !gpio_is_ready_dt(&fem_rx_en) ||
	    !gpio_is_ready_dt(&fem_pdn) ||
	    !gpio_is_ready_dt(&fem_ant_sel)) {
		return -ENODEV;
	}
	if (!spi_is_ready_dt(&fem_spi)) {
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&fem_tx_en, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		return err;
	}
	err = gpio_pin_configure_dt(&fem_rx_en, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		return err;
	}
	err = gpio_pin_configure_dt(&fem_ant_sel, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		return err;
	}

	/* ANT_SEL=0 selects ANT1; ANT_SEL=1 selects ANT2. */
	err = gpio_pin_set_dt(&fem_ant_sel, NRF21540_SELECTED_ANTENNA);
	if (err != 0) {
		return err;
	}

	/* Read back the GPIO output state for startup reporting. */
	int ant_sel_state = gpio_pin_get_dt(&fem_ant_sel);
	if (ant_sel_state < 0) {
		return ant_sel_state;
	}
	if (ant_sel_state != NRF21540_SELECTED_ANTENNA) {
		return -EIO;
	}

	startup_antenna_selected = (uint8_t)ant_sel_state;
	err = gpio_pin_configure_dt(&fem_pdn, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		return err;
	}

	k_busy_wait(1000);
	err = gpio_pin_set_dt(&fem_pdn, 1);
	if (err != 0) {
		return err;
	}
	k_busy_wait(1000);
	return 0;
}

static int verify_nrf21540_early(void)
{
	int err;

	startup_verification_result = initialize_fem_for_precheck();
	if (startup_verification_result != 0) {
		return 0;
	}

	for (size_t i = 0; i < ARRAY_SIZE(readable_registers); i++) {
		startup_register_results[i].attempted = true;
		err = nrf21540_read_register(readable_registers[i].address,
					     startup_register_results[i].rx_frame);
		startup_register_results[i].error = err;
		if (err != 0) {
			startup_verification_result = err;	
			return 0;
		}
	}

	if (startup_register_results[2].rx_frame[1] !=
	    NRF21540_PARTNUMBER_VALUE) {
		startup_verification_result = -ENODEV;
		return 0;
	}

	startup_verification_result = 0;
	return 0;
}

SYS_INIT(verify_nrf21540_early, POST_KERNEL,
	 NRF21540_PRECHECK_INIT_PRIORITY);

static void print_startup_register_results(void)
{
	
	printk("Reading nRF21540 registers before Bluetooth starts:\n");
	for (size_t i = 0; i < ARRAY_SIZE(readable_registers); i++) {
		const struct register_result *result =
			&startup_register_results[i];
		uint8_t address = readable_registers[i].address;
		uint8_t command = NRF21540_READ_COMMAND |
				  (address & NRF21540_MAX_REGISTER_ADDRESS);

		if (!result->attempted) {
			continue;
		}
		if (result->error != 0) {
			printk("%s [0x%02X]: SPI read failed (err %d)\n",
			       readable_registers[i].name, address,
			       result->error);
			continue;
		}
		printk("%s [0x%02X]: MOSI=[0x%02X 0x00] "
		       "MISO=[0x%02X 0x%02X] value=0x%02X\n",
		       readable_registers[i].name, address, command,
		       result->rx_frame[0], result->rx_frame[1],
		       result->rx_frame[1]);
	}
}

#endif /* NRF21540_USE_FEM */

static int set_tx_power(uint8_t handle_type, uint16_t handle,
			int8_t requested_power)
{
	struct bt_hci_cp_vs_write_tx_power_level *cp;
	struct bt_hci_rp_vs_write_tx_power_level *rp;
	struct net_buf *buf;
	struct net_buf *rsp = NULL;
	int err;

	buf = bt_hci_cmd_alloc(K_FOREVER);
	if (buf == NULL) {
		printk("Could not allocate HCI TX power command buffer\n");
		return -ENOMEM;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);
	cp->handle_type = handle_type;
	cp->tx_power_level = requested_power;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				   buf, &rsp);
	if (err != 0) {
		printk("HCI TX power write failed (err %d)\n", err);
		return err;
	}

	if (rsp == NULL || rsp->len < sizeof(*rp)) {
		if (rsp != NULL) {
			net_buf_unref(rsp);
		}
		printk("HCI TX power write returned a short response\n");
		return -EBADMSG;
	}

	rp = (void *)rsp->data;
	if (rp->status != 0U) {
		printk("HCI TX power write returned status 0x%02X\n",
		       rp->status);
		net_buf_unref(rsp);
		return -EIO;
	}

	printk("TX power requested=%d dBm, controller selected=%d dBm\n",
	       requested_power, rp->selected_tx_power);
	net_buf_unref(rsp);
	return 0;
}

static int read_tx_power(uint8_t handle_type, uint16_t handle)
{
	struct bt_hci_cp_vs_read_tx_power_level *cp;
	struct bt_hci_rp_vs_read_tx_power_level *rp;
	struct net_buf *buf;
	struct net_buf *rsp = NULL;
	int err;

	buf = bt_hci_cmd_alloc(K_FOREVER);
	if (buf == NULL) {
		printk("Could not allocate HCI TX power read buffer\n");
		return -ENOMEM;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle_type = handle_type;
	cp->handle = sys_cpu_to_le16(handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
				   buf, &rsp);
	if (err != 0) {
		printk("HCI TX power read failed (err %d)\n", err);
		return err;
	}

	if (rsp == NULL || rsp->len < sizeof(*rp)) {
		if (rsp != NULL) {
			net_buf_unref(rsp);
		}
		printk("HCI TX power read returned a short response\n");
		return -EBADMSG;
	}

	rp = (void *)rsp->data;
	if (rp->status != 0U) {
		printk("HCI TX power read returned status 0x%02X\n",
		       rp->status);
		net_buf_unref(rsp);
		return -EIO;
	}

	printk("TX power readback: handle type %u handle %u = %d dBm\n",
	       handle_type, handle, rp->tx_power_level);
	net_buf_unref(rsp);
	return 0;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	uint16_t handle;
	int ret;

	if (err != 0U) {
		printk("Bluetooth connection failed (err 0x%02X)\n", err);
		return;
	}

	ret = bt_hci_get_conn_handle(conn, &handle);
	if (ret != 0) {
		printk("Could not get connection handle (err %d)\n", ret);
		return;
	}

	printk("Bluetooth connected; setting connection TX power.\n");
	ret = set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN, handle,
			   NRF21540_MAX_TX_POWER_DBM);
	if (ret == 0) {
		ret = read_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN, handle);
	}
	if (ret != 0) {
		printk("Connection TX power setup failed (err %d)\n", ret);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	printk("Bluetooth disconnected (reason 0x%02X)\n", reason);
	/*
	 * Do NOT restart advertising here. The connection object is not
	 * freed yet, so bt_le_adv_start() would fail with -ENOMEM (-12).
	 * Advertising is restarted in recycled() instead.
	 */
}

static void recycled(void)
{
	int err;

	printk("Connection object freed; restarting advertising.\n");

	err = set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0,
			   NRF21540_MAX_TX_POWER_DBM);
	if (err == 0) {
		err = read_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0);
	}
	if (err != 0) {
		printk("Advertising TX power setup failed (err %d)\n", err);
		return;
	}

	err = bt_le_adv_start(advertising_parameters, advertising_data,
			      ARRAY_SIZE(advertising_data), NULL, 0);
	if (err == -EALREADY) {
		/* Advertising is already running: nothing to do. */
		printk("Advertising already running.\n");
		return;
	}
	if (err != 0) {
		printk("Could not restart advertising (err %d)\n", err);
		return;
	}

	printk("Connectable advertising restarted.\n");
}

BT_CONN_CB_DEFINE(connection_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
	.recycled     = recycled,
};

int main(void)
{
	int err;

#if NRF21540_USE_FEM
	printk("\n=== nRF54L15 + nRF21540EK test ===\n");
	if (startup_antenna_selected == 0U) {
		printk("Antenna selected: ANT1 (ANT_SEL=0).\n");
	} else {
		printk("Antenna selected: ANT2 (ANT_SEL=1).\n");
	}
	print_startup_register_results();
	err = startup_verification_result;
	if (err != 0) {
		printk("FEM verification failed (err %d); Bluetooth will not start.\n",
		       err);
		return err;
	}

	printk("nRF21540 verified (PARTNUMBER=0x%02X).\n",
	       startup_register_results[2].rx_frame[1]);
	printk("FEM check complete; starting Bluetooth.\n");
#else
	printk("\n=== nRF54L15 without external nRF21540EK ===\n");
	printk("FEM GPIO/SPI initialization skipped.\n");
	printk("Using nRF54L15 internal radio; requested TX power=%d dBm.\n",
	       NRF21540_MAX_TX_POWER_DBM);
#endif
	err = bt_enable(NULL);
	if (err != 0) {
		printk("Bluetooth initialization failed (err %d)\n", err);
		return err;
	}

	printk("Bluetooth initialized.\n");
	err = set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0,
			   NRF21540_MAX_TX_POWER_DBM);
	if (err == 0) {
		err = read_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0);
	}
	if (err != 0) {
		printk("Advertising TX power setup failed (err %d)\n", err);
		return err;
	}

	err = bt_le_adv_start(advertising_parameters, advertising_data,
			      ARRAY_SIZE(advertising_data), NULL, 0);
	if (err != 0) {
		printk("Could not start advertising (err %d)\n", err);
		return err;
	}

	printk("Connectable advertising started.\n");
	while (1) {
		k_sleep(K_FOREVER);
	}
}
