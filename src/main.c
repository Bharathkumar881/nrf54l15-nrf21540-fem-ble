/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * nRF54L15 DK with OPTIONAL nRF21540EK - one firmware for both cases.
 *
 * At boot (before Bluetooth starts) the firmware reads the nRF21540
 * PARTNUMBER register over SPI:
 *
 *   PARTNUMBER == 0x0C  -> "nRF21540EK detected: YES"
 *                          The FEM is used, TX power = +20 dBm on air
 *                          (nRF54L15 ~0 dBm + nRF21540 PA +20 dB).
 *
 *   anything else       -> "nRF21540EK detected: NO"
 *                          FEM pins are parked (PDN low = FEM off) and the
 *                          nRF54L15 radio is driven at its own maximum
 *                          (+8 dBm) through the DK's PCB antenna.
 *
 * Bluetooth starts in both cases. No rebuild is needed when you plug or
 * unplug the EK - just reset the board.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <mpsl_fem_power_model.h>
#include <mpsl_fem_protocol_api.h>
#include <mpsl_tx_power.h>

/* Wanted on-air TX power when the nRF21540EK is present. */
#define TX_POWER_WITH_FEM_DBM 25

/* Wanted on-air TX power without the EK (nRF54L15 radio maximum). */
#define TX_POWER_SOC_ONLY_DBM 10

/* Channel used only to ask MPSL how it splits TX power (2440 MHz = mid band). */
#define POWER_SPLIT_FREQ_MHZ 2440

#define FEM_SPI_NODE DT_NODELABEL(nrf_radio_fem_spi)
#define NRF21540_PARTNUMBER_INDEX 2 /* index in readable_registers[] */
#define NRF21540_PARTNUMBER_VALUE 0x0C
#define NRF21540_READ_COMMAND 0x80
#define NRF21540_MAX_REGISTER_ADDRESS 0x3F

/*
 * Antenna selection on the EK:
 *   0 = ANT1, 1 = ANT2. Put your antenna on the matching SMA connector.
 */
#define NRF21540_SELECTED_ANTENNA 0

#if (NRF21540_SELECTED_ANTENNA != 0) && (NRF21540_SELECTED_ANTENNA != 1)
#error "NRF21540_SELECTED_ANTENNA must be 0 (ANT1) or 1 (ANT2)"
#endif

/* Must run after SPI init and before MPSL takes over the FEM pins. */
#define NRF21540_DETECT_INIT_PRIORITY 51

#if !DT_NODE_HAS_STATUS(FEM_SPI_NODE, okay)
#error "The target board must provide an enabled nRF21540 FEM SPI node"
#endif

BUILD_ASSERT(IS_ENABLED(CONFIG_BT_HAS_HCI_VS),
	     "This application requires Bluetooth HCI vendor extensions");
BUILD_ASSERT(CONFIG_MPSL_FEM_INIT_PRIORITY > NRF21540_DETECT_INIT_PRIORITY,
	     "FEM detection must run before MPSL claims the FEM SPI bus");
BUILD_ASSERT(CONFIG_SPI_INIT_PRIORITY < NRF21540_DETECT_INIT_PRIORITY,
	     "The FEM SPI device must initialize before FEM detection");

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

/* Result of detection, filled in before main() runs. */
static int fem_detect_error = -EINPROGRESS; /* 0 = SPI reads OK */
static bool fem_present;

/* TX power value sent to the controller (decided in main()). */
static int8_t tx_power_request_dbm;

static const struct bt_data advertising_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_le_adv_param *const advertising_parameters =
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN,
			BT_GAP_MS_TO_ADV_INTERVAL(100),
			BT_GAP_MS_TO_ADV_INTERVAL(100), NULL);

/* ------------------------------------------------------------------ */
/* nRF21540 detection                                                  */
/* ------------------------------------------------------------------ */

static int nrf21540_read_register(uint8_t address, uint8_t rx_frame[2])
{
	uint8_t tx_frame[2] = {
		(uint8_t)(NRF21540_READ_COMMAND |
			  (address & NRF21540_MAX_REGISTER_ADDRESS)),
		0x00,
	};
	const struct spi_buf tx_buf = { .buf = tx_frame, .len = sizeof(tx_frame) };
	const struct spi_buf rx_buf = { .buf = rx_frame, .len = 2 };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	rx_frame[0] = 0;
	rx_frame[1] = 0;
	return spi_transceive_dt(&fem_spi, &tx_set, &rx_set);
}

static int fem_pins_configure(void)
{
	int err;

	if (!gpio_is_ready_dt(&fem_tx_en) || !gpio_is_ready_dt(&fem_rx_en) ||
	    !gpio_is_ready_dt(&fem_pdn) || !gpio_is_ready_dt(&fem_ant_sel) ||
	    !spi_is_ready_dt(&fem_spi)) {
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&fem_tx_en, GPIO_OUTPUT_INACTIVE);
	if (err == 0) {
		err = gpio_pin_configure_dt(&fem_rx_en, GPIO_OUTPUT_INACTIVE);
	}
	if (err == 0) {
		err = gpio_pin_configure_dt(&fem_ant_sel,
					    NRF21540_SELECTED_ANTENNA ?
						    GPIO_OUTPUT_ACTIVE :
						    GPIO_OUTPUT_INACTIVE);
	}
	if (err == 0) {
		err = gpio_pin_configure_dt(&fem_pdn, GPIO_OUTPUT_INACTIVE);
	}
	return err;
}

/* PDN low = nRF21540 powered down. Used when no EK is found. */
static void fem_pins_park(void)
{
	(void)gpio_pin_set_dt(&fem_tx_en, 0);
	(void)gpio_pin_set_dt(&fem_rx_en, 0);
	(void)gpio_pin_set_dt(&fem_pdn, 0);
}

static int detect_nrf21540_early(void)
{
	int err;

	fem_present = false;

	err = fem_pins_configure();
	if (err != 0) {
		fem_detect_error = err;
		return 0;
	}

	/* Wake the nRF21540 (PDN high) so it can answer on SPI. */
	k_busy_wait(1000);
	(void)gpio_pin_set_dt(&fem_pdn, 1);
	k_busy_wait(1000);

	for (size_t i = 0; i < ARRAY_SIZE(readable_registers); i++) {
		startup_register_results[i].attempted = true;
		err = nrf21540_read_register(readable_registers[i].address,
					     startup_register_results[i].rx_frame);
		startup_register_results[i].error = err;
		if (err != 0) {
			fem_detect_error = err;
			fem_pins_park();
			return 0;
		}
	}

	fem_detect_error = 0;
	fem_present = (startup_register_results[NRF21540_PARTNUMBER_INDEX]
			       .rx_frame[1] == NRF21540_PARTNUMBER_VALUE);

	if (!fem_present) {
		fem_pins_park();
	}
	return 0;
}

SYS_INIT(detect_nrf21540_early, POST_KERNEL, NRF21540_DETECT_INIT_PRIORITY);

static void print_startup_register_results(void)
{
	printk("FEM pin map: TX_EN=%s.%u RX_EN=%s.%u PDN=%s.%u ANT_SEL=%s.%u\n",
	       fem_tx_en.port->name, fem_tx_en.pin,
	       fem_rx_en.port->name, fem_rx_en.pin,
	       fem_pdn.port->name, fem_pdn.pin,
	       fem_ant_sel.port->name, fem_ant_sel.pin);
	printk("Reading nRF21540 registers:\n");

	for (size_t i = 0; i < ARRAY_SIZE(readable_registers); i++) {
		const struct register_result *r = &startup_register_results[i];
		uint8_t address = readable_registers[i].address;

		if (!r->attempted) {
			continue;
		}
		if (r->error != 0) {
			printk("  %-11s [0x%02X]: SPI read failed (err %d)\n",
			       readable_registers[i].name, address, r->error);
			continue;
		}
		printk("  %-11s [0x%02X]: MISO=[0x%02X 0x%02X] value=0x%02X\n",
		       readable_registers[i].name, address, r->rx_frame[0],
		       r->rx_frame[1], r->rx_frame[1]);
	}
}

/* ------------------------------------------------------------------ */
/* FEM power model (replaces the MPSL built-in nRF21540 model)         */
/* ------------------------------------------------------------------ */

/*
 * MPSL asks this model how to split a requested on-air power into
 * "nRF54L15 radio power" + "nRF21540 PA setting".
 *
 *   EK present : radio = requested - PA gain (POUTA, ~20 dB)
 *   EK absent  : radio = requested            (PA gain = 0 dB)
 *
 * Without this, MPSL always assumes the 20 dB PA gain, and since the
 * Bluetooth controller caps requests at +20 dBm the radio would never go
 * above 0 dBm when the EK is missing.
 */
#if defined(CONFIG_APP_FEM_POWER_MODEL_AUTO_DETECT)

/* Defaults until MPSL hands us the chip's calibration data. */
static mpsl_fem_calibration_point_t pa_pouta = {
	.gain_db = CONFIG_MPSL_FEM_NRF21540_TX_GAIN_DB_POUTA,
	.fem_pa_power_control = 0,
};

static void app_power_model_init(const mpsl_fem_calibration_data_t *cal)
{
	/* Only trust calibration data that came from a real nRF21540. */
	if (fem_present && cal != NULL &&
	    cal->nrf21540_gpio_spi.pouta.gain_db > 0) {
		pa_pouta = cal->nrf21540_gpio_spi.pouta;
	}
}

/* Runs in radio interrupt context: must stay short (< 2 us). */
static void app_power_model_fetch(int8_t requested_power, mpsl_phy_t phy,
				  uint16_t freq_mhz,
				  mpsl_fem_power_model_output_t *out,
				  bool tx_power_ceiling)
{
	ARG_UNUSED(phy);
	ARG_UNUSED(freq_mhz);

	int8_t pa_gain = fem_present ? pa_pouta.gain_db : 0;

	out->soc_pwr = mpsl_tx_power_radio_supported_power_adjust(
		(mpsl_tx_power_t)(requested_power - pa_gain), tx_power_ceiling);
	out->fem_pa_power_control = pa_pouta.fem_pa_power_control;
	out->achieved_pwr = out->soc_pwr + pa_gain;
}

static const mpsl_fem_power_model_t app_power_model = {
	.fetch = app_power_model_fetch,
	.init = app_power_model_init,
};

/* Called by the NCS FEM driver (priority 60, after detection at 51). */
const mpsl_fem_power_model_t *mpsl_fem_power_model_to_use_get(void);
const mpsl_fem_power_model_t *mpsl_fem_power_model_to_use_get(void)
{
	return &app_power_model;
}

#endif /* CONFIG_APP_FEM_POWER_MODEL_AUTO_DETECT */

/* ------------------------------------------------------------------ */
/* TX power selection                                                  */
/* ------------------------------------------------------------------ */

/* What the nRF54L15 radio itself outputs for a given on-air request. */
static int8_t soc_radio_power_for_request(int8_t request_dbm)
{
	mpsl_tx_power_split_t split;

	(void)mpsl_fem_tx_power_split(request_dbm, &split, MPSL_PHY_BLE_1M,
				      POWER_SPLIT_FREQ_MHZ, false);
	return split.radio_tx_power;
}

static void choose_tx_power(void)
{
	tx_power_request_dbm = fem_present ? TX_POWER_WITH_FEM_DBM :
					     TX_POWER_SOC_ONLY_DBM;
}

static void print_on_air_power(int8_t controller_selected_dbm)
{
	int8_t soc_dbm = soc_radio_power_for_request(controller_selected_dbm);

	if (fem_present) {
		printk("On-air TX power: nRF54L15 %d dBm + nRF21540 PA "
		       "%d dB = %d dBm (out of EK ANT%d)\n",
		       soc_dbm, controller_selected_dbm - soc_dbm,
		       controller_selected_dbm, NRF21540_SELECTED_ANTENNA + 1);
	} else {
		printk("On-air TX power: %d dBm (nRF54L15 only, DK PCB antenna)\n",
		       soc_dbm);
	}
}

/* ------------------------------------------------------------------ */
/* HCI vendor-specific TX power commands                               */
/* ------------------------------------------------------------------ */

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

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, buf, &rsp);
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
		printk("HCI TX power write returned status 0x%02X\n", rp->status);
		net_buf_unref(rsp);
		return -EIO;
	}

	printk("TX power: requested=%d dBm, controller selected=%d dBm\n",
	       requested_power, rp->selected_tx_power);
	print_on_air_power(rp->selected_tx_power);
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

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_TX_POWER_LEVEL, buf, &rsp);
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
		printk("HCI TX power read returned status 0x%02X\n", rp->status);
		net_buf_unref(rsp);
		return -EIO;
	}

	printk("TX power readback: handle type %u handle %u = %d dBm\n",
	       handle_type, handle, rp->tx_power_level);
	net_buf_unref(rsp);
	return 0;
}

static int apply_tx_power(uint8_t handle_type, uint16_t handle)
{
	int err = set_tx_power(handle_type, handle, tx_power_request_dbm);

	if (err == 0) {
		err = read_tx_power(handle_type, handle);
	}
	return err;
}

/* ------------------------------------------------------------------ */
/* Connection callbacks                                                */
/* ------------------------------------------------------------------ */

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
	ret = apply_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN, handle);
	if (ret != 0) {
		printk("Connection TX power setup failed (err %d)\n", ret);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	printk("Bluetooth disconnected (reason 0x%02X)\n", reason);
	/* Advertising is restarted in recycled(), when the conn object is free. */
}

static void recycled(void)
{
	int err;

	printk("Connection object freed; restarting advertising.\n");

	err = apply_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0);
	if (err != 0) {
		printk("Advertising TX power setup failed (err %d)\n", err);
		return;
	}

	err = bt_le_adv_start(advertising_parameters, advertising_data,
			      ARRAY_SIZE(advertising_data), NULL, 0);
	if (err == -EALREADY) {
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

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
	int err;

	printk("\n=== nRF54L15 DK + optional nRF21540EK ===\n");
	print_startup_register_results();

	if (fem_detect_error != 0) {
		printk("FEM SPI/GPIO error (err %d) - treating as no EK.\n",
		       fem_detect_error);
	}

	if (fem_present) {
		printk("nRF21540EK detected: YES (PARTNUMBER=0x%02X)\n",
		       startup_register_results[NRF21540_PARTNUMBER_INDEX]
			       .rx_frame[1]);
		printk("Mode: nRF54L15 + nRF21540 PA, antenna ANT%d on the EK.\n",
		       NRF21540_SELECTED_ANTENNA + 1);
		printk("  -> RF coax DK->EK must be connected.\n");
	} else {
		printk("nRF21540EK detected: NO (PARTNUMBER=0x%02X, expected 0x%02X)\n",
		       startup_register_results[NRF21540_PARTNUMBER_INDEX]
			       .rx_frame[1],
		       NRF21540_PARTNUMBER_VALUE);
		printk("Mode: nRF54L15 DK only, DK PCB antenna. FEM pins parked.\n");
		printk("  -> RF coax must be REMOVED from the DK.\n");
	}

	choose_tx_power();

	err = bt_enable(NULL);
	if (err != 0) {
		printk("Bluetooth initialization failed (err %d)\n", err);
		return err;
	}
	printk("Bluetooth initialized.\n");

	err = apply_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0);
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
