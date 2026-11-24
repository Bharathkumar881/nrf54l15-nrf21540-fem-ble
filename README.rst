.. zephyr:code-sample:: bluetooth_hci_pwr_ctrl
   :name: HCI Power Control
   :relevant-api: bt_hrs bluetooth

   Dynamically control the Tx power of a Bluetooth LE Controller using HCI vendor-specific commands.

Overview
********

This application starts Bluetooth LE advertising without requiring a
connection, requests 20 dBm advertising TX power using Zephyr HCI
vendor-specific commands, and prints the requested, controller-selected, and
read-back TX-power values. It is a TX-power read-back test; it does not measure
the RF output power or read the nRF21540 internal registers. Measure the
antenna-port output with appropriate RF test equipment to determine actual
transmitted RF power.

Requirements
************

* BlueZ running on the host, or
* A board with Bluetooth LE support
* A central device & monitor (e.g. nRF Connect) to check the RSSI values
  obtained from the peripheral.

Building and Running
********************

This sample can be found under :zephyr_file:`samples/bluetooth/hci_pwr_ctrl`
in the Zephyr tree.

See :zephyr:code-sample-category:`bluetooth` samples for details.

nRF54L15 DK with nRF21540 EK (GPIO+SPI mode)
****************************************

The project includes a GPIO+SPI nRF21540 FEM configuration for the
``nrf54l15dk/nrf54l15/cpuapp`` target. It uses the P1 connections below:

* nRF21540 ``TX_EN`` -> DK ``P1.0``
* nRF21540 ``RX_EN`` -> DK ``P1.1``
* nRF21540 ``PDN`` -> DK ``P1.2``
* nRF21540 ``ANT_SEL`` -> DK ``P1.3``
* nRF21540 ``CSN`` -> DK ``P1.4``
* nRF21540 ``SCK`` -> DK ``P1.5`` (SPI21)
* nRF21540 ``MISO`` -> DK ``P1.6`` (SPI21)
* nRF21540 ``MOSI`` -> DK ``P1.7`` (SPI21)

The FEM devicetree configuration is in
``boards/nrf54l15dk_nrf54l15_cpuapp.overlay``. The project selects
``CONFIG_MPSL_FEM_NRF21540_GPIO_SPI`` and uses SPI21 at 8 MHz. The
``CONFIG_MPSL_FEM_NRF21540_RUNTIME_PA_GAIN_CONTROL`` option enables MPSL to
adjust the FEM PA gain through SPI when the requested transmit power changes.
The FEM node sets ``supply-voltage-mv = <3000>`` for the built-in power model;
change this if the nRF21540 EK is powered at a voltage other than 3.0 V.
The Bluetooth controller's
``CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL`` option is enabled so the
application's vendor-specific HCI TX-power read/write commands are supported.
Connect the EK and DK grounds together and ensure the EK is powered according
to its hardware documentation. Connect the EK SCK, MISO, MOSI and active-low
CSN pins to the listed SPI signals.

Build for this DK with:

.. code-block:: console

   west build -b nrf54l15dk/nrf54l15/cpuapp .

ANT_SEL is configured as an active-high GPIO and initialized inactive by the
FEM driver; application-level antenna switching is not implemented. The
configured POUTA/POUTB gain values use the nRF21540 factory defaults (20 dB
and 10 dB). The application requests 20 dBm and reports the value selected by
the Bluetooth controller. That read-back is not an RF power measurement or a
direct read of the nRF21540 registers; use RF test equipment to measure output
power at the antenna port.
