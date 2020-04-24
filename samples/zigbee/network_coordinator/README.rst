.. _zigbee_network_coordinator_sample:

Zigbee: Network coordinator
###########################

This Zigbee network coordinator sample establishes the Zigbee network and commissions Zigbee devices that want to join it.

You can use this sample together with :ref:`zigbee_light_bulb_sample` and :ref:`zigbee_light_switch_sample` to set up a basic Zigbee network.

See :ref:`ug_zigbee` for more information.

Overview
********

This Zigbee network coordinator sample demonstrates the Zigbee Coordinator role.
It is a minimal implementation that supports only the network steering commissioning mechanism.

Requirements
************

* One of the following development kits:

  * |nRF52840DK| 
  * |nRF52840Dongle|
  * |nRF52833DK|

* One or both of the following samples:

  * The :ref:`zigbee_light_bulb_sample` application programmed on one or more separate devices.
  * The :ref:`zigbee_light_switch_sample` application programmed on one or more separate devices.

You can mix different development kits.

User interface
**************

LED 3:
    Indicates whether the network is open or closed.

    .. note::
         When you use the coordinator on the |nRF52840Dongle|, it is the LED 3 that informs about the successful network joining.

Button 1:
    Reopens the network for 180 seconds.

    .. note::
         The network is also opened after start-up.

Building and running
********************
.. |sample path| replace:: :file:`samples/zigbee/network_coordinator`

.. include:: /includes/build_and_run.txt

Make sure to enable the Zigbee stack before building and testing this sample.

.. _zigbee_network_coordinator_sample_testing:

Testing
=======

After programming the sample to your development kit, test it by performing the following steps:

1. Turn on the coordinator development kit. 
   When LED 3 turns on, the development kit has become the coordinator of the Zigbee network.
#. Turn on the other development kits that you programmed:

   * When LED 3 turns on on the light bulb development kit, it has become a Router inside the network.
   * When LED 3 turns on on the light switch development kit, it has become an End Device, connected directly to the Coordinator.

   If LED 3 on the other development kits does not turn on, press Button 1 on the coordinator to reopen the network.
#. Optionally, if you are testing with both the light bulb and the light switch samples, complete the following additional steps:

   #. Wait until LED 4 on the light switch development kit turns on.
      This LED indicates that the switch found a light bulb to control.
   #. Use buttons on the light switch development kit to control the light bulb, as described in the light switch User interface section.

Dependencies
************

This sample uses the following |NCS| libraries:

* Zigbee subsystem:

  * ``zb_nrf_platform.h``
  * ``zigbee_helpers.h``
  * ``zb_error_handler.h``

* :ref:`dk_buttons_and_leds_readme`

This sample uses the following `nrfxlib`_ libraries:

* ZBOSS Zigbee Stack

In addition, it uses the following Zephyr libraries:

* ``include/zephyr.h``
* ``include/device.h``
* :ref:`zephyr:settings`
* :ref:`zephyr:logger`
