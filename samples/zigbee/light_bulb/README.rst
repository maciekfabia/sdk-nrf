.. _zigbee_light_bulb_sample:

Zigbee: Light bulb
##################

This Zigbee light bulb sample demonstrates a simple light bulb whose brightness can be regulated by a different device.

You can use this sample together with :ref:`Zigbee network coordinator <zigbee_network_coordinator_sample>` and Zigbee light switch to set up a basic Zigbee network.

Overview
********

The Zigbee light bulb sample assumes the Zigbee Router role and implements the Dimmable Light profile.
This profile allows to change the brightness level of a LED on the light bulb development kit.

In the default sample configuration, the changes to the light bulb brightness are reflected on LED 4.

Requirements
************

* One or more of the following development kits:

  * |nRF52840DK|
  * |nRF52840Dongle|
  * |nRF52833DK|

* The :ref:`Zigbee network coordinator <zigbee_network_coordinator_sample>` application programmed on one separate device.
* The Zigbee light switch application programmed on one or more separate devices.

You can mix different development kits.

User interface
**************

LED 1 and LED 2:
    Indicates the |BLE| status:

    * Blinking - |BLE| advertising ongoing.
    * Slow blinking (period of 200 ms when on, period of 800 ms when off) - Device is not connecting to a Zigbee mesh network.
    * Rapid blinking (period of 100 ms) - Device is connecting to a Zigbee mesh network.
    * Solid - Device is connected over |BLE|.

LED 3:
    Indicates whether the network is open or closed.

    .. note::
         When you use the light bulb on the |nRF52840Dongle|, it is the LED 3 that informs about the successful network joining.

LED 4:
    Indicates the dimmable light option.

Button 4:
    Puts the light bulb in the Identify mode.

Building and running
********************
.. |sample path| replace:: :file:`samples/zigbee/light_bulb`

|enable_zigbee_before_testing|

.. include:: /includes/build_and_run.txt

.. _zigbee_light_bulb_sample_testing:

Testing
=======

After programming the sample to your development kits, test it by performing the following steps:

1. Turn on the coordinator development kit.
   When LED 3 turns on, this development kit has become the coordinator of the Zigbee network.
#. Turn on the light bulb development kit.
   When LED 3 turns on on the light bulb development kit, it has become a Router inside the network.

   .. tip::
        If LED 3 does not turn on, press Button 1 on the coordinator to reopen the network.

#. Turn on the light switch development kit.
   When LED 3 turns on on the light switch development kit, it has become an End Device, connected directly to the Coordinator.
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
* :ref:`zephyr:settings_api`
* :ref:`zephyr:logging_api`
* :ref:`zephyr:pwm_api`
