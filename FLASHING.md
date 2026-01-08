# Flashing ESP32 on Aqara FP2

You may also find the official ESPHome flashing guide useful: https://esphome.io/guides/physical_device_connection/

## Update your device

Before starting the process, make sure you OTA update to the latest firmware with the Aqara app.

The Radar chip has separate firmware, and if you don't upgrade before flashing you might not be able to use all functionality.

We don't support updating the Radar firmware from ESPHome yet.

## Disassembling

Remove the 4 rubber plugs in the rear of the device and unscrew the 4 screws.

![back_screws.jpg](back_screws.jpg)

After the screws have been removed, pull the front and rear of the device apart firmly. Removing the rear will unplug a header connecting to the USB port.

## Connecting wires

Connecting to the device can be a little bit tricky, you will need to connect to a combination of test points + a pad on the ESP module.

I marked 2 different ground points, you may use either. You may either solder a wire to supply the board with 5V when flashing, or you may plug in the USB daughter board and use a USB cable for power.

![flashing_connections.jpg](flashing_connections.jpg)

TODO connection table

![wires_connected.jpg](wires_connected.jpg)

There are plastic clips in the case which makes removing the PCB entirely a little bit annoying. I found it easier to just prop up the board sideways with a toothpick without removing it entirely.

You may find it easier to remove the board from the case entirely, If you go for this route, make sure to unplug the antenna cable from the ESP module first.

NOTE: I connected one wire wrong in the image above, make sure to follow the diagram!

![wires_and_usb_connected.jpg](wires_and_usb_connected.jpg)

I chose to power the board with USB when flashing it here. Make sure you plug in the USB connector board the correct way if you do!

Connect the wires you soldered to the serial module you use to flash ESPs.

## Backing up firmware

Before flashing ESPHome, it is recommended to take a backup of the stock firmware on the device. If you don't do this it will likely be difficult to go back to stock if you want in the future.

Install `esptool` and run:

```
esptool read_flash 0x0 0x1000000 aqara_fp2_[homekit digits].bin
```

NOTE: It is recommended to note which specific unit the firmware came from in the filename. There may be calibration data in flash which is unit specific.

## Flashing ESPHome

Flash ESPHome firmware on your device through whatever method you prefer.

IMPORTANT: The FP2 uses a variant of the ESP32 with only one core and some other particularities. The below config block is REQUIRED in your `yaml` config for the device to boot properly.

```
esp32:
  board: esp32-solo1
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_FREERTOS_UNICORE: "y"
      CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE: "y"
    advanced:
      ignore_efuse_mac_crc: true
      ignore_efuse_custom_mac: true
```

Other than this, you may customize the config as you want. I would recommend using the `example.yaml` in the repository root as a starting point.

## Reassemble device

After flashing you may desolder the wires and reassemble the device. Just follow disassembly steps in reverse.
