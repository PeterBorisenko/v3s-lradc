# Allwinner V3s LRADC
This driver allows to use low resolution keyboard ADC as a general purpose ADC
The repo is an overlay and it's contents must be written on the top of the kernel directory before compiling.
Please ensure to enable kernel configuration option adding `ADC_SYSFS` and `SUN4I_LRADC` to your `.config` file or selecting them through `menuconfig`.

The driver uses external multiplexer with 3 pins as to select the channel.
GPIO pins used to control multiplexer are selected with device tree configuration.

## How to use
The selection of channels to convert is performed through sysfs.
Selecting the channel N:
`echo N > /sys/kernel/lradc/mux/select`
Deselecting the channel N:
`echo N > /sys/kernel/lradc/mux/deselect`
Reading the data from the channel N:
`echo N > /sys/kernel/lradc/channelN/value`



### DTSI configuration example
```
adc_mux_pins: adc-mux-pins {
	pins = "PE12", "PE13", "PE14";
};
```

## Limitations
* An internal logic of LRADC is triggered on levels higher than 2.0V. Hence normal operation is corrupting if any channel input exceed that level.
* The multiplexer supports only 3 address lines.
