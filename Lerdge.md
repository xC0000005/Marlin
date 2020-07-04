# Lerdge Support in Marlin

This branch is a combination of pull reqeuests, combining:
Lerdge support (for pins and automatic encryption), 
TFT + touch screen (for LCD, SDIO and XPT)
STM32 core update to 8.0 (for I2C fixes)

This has been tested on Lerdge-S and Lerdge-X. Lerdge-K support is underway, but the K board has more bells and whistles, and anything more than a basic (read, no UART/SPI TMC) config is unlikely to work yet.

Thermistor/Thermocouples may have a few bugs - I'll update this document when it's worked out.

##How to build for Lerdge X, S, K in Marlin:

1. Set your motherboard to S, X, or K.
2. If you are running an S, change TFT_DRIVER to ST7796 instead of LERDGE_ST7796
3. Enable the features you want, setting the steps and so on. Once the lerdge support PR is merged
   it may be easier to start from an example config.
4. In the PIO terminal, run: pio run -e [board, where board is LERDGEX, LERDGES, or LERDGEK]
5. The resulting firmware is encrypted for a disk based update. This means you put it on an SD card
   in the correct directory structure ("Lerdge_[X,K]_System\Firmware\ [for S, 'Firmwave\']) and power cycle
6. Calibrate your touch screen. To do so, touch and hold on the status screen until calibration starts.
    Use a stylus or calibration may fail. Do not end calibraiton with the rotary encoder click (this is a bug which will be fixed soon)

