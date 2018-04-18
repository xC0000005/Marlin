pio run -e malyanm200
rm /volumes/PRINTER/update.bin
cp .pioenvs/malyanm200/firmware.bin /volumes/PRINTER/update.bin
diskutil unmountDisk /dev/disk3
