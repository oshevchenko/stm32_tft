#!/bin/sh
sshpass -p "123" scp root@192.168.1.1:/tmp/scrn.raw .
#dd  bs=1 count=153600 if=scrn.raw of=scrn_data.raw
dd  bs=1 count=153600 seek=70 if=scrn.raw of=scrn_image.bmp