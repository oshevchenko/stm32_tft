#!/bin/sh
sshpass -p "123" scp ./original/*.ko root@192.168.1.1:/tmp/
sshpass -p "123" scp ./insmod_drv_on_box.sh root@192.168.1.1:/tmp/

