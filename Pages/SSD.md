
# SSD Setup
If you need to replace/add a SSD on the Jetson NX, you can easliy follow these instruction to setup. Essentially, it is same as an Ubuntu Computer. If this instruction didn't work, please google or ask ChatGPT for solution.  


### Find The Drive
```bash
$ lsblk
```
You should able to see something like "nvme0n1", I would use it as my SSD's name all time here. If your device has differnt name, you should replace "nvme0n1" with your device's name. Then, let's make a directory for the SSD as a mount point.
```bash
$ sudo mount /mnt/nvme0n1
# Then connecting the device and the mount. If you can't see your SSD in /dev, you should check the connection or your SSD.
$ sudo mount /dev/nvme0n1 /mnt/nvme0n1
```


### Create a partition table and auto-mount
```bash
#Find the device
$ sudo parted -l
#Use the parted command to create a new partition table. You can choose to use either GPT or MBR partition table. For newer systems and larger disks, GPT is generally recommended. Enter the following command in the terminal:
$ sudo parted /dev/nvme0n1 mklabel gpt
#  Double check
$ sudo blkid
# Create a new partition table, since we only need 1, I assigned as 100%
$ sudo parted -a opt /dev/nvme0n1 mkpart primary ext 0% 100%
# Check the valume and save the UUID for later
$ sudo blkid
# Format the new disk
$ sudo mkfs.ext4 /dev/nvm0n1p1
```

To mount the device automatically, copy the UUID and edit /etc/fstab. CAUTION: It might cause boot issue. If you are not sure, stop here and look for help.  
We are using Vim as our editor, you can use any editor you want but install it ahead.
```bash
$ sudo blkid
$ sudo vim /etc/fstab
# Add the UUID at the end. For instant, "UUID=your-uuid /mnt/nvme0n1 ext4 defaults 0 2". SAVE and EXIT
# Umount manually and let it mount as we set
$ sudo umount /mnt/nvme0n1
$ sudo mount -a
# Reboot and see if it works
$ sudo reboot
$ cd /mnt
$ ll
```

### Transfer files to new disk and create a link at the directory you want
Move the files taking space to your SSD and create a quick link at the same dircetory. Therefore, you can using same directory to find the files!
```bash
$ sudo mv Your_files/folders_directory /mnt/nvme0n1/
$ sudo ln -s /mnt/nvme0n1/Your_files /Previous_directory
```


