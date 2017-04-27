sudo chmod 640 /etc/redis/redis.conf
sudo aptitude purge
#debsums
sudo sh -c 'echo "nameserver 8.8.8.8" > /etc/resolv.conf'
sudo sh -c 'echo "nameserver 8.8.4.4" > /etc/resolv.conf'
sudo apt-get install arpwatch
sudo arpwatch -i wlan0

/etc/ssh/sshd_config
AllowTcpForwarding no
ClientAliveCountMax 2
Compression no
MaxAuthTries 2
MaxSessions 2
UseDNS no
