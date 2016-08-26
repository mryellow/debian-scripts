# Pre-requisites

```
# As root
adduser username sudo

# As user
sudo apt-get install -y git
git config --global user.name "Username"
git config --global user.email "email@example.com"
git config --global push.default simple
ssh-keygen -t rsa -b 4096 -C "email@example.com"
git clone git@github.com:mryellow/debian-scripts.git ~/scripts
```

# Scripts for Debian desktop setup

```
# Post-install configuration
./scripts/post-install.sh
```
