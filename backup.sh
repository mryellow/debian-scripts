#!/bin/bash

rsync -aP ~/caddy/Pictures/ ~/Pictures
#rsync -aP ~/caddy/Pictures/Canon/converted/ ~/Dropbox/Pictures/Canon/converted
rsync -aP ~/caddy/Pictures/ /media/$USER/9cddbbd5-d369-4959-9aa2-240df4697642/Pictures

