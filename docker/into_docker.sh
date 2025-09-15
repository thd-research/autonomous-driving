#!/usr/bin/env bash

# Some distro requires that the absolute path is given when invoking lspci
# e.g. /sbin/lspci if the user is not root.
gpu=$(lspci | grep -i '.* vga .* nvidia .*')

docker exec -ti autonomous-driving bash
