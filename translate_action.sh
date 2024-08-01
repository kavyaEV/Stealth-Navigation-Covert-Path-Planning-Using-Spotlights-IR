#!/bin/bash

awk -W interactive -F : '/^action/ {if ($2 == 0) move="up"; else if ($2 == 1) move="down"; else if ($2 == 2) move="left"; else move="right"; printf "move -> %s\n", move}'
