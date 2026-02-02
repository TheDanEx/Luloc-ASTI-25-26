#!/bin/bash

sudo apt update
sudo apt install python3
sudo apt install build-essential -y

echo "Para compilar: g++ main.cpp -o my_gpio_app -lgpiodcxx"
