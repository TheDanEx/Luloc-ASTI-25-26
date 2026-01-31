#!/bin/bash

sudo apt update
sudo apt install gpiod libgpiod-dev

echo "Para compilar: g++ main.cpp -o my_gpio_app -lgpiodcxx"
