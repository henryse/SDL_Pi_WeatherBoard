#!/usr/bin/env bash
sudo pip install -r requirements.txt

sudo python WeatherServer.py --host=0.0.0.0 --port=80
