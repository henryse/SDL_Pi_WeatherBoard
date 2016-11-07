#!/usr/bin/env bash
sudo pip install -r requirements.txt

export FLASK_APP=/home/pi/SDL_Pi_WeatherBoard/WeatherServer.py
flash run --host=0.0.0.0 --port=80
