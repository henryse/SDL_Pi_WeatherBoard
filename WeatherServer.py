from flask import json
from flask import Flask
from WeatherBoard import get_weather_data
from WeatherBoard import check_weather_health

app = Flask(__name__)


@app.route('/isActive')
def get_is_active():
    return 'ACTIVE'


@app.route('/health')
def get_health():
    response = {'status': 'DOWN'}

    if check_weather_health():
        response = {'status': 'UP'}

    return json.dumps(response, ensure_ascii=False)

