from flask import json
from flask import Flask
from WeatherBoard import check_weather_health
from WeatherBoard import get_weather_data

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


@app.route('/')
def get_weather():
    return json.dumps(get_weather_data(), ensure_ascii=False)


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
