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
    data = get_weather_data()
    response = {'EnglishOrMetric': 0,
                'OurWeatherTime': data.get('DS3231', None).get('time', None),
                'BarometricPressure': data.get('BMP280', None).get('pressure', None),
                'Altitude': data.get('BMP280', None).get('pressure', None),
                'OutdoorTemperature': data.get('AM2315', None).get('temperature', None),
                'OutdoorHumidity': data.get('AM2315', None).get('humidity', None),
                'CurrentWindSpeed': data.get('weather_rack', None).get('wind_speed', None),
                'CurrentWindGust': data.get('weather_rack', None).get('wind_gust', None),
                'CurrentWindDirection':data.get('ADS1015', None).get('wind_direction', None),
                'RainTotal': data.get('weather_rack', None).get('rain_total', None)}
    return json.dumps(response, ensure_ascii=False)


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
