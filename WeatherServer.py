from flask import json
from flask import Flask

app = Flask(__name__)


@app.route('/isActive')
def get_is_active():
    return 'ACTIVE'


@app.route('/health')
def get_health():
    response = {'status': 'DOWN'}

    # if airport_database.validate_airport_database():
    #    response = {'status': 'UP'}
    return json.dumps(response, ensure_ascii=False)

