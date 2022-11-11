from bottle import Bottle, run, template, static_file, response, request

def enable_cors(fn):
    def _enable_cors(*args, **kwargs):
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = 'GET, POST, PUT, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Origin, Accept, Content-Type, X-Requested-With, X-CSRF-Token'
        if request.method != 'OPTIONS':
            # actual request; reply with the actual response
            return fn(*args, **kwargs)
    return _enable_cors

app = Bottle()

@app.route('/', method='GET')
@enable_cors
def index():
    return template('app/index.html')

@app.route('/webpyux/core/<filename:path>', method='GET')
@enable_cors
def get_core(filename):
    return static_file(filename, root='webpyux/core')

@app.route('/webpyux/assets/<filename:path>', method='GET')
@enable_cors
def get_assets(filename):
    return static_file(filename, root='webpyux/assets')

@app.route('/app/<filename:path>', method='GET')
@enable_cors
def get_app(filename):
    return static_file(filename, root='app')

# simple token as per https://github.com/agile4you/bottle-jwt
@app.route('/session/token', method='POST')
@enable_cors
def post_token():
    return {
        "status": 200, 
        "user_data": {
            "first_name": "a",
            "last_name": "b",
            "picture": "",
            "access_token": "secret",
            "email": "a@b.com",
            "password": "hashed"
        }
    }

# simple token as per https://github.com/agile4you/bottle-jwt
@app.route('/rest/auth', method=['GET', 'POST'])
@enable_cors
def get_auth():
    return {
        "status": 200, 
        "user_data": {
            "first_name": "a",
            "last_name": "b",
            "picture": "",
            "access_token": "secret",
            "email": "a@b.com",
            "password": "hashed"
        }
    }

@app.route('/rest/login', method=['POST', 'GET'])
@enable_cors
def post_login():
    return {
        "status": 200, 
        "user_data": {
            "first_name": "a",
            "last_name": "b",
            "picture": "",
            "access_token": "secret",
            "email": "a@b.com",
            "password": "hashed"
        }
    }

run(app, host='localhost', port=8000, reloader=True)