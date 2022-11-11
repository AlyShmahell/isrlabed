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
def get_static(filename):
    return static_file(filename, root='webpyux/core')

@app.route('/webpyux/assets/<filename:path>', method='GET')
@enable_cors
def get_static(filename):
    return static_file(filename, root='webpyux/assets')

@app.route('/app/<filename:path>', method='GET')
@enable_cors
def get_static(filename):
    return static_file(filename, root='app')

# simple token as per https://github.com/agile4you/bottle-jwt
@app.route('/session/token', method='POST')
@enable_cors
def hello():
    return {"token": "secret"}

# simple token as per https://github.com/agile4you/bottle-jwt
@app.route('/rest/auth', method=['GET', 'POST'])
@enable_cors
def hello():
    return {"token": "secret"}

@app.route('/rest/login', method='POST')
@enable_cors
def hello():
    return {"status": 200, "user_data": ""}

run(app, host='localhost', port=8000, reloader=True)