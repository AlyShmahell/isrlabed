from bottle import Bottle, run, template, static_file

app = Bottle()

@app.get('/')
def index():
    return template('app/index.html')

@app.get('/webpyux/core/<filename:path>')
def get_static(filename):
    return static_file(filename, root='webpyux/core')

@app.get('/webpyux/assets/<filename:path>')
def get_static(filename):
    return static_file(filename, root='webpyux/assets')

@app.get('/app/<filename:path>')
def get_static(filename):
    return static_file(filename, root='app')

@app.post('/session/token')
def hello():
    return "token"

@app.post('/rest/auth')
def hello():
    return "auth"

@app.post('/rest/login')
def hello():
    return {"status": 200, "user_data": ""}

run(app, host='localhost', port=8000, reloader=True)