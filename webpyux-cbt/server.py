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
    return "session_token"

# simple token as per https://github.com/agile4you/bottle-jwt
@app.route('/rest/auth', method=['GET'])
@enable_cors
def get_auth():
    return {
        "status": 200, 
        "message": "user authorized"
    }

@app.route('/rest/login', method=['POST'])
@enable_cors
def post_login():
    return {
        "status": 200, 
        "user_data": {
            "first_name": "a",
            "last_name": "b",
            "picture": "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADIAAAAyCAQAAAC0NkA6AAAAL0lEQVR42u3NIQEAAAgDMG7oXxdHBgRuK7D01LtIJBKJRCKRSCQSiUQikUgkkpsF1bQyAU6FgDwAAAAASUVORK5CYII=",
            "access_token": "secret",
            "email": "a@b.c",
            "password": "hashed"
        }
    }

@app.route('/rest/menu/test', method=['get'])
@enable_cors
def test():
    return {
   "test":{
      "label":"Test Widget",
      "dataset":{
         "widget-1":{
            "label":"Dataset Test 1",
            "widgets":[
               {
                  "label":"Widget 1",
                  "class":"test",
                  "url":"/rest/menu/test/widget/1"
               },
               {
                  "label":"Widget 2",
                  "class":"test",
                  "url":"/rest/menu/test/widget/2"
               },
               {
                  "label":"Chart Widget",
                  "class":"chart",
                  "type": "doughnut",
                  "url":"/rest/menu/test/widget/chart"
               }
            ]
         }
      }
   },
   "element_no_dataset":{
     "label":"Widget no Dataset",
     "class" :"simple_test",
     "url":"/rest/menu/test/widget/3"
   }
}

@app.route('/rest/menu/test/widget/1', method=['get'])
@enable_cors
def widget1():
    return {
        "label": "First widget with dataset",
        "list_view_dataset": "Dataset Test 1",
        "list_view_label": "Test",
        "class": "test",
        "body" : "Lorem ipsum widget test 1",
        "size": 2522
    }

@app.route('/rest/menu/test/widget/2', method=['get'])
@enable_cors
def widget2():
    return {
        "label": "First widget with dataset",
        "list_view_dataset": "Dataset Test 1",
        "list_view_label": "Test 2",
        "class": "test",
        "body" : "Lorem ipsum widget test 2",
        "size": 2522
    }

@app.route('/rest/menu/test/widget/3', method=['get'])
@enable_cors
def widget3():
    return {
        "label": "First widget with dataset",
        "list_view_dataset": "Test No dataset",
        "list_view_label": "Test 1 no dataset",
        "class": "simple_test",
        "test_custom_label_data" : "Test test test test",
        "size": 2522
    }

@app.route('/rest/menu/test/widget/chart', method=['get'])
@enable_cors
def widget4():
    return {
        "label" : "DATA" ,
            "list_view_dataset" : "DATA" ,
            "list_view_label" : "doughnut" ,
            "class" : "chart" ,
            "data" : {
                "labels" : [
                "Red" ,
                "Blue" ,
                "Yellow"
                ],
            "datasets" : [
                {
                    "label" : "chart" ,
                    "data" : [
                        300,
                        50,
                        100
                    ] ,
                    "backgroundColor" : [
                    "rgb(255, 0, 0)" ,
                    "rgb(0, 255, 0)" ,
                    "rgb(0, 0, 255)"
                    ] ,
                    "hoverOffset" : 4
                }
            ]
            } ,
            "size" : 485
    }


run(app, host='localhost', port=8000, reloader=True)