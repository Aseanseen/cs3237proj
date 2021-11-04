from flask import Flask, request, render_template
import joblib
import numpy as np
import json

# Create the Flask object
app = Flask(__name__)

# Endpoints are modeled as "routes". We create one for /. We
# specify access methods as a list to the methods parameter.

model = None

@app.route('/', methods = ['GET'])
def root():
    return 'CS3237 Team 21 Site', 200

# Examples of how to render a template. Also note
# how we use requests.args.get to extract GET parameters
@app.route('/index', methods = ['GET'])
def index():
    """ Demo routine to show how to pass parameters through GET """

    # Extract GET parameters from request object
    name = request.args.get('name')

    if name is None:
        name = 'Bob Jones'

    return render_template('index.html', info = {"title":"Hello World", "name":name}), 200

def string_to_float_list(string_of_list):
    list_of_strings = string_of_list.strip('][').split(', ')
    list_of_floats = [float(i) for i in list_of_strings]
    return list_of_floats    

@app.route('/classify', methods = ['GET'])
def classify():
    label_dict = {0: 'Good posture', 1: 'F1', 2: 'F2', 
                  3: 'F3', 4: 'F4'}
    
    # Extract GET parameters from request object
    acc_list_str = request.args.get('acc_list')
    if acc_list_str is None:
        return 'Input cannot be empty', 200
    
    input_array = string_to_float_list(acc_list_str)
    if (len(input_array) < 12):
        return 'Input must have 12 numbers', 200
    
    model_input = np.array(input_array).reshape(1, -1)
    result = model.predict(model_input)[0]
    
    #Create an answer object
    answer_dict = {}
    answer_dict['classification'] = label_dict[result]
    
    json_response = json.dumps(answer_dict, indent = 4) 
    
    return str(json_response), 200

def load_model():
    global model 
    filename = 'rfc_model.sav'
    model = joblib.load(filename)
    
# Main code

def main():
    load_model()
    global app
    app.run(port = 3237)

if __name__ == '__main__':
    main()


