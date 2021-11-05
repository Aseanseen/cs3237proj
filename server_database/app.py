from flask import Flask , render_template, jsonify, request, redirect, url_for, jsonify
from flask_sqlalchemy import SQLAlchemy

app = Flask(__name__)

app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///User.sqlite3'
	
db = SQLAlchemy(app)
class User(db.Model):
	# Defines the Table Name user
	__tablename__ = "user"

	# Makes three columns into the table id, name, email
	_id = db.Column(db.Integer, primary_key=True, autoincrement=True)
	timecollect = db.Column(db.Integer)
	acc_x_neck = db.Column(db.Float(), nullable=False)
	acc_y_neck = db.Column(db.Float(), nullable=False)
	acc_z_neck = db.Column(db.Float(), nullable=False)
	classification = db.Column(db.Integer)


	# A constructor function where we will pass the name and email of a user and it gets add as a new entry in the table.
	def __init__(self, timecollect, acc_x_neck, acc_y_neck, acc_z_neck, classification):
		self.timecollect = timecollect
		self.acc_x_neck = acc_x_neck
		self.acc_y_neck = acc_y_neck
		self.acc_z_neck = acc_z_neck
		self.classification = classification

# Control will come here and then gets redirect to the index function
@app.route("/")
def home():
	return render_template("demo3237.html", user_data = User.query.all())

@app.route('/add_data', methods = ["GET", "POST", "PUT"]) 
def add_data():
	if request.method == 'PUT': # When a user clicks submit button it will come here.
		#data = request.args.get # request the data from the form in index.html file
		timecollect = int(request.args.get("timecollect"))
		acc_x_neck = float(request.args.get("acc_x_neck"))
		acc_y_neck = float(request.args.get("acc_y_neck"))
		acc_z_neck = float(request.args.get("acc_z_neck"))
		classification = float(request.args.get("classification"))
		new_data = User(timecollect, acc_x_neck, acc_y_neck, acc_z_neck, classification)
		db.session.add(new_data)
		db.session.commit()

		user_data = User.query.all()
		return render_template("demo3237.html", user_data = user_data) # passes user_data variable into the index.html file.

	return render_template("demo3237.html", user_data = User.query.all())

@app.route('/get_data', methods = ["GET"]) 
def get_data():
	if request.method == 'GET': # When a user clicks submit button it will come here.
		start_time = int(request.args.get("start_time"))
		end_time = float(request.args.get("end_time"))
		
		#TODO: Comput the result based on start_time and end_times
		#user_data = User.query.all()

		result = {'Straight': 20, 'Lean forward': 20, 'Lean backward': 20, 'Lean right': 20, 'Lean left': 20}
	return result, 200

if __name__=="__main__":
	db.create_all()
	app.run(debug=True)