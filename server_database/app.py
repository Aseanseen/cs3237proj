from flask import Flask , render_template, jsonify, request, redirect, url_for, jsonify
from flask_sqlalchemy import SQLAlchemy
from commons.commons import CLASSIFICATION_ENUM_TO_NAME
import datetime

from analytics.controller import (
	get_plot,
	get_advice
)
from utils.utils import get_base64string_from_img_path

app = Flask(__name__)

app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///User.sqlite3'
# app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///User.sqlite3'
	
db = SQLAlchemy(app)
class User(db.Model):
	# Defines the Table Name user
	__tablename__ = "user"

	# Makes three columns into the table id, name, email
	_id = db.Column(db.Integer, primary_key=True, autoincrement=True)
	name = db.Column(db.String(100))
	timecollect = db.Column(db.Integer)
	acc_x_neck = db.Column(db.Float(), nullable=False)
	acc_y_neck = db.Column(db.Float(), nullable=False)
	acc_z_neck = db.Column(db.Float(), nullable=False)
	classification = db.Column(db.Integer)


	# A constructor function where we will pass the name and email of a user and it gets add as a new entry in the table.
	def __init__(self, name, timecollect, acc_x_neck, acc_y_neck, acc_z_neck, classification):
		self.name = name
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
		name = request.args.get("name")
		timecollect = int(request.args.get("timecollect"))
		acc_x_neck = float(request.args.get("acc_x_neck"))
		acc_y_neck = float(request.args.get("acc_y_neck"))
		acc_z_neck = float(request.args.get("acc_z_neck"))
		classification = float(request.args.get("classification"))
		new_data = User(name, timecollect, acc_x_neck, acc_y_neck, acc_z_neck, classification)
		db.session.add(new_data)
		db.session.commit()

		user_data = User.query.all()
		return render_template("demo3237.html", user_data = user_data) # passes user_data variable into the index.html file.

	return render_template("demo3237.html", user_data = User.query.all())

@app.route('/get_data', methods = ["GET"]) 
def get_data():
	if request.method == 'GET': # When a user clicks submit button it will come here.
		start_time = int(request.args.get("start_time"))
		end_time = int(request.args.get("end_time"))
		name = request.args.get("name")
		entries_to_analyse = User.query.\
			filter(User.name == name).\
			filter(User.timecollect >= start_time).\
			filter(User.timecollect <= end_time).\
			all()
		
		list_of_timestamps = [datetime.datetime.fromtimestamp(user.timecollect) for user in entries_to_analyse]
		list_of_timestamps_str = [timestamp.strftime("%m/%d/%Y, %H:%M:%S") for timestamp in list_of_timestamps]
		list_of_classifications = [user.classification for user in entries_to_analyse]
		print(list_of_timestamps_str)
		print(list_of_classifications)

		print(entries_to_analyse)

		get_plot(list_of_classifications, list_of_timestamps)
		
		advice = get_advice(list_of_classifications, list_of_timestamps)
		#TODO: Comput the result based on start_time and end_times
		#user_data = User.query.all()
		payload = {
			"img" : get_base64string_from_img_path("analytics.png"),
			"advice" : advice
		}
		print(payload)
		return payload



		result = {'Straight': 20, 'Lean forward': 20, 'Lean backward': 20, 'Lean right': 20, 'Lean left': 20}
	return result, 200

@app.route('/remove_data_all', methods = ["POST"]) 
def remove_data_all():
	if request.method == 'POST': # When a user clicks submit button it will come here.
		name = request.args.get("name")
		entries_to_delete = User.query.filter_by(name=name).all()
		for entry in entries_to_delete:
			db.session.delete(entry)
			db.session.commit()

	return "None", 200


@app.route('/get_analytics_data', methods = ["GET"]) 
def get_analytics_data():
	if request.method == 'GET': # When a user clicks submit button it will come here.
		start_timestamp = str(request.args.get("start_time"))
		end_timestamp = str(request.args.get("end_time"))
		
		
		#TODO: Comput the result based on start_time and end_times
		#user_data = User.query.all()

		

		result = {'Straight': 20, 'Lean forward': 20, 'Lean backward': 20, 'Lean right': 20, 'Lean left': 20}
	return result, 200


if __name__=="__main__":
	db.create_all()
	app.run(debug=True)