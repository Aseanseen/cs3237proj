from flask import Flask , render_template, jsonify, request, redirect, url_for, jsonify
from flask_sqlalchemy import SQLAlchemy
from commons.commons import CLASSIFICATION_ENUM_TO_NAME
import datetime

from analytics.controller import (
	get_plot,
	get_advice,
	get_stack_bar_plot,
	STACK_BAR_PLOT_PATH,
	get_ave
)
from utils.utils import get_base64string_from_img_path

from commons.commons import (
	CLASSIFICATION_PROPER, 
	CLASSIFICATION_FORWARD, 
	CLASSIFICATION_BACKWARD, 
	CLASSIFICATION_LEFT, 
	CLASSIFICATION_RIGHT,
	CLASSIFICATIONS
)

app = Flask(__name__)

app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///User.sqlite3'
# app.config['SQLALCHEMY_DATABASE_URI'] = 'postgres://twzrkawqgsbszj:bbea1188c97217884a56f6eb3f5a53da5010937789bacb879619a5b3f1e9e645@ec2-3-225-30-189.compute-1.amazonaws.com:5432/d46843sjl1lg0l'

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

'''
Given a name, timestamp, sensor data, classification
Add that entry into the database
'''
@app.route('/add_data', methods = ["GET", "POST", "PUT"]) 
def add_data():
	if request.method == 'PUT':
		#data = request.args.get # request the data from the form in index.html file
		name = request.args.get("name")
		timecollect = int(float(request.args.get("timecollect")))
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

'''
Given a start timestamp, end timestamp, name
Give the stack bar plot of the date and the count of bad posture
Give the percentage of time in each posture
'''
@app.route('/get_data', methods = ["GET"]) 
def get_data():
	if request.method == 'GET':
		start_time = int(request.args.get("start_time"))
		end_time = int(request.args.get("end_time"))
		name = str(request.args.get("name"))

		# Filter based on given params
		entries_to_analyse = User.query.\
			filter(User.name == name).\
			filter(User.timecollect >= start_time).\
			filter(User.timecollect <= end_time).\
			all()

		# If get is invalid, return empty dictionary
		if not entries_to_analyse:
			return {}, 200
		
		print(entries_to_analyse)
		
		list_of_datetime = [datetime.datetime.fromtimestamp(user.timecollect) for user in entries_to_analyse]
		# list_of_datetime_str = [datetime.strftime("%m/%d/%Y, %H:%M:%S") for datetime in list_of_datetime]
		list_of_classifications = [user.classification for user in entries_to_analyse]
		list_of_dates = [datetime.strftime("%m/%d/%Y") for datetime in list_of_datetime]

		result = {}

		# Creates a dictionary where key is date and values is a list of the classification in that date
		for date, classification in zip(list_of_dates, list_of_classifications):
			if date in result:
				result[date].append(classification)
			else:
				result[date] = [classification]

		# Creates a dictionary where key is date and values are count of each classification in that date
		# Sequence of the count follows the sequence of CLASSIFICATIONS
		for key in result:
			result[key] = [result[key].count(classification) for classification in CLASSIFICATIONS]

		get_stack_bar_plot(result)
		advice = get_advice(result)

		# Sends a dict of 
		# 1. a image in bas64 string format
		# 2. dict where keys are dates, values are string advice for each date
		payload = {
			"img" : get_base64string_from_img_path(STACK_BAR_PLOT_PATH),
			"advice" : advice
		}
		print(payload)
	return payload


'''
Given a name
Give a per day all time average percentage of each posture
'''
@app.route('/get_all_time_ave', methods = ["GET"]) 
def get_all_time_ave():
	if request.method == 'GET':
		name = str(request.args.get("name"))

		# Filter based on given params
		entries_to_analyse = User.query.\
			filter(User.name == name).\
			all()

		# If get is invalid, return empty dictionary
		if not entries_to_analyse:
			return {}, 200
		
		print(entries_to_analyse)
		
		list_of_datetime = [datetime.datetime.fromtimestamp(user.timecollect) for user in entries_to_analyse]
		# list_of_datetime_str = [datetime.strftime("%m/%d/%Y, %H:%M:%S") for datetime in list_of_datetime]
		list_of_classifications = [user.classification for user in entries_to_analyse]
		list_of_dates = [datetime.strftime("%m/%d/%Y") for datetime in list_of_datetime]

		result = {}

		# Creates a dictionary where key is date and values is a list of the classification in that date
		for date, classification in zip(list_of_dates, list_of_classifications):
			if date in result:
				result[date].append(classification)
			else:
				result[date] = [classification]

		# Creates a dictionary where key is date and values are count of each classification in that date
		# Sequence of the count follows the sequence of CLASSIFICATIONS
		for key in result:
			result[key] = [result[key].count(classification) for classification in CLASSIFICATIONS]

		payload = get_ave(result)

		print(payload)
	return payload

'''
Given a name
Delete every entry of that name in the database
'''
@app.route('/delete_data_all', methods = ["DELETE"]) 
def delete_data_all():
	if request.method == 'DELETE':
		name = request.args.get("name")
		entries_to_delete = User.query.filter_by(name=name).all()
		for entry in entries_to_delete:
			db.session.delete(entry)
			db.session.commit()
	return "None"

'''
Given a name
Delete every entry of that name in the database
'''
@app.route('/clear_db', methods = ["DELETE"]) 
def clear_db():
	if request.method == 'DELETE':
		for entry in User.query.all():
			db.session.delete(entry)
			db.session.commit()
	return "None"

@app.route('/get_analytics_data', methods = ["GET"]) 
def get_analytics_data():
	if request.method == 'GET':
		start_timestamp = str(request.args.get("start_time"))
		end_timestamp = str(request.args.get("end_time"))
		
		
		#TODO: Comput the result based on start_time and end_times
		#user_data = User.query.all()

		

		result = {'Straight': 20, 'Lean forward': 20, 'Lean backward': 20, 'Lean right': 20, 'Lean left': 20}
	return result, 200


if __name__=="__main__":
	db.create_all()
	app.run(debug=True)