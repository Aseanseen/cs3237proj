from flask import Flask , render_template, jsonify, request, redirect, url_for, jsonify
from flask_sqlalchemy import SQLAlchemy
from commons.commons import CLASSIFICATION_ENUM_TO_NAME
import datetime

from analytics.controller import(
get_plot,
get_stack_bar_plot,
# get_advice,
get_sum,
get_pie_plot
)

from analytics.paths import(
STACK_BAR_PLOT_DATE_PATH,
STACK_BAR_PLOT_DATE_BAD_PATH,

STACK_BAR_PLOT_HOUR_PATH,
STACK_BAR_PLOT_HOUR_ALL_PATH,

STACK_BAR_PLOT_HOUR_BAD_PATH,
STACK_BAR_PLOT_HOUR_BAD_ALL_PATH,

PIE_PATH,
PIE_ALL_PATH
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
# app.config['SQLALCHEMY_DATABASE_URI'] = 'postgres://mfujthfmebgsad:39eadddca0737a0aefaf2c00568fcb6bcd96342f00d8dfcbe1e9fc82811c6c07@ec2-3-230-149-158.compute-1.amazonaws.com:5432/d6gkgi2oa7pokv'

db = SQLAlchemy(app)

class User(db.Model):
	# Defines the Table Name user
	__tablename__ = "user"

	# Makes three columns into the table id, name, email
	_id = db.Column(db.Integer, primary_key=True, autoincrement=True)
	name = db.Column(db.String(100))
	timecollect = db.Column(db.Integer)
	classification = db.Column(db.Integer)


	# A constructor function where we will pass the name and email of a user and it gets add as a new entry in the table.
	def __init__(self, name, timecollect, classification):
		self.name = name
		self.timecollect = timecollect
		self.classification = classification
		

# Control will come here and then gets redirect to the index function
@app.route("/")
def home():
	list_of_date = [datetime.datetime.fromtimestamp(user.timecollect).date() for user in User.query.all()]
	return render_template("demo3237.html", user_data = User.query.all(), list_of_date = list_of_date)

'''
Given a name, timestamp, sensor data, classification
Add that entry into the database
'''
@app.route('/add_data', methods = ["PUT"]) 
def add_data():
	if request.method == 'PUT':
		#data = request.args.get # request the data from the form in index.html file
		name = request.args.get("name")
		timecollect = int(float(request.args.get("timecollect")))
		classification = int(float(request.args.get("classification")))
		new_data = User(name, timecollect, classification)
		db.session.add(new_data)
		db.session.commit()

		user_data = User.query.all()
		list_of_date = [datetime.datetime.fromtimestamp(user.timecollect).date() for user in user_data]
		return render_template("demo3237.html", user_data = user_data, list_of_date = list_of_date)

'''
Given a start timestamp, end timestamp, name
Show the filtered results in html
'''
@app.route('/get_data', methods = ["GET"])
def get_data():
	if request.method == 'GET':
		name = str(request.args.get("name"))
		start_time = request.args.get("start_time")
		end_time = request.args.get("end_time")

		if name and (not start_time) and (not end_time):
			# Filter based on given params
			entries_to_analyse = User.query.\
				filter(User.name == name).\
				all()
			list_of_date = [datetime.datetime.fromtimestamp(user.timecollect).date() for user in entries_to_analyse]
			return render_template("demo3237.html", user_data = entries_to_analyse, list_of_date = list_of_date)
		else:
			start_time = int(start_time)
			end_time = int(end_time)
			# Filter based on given params
			entries_to_analyse = User.query.\
				filter(User.name == name).\
				filter(User.timecollect >= start_time).\
				filter(User.timecollect <= end_time).\
				all()
			list_of_date = [datetime.datetime.fromtimestamp(user.timecollect).date() for user in entries_to_analyse]
			return render_template("demo3237.html", user_data = entries_to_analyse, list_of_date = list_of_date)

'''
Given a start timestamp, end timestamp, name
Give the stack bar plot of the date
Give the percentage of time in each posture
'''
@app.route('/get_stack_bar_plot_date', methods = ["GET"]) 
def get_stack_bar_plot_date():
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
			return {}
		
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

		# Saves a stack bar plot to the path specified
		get_stack_bar_plot(result, STACK_BAR_PLOT_DATE_PATH)
		payload = get_base64string_from_img_path(STACK_BAR_PLOT_DATE_PATH)
		# advice = get_advice(result)

		# # Sends a dict of 
		# # 1. a image in bas64 string format
		# # 2. dict where keys are dates, values are string advice for each date
		# payload = {
		# 	"img" : get_base64string_from_img_path(STACK_BAR_PLOT_PATH),
		# 	"advice" : advice
		# }
		# print(payload)
	return payload

'''
Given a start timestamp, end timestamp, name
Give the stack bar plot of the date
Give the percentage of time in each posture
'''
@app.route('/get_stack_bar_plot_date_bad', methods = ["GET"]) 
def get_stack_bar_plot_date_bad():
	if request.method == 'GET':
		start_time = int(request.args.get("start_time"))
		end_time = int(request.args.get("end_time"))
		name = str(request.args.get("name"))

		# Filter based on given params
		entries_to_analyse = User.query.\
			filter(User.name == name).\
			filter(User.timecollect >= start_time).\
			filter(User.timecollect <= end_time).\
			filter(User.classification != CLASSIFICATION_PROPER).\
			all()

		# If get is invalid, return empty dictionary
		if not entries_to_analyse:
			return {}
		
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

		# Saves a stack bar plot to the path specified
		get_stack_bar_plot(result, STACK_BAR_PLOT_DATE_BAD_PATH)
		payload = get_base64string_from_img_path(STACK_BAR_PLOT_DATE_BAD_PATH)
		# advice = get_advice(result)

		# # Sends a dict of 
		# # 1. a image in bas64 string format
		# # 2. dict where keys are dates, values are string advice for each date
		# payload = {
		# 	"img" : get_base64string_from_img_path(STACK_BAR_PLOT_PATH),
		# 	"advice" : advice
		# }
		# print(payload)
	return payload

'''
Given a start timestamp, end timestamp, name
Give the plot of the hours of the day
Give the percentage of time in each posture
'''
@app.route('/get_stack_bar_plot_hour', methods = ["GET"]) 
def get_stack_bar_plot_hour():
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
			return {}
		
		list_of_datetime = [datetime.datetime.fromtimestamp(user.timecollect) for user in entries_to_analyse]
		# list_of_datetime_str = [datetime.strftime("%m/%d/%Y, %H:%M:%S") for datetime in list_of_datetime]
		list_of_classifications = [user.classification for user in entries_to_analyse]
		list_of_hours = [int(datetime.strftime("%H")) for datetime in list_of_datetime]

		result = {}

		# Creates a dictionary where key is hour and values is a list of the classification in that date
		for hour, classification in zip(list_of_hours, list_of_classifications):
			if hour in result:
				result[hour].append(classification)
			else:
				result[hour] = [classification]

		# Creates a dictionary where key is date and values are count of each classification in that date
		# Sequence of the count follows the sequence of CLASSIFICATIONS
		for key in result:
			result[key] = [result[key].count(classification) for classification in CLASSIFICATIONS]

		# Saves a stack bar plot to the path specified
		get_stack_bar_plot(result, STACK_BAR_PLOT_HOUR_PATH)

		payload = get_base64string_from_img_path(STACK_BAR_PLOT_HOUR_PATH)
	return payload

'''
Given a name
Give the plot of the hours of the day and the count of bad posture
Give the percentage of time in each posture
'''
@app.route('/get_stack_bar_plot_hour_all', methods = ["GET"]) 
def get_stack_bar_plot_hour_all():
	if request.method == 'GET':
		name = str(request.args.get("name"))

		# Filter based on given params
		entries_to_analyse = User.query.\
			filter(User.name == name).\
			all()

		# If get is invalid, return empty dictionary
		if not entries_to_analyse:
			return {}
		
		list_of_datetime = [datetime.datetime.fromtimestamp(user.timecollect) for user in entries_to_analyse]
		# list_of_datetime_str = [datetime.strftime("%m/%d/%Y, %H:%M:%S") for datetime in list_of_datetime]
		list_of_classifications = [user.classification for user in entries_to_analyse]
		list_of_hours = [int(datetime.strftime("%H")) for datetime in list_of_datetime]

		result = {}

		# Creates a dictionary where key is hour and values is a list of the classification in that date
		for hour, classification in zip(list_of_hours, list_of_classifications):
			if hour in result:
				result[hour].append(classification)
			else:
				result[hour] = [classification]

		# Creates a dictionary where key is date and values are count of each classification in that date
		# Sequence of the count follows the sequence of CLASSIFICATIONS
		for key in result:
			result[key] = [result[key].count(classification) for classification in CLASSIFICATIONS]

		# Saves a stack bar plot to the path specified
		get_stack_bar_plot(result, STACK_BAR_PLOT_HOUR_ALL_PATH)

		payload = get_base64string_from_img_path(STACK_BAR_PLOT_HOUR_ALL_PATH)
	return payload

'''
Given a start timestamp, end timestamp, name
Give the plot of the hours of the day and the count of bad posture
Give the percentage of time in each posture
'''
@app.route('/get_stack_bar_plot_hour_bad', methods = ["GET"]) 
def get_stack_bar_plot_hour_bad():
	if request.method == 'GET':
		start_time = int(request.args.get("start_time"))
		end_time = int(request.args.get("end_time"))
		name = str(request.args.get("name"))

		# Filter based on given params
		entries_to_analyse = User.query.\
			filter(User.name == name).\
			filter(User.timecollect >= start_time).\
			filter(User.timecollect <= end_time).\
			filter(User.classification != CLASSIFICATION_PROPER).\
			all()

		# If get is invalid, return empty dictionary
		if not entries_to_analyse:
			return {}
		
		list_of_datetime = [datetime.datetime.fromtimestamp(user.timecollect) for user in entries_to_analyse]
		# list_of_datetime_str = [datetime.strftime("%m/%d/%Y, %H:%M:%S") for datetime in list_of_datetime]
		list_of_classifications = [user.classification for user in entries_to_analyse]
		list_of_hours = [int(datetime.strftime("%H")) for datetime in list_of_datetime]

		result = {}

		# Creates a dictionary where key is hour and values is a list of the classification in that date
		for hour, classification in zip(list_of_hours, list_of_classifications):
			if hour in result:
				result[hour].append(classification)
			else:
				result[hour] = [classification]

		# Creates a dictionary where key is date and values are count of each classification in that date
		# Sequence of the count follows the sequence of CLASSIFICATIONS
		for key in result:
			result[key] = [result[key].count(classification) for classification in CLASSIFICATIONS]

		# Saves a stack bar plot to the path specified
		get_stack_bar_plot(result, STACK_BAR_PLOT_HOUR_BAD_PATH)

		payload = get_base64string_from_img_path(STACK_BAR_PLOT_HOUR_BAD_PATH)
	return payload

'''
Given a name
Give the plot of the hours of the day and the count of bad posture
Give the percentage of time in each posture
'''
@app.route('/get_stack_bar_plot_hour_bad_all', methods = ["GET"]) 
def get_stack_bar_plot_hour_bad_all():
	if request.method == 'GET':
		name = str(request.args.get("name"))

		# Filter based on given params
		entries_to_analyse = User.query.\
			filter(User.name == name).\
			filter(User.classification != CLASSIFICATION_PROPER).\
			all()

		# If get is invalid, return empty dictionary
		if not entries_to_analyse:
			return {}
		
		list_of_datetime = [datetime.datetime.fromtimestamp(user.timecollect) for user in entries_to_analyse]
		# list_of_datetime_str = [datetime.strftime("%m/%d/%Y, %H:%M:%S") for datetime in list_of_datetime]
		list_of_classifications = [user.classification for user in entries_to_analyse]
		list_of_hours = [int(datetime.strftime("%H")) for datetime in list_of_datetime]

		result = {}

		# Creates a dictionary where key is hour and values is a list of the classification in that date
		for hour, classification in zip(list_of_hours, list_of_classifications):
			if hour in result:
				result[hour].append(classification)
			else:
				result[hour] = [classification]

		# Creates a dictionary where key is date and values are count of each classification in that date
		# Sequence of the count follows the sequence of CLASSIFICATIONS
		for key in result:
			result[key] = [result[key].count(classification) for classification in CLASSIFICATIONS]

		# Saves a stack bar plot to the path specified
		get_stack_bar_plot(result, STACK_BAR_PLOT_HOUR_BAD_ALL_PATH)

		payload = get_base64string_from_img_path(STACK_BAR_PLOT_HOUR_BAD_ALL_PATH)
	return payload

# '''
# Given a name
# Give all time average per day percentage of each posture
# '''
# @app.route('/get_all_time_ave', methods = ["GET"]) 
# def get_all_time_ave():
# 	if request.method == 'GET':
# 		name = str(request.args.get("name"))

# 		# Filter based on given params
# 		entries_to_analyse = User.query.\
# 			filter(User.name == name).\
# 			all()

# 		# If get is invalid, return empty dictionary
# 		if not entries_to_analyse:
# 			return {}
		
# 		print(entries_to_analyse)
		
# 		list_of_datetime = [datetime.datetime.fromtimestamp(user.timecollect) for user in entries_to_analyse]
# 		# list_of_datetime_str = [datetime.strftime("%m/%d/%Y, %H:%M:%S") for datetime in list_of_datetime]
# 		list_of_classifications = [user.classification for user in entries_to_analyse]
# 		list_of_dates = [datetime.strftime("%m/%d/%Y") for datetime in list_of_datetime]

# 		result = {}

# 		# Creates a dictionary where key is date and values is a list of the classification in that date
# 		for date, classification in zip(list_of_dates, list_of_classifications):
# 			if date in result:
# 				result[date].append(classification)
# 			else:
# 				result[date] = [classification]

# 		# Creates a dictionary where key is date and values are count of each classification in that date
# 		# Sequence of the count follows the sequence of CLASSIFICATIONS
# 		for key in result:
# 			result[key] = [result[key].count(classification) for classification in CLASSIFICATIONS]

# 		payload = get_ave(result)

# 		print(payload)
# 	return payload

'''
Given a name, start, end
Give pie chart of all time average per day percentage of each posture
'''
@app.route('/get_pie', methods = ["GET"]) 
def get_pie():
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
			return {}
		
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

		get_pie_plot(result, PIE_PATH)

		payload = get_base64string_from_img_path(PIE_PATH)
	return payload

'''
Given a name
Give pie chart of all time average per day percentage of each posture
'''
@app.route('/get_pie_all', methods = ["GET"]) 
def get_pie_all():
	if request.method == 'GET':
		name = str(request.args.get("name"))

		# Filter based on given params
		entries_to_analyse = User.query.\
			filter(User.name == name).\
			all()

		# If get is invalid, return empty dictionary
		if not entries_to_analyse:
			return {}
		
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

		get_pie_plot(result, PIE_ALL_PATH)

		payload = get_base64string_from_img_path(PIE_ALL_PATH)
	return payload

'''
Given a name
Delete every entry of that name in the database
'''
@app.route('/delete_user', methods = ["DELETE"]) 
def delete_user():
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
# WARNING: DO NOT USE, WILL CRASH THE ENTIRE DATABASE
# @app.route('/clear_db', methods = ["DELETE"]) 
# def clear_db():
# 	if request.method == 'DELETE':
# 		for entry in User.query.all():
# 			db.session.delete(entry)
# 			db.session.commit()
# 	return "None"

if __name__=="__main__":
	db.create_all()
	app.run(debug=True)
