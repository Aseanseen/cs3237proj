# References
- https://rg2021.medium.com/flask-with-sqlalchemy-database-39fc0959609c
- https://medium.com/analytics-vidhya/heroku-deploy-your-flask-app-with-a-database-online-d19274a7a749

# Steps to use on your computer
1. Make heroku account
2. Replace the DB URL
```python
heroku login
heroku addons:create heroku-postgresql:hobby-dev --app app_name # Replace app_name with your name of the app
heroku config --app app_name # Replace app_name with your name of the app
```
3. Use the prompted link to replace the placeholder for database URL in app.py

4. Deploy
```
git add .
git commit -m "..."
heroku git:remote -a app_name
git push heroku master
```
5. Create the Tables and their structure
```
heroku run python
```
```python
from app import db
db.create_all()
exit()
```

# API for Flask-SQLAlchemy
We set up a Heroku server at: https://demoiot3237.herokuapp.com/

We run a Flask application on the server. Using Heroku addons, we create a PostgreSQL database. We use Flask-SQLAlchemy to access the database easily.

## Possible actions
ADD_DATA_POSTFIX = "add_data"

STACK_BAR_PLOT_DATE_POSTFIX = "get_stack_bar_plot_date"

STACK_BAR_PLOT_DATE_BAD_POSTFIX = "get_stack_bar_plot_date_bad"

STACK_BAR_PLOT_HOUR_POSTFIX = "get_stack_bar_plot_hour"

STACK_BAR_PLOT_HOUR_ALL_POSTFIX = "get_stack_bar_plot_hour_all"

STACK_BAR_PLOT_HOUR_BAD_POSTFIX = "get_stack_bar_plot_hour_bad"

STACK_BAR_PLOT_HOUR_BAD_ALL_POSTFIX = "get_stack_bar_plot_hour_bad_all"

PIE_POSTFIX = "get_pie"

PIE_ALL_POSTFIX = "get_pie_all"

DELETE_USER_POSTFIX = "delete_user"

### Adding data to database
Type of request: `PUT`

ADD_DATA_POSTFIX = "add_data"

Arguments required:
1. name
2. timecollect
3. classification

### Get stack bar plot with dates
Type of request: `GET`

STACK_BAR_PLOT_DATE_POSTFIX = "get_stack_bar_plot_date"

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

### Get stack bar plot with dates, but only showing bad posture
Type of request: `GET`

STACK_BAR_PLOT_DATE_BAD_POSTFIX = "get_stack_bar_plot_date_bad"

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

### Get stack bar plot with hours
Type of request: `GET`

STACK_BAR_PLOT_HOUR_POSTFIX = "get_stack_bar_plot_hour"

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

### Get stack bar plot with hours, all time record
Type of request: `GET`

STACK_BAR_PLOT_HOUR_ALL_POSTFIX = "get_stack_bar_plot_hour_all"

Arguments required:
1. name

### Get stack bar plot with hours, but only showing bad posture
Type of request: `GET`

STACK_BAR_PLOT_HOUR_BAD_POSTFIX = "get_stack_bar_plot_hour_bad"

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

### Get stack bar plot with hours, but only showing bad posture, all time record
Type of request: `GET`

STACK_BAR_PLOT_HOUR_BAD_ALL_POSTFIX = "get_stack_bar_plot_hour_bad_all"

Arguments required:
1. name

### Get pie chart plot showing percentages of each classification
Type of request: `GET`

PIE_POSTFIX = "get_pie"

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

### Get pie chart plot showing percentages of each classification, all time record
Type of request: `GET`

PIE_ALL_POSTFIX = "get_pie_all"

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

### Delete all the data of a given name
Type of request: `DELETE`

DELETE_USER_POSTFIX = "delete_user"

Arguments required:
1. name

## Examples for API

https://demoiot3237.herokuapp.com/add_data?name=Karthig&timecollect=1640303800&classification=0
