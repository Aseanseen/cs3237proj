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
- add_data
- get_data
- get_stack_bar_plot_date
- get_stack_bar_plot_date_bad
- get_stack_bar_plot_hour
- get_stack_bar_plot_hour_all
- get_stack_bar_plot_hour_bad
- get_stack_bar_plot_hour_bad_all
- get_pie
- get_pie_all
- get_last
- delete_user
---
### Adding data to database
>***add_data***
---
Type of request: `PUT`

Arguments required:
1. name
2. timecollect
3. classification

Returns:

New HTML page

---
### Visualising data from the database
>***get_data***
---
Type of request: `GET`

Arguments required:
1. name

OR
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

Returns:

New HTML page

---
### Get stack bar plot with dates
>***get_stack_bar_plot_date***
---
Type of request: `GET`

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

Returns:

Base64 String of the requested plot

---
### Get stack bar plot with dates, but only showing bad posture
>***get_stack_bar_plot_date_bad***
---
Type of request: `GET`

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

Returns:

Base64 String of the requested plot

---
### Get stack bar plot with hours
>***get_stack_bar_plot_hour***
---
Type of request: `GET`

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

Returns:

Base64 String of the requested plot

---
### Get stack bar plot with hours, all time record
>***get_stack_bar_plot_hour_all***
---
Type of request: `GET`

Arguments required:
1. name

Returns:

Base64 String of the requested plot

---
### Get stack bar plot with hours, but only showing bad posture
>***get_stack_bar_plot_hour_bad***
---
Type of request: `GET`

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

Returns:

Base64 String of the requested plot

---
### Get stack bar plot with hours, but only showing bad posture, all time record
>***get_stack_bar_plot_hour_bad_all***
---
Type of request: `GET`

Arguments required:
1. name

Returns:

Base64 String of the requested plot

---
### Get pie chart plot showing percentages of each classification
>***get_pie***
---
Type of request: `GET`

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

Returns:

Base64 String of the requested plot

---
### Get pie chart plot showing percentages of each classification, all time record
>***get_pie_all***
---
Type of request: `GET`

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

Returns:

Base64 String of the requested plot

---
### Get last lag number of entries from the database
>***get_last***
---
Type of request: `GET`

Arguments required:
1. name
2. lag

Returns:

String of the last lag number of entries
Name:
Date Time: %Y-%m-%d %H:%M:%S
Classification:

---
### Delete all the data of a given name
>***delete_user***
---
Type of request: `DELETE`

Arguments required:
1. name

Returns:

None

---
## Examples for API

https://demoiot3237.herokuapp.com/get_stack_bar_plot_date_bad?name=Karthig&start_time=1640303800&end_time=1640403800

https://demoiot3237.herokuapp.com/get_stack_bar_plot_hour_bad_all?name=Karthig

https://demoiot3237.herokuapp.com/get_pie_all?name=Karthig

https://demoiot3237.herokuapp.com/get_last?name=Karthig&lag=2
