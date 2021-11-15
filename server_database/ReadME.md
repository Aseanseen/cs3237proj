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
### Adding data to database
Type of request: `PUT`

URL: https://demoiot3237.herokuapp.com/add_data

Arguments required:
1. name
2. timecollect
3. classification

### Get stack bar plot, percentage of time in each posture
Type of request: `GET`

URL: https://demoiot3237.herokuapp.com/get_data

Arguments required:
1. name
2. start_time
    - Must be in the form of datetime.datetime.timestamp
3. end_time
    - Must be in the form of datetime.datetime.timestamp

### Get per day all time average percentage of each posture
Type of request: `GET`

URL: https://demoiot3237.herokuapp.com/get_all_time_ave

Arguments required:
1. name

### Delete all the data of a given name
Type of request: `DELETE`

URL: https://demoiot3237.herokuapp.com/delete_data_all

Arguments required:
1. name

### Clear the entire database
Type of request: `DELETE`

URL: https://demoiot3237.herokuapp.com/clear_db

Arguments required:
None
