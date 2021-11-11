You can always refer this link
https://rg2021.medium.com/flask-with-sqlalchemy-database-39fc0959609c
https://medium.com/analytics-vidhya/heroku-deploy-your-flask-app-with-a-database-online-d19274a7a749

Step 1. Create a new app on your heroku 

Step 2: Replace the DB URL
$ heroku login
$ heroku addons:create heroku-postgresql:hobby-dev --app app_name
(Replace app_name with your name of the app)
$ heroku config --app app_name
Then, use the prompted link to replace the placeholder for database URL in app.py

Step 3: Deploy
$ git add .
$ git commit -m "..."
$ heroku git:remote -a app_name
$ git push heroku master

Step 4: Create the Tables and their structure
$ heroku run python
>> from app import db
>> db.create_all()
>> exit()