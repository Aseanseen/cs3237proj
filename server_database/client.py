from flask import Flask, request, render_template
import requests
import json
import datetime
from commons import commons
from random import randint

TEST_USER_1 = "Karthig"
TEST_USER_2 = "Long"

url = "https://demoiot3237.herokuapp.com"
# url = "http://127.0.0.1:5000"

url_add_data = url + "/add_data"
url_get_data = url + "/get_data"
url_get_all_time_ave = url + "/get_all_time_ave"
url_delete_user = url + "/delete_user"
url_clear_db = url + "/clear_db"

def main():
    fill_with_1_year_1_user(TEST_USER_1)
    # test_get_all_time_ave(TEST_USER_1)
    # delete_data_all(TEST_USER_1)
    # clear_db()

def test_get_all_time_ave(user):
    fill_with_1_year_1_user(user)

    dict_to_enter = {
        "name" : user
    }
    requests.get(url_get_all_time_ave, params=dict_to_enter)

def url_delete_user(user):
    dict_to_enter = {
        "name" : user
    }
    requests.delete(url_delete_user, params=dict_to_enter)

# def clear_db():
#     requests.delete(url_clear_db)

def fill_with_1_year_1_user(user):
    year = 2021
    minute = 0

    for month in range(1, 13):
        day = randint(1, 28)
        hour = randint(0, 23)
        
        # year, month, day, hour, minute
        obj = datetime.datetime(year, month, day, hour, minute)
        classification = randint(0, 4)

        if minute < 55:
            minute += 5
        else:
            minute -= 5

        t1 = {
            'name': user,
            'timecollect': datetime.datetime.timestamp(obj), 
            "classification": classification
        }
        requests.put(url_add_data, params=t1)

def fill_with_1_month_1_user(user):
    year = 2021
    month = 10
    minute = 0
    for _ in range(10):
        day = randint(1, 28)
        hour = randint(0, 23)
        # year, month, day, hour, minute
        obj = datetime.datetime(year, month, day, hour, minute)
        classification = randint(0, 4)

        if minute < 55:
            minute += 5
        else:
            minute -= 5

        t1 = {
            'name': user,
            'timecollect': datetime.datetime.timestamp(obj), 
            "classification": classification
        }
        requests.put(url_add_data, params=t1)

def fill_with_1_month_2_user(user1, user2):
    year = 2021
    month = 9
    minute = 0
    for _ in range(10):
        day = randint(1, 28)
        hour = randint(0, 23)
        # year, month, day, hour, minute
        obj = datetime.datetime(year, month, day, hour, minute)
        classification = randint(0, 4)

        if minute < 55:
            minute += 5
        else:
            minute -= 5

        t1 = {
            'name': user1,
            'timecollect': datetime.datetime.timestamp(obj), 
            "classification": classification
        }
        requests.put(url_add_data, params=t1)

    year = 2021
    month = 9
    minute = 0
    for _ in range(10):
        day = randint(1, 28)
        hour = randint(0, 23)

        # year, month, day, hour, minute
        obj = datetime.datetime(year, month, day, hour, minute)
        classification = randint(0, 4)

        if minute < 55:
            minute += 5
        else:
            minute -= 5

        t2 = {
            'name': user2,
            'timecollect': datetime.datetime.timestamp(obj), 
            "classification": classification
        }
        requests.put(url_add_data, params=t2)

if __name__ == '__main__':
    main()

