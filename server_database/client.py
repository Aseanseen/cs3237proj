from flask import Flask, request, render_template
import requests
import json
import datetime
import time
from commons import commons
from random import randint

TEST_USER_1 = "Karthig"
TEST_USER_2 = "Long"

# url = "https://demoiot3237.herokuapp.com/"
url = "http://127.0.0.1:5000/"

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

def main():
    # test_get_stack_bar_plot_date(TEST_USER_1)
    # # time.sleep(10)
    # test_get_stack_bar_plot_date_bad(TEST_USER_1)
    # # time.sleep(10)
    # test_get_stack_bar_plot_hour(TEST_USER_1)
    # # time.sleep(10)
    # test_get_stack_bar_plot_hour_all(TEST_USER_1)
    # # time.sleep(10)
    # test_get_stack_bar_plot_hour_bad(TEST_USER_1)
    # # time.sleep(10)
    # test_get_stack_bar_plot_hour_bad_all(TEST_USER_1)
    # time.sleep(10)
    test_get_pie(TEST_USER_1)
    # time.sleep(5)
    # test_get_pie_all(TEST_USER_1)

    # delete_user(TEST_USER_1)
    # fill_with_1_day_1_user(TEST_USER_1)
    # fill_with_1_year_1_user(TEST_USER_1)

def test_get_stack_bar_plot_date(user):
    # fill_with_1_day_1_user(user)
    dict_to_enter = {
        "name" : user,
        "start_time" : 1627775100,
        "end_time" : 1627826400
    }
    requests.get(url + STACK_BAR_PLOT_DATE_POSTFIX, params=dict_to_enter)

def test_get_stack_bar_plot_date_bad(user):
    # fill_with_1_day_1_user(user)
    dict_to_enter = {
        "name" : user,
        "start_time" : 1627775100,
        "end_time" : 1627826400
    }
    requests.get(url + STACK_BAR_PLOT_DATE_BAD_POSTFIX, params=dict_to_enter)

def test_get_stack_bar_plot_hour(user):
    # fill_with_1_day_1_user(user)
    dict_to_enter = {
        "name" : user,
        "start_time" : 1627775100,
        "end_time" : 1627826400
    }
    requests.get(url + STACK_BAR_PLOT_HOUR_POSTFIX, params=dict_to_enter)

def test_get_stack_bar_plot_hour_all(user):
    # fill_with_1_day_1_user(user)
    dict_to_enter = {
        "name" : user
    }
    requests.get(url + STACK_BAR_PLOT_HOUR_ALL_POSTFIX, params=dict_to_enter)

def test_get_stack_bar_plot_hour_bad(user):
    # fill_with_1_day_1_user(user)
    dict_to_enter = {
        "name" : user,
        "start_time" : 1627775100,
        "end_time" : 1627826400
    }
    requests.get(url + STACK_BAR_PLOT_HOUR_BAD_POSTFIX, params=dict_to_enter)

def test_get_stack_bar_plot_hour_bad_all(user):
    # fill_with_1_day_1_user(user)
    dict_to_enter = {
        "name" : user
    }
    requests.get(url + STACK_BAR_PLOT_HOUR_BAD_ALL_POSTFIX, params=dict_to_enter)

def test_get_pie(user):
    # fill_with_1_day_1_user(user)
    dict_to_enter = {
        "name" : user,
        "start_time" : 1627775100,
        "end_time" : 1627826400
    }
    requests.get(url + PIE_POSTFIX, params=dict_to_enter)

def test_get_pie_all(user):
    # fill_with_1_day_1_user(user)
    dict_to_enter = {
        "name" : user
    }
    requests.get(url + PIE_ALL_POSTFIX, params=dict_to_enter)

def delete_user(user):
    dict_to_enter = {
        "name" : user
    }
    requests.delete(url + DELETE_USER_POSTFIX, params=dict_to_enter)

# def test_get_all_time_ave(user):
#     fill_with_1_year_1_user(user)

#     dict_to_enter = {
#         "name" : user
#     }
#     requests.get(url_get_all_time_ave, params=dict_to_enter)


# def clear_db():
#     requests.delete(url_clear_db)

def fill_with_1_day_1_user(user):
    year = 2021
    month = 8
    minute = 0
    day = 1
    for _ in range(10):
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

