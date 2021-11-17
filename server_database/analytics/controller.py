# import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import numpy as np
import matplotlib.dates as mdates
from datetime import datetime
import io
import base64

from commons.commons import (
    CLASSIFICATIONS,
    CLASSIFICATION_PROPER,
    CLASSIFICATION_ENUM_TO_NAME,
    MQTT_CLASSIFICATIONS
)

from analytics.paths import(
STACK_BAR_PLOT_TITLE,
PIE_PLOT_TITLE
)

# def get_plot(posture, dates):
#     posture
#     # Choose some nice levels
#     levels = np.tile([-3, 3, -2, 2, -1, 1],
#                     int(np.ceil(len(posture)/6)))[:len(posture)]

#     # Create figure and plot a stem plot with the date
#     fig, ax = plt.subplots(figsize=(8.8, 4), constrained_layout=True)
#     ax.set(title="Posture Plot")

#     ax.vlines(dates, 0, levels, color="tab:red")  # The vertical stems.
#     ax.plot(dates, np.zeros_like(dates), "-o",
#             color="k", markerfacecolor="w")  # Baseline and markers on it.

#     # annotate lines
#     for d, l, r in zip(dates, levels, posture):
#         ax.annotate(r, xy=(d, l),
#                     xytext=(-3, np.sign(l)*3), textcoords="offset points",
#                     horizontalalignment="right",
#                     verticalalignment="bottom" if l > 0 else "top")

#     # format xaxis with 4 month intervals
#     ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=4))
#     ax.xaxis.set_major_formatter(mdates.DateFormatter("%b-%d-%Y %H:%M:%S"))
#     plt.setp(ax.get_xticklabels(), rotation=30, ha="right")

#     # remove y axis and spines
#     ax.yaxis.set_visible(False)
#     ax.spines[["left", "top", "right"]].set_visible(False)

#     ax.margins(y=0.1)
#     # plt.show()
#     plt.savefig("analytics.png")

'''
Given a dict where keys are the dates/hour and values are the count of the types of postures in that day/hour
Plot a stack bar plot
Save it into a png
'''
# https://stackoverflow.com/questions/38061267/matplotlib-graphic-image-to-base64
# https://pretagteam.com/question/fast-conversion-of-matplotlib-charts-into-base64-string-representation-or-other-serving-method-of-charts-in-flask
def get_stack_bar_plot(sum_dict):
    fig = Figure()
    ax = fig.subplots()

    x = sum_dict.keys()
    y0 = np.array([item[0] for item in sum_dict.values()])
    y1 = np.array([item[1] for item in sum_dict.values()])
    y2 = np.array([item[2] for item in sum_dict.values()])
    y3 = np.array([item[3] for item in sum_dict.values()])
    y4 = np.array([item[4] for item in sum_dict.values()])
    order = [y0, y1, y2, y3, y4]
    ax.bar(x, y0, color='y')
    ax.bar(x, y1, bottom=order[0], color='b')
    ax.bar(x, y2, bottom=sum(order[:2]), color='r')
    ax.bar(x, y3, bottom=sum(order[:3]), color='m')
    ax.bar(x, y4, bottom=sum(order[:4]), color='g')
    ax.set_title(STACK_BAR_PLOT_TITLE)
    ax.legend([CLASSIFICATION_ENUM_TO_NAME[classification] for classification in CLASSIFICATIONS])

    my_stringIObytes = io.BytesIO()
    fig.savefig(my_stringIObytes, format='png')
    my_stringIObytes.seek(0)
    # my_base64_pngData = base64.b64encode(my_stringIObytes.read()).decode("utf-8")
    my_base64_pngData = base64.b64encode(my_stringIObytes.getvalue()).decode("utf-8")
    return my_base64_pngData

# https://www.youtube.com/watch?v=cKMEL9xgq2I
'''
Given a Pandas Series indexed by a DatetimeIndex
Plot a calendar heat map plot
Save it into a png
'''
# def get_cal_heat_map_plot(heatmap_series):
#     plt.figure(figsize=(16, 8))
#     calmap.yearplot(data=heatmap_series, year=2021)
#     plt.suptitle('Calendar Heatmap', y=.65, fontsize=20)
#     plt.savefig("calmap.png")

'''
Given a dict where keys are the dates and values are the count of the types of postures in that day
Get the analysis of the time spent in each posture for each day
Returns a dict where keys are dates, values are advice for that date

Note: The dict passed in must have values where the classifation is in the same sequence as in CLASSIFICATIONS
'''
def get_advice(sum_dict):
    res_dict = {}
    classes = [CLASSIFICATION_ENUM_TO_NAME[classification] for classification in CLASSIFICATIONS]
    # Iterate through days
    for key, value in sum_dict.items():
        advice = "Date: %s\n" %key
        # For every day, iterate through the postures
        for i in range(len(classes)):
            advice += "[%s] : You were in this posture %.3f percent of the time!\n" % (classes[i], ((value[i]/sum(value)) * 100))
        res_dict[key] = advice
    return res_dict

'''
Given a dict where keys are the dates and values are the count of the types of postures in that day
Get the all time sum for each posture
Returns a dict

Note: The dict passed in must have values where the classifation is in the same sequence as in CLASSIFICATIONS
'''
def get_sum(sum_dict):
    res_dict = {}
    sum_of_posture_percentage = [0 for classification in CLASSIFICATIONS]
    classes = [CLASSIFICATION_ENUM_TO_NAME[classification] for classification in CLASSIFICATIONS]
    # Iterate through days
    for key, value in sum_dict.items():
        # For every day, iterate through the postures
        for i in range(len(classes)):
            sum_of_posture_percentage[i] += (value[i] / sum(value))
    # Find the overall average percentage
    for i in range(len(classes)):
        # res_dict[classes[i]] = sum_of_posture_percentage[i] / len(sum_dict)
        res_dict[classes[i]] = sum_of_posture_percentage[i]
    return res_dict

'''
Given a dict where keys are the dates and values are the count of the types of postures in that day
Get the all time average for each posture
Plot a pie chart

Note: The dict passed in must have values where the classifation is in the same sequence as in CLASSIFICATIONS
'''
# https://stackoverflow.com/questions/38061267/matplotlib-graphic-image-to-base64
# https://pretagteam.com/question/fast-conversion-of-matplotlib-charts-into-base64-string-representation-or-other-serving-method-of-charts-in-flask
def get_pie_plot(sum_dict):
    fig = Figure()
    ax = fig.subplots()

    res_dict = get_sum(sum_dict)
    myexplode = [0.2 if i == CLASSIFICATION_PROPER else 0 for i in CLASSIFICATIONS]
    ax.pie(res_dict.values(), labels = res_dict.keys(), explode = myexplode, autopct='%.1f%%', startangle = 90)
    ax.set_title(PIE_PLOT_TITLE)
    # ax.legend(loc='upper left')
    
    my_stringIObytes = io.BytesIO()
    fig.savefig(my_stringIObytes, format='png')
    my_stringIObytes.seek(0)
    # my_base64_pngData = base64.b64encode(my_stringIObytes.read()).decode("utf-8")
    my_base64_pngData = base64.b64encode(my_stringIObytes.getvalue()).decode("utf-8")
    return my_base64_pngData
