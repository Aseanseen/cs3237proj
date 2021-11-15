import matplotlib.pyplot as plt
import numpy as np
import matplotlib.dates as mdates
from datetime import datetime
from commons.commons import (
    CLASSIFICATIONS,
    CLASSIFICATION_PROPER,
    CLASSIFICATION_ENUM_TO_NAME,
    MQTT_CLASSIFICATIONS
)
# import calmap

STACK_BAR_PLOT_TITLE = "Stack Bar Plot of Postures"
STACK_BAR_PLOT_PATH = "bar.png"

def get_plot(posture, dates):
    posture
    # Choose some nice levels
    levels = np.tile([-3, 3, -2, 2, -1, 1],
                    int(np.ceil(len(posture)/6)))[:len(posture)]

    # Create figure and plot a stem plot with the date
    fig, ax = plt.subplots(figsize=(8.8, 4), constrained_layout=True)
    ax.set(title="Posture Plot")

    ax.vlines(dates, 0, levels, color="tab:red")  # The vertical stems.
    ax.plot(dates, np.zeros_like(dates), "-o",
            color="k", markerfacecolor="w")  # Baseline and markers on it.

    # annotate lines
    for d, l, r in zip(dates, levels, posture):
        ax.annotate(r, xy=(d, l),
                    xytext=(-3, np.sign(l)*3), textcoords="offset points",
                    horizontalalignment="right",
                    verticalalignment="bottom" if l > 0 else "top")

    # format xaxis with 4 month intervals
    ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=4))
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%b-%d-%Y %H:%M:%S"))
    plt.setp(ax.get_xticklabels(), rotation=30, ha="right")

    # remove y axis and spines
    ax.yaxis.set_visible(False)
    ax.spines[["left", "top", "right"]].set_visible(False)

    ax.margins(y=0.1)
    # plt.show()
    plt.savefig("analytics.png")

'''
Given a dict where keys are the dates and values are the count of the types of postures in that day
Plot a stack bar plot
Save it into a png
'''
def get_stack_bar_plot(sum_dict):
    x = sum_dict.keys()
    y0 = np.array([item[0] for item in sum_dict.values()])
    y1 = np.array([item[1] for item in sum_dict.values()])
    y2 = np.array([item[2] for item in sum_dict.values()])
    y3 = np.array([item[3] for item in sum_dict.values()])
    y4 = np.array([item[4] for item in sum_dict.values()])
    order = [y0, y1, y2, y3, y4]
    plt.bar(x, y0, color='y')
    plt.bar(x, y1, bottom=order[0], color='b')
    plt.bar(x, y2, bottom=sum(order[:2]), color='r')
    plt.bar(x, y3, bottom=sum(order[:3]), color='m')
    plt.bar(x, y4, bottom=sum(order[:4]), color='g')
    plt.title(STACK_BAR_PLOT_TITLE)
    plt.legend([CLASSIFICATION_ENUM_TO_NAME[classification] for classification in CLASSIFICATIONS])
    plt.savefig(STACK_BAR_PLOT_PATH)

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
Returns a dict

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
            advice += "[%s] : You were in this posture %.3f percent of the time!\n" % (classes[i], (value[i]/sum(value)))
        res_dict[key] = advice
    return res_dict
