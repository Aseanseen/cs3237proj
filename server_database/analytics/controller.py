import matplotlib.pyplot as plt
import numpy as np
import matplotlib.dates as mdates
from datetime import datetime
from commons.commons import (
    CLASSIFICATIONS,
    MQTT_CLASSIFICATIONS
)
import calmap

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
Given a 2D list of the count of the types of postures and 1D list of dates
2D list of count:
[[CLASSIFICATION_PROPER, CLASSIFICATION_FORWARD, CLASSIFICATION_BACKWARD, CLASSIFICATION_LEFT, CLASSIFICATION_RIGHT]]
Plot a stack bar plot
Save it into a png
'''
def get_stack_bar_plot(postures, dates):
    x = dates
    y1 = [i[0] for i in postures]
    y2 = [i[1] for i in postures]
    y3 = [i[2] for i in postures]
    y4 = [i[3] for i in postures]
    y5 = [i[4] for i in postures]
    plt.bar(x, y1, color='g')
    plt.bar(x, y2, bottom=y1, color='b')
    plt.bar(x, y3, bottom=y1 + y2, color='r')
    plt.bar(x, y4, bottom=y1 + y2 + y3, color='g')
    plt.bar(x, y4, bottom=y1 + y2 + y3 + y4, color='y')
    plt.set(title="Stack Bar Plot of Postures")
    plt.savefig("bar.png")

'''
Given a 2D list of the count of the types of postures and 1D list of dates
2D list of count:
[[CLASSIFICATION_PROPER, CLASSIFICATION_FORWARD, CLASSIFICATION_BACKWARD, CLASSIFICATION_LEFT, CLASSIFICATION_RIGHT]]
Plot a stack bar plot
Save it into a png
'''
def get_stack_bar_plot(postures, dates):
    x = dates
    y1 = [i[0] for i in postures]
    y2 = [i[1] for i in postures]
    y3 = [i[2] for i in postures]
    y4 = [i[3] for i in postures]
    y5 = [i[4] for i in postures]
    plt.bar(x, y1, color='g')
    plt.bar(x, y2, bottom=y1, color='b')
    plt.bar(x, y3, bottom=y1 + y2, color='r')
    plt.bar(x, y4, bottom=y1 + y2 + y3, color='g')
    plt.bar(x, y4, bottom=y1 + y2 + y3 + y4, color='y')
    plt.set(title="Stack Bar Plot of Postures")
    plt.savefig("bar.png")

# https://www.youtube.com/watch?v=cKMEL9xgq2I
'''
Given a Pandas Series indexed by a DatetimeIndex
Plot a calendar heat map plot
Save it into a png
'''
def get_cal_heat_map_plot(heatmap_series):
    plt.figure(figsize=(16, 8))
    calmap.yearplot(data=heatmap_series, year=2021)
    plt.suptitle('Calendar Heatmap', y=.65, fontsize=20)

def get_advice(posture, dates):
    count = {}
    advice = ""
    for classification in posture:
        count[classification] = 1 if classification not in count.keys() else count[classification] + 1
    
    for classification in CLASSIFICATIONS:
        advice += "\n[%s] : You were in this posture %.3f percent of the time!\n" % (classification, (count[classification]/len(posture)))
    return advice
