from datetime import datetime
import pandas as pd
import os

def getTimeStamp():
    return datetime.timestamp(datetime.now())

def getDataframeFromDatalist(datalist, deviceId):
    dev_data = ["accX", "accY", "accZ", "magX", "magY", "magZ", "gyroX", "gyroY", "gyroZ"]
    column = ["Timestamp"] 
    column += [col + "_%s" % (deviceId) for col in dev_data]
    print(column)

    tmp = []

    for id in range(len(datalist["acc"])):
        tmp.append(
            [ datalist["acc"][id][0], *datalist["acc"][id][1], *datalist["gyro"][id][1], *datalist["mag"][id][1]]
        )
    
        df = pd.DataFrame(tmp, columns = column)
    return df


def loadDataframeFromCsv(filename):
    df = pd.read_csv(filename)
    return df

def saveDataframeToCsv(df, filename):
    with open(filename, "w") as f:
        df.to_csv(f, index = False)

def appendDataToDataframe(df, datalist, deviceId):
    print(getDataframeFromDatalist(datalist, deviceId))
    return pd.concat([df, getDataframeFromDatalist(datalist, deviceId)], ignore_index=True)


if __name__ == "__main__":
    datalist1 = \
        {
            "acc": [[1635031407.178796, (0.0, 0.0, 0.0)], [1635031408.137214, (0.0908203125, 1.26025390625, 1.4130859375)], [1635031409.159121, (0.072265625, 1.544921875, 1.244140625)], [1635031410.178374, (0.0244140625, 1.6533203125, 1.23779296875)], [1635031411.138511, (0.017578125, 1.6611328125, 1.2216796875)], [1635031412.218498, (0.01416015625, 1.66455078125, 1.21826171875)], [1635031413.178628, (0.021484375, 1.67333984375, 1.2216796875)], [1635031414.137772, (0.0341796875, 1.669921875, 1.21923828125)], [1635031415.158625, (0.02880859375, 1.67919921875, 1.22216796875)]],

            "gyro": [[1635031407.178796, (0.0, 0.0, 0.0)], [1635031408.137214, (-9.002685546875, 21.67510986328125, -0.1983642578125)], [1635031409.159121, (17.6544189453125, -7.85064697265625, 1.60980224609375)], [1635031410.178374, (-0.89263916015625, 0.66375732421875, 2.31170654296875)], [1635031411.138511, (-1.129150390625, 0.701904296875, 2.655029296875)], [1635031412.218498, (-1.02996826171875, 0.8392333984375, 2.38037109375)], [1635031413.178628, (-0.579833984375, 1.0223388671875, 2.227783203125)], [1635031414.137772, (-1.220703125, 0.8697509765625, 2.40325927734375)], [1635031415.158625, (-0.77056884765625, 0.8697509765625, 2.1820068359375)]],

            "mag" : [[1635031407.178796, (0.0, 0.0, 0.0)], [1635031408.137214, (-83.36605616605617, -20.09181929181929, 24.29010989010989)], [1635031409.159121, (-83.66593406593407, -22.19096459096459, 28.338461538461537)], [1635031410.178374, (-79.61758241758241, -22.34090354090354, 31.637118437118435)], [1635031411.138511, (-78.56800976800976, -22.79072039072039, 32.686691086691084)], [1635031412.218498, (-79.01782661782661, -21.89108669108669, 32.536752136752135)], [1635031413.178628, (-78.71794871794872, -22.94065934065934, 32.536752136752135)], [1635031414.137772, (-77.66837606837606, -21.14139194139194, 32.236874236874236)], [1635031415.158625, (-79.01782661782661, -22.64078144078144, 31.487179487179485)]]
        }
    datalist2 = \
        {
            "acc": [[1635031416.175526, (0.02587890625, 1.67529296875, 1.21044921875)]],

            "gyro": [ [1635031416.175526, (-1.0986328125, 0.37384033203125, 2.227783203125)]],

            "mag" : [[1635031416.175526, (-79.91746031746031, -23.54041514041514, 31.637118437118435)]]
        }
    if os.path.exists("out.csv"):
        df = loadDataframeFromCsv("out.csv")
        print(df)
    else:
        df = getDataframeFromDatalist(datalist1, 1)
        saveDataframeToCsv(df, "out.csv")
    df = appendDataToDataframe(df, datalist2, 1)
    print(df)
    saveDataframeToCsv(df, "out.csv")