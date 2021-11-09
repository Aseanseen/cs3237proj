import numpy as np
from quatviz import Quaternion_naive as quat
import os
import json
import fasteners

from commons.commons import (
    BLE_NAME_LIST,
    IO_DIR
)

lock = fasteners.InterProcessLock('%s/tmp_lock_file'% IO_DIR)

# Node stores each point of the block
class Node:
    def __init__(self, coordinates, color):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.z = coordinates[2]
        self.color = color

# Face stores 4 nodes that make up a face of the block
class Face:
    def __init__(self, nodes, color):
        self.nodeIndexes = nodes
        self.color = color

# Wireframe stores the details of a block
class Wireframe:
    def __init__(self):
        self.nodes = []
        self.edges = []
        self.faces = []
        self.quaternion = quat.Quaternion()

    # Inteface with gateway ble functions
    def get_data_func(self):
        filepaths = [os.path.join(IO_DIR, i+".json") for i in BLE_NAME_LIST]
            
        exists = [os.path.exists(filepath) for filepath in filepaths]
        # If all files exist
        if all(exist for exist in exists):
            # Merge the json files
            send_dict = {}
            for filepath in filepaths:
                with open(filepath, "r") as f:
                    dict_data = json.load(f)
                    send_dict.update(dict_data)
            return send_dict
        else:
            return

    def getQuatFromJson(self):
        dev = BLE_NAME_LIST[0]
        with lock:
            q_inp = self.get_data_func()
        l = ["q0_", "q1_", "q2_", "q3_"]
        self.quaternion.q = [q_inp[i+dev] for i in l]


    def addNodes(self, nodeList, colorList):
        for node, color in zip(nodeList, colorList):
            self.nodes.append(Node(node, color))

    def addFaces(self, faceList, colorList):
        for indexes, color in zip(faceList, colorList):
            self.faces.append(Face(indexes, color))

    def quatRotate(self, w, dt):
        self.quaternion.rotate(w, dt)

    def rotatePoint(self, point):
        rotationMat = quat.getRotMat(self.quaternion.q)
        return np.matmul(rotationMat, point)

    def convertToComputerFrame(self, point):
        computerFrameChangeMatrix = np.array([[-1, 0, 0], [0, 0, -1], [0, -1, 0]])
        return np.matmul(computerFrameChangeMatrix, point)

    def getAttitude(self):
        return quat.getEulerAngles(self.quaternion.q)

    def outputNodes(self):
        print("\n --- Nodes --- ")
        for i, node in enumerate(self.nodes):
            print(" %d: (%.2f, %.2f, %.2f) \t Color: (%d, %d, %d)" %
                 (i, node.x, node.y, node.z, node.color[0], node.color[1], node.color[2]))

    def outputFaces(self):
        print("\n --- Faces --- ")
        for i, face in enumerate(self.faces):
            print("Face %d:" % i)
            print("Color: (%d, %d, %d)" % (face.color[0], face.color[1], face.color[2]))
            for nodeIndex in face.nodeIndexes:
                print("\tNode %d" % nodeIndex)