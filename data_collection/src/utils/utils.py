from datetime import datetime
import pandas as pd
import os
import numpy as np

def getTimeStamp():
    return datetime.timestamp(datetime.now())

class NP_Q:
    
    """
        Set of Numpy based functions to work with quaternions
    """
    
    @classmethod
    def norm(cls, q):
        return np.sqrt(np.dot(q,q))
    
    @classmethod
    def mult(cls, p,q):
        s = p[0]*q[0] - np.dot(p[1:], q[1:])
        v = p[0]*q[1:] + q[0]*p[1:] + np.cross(p[1:], q[1:])
        return np.append([s], v)
    
    @classmethod
    def conjugate(cls, q):
        return np.array([q[0], -q[1], -q[2], -q[3]])
    
    @classmethod
    def inverse(cls, q):
        return cls.conjugate(q) / np.dot(q,q)
    
    @classmethod
    def log(cls, q):
        v = q[1:]
        a = q[0]
        x = a/cls.norm(q)
        real_part = np.log(cls.norm(q))
        vec_part = v/np.linalg.norm(v, axis=-1) * np.arccos(x)
        return np.append([real_part], vec_part)
    
    @classmethod
    def geodesic_dist(cls, q1, q2):
        x = cls.mult(cls.inverse(q1), q2)
        print(x)
        return cls.norm(cls.log(x))
    
    @classmethod
    def angle_dist(cls, q1,q2):
        x = 2*(np.dot(q1,q2))**2 - 1
        return np.arccos(x) / np.pi * 180
    
    @classmethod
    def quad_diff(cls, q1, q2):
        x = cls.mult(cls.inverse(q1), q2)
        return x