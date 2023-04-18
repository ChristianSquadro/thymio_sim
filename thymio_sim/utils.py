from collections import Counter
import numpy as np

def mkrot(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
        ])

def mktransl(xt, yt):
    return np.array([
        [1, 0, xt],
        [0, 1, yt],
        [0, 0, 1]
        ])

def list_mode(lst):
    data = Counter(lst)
    return data.most_common(1)[0][0]