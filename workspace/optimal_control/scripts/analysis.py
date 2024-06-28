#!/usr/bin/python3

import matplotlib.pyplot as plt

from sys import argv

import numpy as np

if __name__ == "__main__":
    data = np.genfromtxt(argv[1], delimiter=",")

    print(np.mean(data[:, 2]), np.var(data[:, 2]))

    # shape = (21, 21)

    # x = np.asarray(data[:, 0]).reshape(shape)
    # y = np.asarray(data[:, 1]).reshape(shape)
    # z = np.asarray(data[:, 2]).reshape(shape)

    # plt.pcolormesh(x, y, z, cmap="hot")

    # plt.show()