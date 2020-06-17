import Astar_ros
import re
import numpy
import numpy as np
import matplotlib.pyplot as plt
import cv2

def read_pgm(filename, byteorder='>'):

    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return numpy.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))

if __name__ == "__main__":

    sx = 67
    sy = 247
    gx = 322
    gy = 280
    grid_size = 1
    robot_radius = 8

    image = read_pgm("map.pgm", byteorder='<')
    y,x = image.shape
    ox, oy = [], []
    draw_rad = 1
    saturation = 0
    black = [0,0,0]
    white = [255,255,255]
    blue = [255,0,0]
    green = [0,255,0]
    red = [0,0,255]
    recreated_img = np.zeros((y, x,3), np.uint8)
    for i in range(y):
        for j in range(x):
            recreated_img[i,j] = white
            if image[i,j] < 20:
                oy.append(int(i))
                ox.append(int(j))
                cv2.circle(recreated_img, (j,i), draw_rad, black, -1)

    cv2.circle(recreated_img, (sx,sy), 3, green, -1)
    cv2.circle(recreated_img, (gx,gy), 3, red, -1)

    wx,wy = Astar_ros.Amap(sx,sy,gx,gy,grid_size,robot_radius,ox,oy)
    for i in range(len(wx)):
        cv2.circle(recreated_img, (int(wx[i]),int(wy[i])), 1, blue, -1)

    cv2.imshow('img', recreated_img)
    cv2.waitKey(0)
