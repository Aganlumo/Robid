import Astar_ros
import re
import numpy as np
import cv2
import mapcropper

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
    return np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))

def mapWAstar(sx,sy,gx,gy,grid_size,robot_radius,image):

    #Inicializacion
    y,x = image.shape
    ox, oy = [], []
    draw_rad = 1
    black = [0,0,0]
    white = [255,255,255]
    blue = [255,0,0]
    green = [0,255,0]
    red = [0,0,255]
    whiteTreshold = 250
    recreated_img = np.zeros((y, x,3), np.uint8)

    #Obtencion de puntos en el mapa
    for i in range(y):
        for j in range(x):
            recreated_img[i,j] = white
            if image[i,j] == 0:
                oy.append(int(i))
                ox.append(int(j))
                cv2.circle(recreated_img, (j,i), draw_rad, black, -1)
            elif image[i,j] > whiteTreshold:
                recreated_img[i,j] = white
            else:
                recreated_img[i,j] = black

    cv2.circle(recreated_img, (sx,sy), 3, green, -1)
    cv2.circle(recreated_img, (gx,gy), 3, red, -1)

    #Mapeado de respuesta en A*
    wx,wy = Astar_ros.Amap(sx,sy,gx,gy,grid_size,robot_radius,ox,oy)
    for i in range(len(wx)):
        cv2.circle(recreated_img, (int(wx[i]),int(wy[i])), 1, blue, -1)
    cv2.imshow('img', recreated_img)
    cv2.waitKey(0)

    return wx,wy;


if __name__ == "__main__":

    sx = 27
    sy = 158
    gx = 264
    gy = 207
    grid_size = 2
    robot_radius = 2
    whiteTreshold = 250

    image = read_pgm("map.pgm", byteorder='<')
    old_img = image
    image = mapcropper.crop(image)
    y,x = image.shape
    ox, oy = [], []
    draw_rad = 1
    black = [0,0,0]
    white = [255,255,255]
    blue = [255,0,0]
    green = [0,255,0]
    red = [0,0,255]
    recreated_img = np.zeros((y, x,3), np.uint8)
    for i in range(y):
        for j in range(x):
            if image[i,j] == 0:
                oy.append(int(i))
                ox.append(int(j))
                cv2.circle(recreated_img, (j,i), draw_rad, black, -1)
            elif image[i,j] > whiteTreshold:
                recreated_img[i,j] = white
            else:
                recreated_img[i,j] = black

    cv2.circle(recreated_img, (sx,sy), 3, green, -1)
    cv2.circle(recreated_img, (gx,gy), 3, red, -1)

    wx,wy = Astar_ros.Amap(sx,sy,gx,gy,grid_size,robot_radius,ox,oy)
    for i in range(len(wx)):
        cv2.circle(recreated_img, (int(wx[i]),int(wy[i])), 1, blue, -1)
    cv2.imshow('old_map', old_img)
    cv2.imshow('img', recreated_img)
    cv2.waitKey(0)
