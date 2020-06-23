import Astar_ros
import re
import numpy as np
import cv2
def mapWAstar(sx,sy,gx,gy,grid_size,robot_radius,image):

    #reduccion a rectangulo mas pequeno posible
    image = crop(image)

    #Inicializacion
    y,x = image.shape
    ox, oy = [], []
    draw_rad = 1
    black = [0,0,0]
    white = [255,255,255]
    blue = [255,0,0]
    green = [0,255,0]
    red = [0,0,255]
    recreated_img = np.zeros((y, x,3), np.uint8)

    #Obtencion de puntos en el mapa
    for i in range(y):
        for j in range(x):
            recreated_img[i,j] = white
            if image[i,j] < 20:
                oy.append(int(i))
                ox.append(int(j))

    #Mapeado de respuesta en A*
    wx,wy = Astar_ros.Amap(sx,sy,gx,gy,grid_size,robot_radius,ox,oy)

    return wx,wy;

def crop(img):
    # X son filas; y, columnas
    x,y = img.shape
    minx = x;
    maxx = 0;
    miny = y;
    maxy = 0;

    #find min rectangle
    for i in range(x):
        for j in range(y):
            if (img[i,j] == 0):
                if(minx > i):
                    minx = i
                if(miny > j):
                    miny = j
                if(maxx < i):
                    maxx = i
                if(maxy < j):
                    maxy = j

    newx = maxx + 1 - minx
    newy = maxy + 1 - miny

    #remapeado de pixeles a minimo rectangulo posible
    new_img = np.zeros((newx,newy))
    for i in range(x):
        for j in range(y):
            if(i >= minx and i <= maxx):
                if(j >= miny and j <= maxy):
                    new_img[(i-minx),(j-miny)] = img[i,j]
    return new_img
