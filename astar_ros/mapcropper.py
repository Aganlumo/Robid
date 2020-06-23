import numpy as np
import cv2

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

    #Reescalando al tamano original:
    #new_img = cv2.resize(new_img, (y,x), interpolation = cv2.INTER_AREA)

    return new_img
