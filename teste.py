#!/usr/bin/env python3.5
'''
Created on 20170822
Update on 20170822
@author: Eduardo Pagotto
'''

#pylint: disable=C0301
#pylint: disable=C0103
#pylint: disable=W0703
#pylint: disable=R0913

import cv2
import numpy as np


if __name__ == '__main__':

    #dado = 0x00202020
    dado = 255

    width = 256
    height = 128

    #rgb = np.zeros((height, width), dtype=int)
    #rgb = np.zeros((height, width), np.uint8)
    rgb = np.zeros((height, width), dtype=np.uint8)

    #print('Tamanho:' + rgb.shape)

    for x in range(0, height):
        for y in range(0, width):
            if (x % 10) == 0:
                rgb[x][y] = dado

    #vis2 = cv2.cvtColor(rgb, cv2.COLOR_GRAY2BGR)

    cv2.imwrite('arquivoZ1.bmp', rgb)