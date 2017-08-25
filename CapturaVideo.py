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

#from PIL import Image
import cv2
#import matplotlib.pyplot as plt


import numpy as np

import time
import serial
import struct

def InfoComSerial(dados_device):
    '''Retorna o nome e a configuracao da porta'''
    print('\nObtendo informacoes sobre a comunicacao serial\n')

    # Iniciando conexao serial
    comport = serial.Serial(port = dados_device['port'], baudrate = dados_device['baudrate'], timeout=dados_device['timeout'], bytesize=dados_device['bytesize'])
    time.sleep(1.8) # Entre 1.5s a 2s

    print('\nStatus Porta: %s ' % (comport.isOpen()))
    print('Device conectado: %s ' % (comport.name))
    print('Dump da configuracao:\n %s ' % (comport))
    print('\n###############################################\n')

    # Fechando conexao serial
    comport.close()


def isImageStart(ser, key, indice):

    if indice < len(key):

        val1 = key[indice]
        val2 = struct.unpack(">B", ser.read())[0]

        if val1 == val2:
            indice += 1
            return isImageStart(ser, key, indice)
        else:
            return False

    return True


def teste_c2(dados_device):
    '''Teste 1'''
    try:
        ser = serial.Serial(port = dados_device['port'], baudrate = dados_device['baudrate'], timeout=dados_device['timeout'], bytesize=dados_device['bytesize'])
        ser.flushInput()
        key = bytes([42, 82, 68, 89, 42, 13, 10])

        buffer = bytearray(76800)

        contador = 0
        while True:
            
            for indice in range(0, len(buffer)):
                buffer[indice] = 0

            print('Procutando Imagem..')
            while isImageStart(ser, key, 0) is False:
                pass

            print('Encontrada Imagem..')
            tot = ser.readinto(buffer)


            WIDTH = 320#; //640;
            HEIGHT = 240#; //480;

            rgb = np.zeros((HEIGHT, WIDTH, 1), dtype=np.uint8)
            #rgb2 = np.zeros((WIDTH, HEIGHT, 1), dtype=np.uint8)

            for y in range(0, WIDTH):
                for x in range(0, HEIGHT):
                    indice = (y * HEIGHT) + x
                    rgb[x][y] = buffer[indice]
                    

            print('Dados imagen completos:{0}'.format(contador))

            saveBMP('img_{0}.bmp'.format(contador), rgb)

            contador += 1

    except Exception as exp:
        print(exp)

def teste_c1(dados_device):
    '''Teste 1'''
    try:
        ser = serial.Serial(port = dados_device['port'], baudrate = dados_device['baudrate'], timeout=dados_device['timeout'], bytesize=dados_device['bytesize'])
        ser.flushInput()
        key = bytes([42, 82, 68, 89, 42, 13, 10])

        contador = 0

        while True:

            print('Procutando Imagem..')
            while isImageStart(ser, key, 0) is False:
                pass

            print('Encontrada Imagem..')

            WIDTH = 320#; //640;
            HEIGHT = 240#; //480;

            rgb = np.zeros((HEIGHT, WIDTH, 1), dtype=np.uint8)
            #rgb2 = np.zeros((WIDTH, HEIGHT, 1), dtype=np.uint8)

            for x in range(0, HEIGHT):
                for y in range(0, WIDTH):
                    val = ser.read()
                    temp = struct.unpack(">B", val)[0]
                    rgb[x][y] = temp

            # for y in range(0, WIDTH):
            #     for x in range(0, HEIGHT):
            #         val = ser.read()
            #         temp = struct.unpack(">B", val)[0]
            #         rgb[x][y] = temp

            # for y in range(0, HEIGHT):
            #     for x in range(0, WIDTH):
            #         rgb2[x][y]=rgb[y][x]


            print('Dados imagen completos')

            saveBMP('img_{0}.bmp'.format(contador), rgb)

            contador += 1

    except Exception as exp:
        print(exp)


def saveBMP(filename, rgbValues):

    print('Dados imagen completos:{0}'.format(filename))
    cv2.imwrite(filename, rgbValues)

if __name__ == '__main__':

    dados_device = {}
    dados_device['port'] = '/dev/ttyACM0'
    dados_device['baudrate'] = 460800
    dados_device['timeout'] = None
    dados_device['bytesize'] = serial.EIGHTBITS
    dados_device['parity'] = serial.PARITY_NONE
    dados_device['stopbits'] = serial.STOPBITS_ONE

    teste_c1(dados_device)
    #teste_c2(dados_device)
