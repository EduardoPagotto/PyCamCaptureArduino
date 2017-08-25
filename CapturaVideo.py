#!/usr/bin/env python3.5
'''
Created on 20170816
Update on 20170824
@author: Eduardo Pagotto
'''

#pylint: disable=C0301
#pylint: disable=C0103
#pylint: disable=W0703
#pylint: disable=R0913

from datetime import datetime
import time
import struct
import cv2
import numpy as np
import serial

def InfoComSerial(dados_device):
    '''Retorna o nome e a configuracao da porta'''
    print('\nObtendo informacoes sobre a comunicacao serial\n')

    # Iniciando conexao serial
    comport = serial.Serial(port = dados_device['port'], baudrate = dados_device['baudrate'], timeout = dados_device['timeout'], bytesize = dados_device['bytesize'])
    time.sleep(1.8) # Entre 1.5s a 2s

    print('\nStatus Porta: %s ' % (comport.isOpen()))
    print('Device conectado: %s ' % (comport.name))
    print('Dump da configuracao:\n %s ' % (comport))
    print('\n###############################################\n')

    # Fechando conexao serial
    comport.close()

def isImageStart(ser, key, indice):
    '''Localiza a string de *RDY* e posiciona proxima imagem'''
    if indice < len(key):

        val1 = key[indice]
        val2 = struct.unpack(">B", ser.read())[0]

        if val1 == val2:
            indice += 1
            return isImageStart(ser, key, indice)
        else:
            return False

    return True

def teste_c1(dados_device):
    '''Teste 1'''
    try:
        ser = serial.Serial(port = dados_device['port'], baudrate = dados_device['baudrate'], timeout=dados_device['timeout'], bytesize=dados_device['bytesize'])
        ser.flushInput()
        key = bytes([42, 82, 68, 89, 42, 13, 10])# *RDY*\n\r

        contador = 0

        #geometria do canvas
        WIDTH = 320
        HEIGHT = 240

        #buffer do canvas 1 byte por pixel!!
        rgb = np.zeros((HEIGHT, WIDTH, 1), dtype=np.uint8)
        while True:

            print('Procutando Imagem..')
            while isImageStart(ser, key, 0) is False:
                pass

            print('Encontrada Imagem..')

            #carrega cada byte(pixel) do serial para o buffer de canvas
            for x in range(0, HEIGHT):
                for y in range(0, WIDTH):
                    rgb[x][y] = struct.unpack(">B", ser.read())[0]

            #salva Imagen
            val = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
            saveBMP('imagens/img_{0}_{1}.jpg'.format(contador, val), rgb)

            contador += 1

    except Exception as exp:
        print(exp)

def saveBMP(filename, rgbValues):

    print('Dados imagen completos:{0}'.format(filename))
    cv2.imwrite(filename, rgbValues)

if __name__ == '__main__':

    #definicao da porta serial
    dados_device = {}
    dados_device['port'] = '/dev/ttyACM0'
    dados_device['baudrate'] = 460800
    dados_device['timeout'] = None
    dados_device['bytesize'] = serial.EIGHTBITS
    dados_device['parity'] = serial.PARITY_NONE
    dados_device['stopbits'] = serial.STOPBITS_ONE

    #rotina de carga serial
    teste_c1(dados_device)
