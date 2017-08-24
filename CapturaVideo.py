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

            rgb = np.empty((HEIGHT, WIDTH), dtype=int)
            rgb2 = np.empty((WIDTH, HEIGHT,), dtype=int)

            for y in range(0, HEIGHT):
                for x in range(0, WIDTH):
                    val = ser.read()
                    temp = struct.unpack(">B", val)[0]
                    rgb[y][x] = ((temp & 0xFF) << 16) | ((temp & 0xFF) << 8) | (temp & 0xFF)
                
            for y in range(0, HEIGHT):
                for x in range(0, WIDTH):
                    rgb2[x][y]=rgb[y][x]
            

            print('Dados imagen completos')

            saveBMP('img_{0}.bmp'.format(contador), rgb2)

            contador += 1

    except Exception as exp:
        print(exp)


def saveBMP(filename, rgbValues):

    #cv2.imwrite()

    #plt.imshow(rgbValues)
    #plt.savefig(filename)

    #cv2.imwrite()
    cv2.imwrite(filename, rgbValues)

    # im = Image.fromarray(rgbValues)
    # im.save(filename)


    # imgSize = (320,240)# the image size
    # img = Image.frombytes('L', imgSize, rgbValues)
    # img.save("filename")# can give any format you like .png




    # try:
    #     #FileOutputStream fos = new FileOutputStream(new File(filename));
    #     with open(filename, "wb") as file:
    #         #file.write(pack("<IIIII", *bytearray([120, 3, 255, 0, 100])))

    #         tw = len(rgbValues)
    #         th = len(rgbValues[0])
    #         tamanho_arquivo = 54 + 3 * tw * th
    #         buffer = bytearray(tamanho_arquivo)
    #         #buffer = bytes(tamanho_arquivo)

    #         #header
    #         buffer[0] = ord('B')#struct.unpack(">B", 'B')[0]
    #         buffer[1] = ord('M')#struct.unpack(">B", 'M')[0]

    #         buffer[5] = tamanho_arquivo
    #         buffer[4] = tamanho_arquivo >> 8
    #         buffer[3] = tamanho_arquivo >>16
    #         buffer[2] = tamanho_arquivo >>24

    #         #data offset
    #         buffer[10] = 54


    #         #bytes = new byte[54 + 3*rgbValues.length*rgbValues[0].length];
    #         #saveFileHeader();
    #         #saveInfoHeader(rgbValues.length, rgbValues[0].length);
    #         #saveBitmapData(rgbValues);

    #         file.write(buffer)

    #         #fos.close();

    # except Exception as e:
    #     print(e)

	# public void saveBMP(String filename, int [][] rgbValues){
	# 	try {
	# 		FileOutputStream fos = new FileOutputStream(new File(filename));
			
	# 		bytes = new byte[54 + 3*rgbValues.length*rgbValues[0].length];

	# 		saveFileHeader();
	# 		saveInfoHeader(rgbValues.length, rgbValues[0].length);
	# 		saveBitmapData(rgbValues);

	# 		fos.write(bytes);
			
	# 		fos.close();
	# 	} catch (IOException e) {
	# 		throw new IllegalStateException(e);
	# 	}
	# }


if __name__ == '__main__':

    dados_device = {}
    dados_device['port'] = '/dev/ttyACM0'
    dados_device['baudrate'] = 460800
    dados_device['timeout'] = None
    dados_device['bytesize'] = serial.EIGHTBITS
    dados_device['parity'] = serial.PARITY_NONE
    dados_device['stopbits'] = serial.STOPBITS_ONE

    teste_c1(dados_device)

    #InfoComSerial(dados_device)


    #ser = serial.Serial(DEVICE, SPEED)
    #ser.write(b'5')
    ##ser.write(b'5') #Prefixo b necessario se estiver utilizando Python 3.X
    #ser.read()
