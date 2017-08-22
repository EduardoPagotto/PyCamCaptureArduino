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

import time
import serial

port = '/dev/ttyUSB0'
baudrate = 9600
timeout = None
bytesize = EIGHTBITS

def InfoComSerial():
    '''Retorna o nome e a configuracao da porta'''
    print('\nObtendo informacoes sobre a comunicacao serial\n')

    # Iniciando conexao serial
    comport = serial.Serial(port = port, baudrate = baudrate, timeout=timeout, bytesize=EIGHTBITS, , )
    time.sleep(1.8) # Entre 1.5s a 2s

    print('\nStatus Porta: %s ' % (comport.isOpen()))
    print('Device conectado: %s ' % (comport.name))
    print('Dump da configuracao:\n %s ' % (comport))
    print('\n###############################################\n')

    # Fechando conexao serial
    comport.close()

if __name__ == '__main__':

    ser = serial.Serial(DEVICE, SPEED)
    ser.write(b'5')
    #ser.write(b'5') #Prefixo b necessario se estiver utilizando Python 3.X
    ser.read()
