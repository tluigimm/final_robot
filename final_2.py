#! /usr/bin/env python
# -*- coding:utf-8 -*-

# Robo minhoca comuista

__author__ = ["Thiago Luigi", "Luis Vitor"]

import rospy
import numpy as np
import tf
import math
import cv2
import argparse
import time
import cormodule
import visao_module
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

cv_image = None

media = []
centro = []
contornos = None

atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

bird = False
centro_bird = []

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados.
# Descarta imagens que chegam atrasadas demais
check_delay = False

x = None
oi = 0
dx = 0
red = False

def scaneou_lazer(dado):
	global x

	x = list(dado.ranges)

def scaneou_bumpper(dado):
	global oi

	k  = str(dado)
	oi = int(k[len(k)-1])

def vetir(x):
	lista_esquerda = []
	lista_direita = []
	if x is not None:
		for e in range(len(x)):
			if e <= 90:
				if x[e] == float("Inf") or x[e] == 0:
					lista_direita.append(10)
				else:
					lista_direita.append(x[e])
			elif 360>=e>=270:
				if x[e] == float("Inf") or x[e] == 0:
					lista_esquerda.append(10)
				else:
					lista_esquerda.append(x[e])


	if min(lista_esquerda) < 0.15 and min(lista_esquerda) > 0.12:
		print("Esquerda:")
		return "Esquerda"
	elif min(lista_direita) < 0.15 and min(lista_direita) > 0.12:
		print("Direita:")
		return "Direita"
	elif min(lista_direita) < 0.15 and min(lista_esquerda) < 0.15 and min(lista_esquerda) > 0.12 and min(lista_direita) > 0.12:
		print('BOTH')
		return "Direita"
	else:
		return "Normal"


# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global contornos
	global dx
	global stop
	global bird
	global contador
	global centro_bird
	global red

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame: ", delay)
		return
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		dx = cv_image.shape[1] // 10
		depois = time.clock()
		media, centro, area = cormodule.identifica_cor(cv_image)

		if media[0] + media[1] != 0:
			red = True
		else:
			red = False

		centro, imagem, resultados = visao_module.processa(cv_image)

		cv2.line(cv_image, (centro[0]+dx,0), (centro[0]+dx,cv_image.shape[1]), (0,0,255), 2)
		cv2.line(cv_image, (centro[0]-dx,0), (centro[0]-dx,cv_image.shape[1]), (0,0,255), 2)

		if resultados[0][0] == "bird":
			print("PASSAROOOOOOOOOOOOOOO")

			bird = True

			x0, y0 = resultados[0][2]
			x1, y1 = resultados[0][3]

			centro_bird = [x0 + (x1-x0)//2, y0 + (y1-y0)//2]

			cv2.circle(cv_image,(centro_bird[0],centro_bird[1]), 5, (0,0,255), -1)

		else:
			bird = False

		cv2.imshow("Camera", cv_image)

	except CvBridgeError as e:
		print('ex', e)


if __name__=="__main__":
	rospy.init_node("cor")

	topico_imagem = "/kamera"

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou_lazer)
	recebe_bumper = rospy.Subscriber("/bumper", UInt8, scaneou_bumpper)

	try:
		while not rospy.is_shutdown():

			rospy.sleep(1)
			lazer = vetir(x)

			print(oi)

			# bateu o bumpper
			if oi != 0:
				print("BATEU AI O CARAMBAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
				if oi == 1 or oi == 2:
					print("BATEU NA FRENTEEEEEE")
					velocidade_saida.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
					rospy.sleep(1)
					velocidade_saida.publish(Twist(Vector3(-0.1,0,0),Vector3(0,0,0)))
					rospy.sleep(1)
					velocidade_saida.publish(Twist(Vector3(0,0,0),Vector3(0,0,0.5)))
					rospy.sleep(1)

				else:
					print("BATEU ATRAAAAAAAAAASSSS")
					velocidade_saida.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
					rospy.sleep(1)
					velocidade_saida.publish(Twist(Vector3(0.1,0,0),Vector3(0,0,0)))
					rospy.sleep(1)
					velocidade_saida.publish(Twist(Vector3(0,0,0),Vector3(0,0,0.5)))
					rospy.sleep(1)

				oi = 0

			else:
				# da re durante 3s ao ver um passaro
				if bird:
					print("oh nao, um passaro")
					velocidade_saida.publish(Twist(Vector3(-0.1,0,0),Vector3(0,0,0)))
					rospy.sleep(3)

				# lazer detecta na direita
				if lazer == 'Direita':
					velocidade_saida.publish(Twist(Vector3(0.1,0,0),Vector3(0,0,-0.5)))

				# lazer detecta na esquerda
				elif lazer == "Esquerda":
					velocidade_saida.publish(Twist(Vector3(0.1,0,0),Vector3(0,0,0.5)))

				# lazer não detecta nada
				elif lazer == 'Normal':

					# segue vermelho caso não haja passaro
					# if red and not bird:

					# vai para frente quando tracking centralizado
					if centro[0]+dx >= media[0] >= centro[0]-dx:
						print("CENTRALIZADOOOO")
						vel = Twist(Vector3(0.1,0,0),Vector3(0,0,0))

					# gira sentido horário e vai para frente quando centro do tracking esta a direita
					elif media[0] > centro[0]+dx:
						print("HORARIO")
						vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.5))

					# gira sentido anti-horário e vai para frente quando centro do tracking esta a esquerda
					elif media[0] < centro[0]-dx:
						print("ANTI-HORARIO")
						vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.5))

					# Caso centro do tracking saia da tela, redefine contador como 0 e procura novo tracking
					else:
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.5))

					velocidade_saida.publish(vel)
					rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
