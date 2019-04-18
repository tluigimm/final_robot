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

dx = 0

x = None
y = None
z = None
oi = 0

def scaneou(dado):
	global x
	global y

	x = list(dado.ranges)

def scaneou2(dado):

	global oi

	k  = str(dado)
	oi = int(k[len(k)-1])

	recebe_bumper = rospy.Subscriber("/bumper", UInt8, scaneou)

def Bumber(test):
	global oi
	if test == 0:
			print("ta tudo normal")
			return 'Normal'

	if test!=0:
		if test == 1 or test == 2:
			oi = 0
			return 'Frente'

		else:
			oi = 0
			return 'Atras'

def vetir(x):
	lista_esquerda = []
	lista_direita = []
	for e in range(len(x)):
		if e <= 90:
			if x[e] == float("Inf"):
				lista_direita.append(10)
			else:
				lista_direita.append(x[e])
		elif 360>=e>=270:
			if x[e] == float("Inf"):
				lista_esquerda.append(10)
			else:
				lista_esquerda.append(x[e])


	if min(lista_esquerda)<0.8:
		print("Esquerda:")
		print(min(lista_esquerda))
		return "Esquerda"
	elif min(lista_direita)<0.8:
		print("Direita:")
		print(min(lista_direita))
		return "Direita"
	elif min(lista_direita)<0.8 and min(lista_esquerda)<0.8:
		print('BOTH')
		return "Direita"
	else:
		return "Normal"

def sobrevivencia(Bum, Scan):
	global oi
	k = vetir(Scan)
	io = Bumber(Bum)
	if io == 'Frente':
		return io
	elif io == 'Atras':
		return io
	elif vetir == 'Direita':
		return k
	elif vetir == 'Esquerda':
		return k




# funcao que devolve o tracker
def tracker_funcao():
	ap = argparse.ArgumentParser()
	ap.add_argument("-v", "--video", type=str,
		help="path to input video file")
	ap.add_argument("-t", "--tracker", type=str, default="kcf",
		help="OpenCV object tracker type")
	args = vars(ap.parse_args())

	(major, minor) = cv2.__version__.split(".")[:2]

	if int(major) == 3 and int(minor) < 3:
		tracker = cv.Tracker_create(args["tracker"].upper())

	else:
		OPENCV_OBJECT_TRACKERS = {
			"csrt": cv2.TrackerCSRT_create,
			"kcf": cv2.TrackerKCF_create,
			"boosting": cv2.TrackerBoosting_create,
			"mil": cv2.TrackerMIL_create,
			"tld": cv2.TrackerTLD_create,
			"medianflow": cv2.TrackerMedianFlow_create,
			"mosse": cv2.TrackerMOSSE_create
		}

		tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()

	return tracker

# função que faz o tracking
def tracker_(img):
	# grab the new bounding box coordinates of the object
	(success, box) = tracker.update(img)
	return [success, box]

stop = False

# conta quantos frames tem vermelho na tela
contador = 0

dx = 0


red = False
bird_coordenadas = []

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
	global bird_coordenadas

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
		dx = cv_image.shape[1] // 20
		depois = time.clock()
		media, centro, area, contornos =  cormodule.identifica_cor(cv_image)

		print(media)

		if media[0] + media[1] != 0:
			red = True
		else:
			red = False

		# if not stop:
		centro, imagem, resultados = visao_module.processa(cv_image)

		cv2.line(cv_image, (centro[0]+dx,0), (centro[0]+dx,cv_image.shape[1]), (0,0,255), 2)
		cv2.line(cv_image, (centro[0]-dx,0), (centro[0]-dx,cv_image.shape[1]), (0,255,0), 2)

		for e in resultados:
			obj = e[0]

		if obj is not None and obj == "bird":
			print("eu acho que vi um passarinho")

			bird = True

			x0, y0 = resultados[0][2]
			x1, y1 = resultados[0][3]

			bird_coordenadas = [(x0,y0), (x1,y1)]
			centro_bird = [x0 + (x1-x0)//2, y0 + (y1-y0)//2]

			cv2.circle(cv_image,(centro_bird[0],centro_bird[1]), 5, (0,0,255), -1)

			contador += 1

		else:
			bird = False


		cv2.imshow("Camera", cv_image)

	except CvBridgeError as e:
		print('ex', e)

# inicia tracker
tracker = tracker_funcao()

initBB = None

if __name__=="__main__":
	rospy.init_node("cor")

	topico_imagem = "/kamera"

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	recebe_bumper = rospy.Subscriber("/bumper", UInt8, scaneou2)


	try:
		while not rospy.is_shutdown():
			rospy.sleep(1)
			palavra = vetir(x)

			vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
			print(palavra)
			if palavra == 'Normal' and oi == 0:

			# se existe passaro, segue:
				if bird:
					# inicia tracking apos 5 frames seguidos com vermelho
						if initBB is None:
							x_min = bird_coordenadas[0][0]
							y_min = bird_coordenadas[0][1]
							x_max = bird_coordenadas[1][0]
							y_max = bird_coordenadas[1][1]

							initBB = (x_min, y_min, x_max, y_max)

						# else:
						if not stop:
							print('COMEÇOU O TRACKER AI CARAMBA')
							tracker.init(cv_image, initBB)

						success, box = tracker_(cv_image)[0:2]

						# sucesso em fazer o tracking
						if success:
							print('tracking')

							# determina o retangulo a ser trackeado
							(x, y, w, h) = [int(v) for v in box]
							cv2.rectangle(cv_image, (x, y), (w, h), (255, 0, 0), 2)

							stop = True

							# centro do objeto que vai ser trackeado
							media_tracking = ((x+w)//2, (y+h)//2)

							cv2.circle(cv_image,(media_tracking[0],media_tracking[1]), 5, (0,0,255), -1)

							# vai para frente quando tracking centralizado
							if centro[0]+dx >= media_tracking[0] >= centro[0]-dx:
								print("tracking - CENTRALIZADOOOO")
								vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))

							# gira sentido horário e vai para frente quando centro do tracking esta a direita
							elif media_tracking[0] > centro[0]+dx:
								print("tracking - HORARIO")
								vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.5))

							# gira sentido anti-horário e vai para frente quando centro do tracking esta a esquerda
							elif media_tracking[0] < centro[0]-dx:
								print("tracking - ANTI-HORARIO")
								vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.5))

							# Caso centro do tracking saia da tela, redefine contador como 0 e procura novo tracking
							else:
								contador = 0
								stop = False


				# caso veja vermelho, foge dele
				elif red and not bird:
					print("oh nao, um vermelho")
					vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,0))

				

				print("stop: " + str(stop))
			elif palavra == 'Direita' and oi == 0:
				vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.5))
			elif palavra == "Esquerda" and oi == 0:
				vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.5))
			elif oi!=0:
				if oi == 1 or oi == 2:
					print("BATEU NA FRENTEEEEEE")
					velocidade_saida.publish(para)
					rospy.sleep(1)
					velocidade_saida.publish(re)
			 		rospy.sleep(1)
			 		velocidade_saida.publish(rodadinha)
			 		rospy.sleep(1)
			 		oi = 0

				else:
					print("BATEU ATRAAAAAAAAAASSSS")
					velocidade_saida.publish(para)
					rospy.sleep(1)
					velocidade_saida.publish(normal)
			 		rospy.sleep(1)
			 		velocidade_saida.publish(rodadinha)
			 		rospy.sleep(1)
			 		oi = 0

			velocidade_saida.publish(vel)
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
