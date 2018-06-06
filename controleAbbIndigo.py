#!/usr/bin/env python
import rospy
import roslib
from math import atan2, cos, sin, sqrt, pi, degrees
import time

#import sys

#================ TRABALHO FINAL DE ELEMENTOS DE ROBOTICA ========================================

# Os limites das juntas do robo sao (em graus e radianos em parenteses):
	# theta_1 = -180 (-3.14)	a 	180 (3.14)
	# theta_2 = -90	(-1.57)		a 	150 (2.62)
	# theta_3 = -180 (-3.14) 	a 	75 (1.30)
	# theta_4 = -400 (-6.98)	a 	400 (6.98)
	# theta_5 = -125 (-2.18)	a 	120 (2.1)
	# theta_6 = -400 (-6.98)	a 	400 (6.98)


#=================================================================================================


#imports de mensagens
from geometry_msgs.msg import Accel


#Cria a classe do noo para publicar no motor
class ControleRobo():

	#Metodo criador da classe
	def __init__(self):

		#Enviando uma informacao para o usuario
		rospy.loginfo("Node de Controle do Robo ABB IRB inicializado")

		#Variavel que carrega a posicao atual das juntas em radianos
		self.posJunta = [0 for x in range(6)]

		#Variavel que carrega a posicao atual das juntas em graus
		self.posJuntaGraus = [0 for x in range(6)]

		#Variavel que carrega o comando de posicao a ser enviado para as juntas
		comandoJunta = [0 for x in range(6)]
		
		self.posC = [0 for x in range(3)]
		self.teta = [0 for x in range(6)]
		self.d1 = 0.495		
		self.d4 = 0.960
		self.a1 = 0.175
		self.a2 = 0.900
		self.a3 = 0.175
		self.para = 0	


		#Criando os publishers e subscribers do ROS
		self.pub = rospy.Publisher('/vrep_ros_interface/ABBIRB/atuarNasJuntas', Accel, queue_size=1)
		rospy.Subscriber('/vrep_ros_interface/ABBIRB/posicaoAtualJuntas',Accel,self.juntaPosCallback)
		#rospy.spin()

#====================================================================================================
		#INICIO DO LOOP INFINITO DE CONTROLE DO ROBO
#====================================================================================================
		while not rospy.is_shutdown():
					
			self.espera(1.5)			
		
			while self.para != 1:
				#CHEGA PERTO PRIMEIRA CAIXA ESTANTE
				self.calculaAngulo(-0.723,0.140,1.245)
				for x in range(0,6):
					comandoJunta[x] = self.teta[x]
				print('--------\nComando Junta:\n {}'.format(comandoJunta))
				print('--------\nPos Junta:\n {}'.format(self.posJunta))
				self.aplicarComandoJuntas(comandoJunta)	

				self.espera(5.5)
				
				#EMPURRA A CAIXA
				self.calculaAngulo(-1.223,0.340,1.145)
				for x in range(0,6):
					comandoJunta[x] = self.teta[x]
				print('--------\nComando Junta:\n {}'.format(comandoJunta))
				print('--------\nPos Junta:\n {}'.format(self.posJunta))
				self.aplicarComandoJuntas(comandoJunta)	

				self.espera(4)	

				#AFASTA DA CAIXA
				self.calculaAngulo(-0.723,0.140,1.245)
				for x in range(0,6):
					comandoJunta[x] = self.teta[x]
				print('--------\nComando Junta:\n {}'.format(comandoJunta))
				print('--------\nPos Junta:\n {}'.format(self.posJunta))
				self.aplicarComandoJuntas(comandoJunta)	

				self.espera(3)	
		
				#POSICIONA PARA DERRUBAR A CAIXA DA MESA				
				self.calculaAngulo(1.323,1.2140,1.345)
				for x in range(1,6):
					comandoJunta[0] = self.teta[0]
					comandoJunta[x] = self.teta[x]
				print('--------\nComando Junta:\n {}'.format(comandoJunta))
				print('--------\nPos Junta:\n {}'.format(self.posJunta))
				self.aplicarComandoJuntas(comandoJunta)
	
				self.espera(5)
	
				#DERRUBA PRIMEIRA CAIXA DA MESA			
				self.calculaAngulo(1.7230,1.8140,0.7476)
				for x in range(1,6):
					comandoJunta[0] = self.teta[0]
					comandoJunta[x] = self.teta[x]
				print('--------\nComando Junta:\n {}'.format(comandoJunta))
				print('--------\nPos Junta:\n {}'.format(self.posJunta))
				self.aplicarComandoJuntas(comandoJunta)
	
				self.espera(1.5)
					
				#DERRUBA A SEGUNDA CAIXA DA MESA
				self.calculaAngulo(1.5962,3.5890,0.7476)
				for x in range(1,6):
					comandoJunta[0] = self.teta[0]
					comandoJunta[x] = self.teta[x]
				print('--------\nComando Junta:\n {}'.format(comandoJunta))
				print('--------\nPos Junta:\n {}'.format(self.posJunta))
				self.aplicarComandoJuntas(comandoJunta)
	
				self.espera(1.5)
	
				#APROXIMA SEGUNDA CAIXA DA ESTANTE
				self.calculaAngulo(0.9286,0.3390,1.25)
				for x in range(1,6):
					comandoJunta[0] = self.teta[0] - pi
					comandoJunta[x] = self.teta[x]
				print('--------\nComando Junta:\n {}'.format(comandoJunta))
				print('--------\nPos Junta:\n {}'.format(self.posJunta))
				self.aplicarComandoJuntas(comandoJunta)
					
				self.espera(5.5)
	
				#EMPURRA SEGUNDA CAIXA DA ESTANTE
				self.calculaAngulo(1.6,0.7390,1.25)
				for x in range(1,6):
					comandoJunta[0] = self.teta[0] - pi
					comandoJunta[x] = self.teta[x]
				print('--------\nComando Junta:\n {}'.format(comandoJunta))
				print('--------\nPos Junta:\n {}'.format(self.posJunta))
				self.aplicarComandoJuntas(comandoJunta)
					
				self.espera(2)
				
				#VOLTA PARA A POSICAO PROXIMA A CAIXA				
				self.calculaAngulo(0.9286,0.3390,1.25)
				for x in range(1,6):
					comandoJunta[0] = self.teta[0] - pi
					comandoJunta[x] = self.teta[x]
				print('--------\nComando Junta:\n {}'.format(comandoJunta))
				print('--------\nPos Junta:\n {}'.format(self.posJunta))
				self.aplicarComandoJuntas(comandoJunta)
	
				self.espera(2)
			
				
				#APROXIMA CAIXA DE CIMA
				self.calculaAngulo(0.9286,0.00390,1.802)
				for x in range(1,6):
					comandoJunta[0] = self.teta[0] - pi
					comandoJunta[x] = self.teta[x]
				print('--------\nComando Junta:\n {}'.format(comandoJunta))
				print('--------\nPos Junta:\n {}'.format(self.posJunta))
				self.aplicarComandoJuntas(comandoJunta)
	
				self.espera(1.5)
	
				#DERRUBA CAIXA DE CIMA				
				self.calculaAngulo(1.9286,0.0390,1.902)
				for x in range(1,6):
					comandoJunta[0] = self.teta[0] - pi
					comandoJunta[x] = self.teta[x]
				print('--------\nComando Junta:\n {}'.format(comandoJunta))
				print('--------\nPos Junta:\n {}'.format(self.posJunta))
				self.aplicarComandoJuntas(comandoJunta)
	
				self.espera(1.5)

				#VOLTA PARA A POSICAO INICIAL				
				for x in range(1,6):
					comandoJunta[0] = 0
					comandoJunta[x] = 0
				print('--------\nComando Junta:\n {}'.format(comandoJunta))
				print('--------\nPos Junta:\n {}'.format(self.posJunta))
				self.aplicarComandoJuntas(comandoJunta)
				self.para = 1
	

#====================================================================================================
			



	def calculaAngulo(self,p1,p2,p3):
		self.teta[0] = atan2(p2,p1)
		self.s = p3 - self.d1
		self.somar1 = ((p1 - self.a1)*cos(self.teta[0]))**2
		self.somar2 = ((p1 - self.a1)*sin(self.teta[0]))**2
		self.r = sqrt(self.somar1 + self.somar2)
		self.beta = sqrt(self.a3**2 + self.d4**2)
		self.dnum = self.r**2 + self.s**2 - self.a2**2 - self.beta**2
		self.dden = 2*self.a2*self.beta
		self.d = self.dnum / self.dden
		self.d2 = self.d**2
		if self.d2 > 1:
			self.d2 = 1
		self.raizd = sqrt(1 - self.d2)
		self.teta[2] = atan2(self.raizd,self.d) - (pi/2) + atan2(self.a3,self.d4)
		self.atan1 = atan2(self.a3,self.d4)
		self.asin = sin(-self.teta[2] - (pi/2) + self.atan1)
		self.acos = cos(-self.teta[2] - (pi/2) + self.atan1)
		self.atan2 = atan2((self.beta*self.asin), (self.a2 + self.beta*self.acos))
		self.teta[1] = -atan2(self.s,self.r) + self.atan2 + (pi/2)		

	def espera(self,tempo):
		self.end_time = time.time() + tempo
		self.countTimer = 0
		self.sleepTime = 0.500
		while time.time() < self.end_time:
			time.sleep(self.sleepTime)
 			self.countTimer += self.sleepTime
   			print('proximo movimento em {} secs'.format(self.countTimer))
	
	#Funcao que eh chamada sempre que uma mensagem nova chega no topico de posicao das juntas
	def juntaPosCallback(self,data):

		self.posJunta[0]=data.linear.x
		self.posJunta[1]=data.linear.y
		self.posJunta[2]=data.linear.z
		self.posJunta[3]=data.angular.x
		self.posJunta[4]=data.angular.y
		self.posJunta[5]=data.angular.z

		#Converte o valor para graus em outra variavel
		i=0
		for x in self.posJunta:
			self.posJuntaGraus[i]=degrees(x)

	 	#print(self.posJunta)
	 	#print(self.posJuntaGraus)
		#print("------------")

	#Funcao que publica um valor de posicao para as juntas
	def aplicarComandoJuntas(self,data):

		#Variavel que receber o comando a ser enviado para as juntas
		comandoPub=Accel()


	# theta_1 = -180 (-3.14)	a 	180 (3.14)
	# theta_2 = -90	(-1.57)		a 	150 (2.62)
	# theta_3 = -180 (-3.14) 	a 	75 (1.30)
	# theta_4 = -400 (-6.98)	a 	400 (6.98)
	# theta_5 = -125 (-2.18)	a 	120 (2.1)
	# theta_6 = -400 (-6.98)	a 	400 (6.98)
	
		#Montando a variavel que sera publicada

		# Theta 1
		if data[0] < -3.14:
			comandoPub.linear.x = -3.14
		elif data[0] > 3.14:
			comandoPub.linear.x = 3.14
		else:
			comandoPub.linear.x = data[0]

		# Theta 2
		if data[1] < -1.57:
			comandoPub.linear.y = -1.57
		elif data[1] > 2.62:
			comandoPub.linear.y = 2.62
		else:
			comandoPub.linear.y = data[1]

		# Theta 3
		if data[2] < -3.14:
			comandoPub.linear.z = -3.14
		elif data[2] > 1.30:
			comandoPub.linear.z = 1.30
		else: 
			comandoPub.linear.z = data[2]

		# Theta 4
		if data[3] < -6.98:
			comandoPub.angular.x = -6.98
		elif data[3] > 6.98:
			comandoPub.angular.x = 6.98
		else: 
			comandoPub.angular.x = data[3]

		# Theta 5
		if data[4] < -2.18:
			comandoPub.angular.y = -2.18
		elif data[4] > 2.1:
			comandoPub.angular.y = 2.1
		else: 
			comandoPub.angular.y = data[4]

		# Theta 6
		if data[5] < -6.98:
			comandoPub.angular.z = -6.98
		elif data[5] > 6.98:
			comandoPub.angular.z = 6.98
		else: 
			comandoPub.angular.z = data[5]

		#Publicando o comando a ser enviado
		self.pub.publish(comandoPub)		


#Funcao main que chama a classe criada
if __name__ == '__main__':

	#Inicializa o nosso no com o nome
	rospy.init_node('controleAbb', anonymous=True)

	#Instancia a classe e entra em um regime de tratamento de eventuais erros
	try:
		obj_no = ControleRobo()
	except rospy.ROSInterruptException: pass
