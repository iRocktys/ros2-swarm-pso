# Baseado em: #https://davesroboshack.com/the-robot-operating-system-ros/ros2-topics/
# Baseado em: https://github.com/ros2/rclpy/issues/629
# Baseado em: https://answers.ros.org/question/362954/ros2create_subscription-how-to-pass-callback-arguments/
	
import time
import rclpy
import math
from rclpy.action import ActionServer
from rclpy.node import Node
from beswarm_action_interfaces.action import Beswarm
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from random import seed
from random import random
from functools import partial
from turtlesim.msg import Pose


class BeswarmActionServer(Node):

	ITERATION=0
	particles = []
	globalBestFit=100000 # Global inicializa com um grande valor
	globalBestPosX=0
	globalBestPosY=0
	globalMeanFit=0

	def __init__(self):
		super().__init__('beswarm_action_server')
		
        # inicializa gerador de números aleatórios
		seed(1)
		print('Servidor de ação aguardando conexões...')
		self._action_server = ActionServer(
        	self,
        	Beswarm,
        	'beswarm',
        	self.execute_callback)

    
	# Fica em escuta contínua após a função 'execute_callback'
	def listener_pose(self, msg, robot_name, NUM_PARTICLES, EVO):
    	#self.get_logger().info(f'Log2: Partícula {robot_name} encontrada em x: {msg.x}; y: {msg.y}')
		self.ITERATION +=1
		i = int(robot_name)

    	#0-velX,1-velY,2-posX,3-posY,4-fit,5-bestFit,6-bestPosX,7-bestPosY
		velX = self.particles[i][0]
		velY = self.particles[i][1]
		posX = msg.x  # posição atual
		posY = msg.y  # posição atual
		bestFit = self.particles[i][5]
		bestPosX = self.particles[i][6]
		bestPosY = self.particles[i][7]
		angle = self.particles[i][8]
		node = rclpy.create_node('listener_server_pub__')
   	 
    	#Calcula a aptidão
    	#node.get_logger().info('Log3: calcula aptidão')
		self.particles[i][2] = posX
		self.particles[i][3] = posY
		fit = calculateFitness(self, i, EVO)

    	#node.get_logger().info('Log4: Robô:{0} X:{1} Y:{2} Ângulo:{3} Aptidão:{4}'.format(i,posX,posY,angle,fit))

    	#self.get_logger().info(f'Log3: Partícula: {i}; x: {self.particles[i][0]}; y: {self.particles[i][1]}')
		value = '/turtle'
		value += str(i)
		value += '/cmd_vel'
		publisher = node.create_publisher(Twist, value, 10) #10 é o tamanho da fila (padrão) para o assinante tentar ler esta mesma mensagem.
		r1 = random()
		r2 = random()
		w = 0.005
		f1 = 0.002
		f2 = 0.002
		vx = w * velX + f1 * r1 * (bestPosX - posX) + f2 * r2 * (self.globalBestPosX - posX)
    
    	#A partir de agora, não é necessário fornecer detecção de colisão entre os robôs para avaliar a convergência
    	#Há um atraso para mover, então tenho que pegar a posição apenas no próximo passo da escuta
    	#msg.x = vx
		vy = w * velY + f1 * r1 * (bestPosY - posY) + f2 * r2 * (self.globalBestPosY - posY)
    	#msg.y = vy
		self.particles[i][0] += vx #velX
		self.particles[i][1] += vy #velY
		self.particles[i][2] = posX
		self.particles[i][3] = posY
		self.particles[i][4] = fit
		if fit < bestFit:
        	#Melhor aptidão pessoal
			self.particles[i][5] = fit
        	#Melhor posição X
			self.particles[i][6] = posX
        	#Melhor posição Y
			self.particles[i][7] = posY
        	#node.get_logger().info('Log5: Partícula {0} encontrou melhor aptidão: {1}'.format(i,fit))
    	#muda o ângulo
		elif fit > bestFit:
			orientation = int(1+random()*2)
			if orientation==1:
				angle=1.0 #anti-horário
			elif orientation==2:
				angle=-1.0 #horário
           	 
        	#node.get_logger().info('Log7: Partícula {0} muda o ângulo: {1}'.format(i,angle))
			if fit < self.globalBestFit:
				self.globalBestFit = fit
				self.globalBestPosX = bestPosX
				self.globalBestPosY = bestPosY
        	#node.get_logger().info('Log6: (Global) Encontrou nova melhor aptidão global: {0}'.format(self.globalBestFit))
                    	 
    	#self.particles[i][0] = 1.0;
       	 
    	#Cria mensagem para publicar
		msg = Twist()
		msg.linear.x=self.particles[i][0]
		msg.linear.y=self.particles[i][1]
    	#Só muda o ângulo se não melhorar a aptidão
		msg.angular.z=angle
       	 
    	#node.get_logger().info('Publicando: X:{0} Y:{1} Z:{2}'.format(msg.linear.x,msg.linear.y,msg.angular.z)) #mensagem no servidor
		publisher.publish(msg) #servidor publica no tópico

    	#É necessário isso para o cálculo do 'globalMeanFit'
		self.globalMeanFit = 0
		for i in range(1, int(NUM_PARTICLES)+1):
			self.globalMeanFit += self.particles[i][4]
				
		self.globalMeanFit /= int(NUM_PARTICLES);
        # coment
		node.get_logger().info('Iteração: {0} Aptidão média: {1}'.format(self.ITERATION,self.globalMeanFit))
	    # time.sleep(1.0) #devagar
   	 
    
	def generate_callback(self, robot_name, NUM_PARTICLES, EVO):
		return lambda msg: self.listener_pose(msg, robot_name, NUM_PARTICLES, EVO)

	#1-Aqui posso responder ao assinante
	#   Cria partículas e tópicos de escuta
	#   Apenas um feedback
	def execute_callback(self, goal_handle):
		self.get_logger().info('Executando meta...')

    	#cria uma lista de partículas
		NUM_PARTICLES = goal_handle.request.client_id  #recebe 'client_id' do assinante
		EVO = goal_handle.request.evo  #recebe 'evo' do assinante

    	#cria array de robôs
		for line in range(NUM_PARTICLES+1):
			particle = []
			for col in range(9): #velX,velY,posX,posY,aptidão,melhorAptidão,melhorX,melhorY,ângulo
				particle.append(0.0) #inicializa o array
				
			self.particles.append(particle)

    	#Apenas uma resposta ao cliente
		feedback_msg = Beswarm.Feedback()
		feedback_msg.px = 0
		feedback_msg.py = 0
		feedback_msg.pz = 0
		
		node = rclpy.create_node('minimal_publisher')
		
		for i in range(1, NUM_PARTICLES+1):
			self.get_logger().info(f'x: {self.particles[i][0]}; y: {self.particles[i][1]}')
			value = '/turtle'
			value += str(i)
			value += '/cmd_vel'
           	
			publisher = node.create_publisher(Twist, value, 10) #10 é o tamanho da fila (padrão) para o assinante tentar ler esta mesma mensagem.
           	
			self.particles[i][0] = 1.0;
           	 
        	#Cria mensagem de posição
			msg = Twist()
			msg.linear.x=self.particles[i][0]
			msg.linear.y=self.particles[i][1]
			msg.angular.z=self.particles[i][8]

            	#Atualiza a posição atual da partícula
            	#self.particles[i][0] = msg.linear.x #vel
            	 #self.particles[i][1] = msg.linear.y
            	#self.particles[i][2] = msg.linear.z #ângulo
            	#Aptidão inicial
			fit = calculateFitness(self, i,EVO)
			self.particles[i][4] = fit #aptidão
			self.particles[i][5] = fit #melhor aptidão
			self.particles[i][6] = 0.0 #melhorPosX
			self.particles[i][7] = 0.0 #melhorPosY
			self.particles[i][8] = 0.0 #ângulo
           	 

            	#rqt dá posição? R.: não
            	#Tutorial parte 3:
            	#ros2 topic list -t
            	#ros2 topic echo /turtle1/pose
           	 
            	#node.get_logger().info('Publicando: X:{0} Y:{1} Z:{2}'.format(msg.linear.x,msg.linear.y,msg.angular.z)) #mensagem no servidor
			publisher.publish(msg) #servidor publica no tópico
           	 
            	#---
            	#Resposta parcial ao assinante
            	#Campos de Beswarm.action
			feedback_msg.px= self.particles[i][0]
			feedback_msg.py= self.particles[i][1]
			feedback_msg.pz=  self.particles[i][8]
			feedback_msg.pfit=float(self.particles[i][4])
           	 
            	#self.get_logger().info('Feedback: X:{0} Y:{1} Z:{2} Aptidão:{3}'.format(feedback_msg.px,feedback_msg.py,feedback_msg.pz,feedback_msg.pfit))
			goal_handle.publish_feedback(feedback_msg) #resposta parcial ao cliente
			time.sleep(0.1)

    	#fim do for
		goal_handle.succeed()

    	#Assina em cada tópico    
    	#Callback da pose do ouvinte
		for i in range(NUM_PARTICLES+1):    
        	#Preciso do msg.x e msg.y aqui
			self.my_subscriber = self.create_subscription(
            	Pose,
            	'turtle' + str(i) + '/pose',
            	self.generate_callback(str(i),str(NUM_PARTICLES),str(EVO)),
            	10)
    	#fim do for   	 
               	 
    	#É necessário responder
		result = Beswarm.Result() #resposta final ao assinante
		result.x = msg.linear.x #campo de resposta da interface de ação (BeSwarm.action)
		result.y = msg.linear.y
		result.z = msg.angular.z
		return result
		
def calculateFitness(self, robot_name,evo):
	#self.get_logger().info('Log1: função de aptidão: EVO:{0}'.format(evo))
	i = int(robot_name)
	x = self.particles[i][2]
	y = self.particles[i][3]

	EVO = int(evo)
	result=0
	if EVO==1: #Função Esfera: Mínimo global: X:0.0 Y:0.0
    	#self.get_logger().info('Log9: EVO:{0}'.format(EVO))
	  	result=pow(x,2) + pow(y,2)
	elif EVO==2: #Rastrigin: Mínimo global: X:0.0 Y:0.0
		v = [x, y]
		d = len(v)
		sum1 = 0
		for i in range(d):
			sum += (pow(v[i],2)-10*math.cos(2*math.pi*v[i]))
		result=10*d + sum1
	elif EVO==3:  #Ackley: Mínimo global: X:0.0 Y:0.0
		v = [x, y]
		d = len(v)
		c = 2*math.pi
		b = 0.2
		a = 20
		sum1 = 0
		sum2 = 0
		xi=0
		for i in range(d):
			xi = v[i]
			sum1 += pow(xi,2)
			sum2 += math.cos(c*xi)
		term1 = -a * math.exp(-b*math.sqrt(sum1/d))
		term2 = -math.exp(sum2/d)

		result = term1 + term2 + a + math.exp(1)
	elif EVO==4: #Adjiman: Mínimo global: X:-2.0 Y:-2.0
		result=math.cos(x)*math.sin(y)-(x/(pow(y,2)+1))
	else: #Distância Euclidiana para (800.0,800.0) no canto superior direito
		result=math.sqrt(pow(x-800.0,2) + pow(y-800,2))
	return result
    
def main(args=None):
	rclpy.init(args=args)
	beswarm_action_server = BeswarmActionServer()
	rclpy.spin(beswarm_action_server) #executa enquanto estiver ativo
    
if __name__ == '__main__':
	main()
