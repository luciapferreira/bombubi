'''
O status dos sensores é lido com
self.leitura()
que guarda os valores lidos em atributos da classe:
self.bateria, self.distancia, self.botao_start e self.botao_stop
Pode-se mostrar estes valores chamando de seguida:
self.mostra_status()

É preciso ter isto instalado:
sudo apt-get install python3-serial
https://pyserial.readthedocs.io/en/latest/pyserial_api.html

----------------------------------
Códigos dos comandos:

EXECUTAR AÇÕES
"21"= vento_stop()
"22"= vento_start()
"23"= parar()
"24"= motor_dir_frente()
"25"= motor_dir_tras()
"26"= motor_esq_frente()
"27"= motor_esq_tras()
"28"= motor_dir_speed(speed)
"29"= motor_esq_speed(speed)
o valor de speed está em [0,255]
----------------------------------
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CommandSubscriber(Node):

    def __init__(self):
        super().__init__('command_sub')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            2)
        try:
            self.serie = serial.Serial('/dev/ttyS0', 57600, timeout=1)
            self.serie.reset_input_buffer()
        except:
            self.get_logger().error('Erro na abertura da porta série')
            rclpy.shutdown()
            return

        # Minimum speed
        self.min_speed = 50
        # Maximum speed
        self.max_speed = 150

        self.get_logger().info(f'Porta série usada: {self.serie.name}')

    def comando(self, id, param='-1'):
        '''
		envia um comando ao arduino, enventualmente com um parâmetro adicional
            id = id do comando a enviar (ver comentários no início deste ficheiro) : é uma string!
            param = valor a enviar adicionalmente ao comando. Se for = '-1', não envia. É uma string!
            devolve -1 se tiver ocorrido um erro e zero caso contrário

		todas as mensagens enviadas começam com o símbolo # para o arduino saber que está a 
        apanhar a mensagem desde o início
		'''
        id = '#' + id
        try:
            self.serie.write(id.encode())
            self.serie.write('\n'.encode())
        except IOError:
            self.get_logger().error(f'Erro no envio do comando {id}')
            return -1
        
        if param != '-1':
            param = '#' + param
            self.serie.write(param.encode())
            self.serie.write('\n'.encode('utf-8'))
        return 0  # OK

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        left_speed = self.calculate_speed(linear_x)
        right_speed = self.calculate_speed(linear_x)

        if angular_z != 0:
            # Adjust speeds based on angular velocity for turning
            if angular_z > 0:
                right_speed -= int(abs(angular_z) * self.max_speed)
            elif angular_z < 0:
                left_speed -= int(abs(angular_z) * self.max_speed)
            
            # Ensure speeds are within bounds
            left_speed = min(max(left_speed, self.min_speed), self.max_speed)
            right_speed = min(max(right_speed, self.min_speed), self.max_speed)

        # Send speeds to the motors
        self.comando('28', str(abs(left_speed)))
        self.comando('29', str(abs(right_speed)))

        # Move forward
        if linear_x > 0:
            self.get_logger().info('Comando: frente')
            self.comando('24')
            self.comando('26')
        # Move backwards
        elif linear_x < 0:
            self.get_logger().info('Comando: trás')
            self.comando('25')
            self.comando('27')
        # Turn right
        if angular_z > 0:
            self.get_logger().info('Comando: direita')
            self.comando('24')
            self.comando('27')
        # Turn left
        elif angular_z < 0:
            self.get_logger().info('Comando: esquerda')
            self.comando('26')
            self.comando('25')
        # Stop
        time.sleep(0.3)
        self.comando('23')

    def calculate_speed(self, linear_x):
        # Calculate speed based on linear velocity and apply constraints
        speed = int(self.max_speed * abs(linear_x))
        return min(max(speed, self.min_speed), self.max_speed)

    def leitura(self):
        '''
		ler a string de status
		devolve -1 se tiver ocorrido um erro senão devolve o valor lido do sensor
        '''
        response = ''
        self.comando('19')
        # recebe resposta do comando enviado
        while True:
            if self.serie.inWaiting() > 0:
				#print('recebemos resposta...')
                response = self.serie.readline().decode()
				# a msg começa em '#'
                if response[0] == '#':
                    # apanhamos a mensagem no início
                    #print('a mensagem de status recebida é:',response)
                    p = response.find(',')
                    self.distancia = float(response[1:p])
                    q = response.find(',',p+1)
                    self.bateria = float(response[p+1:q])
                    p = response.find(',',q+1)
                    self.botao_start = int(response[q+1:p])
                    q = response.find(',',p+1)
                    self.botao_stop = int(response[p+1:q])
                    return 0
                else:
                    return -1

def main(args=None):
    rclpy.init(args=args)
    node = CommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
