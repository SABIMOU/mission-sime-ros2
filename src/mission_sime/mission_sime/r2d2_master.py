#!/usr/bin/env python3
"""
Nœud R2-D2 Master
- Écrit "MASTER SIME" au centre de l'écran
- Change aléatoirement la couleur du stylo
- Publie sa position sur /r2d2_pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, SetPen, TeleportAbsolute
from std_srvs.srv import Empty
import math
import random
import time

from mission_sime.msg import RobotPose

class R2D2Master(Node):
    def __init__(self):
        super().__init__('r2d2_master')
        
        # Déclaration des paramètres
        self.declare_parameter('initial_x', 5.0)
        self.declare_parameter('initial_y', 5.0)
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('pen_colors', ['red', 'green', 'blue', 'yellow', 'purple'])
        
        # Récupération des paramètres
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.speed = self.get_parameter('speed').value
        self.pen_colors = self.get_parameter('pen_colors').value
        
        self.get_logger().info('🤖 R2-D2 Master initialisé')
        self.get_logger().info(f'Position initiale: ({self.initial_x}, {self.initial_y})')
        self.get_logger().info(f'Vitesse: {self.speed}')
        self.get_logger().info(f'Couleurs disponibles: {self.pen_colors}')
        
        # Publishers
        self.pose_publisher = self.create_publisher(RobotPose, '/r2d2_pose', 10)
        
        # Clients de service
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.clear_client = self.create_client(Empty, '/clear')
        
        # Attendre que les services soient disponibles
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ En attente du service /turtle1/set_pen...')
        
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ En attente du service /turtle1/teleport_absolute...')
            
        # Initialisation
        self.init_turtle()
        
    def init_turtle(self):
        """Initialise la tortue à la position de départ"""
        self.get_logger().info('🎯 Initialisation de R2-D2...')
        
        # Téléportation à la position initiale
        self.teleport(self.initial_x, self.initial_y, 0.0)
        
        # Configuration du stylo initial
        self.set_random_pen_color()
        
        # Effacer l'écran
        self.clear_screen()
        
        # Démarrer l'écriture après un court délai
        self.create_timer(2.0, self.start_writing)
    
    def teleport(self, x, y, theta):
        """Téléporte la tortue à une position absolue"""
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        
        future = self.teleport_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
    def set_random_pen_color(self):
        """Change la couleur du stylo de manière aléatoire"""
        color = random.choice(self.pen_colors)
        
        # Mapping des couleurs vers les valeurs RGB
        color_map = {
            'red': (255, 0, 0),
            'green': (0, 255, 0),
            'blue': (0, 0, 255),
            'yellow': (255, 255, 0),
            'purple': (255, 0, 255),
            'orange': (255, 165, 0),
            'pink': (255, 192, 203)
        }
        
        r, g, b = color_map.get(color, (255, 255, 255))
        
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = 3
        request.off = 0  # Stylo activé
        
        future = self.set_pen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        self.get_logger().info(f'🎨 Couleur du stylo changée: {color}')
    
    def clear_screen(self):
        """Efface l'écran de turtlesim"""
        if self.clear_client.wait_for_service(timeout_sec=1.0):
            request = Empty.Request()
            future = self.clear_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('🧹 Écran effacé')
    
    def start_writing(self):
        """Démarre l'écriture de 'MASTER SIME'"""
        self.get_logger().info('✍️ Début de l\'écriture: MASTER SIME')
        
        # Calcul de la position de départ pour centrer le texte
        start_x = 2.0
        start_y = 6.0
        
        # Téléportation au point de départ
        self.teleport(start_x, start_y, 0.0)
        
        # Écriture lettre par lettre
        text = "MASTER SIME"
        letter_spacing = 1.2
        
        for i, letter in enumerate(text):
            if letter == ' ':
                # Espace - juste avancer
                current_x, current_y, _ = self.get_current_pose()
                self.teleport(current_x + letter_spacing, current_y, 0.0)
                continue
                
            # Changer la couleur pour chaque lettre
            self.set_random_pen_color()
            
            # Dessiner la lettre
            self.draw_letter(letter, start_x + i * letter_spacing, start_y)
            
            # Publier la position après chaque lettre
            self.publish_pose(f"writing_{letter}")
            
            # Pause entre les lettres
            time.sleep(0.5)
        
        self.get_logger().info('✅ Écriture terminée: MASTER SIME')
        self.publish_pose("mission_complete")
    
    def draw_letter(self, letter, x, y):
        """Dessine une lettre à la position spécifiée"""
        self.get_logger().info(f'📝 Dessin de la lettre: {letter}')
        
        # Patterns de dessin pour chaque lettre (simplifiés)
        letter_patterns = {
            'M': [(x, y), (x, y+2), (x+0.7, y+1), (x+1.4, y+2), (x+1.4, y)],
            'A': [(x, y), (x+0.7, y+2), (x+1.4, y), (x+0.2, y+1), (x+1.2, y+1)],
            'S': [(x+1, y), (x, y+0.5), (x+1, y+1), (x, y+1.5), (x+1, y+2)],
            'T': [(x, y+2), (x+1.4, y+2), (x+0.7, y+2), (x+0.7, y)],
            'E': [(x, y), (x, y+2), (x+1, y+2), (x, y+1), (x+1, y+1), (x, y), (x+1, y)],
            'R': [(x, y), (x, y+2), (x+1, y+2), (x+1.4, y+1), (x+1, y), (x+1.4, y)],
            'I': [(x+0.7, y), (x+0.7, y+2)],
            'S': [(x+1, y), (x, y+0.5), (x+1, y+1), (x, y+1.5), (x+1, y+2)],  # Deuxième S
            'E': [(x, y), (x, y+2), (x+1, y+2), (x, y+1), (x+1, y+1), (x, y), (x+1, y)],  # Deuxième E
        }
        
        pattern = letter_patterns.get(letter, [])
        
        if pattern:
            # Se téléporter au premier point
            self.teleport(pattern[0][0], pattern[0][1], 0.0)
            
            # Dessiner la lettre point par point
            for point in pattern[1:]:
                self.teleport(point[0], point[1], 0.0)
                time.sleep(0.1)  # Pause pour visualisation
    
    def get_current_pose(self):
        """Retourne la position actuelle (simulée)"""
        # Dans une vraie implémentation, on souscrirait à /turtle1/pose
        # Pour la simulation, on garde une trace interne
        return self.initial_x, self.initial_y, 0.0
    
    def publish_pose(self, status="writing"):
        """Publie la position actuelle sur /r2d2_pose"""
        x, y, theta = self.get_current_pose()
        
        msg = RobotPose()
        msg.name = "r2d2"
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta)
        msg.status = status
        
        self.pose_publisher.publish(msg)
        self.get_logger().info(f'📡 Position publiée: ({x:.1f}, {y:.1f}) - {status}', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        r2d2_master = R2D2Master()
        rclpy.spin(r2d2_master)
    except KeyboardInterrupt:
        pass
    finally:
        r2d2_master.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
