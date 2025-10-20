#!/bin/bash

echo "🚀 DÉMARRAGE ÉTAPE 4 - Implémentation de C-3PO Follower"
echo "📋 Objectif: Développer le nœud suiveur intelligent"

cd ~/mission_sime_ws/src/mission_sime/mission_sime

# 1. Création du nœud C-3PO Follower
echo "🤖 Création du nœud C-3PO Follower..."

cat > c3po_follower.py << 'EOF'
#!/usr/bin/env python3
"""
Nœud C-3PO Follower
- Suit R2-D2 à distance configurable
- Reproduit les mouvements exacts
- Élimine les ennemis proches
- Interrompt le suivi pour menaces
"""

import rclpy
from rclpy.node import Node
import math
import time
from typing import List, Optional

from mission_sime.msg import RobotPose, EnemyStatus
from mission_sime.srv import RemoveRobot
from turtlesim.srv import Spawn, TeleportAbsolute

class C3POFollower(Node):
    def __init__(self):
        super().__init__('c3po_follower')
        
        # Déclaration des paramètres
        self.declare_parameter('follow_distance', 2.0)
        self.declare_parameter('elimination_range', 1.5)
        self.declare_parameter('spawn_x', 8.0)
        self.declare_parameter('spawn_y', 8.0)
        
        # Récupération des paramètres
        self.follow_distance = self.get_parameter('follow_distance').value
        self.elimination_range = self.get_parameter('elimination_range').value
        self.spawn_x = self.get_parameter('spawn_x').value
        self.spawn_y = self.get_parameter('spawn_y').value
        
        self.get_logger().info('🤖 C-3PO Follower initialisé')
        self.get_logger().info(f'Distance de suivi: {self.follow_distance}')
        self.get_logger().info(f'Portée élimination: {self.elimination_range}')
        
        # État interne
        self.current_mode = "following"  # "following" or "fighting"
        self.r2d2_pose: Optional[RobotPose] = None
        self.enemies: List[EnemyStatus] = []
        self.c3po_pose = RobotPose()
        self.c3po_pose.name = "c3po"
        self.c3po_pose.x = self.spawn_x
        self.c3po_pose.y = self.spawn_y
        self.c3po_pose.theta = 0.0
        self.c3po_pose.status = "initializing"
        
        # Subscribers
        self.r2d2_subscriber = self.create_subscription(
            RobotPose,
            '/r2d2_pose',
            self.r2d2_pose_callback,
            10
        )
        
        self.enemy_subscriber = self.create_subscription(
            EnemyStatus,
            '/enemy_status',
            self.enemy_status_callback,
            10
        )
        
        # Publishers
        self.pose_publisher = self.create_publisher(RobotPose, '/c3po_pose', 10)
        
        # Clients de service
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle2/teleport_absolute')
        self.remove_robot_client = self.create_client(RemoveRobot, '/remove_robot')
        self.spawn_client = self.create_client(Spawn, '/spawn')
        
        # Attendre que les services soient disponibles
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ En attente du service /turtle2/teleport_absolute...')
        
        # Timer pour le comportement principal
        self.create_timer(0.1, self.main_behavior_loop)  # 10 Hz
        
        # Spawn de C-3PO
        self.spawn_c3po()
        
    def spawn_c3po(self):
        """Faire apparaître C-3PO sur l'écran"""
        self.get_logger().info('🎯 Apparition de C-3PO...')
        
        if self.spawn_client.wait_for_service(timeout_sec=5.0):
            request = Spawn.Request()
            request.x = self.spawn_x
            request.y = self.spawn_y
            request.theta = 0.0
            request.name = 'turtle2'
            
            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                self.get_logger().info('✅ C-3PO apparu avec succès')
            else:
                self.get_logger().error('❌ Échec de l\'apparition de C-3PO')
        else:
            self.get_logger().error('❌ Service /spawn non disponible')
    
    def r2d2_pose_callback(self, msg: RobotPose):
        """Callback pour la position de R2-D2"""
        self.r2d2_pose = msg
        self.get_logger().debug(f'📡 Position R2-D2 reçue: ({msg.x:.1f}, {msg.y:.1f})', 
                               throttle_duration_sec=2.0)
    
    def enemy_status_callback(self, msg: EnemyStatus):
        """Callback pour le statut des ennemis"""
        # Mettre à jour ou ajouter l'ennemi
        existing_enemy = None
        for i, enemy in enumerate(self.enemies):
            if enemy.name == msg.name:
                existing_enemy = i
                break
        
        if existing_enemy is not None:
            if msg.state == "eliminated":
                # Supprimer l'ennemi éliminé
                self.enemies.pop(existing_enemy)
                self.get_logger().info(f'🎯 Ennemi éliminé: {msg.name}')
            else:
                # Mettre à jour l'ennemi existant
                self.enemies[existing_enemy] = msg
        else:
            if msg.state != "eliminated":
                # Ajouter un nouvel ennemi
                self.enemies.append(msg)
                self.get_logger().info(f'⚠️ Nouvel ennemi détecté: {msg.name} à ({msg.x:.1f}, {msg.y:.1f})')
    
    def get_nearest_enemy(self) -> Optional[EnemyStatus]:
        """Retourne l'ennemi le plus proche de C-3PO"""
        if not self.enemies:
            return None
        
        nearest_enemy = None
        min_distance = float('inf')
        
        for enemy in self.enemies:
            if enemy.state == "alive":
                distance = self.calculate_distance(
                    self.c3po_pose.x, self.c3po_pose.y, 
                    enemy.x, enemy.y
                )
                if distance < min_distance:
                    min_distance = distance
                    nearest_enemy = enemy
        
        return nearest_enemy if min_distance <= self.elimination_range else None
    
    def calculate_distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calcule la distance entre deux points"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def calculate_follow_pose(self) -> Optional[RobotPose]:
        """Calcule la position de suivi basée sur R2-D2"""
        if self.r2d2_pose is None:
            return None
        
        # Calculer la direction vers R2-D2
        dx = self.r2d2_pose.x - self.c3po_pose.x
        dy = self.r2d2_pose.y - self.c3po_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance == 0:
            return None
        
        # Normaliser et appliquer la distance de suivi
        if distance > self.follow_distance:
            # Se rapprocher
            ratio = (distance - self.follow_distance) / distance
            target_x = self.c3po_pose.x + dx * ratio
            target_y = self.c3po_pose.y + dy * ratio
        elif distance < self.follow_distance - 0.1:
            # S'éloigner un peu
            ratio = (self.follow_distance - distance) / distance
            target_x = self.c3po_pose.x - dx * ratio
            target_y = self.c3po_pose.y - dy * ratio
        else:
            # Distance parfaite, rester sur place
            return None
        
        follow_pose = RobotPose()
        follow_pose.name = "c3po"
        follow_pose.x = target_x
        follow_pose.y = target_y
        follow_pose.theta = math.atan2(dy, dx)  # Orientation vers R2-D2
        follow_pose.status = "following"
        
        return follow_pose
    
    def eliminate_enemy(self, enemy: EnemyStatus):
        """Élimine un ennemi en appelant le service"""
        self.get_logger().info(f'⚔️ Tentative d\'élimination: {enemy.name}')
        
        if self.remove_robot_client.wait_for_service(timeout_sec=1.0):
            request = RemoveRobot.Request()
            request.name = enemy.name
            
            future = self.remove_robot_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f'✅ Ennemi éliminé: {enemy.name}')
                    self.current_mode = "following"
                else:
                    self.get_logger().warning(f'⚠️ Échec élimination: {future.result().message}')
            else:
                self.get_logger().error('❌ Erreur d\'appel au service remove_robot')
        else:
            self.get_logger().error('❌ Service /remove_robot non disponible')
    
    def move_to_pose(self, pose: RobotPose):
        """Déplace C-3PO vers une position spécifique"""
        if self.teleport_client.wait_for_service(timeout_sec=1.0):
            request = TeleportAbsolute.Request()
            request.x = pose.x
            request.y = pose.y
            request.theta = pose.theta
            
            future = self.teleport_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            # Mettre à jour la position interne
            self.c3po_pose.x = pose.x
            self.c3po_pose.y = pose.y
            self.c3po_pose.theta = pose.theta
            self.c3po_pose.status = pose.status
    
    def main_behavior_loop(self):
        """Boucle principale de comportement de C-3PO"""
        # Vérifier s'il y a des ennemis proches
        nearest_enemy = self.get_nearest_enemy()
        
        if nearest_enemy and self.current_mode != "fighting":
            self.get_logger().info(f'🚨 Mode combat activé! Ennemi: {nearest_enemy.name}')
            self.current_mode = "fighting"
        
        if self.current_mode == "fighting" and nearest_enemy:
            # Mode combat - éliminer l'ennemi
            self.c3po_pose.status = "fighting"
            self.eliminate_enemy(nearest_enemy)
            
        elif self.r2d2_pose and self.current_mode == "following":
            # Mode suivi normal
            follow_pose = self.calculate_follow_pose()
            if follow_pose:
                self.move_to_pose(follow_pose)
                self.c3po_pose.status = "following"
            else:
                self.c3po_pose.status = "waiting"
        
        # Publier la position actuelle
        self.publish_pose()
    
    def publish_pose(self):
        """Publie la position actuelle de C-3PO"""
        self.pose_publisher.publish(self.c3po_pose)
        self.get_logger().debug(f'📡 C-3PO position: ({self.c3po_pose.x:.1f}, {self.c3po_pose.y:.1f}) - {self.c3po_pose.status}',
                               throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        c3po_follower = C3POFollower()
        rclpy.spin(c3po_follower)
    except KeyboardInterrupt:
        pass
    finally:
        c3po_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# 2. Rendre le script exécutable
chmod +x c3po_follower.py

echo ""
echo "🎉 ÉTAPE 4 TERMINÉE AVEC SUCCÈS !"
echo ""
echo "📊 RÉCAPITULATIF ÉTAPE 4 :"
echo "   ✅ Nœud C-3PO Follower créé : c3po_follower.py"
echo "   ✅ Souscription à /r2d2_pose pour suivre R2-D2"
echo "   ✅ Calcul de position de suivi avec distance configurable"
echo "   ✅ Détection et élimination d'ennemis via /enemy_status"
echo "   ✅ Gestion des modes : suivi vs combat"
echo "   ✅ Service /remove_robot pour élimination"
echo "   ✅ Publication sur /c3po_pose (RobotPose)"
echo ""
echo "🚀 PROCHAINES ACTIONS :"
echo "   1. Build du package"
echo "   2. Test avec R2-D2"
echo "   3. Commit et push sur GitHub"
echo ""
echo "💡 CONSEIL : Testez avec 'ros2 run mission_sime c3po_follower'"
