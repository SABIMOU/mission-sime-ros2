#!/bin/bash

echo "🚀 DÉMARRAGE ÉTAPE 5 - Implémentation du système ennemi"
echo "📋 Objectif: Créer le générateur d'ennemis et mécaniques de combat"

cd ~/mission_sime_ws/src/mission_sime/mission_sime

# 1. Création du nœud Enemy Spawner
echo "⚔️ Création du nœud Enemy Spawner..."

cat > enemy_spawner.py << 'EOF'
#!/usr/bin/env python3
"""
Nœud Enemy Spawner
- Génère des ennemis aléatoirement selon une fréquence
- Gère le cycle de vie des ennemis
- Répond aux demandes d'élimination
- Publie les statuts d'ennemis
"""

import rclpy
from rclpy.node import Node
import random
import time
from typing import Dict, List
import math

from mission_sime.msg import EnemyStatus
from mission_sime.srv import SpawnRobot, RemoveRobot
from turtlesim.srv import Spawn, Kill

class EnemySpawner(Node):
    def __init__(self):
        super().__init__('enemy_spawner')
        
        # Déclaration des paramètres
        self.declare_parameter('spawn_rate', 5.0)  # secondes entre les spawns
        self.declare_parameter('max_enemies', 5)
        self.declare_parameter('spawn_area_min_x', 1.0)
        self.declare_parameter('spawn_area_max_x', 10.0)
        self.declare_parameter('spawn_area_min_y', 1.0)
        self.declare_parameter('spawn_area_max_y', 10.0)
        
        # Récupération des paramètres
        self.spawn_rate = self.get_parameter('spawn_rate').value
        self.max_enemies = self.get_parameter('max_enemies').value
        self.spawn_area_min_x = self.get_parameter('spawn_area_min_x').value
        self.spawn_area_max_x = self.get_parameter('spawn_area_max_x').value
        self.spawn_area_min_y = self.get_parameter('spawn_area_min_y').value
        self.spawn_area_max_y = self.get_parameter('spawn_area_max_y').value
        
        self.get_logger().info('⚔️ Enemy Spawner initialisé')
        self.get_logger().info(f'Fréquence de spawn: {self.spawn_rate}s')
        self.get_logger().info(f'Nombre max d\'ennemis: {self.max_enemies}')
        
        # État interne
        self.enemies: Dict[str, EnemyStatus] = {}  # name -> EnemyStatus
        self.enemy_counter = 0
        
        # Publisher
        self.enemy_publisher = self.create_publisher(EnemyStatus, '/enemy_status', 10)
        
        # Services (serveur)
        self.spawn_service = self.create_service(SpawnRobot, '/spawn_robot', self.spawn_robot_callback)
        self.remove_service = self.create_service(RemoveRobot, '/remove_robot', self.remove_robot_callback)
        
        # Clients de service turtlesim
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        
        # Attendre que les services soient disponibles
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ En attente du service /spawn...')
        
        # Timer pour le spawn automatique
        self.create_timer(self.spawn_rate, self.auto_spawn_enemy)
        
        # Timer pour la publication régulière des statuts
        self.create_timer(1.0, self.publish_enemy_statuses)  # 1 Hz
        
        self.get_logger().info('✅ Enemy Spawner prêt à générer des ennemis!')
    
    def generate_random_position(self) -> tuple:
        """Génère une position aléatoire dans la zone de spawn"""
        x = random.uniform(self.spawn_area_min_x, self.spawn_area_max_x)
        y = random.uniform(self.spawn_area_min_y, self.spawn_area_max_y)
        return x, y
    
    def generate_enemy_name(self) -> str:
        """Génère un nom unique pour l'ennemi"""
        self.enemy_counter += 1
        return f"enemy_{self.enemy_counter}"
    
    def spawn_turtlesim_robot(self, name: str, x: float, y: float) -> bool:
        """Fait apparaître un robot dans turtlesim"""
        try:
            if self.spawn_client.wait_for_service(timeout_sec=2.0):
                request = Spawn.Request()
                request.x = x
                request.y = y
                request.theta = random.uniform(0, 2 * math.pi)  # Orientation aléatoire
                request.name = name
                
                future = self.spawn_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                
                return future.result() is not None
            else:
                self.get_logger().warn('❌ Service /spawn non disponible')
                return False
        except Exception as e:
            self.get_logger().error(f'❌ Erreur lors du spawn: {e}')
            return False
    
    def auto_spawn_enemy(self):
        """Spawn automatique d'ennemis selon la fréquence configurée"""
        if len(self.enemies) >= self.max_enemies:
            self.get_logger().debug('📊 Nombre max d\'ennemis atteint', throttle_duration_sec=5.0)
            return
        
        # Générer un nouvel ennemi
        enemy_name = self.generate_enemy_name()
        x, y = self.generate_random_position()
        
        self.get_logger().info(f'🎯 Tentative de spawn: {enemy_name} à ({x:.1f}, {y:.1f})')
        
        # Créer le statut d'ennemi
        enemy_status = EnemyStatus()
        enemy_status.name = enemy_name
        enemy_status.x = x
        enemy_status.y = y
        enemy_status.state = "spawning"
        
        # Publier le statut "spawning"
        self.enemy_publisher.publish(enemy_status)
        
        # Spawn dans turtlesim
        if self.spawn_turtlesim_robot(enemy_name, x, y):
            enemy_status.state = "alive"
            self.enemies[enemy_name] = enemy_status
            self.get_logger().info(f'✅ Ennemi spawné: {enemy_name} à ({x:.1f}, {y:.1f})')
            
            # Publier le statut "alive"
            self.enemy_publisher.publish(enemy_status)
        else:
            self.get_logger().error(f'❌ Échec du spawn: {enemy_name}')
            enemy_status.state = "failed"
            self.enemy_publisher.publish(enemy_status)
    
    def remove_robot_callback(self, request: RemoveRobot.Request, response: RemoveRobot.Response):
        """Callback pour le service d'élimination d'ennemi"""
        enemy_name = request.name
        
        self.get_logger().info(f'🎯 Demande d\'élimination: {enemy_name}')
        
        if enemy_name in self.enemies:
            # Tuer la tortue dans turtlesim
            if self.kill_turtlesim_robot(enemy_name):
                # Mettre à jour le statut
                self.enemies[enemy_name].state = "eliminated"
                
                # Publier le statut "eliminated"
                self.enemy_publisher.publish(self.enemies[enemy_name])
                
                # Supprimer de la liste
                del self.enemies[enemy_name]
                
                response.success = True
                response.message = f"Ennemi {enemy_name} éliminé avec succès"
                self.get_logger().info(f'✅ {response.message}')
            else:
                response.success = False
                response.message = f"Échec de l'élimination de {enemy_name}"
                self.get_logger().error(f'❌ {response.message}')
        else:
            response.success = False
            response.message = f"Ennemi {enemy_name} non trouvé"
            self.get_logger().warn(f'⚠️ {response.message}')
        
        return response
    
    def kill_turtlesim_robot(self, name: str) -> bool:
        """Supprime un robot de turtlesim"""
        try:
            if self.kill_client.wait_for_service(timeout_sec=2.0):
                request = Kill.Request()
                request.name = name
                
                future = self.kill_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                
                return future.result() is not None
            else:
                self.get_logger().warn('❌ Service /kill non disponible')
                return False
        except Exception as e:
            self.get_logger().error(f'❌ Erreur lors de l\'élimination: {e}')
            return False
    
    def spawn_robot_callback(self, request: SpawnRobot.Request, response: SpawnRobot.Response):
        """Callback pour le service de spawn manuel"""
        enemy_name = request.name or self.generate_enemy_name()
        x = request.x if request.x != 0.0 else random.uniform(1.0, 10.0)
        y = request.y if request.y != 0.0 else random.uniform(1.0, 10.0)
        
        self.get_logger().info(f'🎯 Spawn manuel demandé: {enemy_name} à ({x:.1f}, {y:.1f})')
        
        # Vérifier si le nom existe déjà
        if enemy_name in self.enemies:
            response.success = False
            response.message = f"Le nom {enemy_name} est déjà utilisé"
            return response
        
        # Créer le statut d'ennemi
        enemy_status = EnemyStatus()
        enemy_status.name = enemy_name
        enemy_status.x = x
        enemy_status.y = y
        enemy_status.state = "spawning"
        
        # Publier le statut "spawning"
        self.enemy_publisher.publish(enemy_status)
        
        # Spawn dans turtlesim
        if self.spawn_turtlesim_robot(enemy_name, x, y):
            enemy_status.state = "alive"
            self.enemies[enemy_name] = enemy_status
            
            response.success = True
            response.message = f"Ennemi {enemy_name} spawné avec succès à ({x:.1f}, {y:.1f})"
            self.get_logger().info(f'✅ {response.message}')
            
            # Publier le statut "alive"
            self.enemy_publisher.publish(enemy_status)
        else:
            response.success = False
            response.message = f"Échec du spawn de {enemy_name}"
            self.get_logger().error(f'❌ {response.message}')
        
        return response
    
    def publish_enemy_statuses(self):
        """Publie régulièrement les statuts de tous les ennemis"""
        for enemy in self.enemies.values():
            self.enemy_publisher.publish(enemy)
        
        # Log du nombre d'ennemis actifs
        active_enemies = len([e for e in self.enemies.values() if e.state == "alive"])
        if active_enemies > 0:
            self.get_logger().debug(f'📊 Ennemis actifs: {active_enemies}/{self.max_enemies}', 
                                  throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        enemy_spawner = EnemySpawner()
        rclpy.spin(enemy_spawner)
    except KeyboardInterrupt:
        pass
    finally:
        enemy_spawner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# 2. Rendre le script exécutable
chmod +x enemy_spawner.py

echo ""
echo "🎉 ÉTAPE 5 TERMINÉE AVEC SUCCÈS !"
echo ""
echo "📊 RÉCAPITULATIF ÉTAPE 5 :"
echo "   ✅ Nœud Enemy Spawner créé : enemy_spawner.py"
echo "   ✅ Génération aléatoire d'ennemis selon fréquence"
echo "   ✅ Gestion du cycle de vie : spawning → alive → eliminated"
echo "   ✅ Service /remove_robot pour l'élimination"
echo "   ✅ Service /spawn_robot pour spawn manuel"
echo "   ✅ Publication sur /enemy_status (EnemyStatus)"
echo "   ✅ Intégration avec services turtlesim : /spawn, /kill"
echo ""
echo "🚀 PROCHAINES ACTIONS :"
echo "   1. Build du package"
echo "   2. Test du système complet"
echo "   3. Commit et push sur GitHub"
echo ""
echo "💡 CONSEIL : Testez avec 'ros2 run mission_sime enemy_spawner'"
EOF
