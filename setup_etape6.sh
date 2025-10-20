#!/bin/bash

echo "🚀 DÉMARRAGE ÉTAPE 6 - Implémentation du Mission Supervisor"
echo "📋 Objectif: Créer le superviseur global du système"

cd ~/mission_sime_ws/src/mission_sime/mission_sime

# 1. Création du nœud Mission Supervisor
echo "👁️ Création du nœud Mission Supervisor..."

cat > mission_supervisor.py << 'EOF'
#!/usr/bin/env python3
"""
Nœud Mission Supervisor
- Surveille tous les composants du système
- Log les événements importants
- Détecte les anomalies
- Publie le statut global de la mission
"""

import rclpy
from rclpy.node import Node
import time
from typing import Dict, Optional
from datetime import datetime

from mission_sime.msg import RobotPose, EnemyStatus
from std_msgs.msg import String

class MissionSupervisor(Node):
    def __init__(self):
        super().__init__('mission_supervisor')
        
        # Déclaration des paramètres
        self.declare_parameter('mission_timeout', 300.0)  # 5 minutes
        self.declare_parameter('communication_timeout', 10.0)  # 10 secondes
        
        # Récupération des paramètres
        self.mission_timeout = self.get_parameter('mission_timeout').value
        self.communication_timeout = self.get_parameter('communication_timeout').value
        
        self.get_logger().info('👁️ Mission Supervisor initialisé')
        self.get_logger().info(f'Timeout mission: {self.mission_timeout}s')
        self.get_logger().info(f'Timeout communication: {self.communication_timeout}s')
        
        # État de la mission
        self.mission_start_time = time.time()
        self.mission_status = "initializing"
        self.last_events = []
        
        # États des composants
        self.components = {
            'r2d2': {'last_seen': None, 'status': 'unknown', 'pose': None},
            'c3po': {'last_seen': None, 'status': 'unknown', 'pose': None},
            'enemy_spawner': {'last_seen': None, 'status': 'unknown'}
        }
        
        self.enemies: Dict[str, EnemyStatus] = {}
        self.mission_stats = {
            'enemies_spawned': 0,
            'enemies_eliminated': 0,
            'letters_written': 0,
            'mode_changes': 0
        }
        
        # Subscribers pour surveiller tous les composants
        self.r2d2_subscriber = self.create_subscription(
            RobotPose,
            '/r2d2_pose',
            self.r2d2_callback,
            10
        )
        
        self.c3po_subscriber = self.create_subscription(
            RobotPose,
            '/c3po_pose',
            self.c3po_callback,
            10
        )
        
        self.enemy_subscriber = self.create_subscription(
            EnemyStatus,
            '/enemy_status',
            self.enemy_callback,
            10
        )
        
        # Publisher pour le statut de mission
        self.mission_publisher = self.create_publisher(String, '/mission_status', 10)
        
        # Timer pour la surveillance périodique
        self.create_timer(2.0, self.supervision_loop)  # Surveillance toutes les 2 secondes
        self.create_timer(5.0, self.publish_mission_status)  # Publication statut toutes les 5 secondes
        
        self.log_event("mission_started", "Mission SIME démarrée")
        self.mission_status = "running"
        
        self.get_logger().info('✅ Mission Supervisor opérationnel - Surveillance active')
    
    def r2d2_callback(self, msg: RobotPose):
        """Callback pour la position de R2-D2"""
        self.components['r2d2']['last_seen'] = time.time()
        self.components['r2d2']['status'] = msg.status
        self.components['r2d2']['pose'] = msg
        
        # Log des changements d'état importants de R2-D2
        if "writing" in msg.status and "writing" not in self.components['r2d2'].get('last_status', ''):
            self.log_event("writing_started", f"R2-D2 a commencé à écrire: {msg.status}")
            if "letter" in msg.status:
                self.mission_stats['letters_written'] += 1
        
        if "complete" in msg.status:
            self.log_event("writing_completed", "R2-D2 a terminé d'écrire MASTER SIME")
        
        self.components['r2d2']['last_status'] = msg.status
    
    def c3po_callback(self, msg: RobotPose):
        """Callback pour la position de C-3PO"""
        self.components['c3po']['last_seen'] = time.time()
        self.components['c3po']['status'] = msg.status
        self.components['c3po']['pose'] = msg
        
        # Log des changements de mode de C-3PO
        current_mode = msg.status
        last_mode = self.components['c3po'].get('last_mode', '')
        
        if current_mode != last_mode:
            if current_mode == "fighting":
                self.log_event("combat_started", "C-3PO est entré en mode combat")
                self.mission_stats['mode_changes'] += 1
            elif current_mode == "following" and last_mode == "fighting":
                self.log_event("combat_ended", "C-3PO est revenu en mode suivi")
                self.mission_stats['mode_changes'] += 1
        
        self.components['c3po']['last_mode'] = current_mode
    
    def enemy_callback(self, msg: EnemyStatus):
        """Callback pour le statut des ennemis"""
        self.components['enemy_spawner']['last_seen'] = time.time()
        self.components['enemy_spawner']['status'] = 'active'
        
        enemy_name = msg.name
        
        # Gérer les différents états d'ennemis
        if msg.state == "spawning":
            if enemy_name not in self.enemies:
                self.log_event("enemy_spawning", f"Apparition de {enemy_name} en cours")
        
        elif msg.state == "alive":
            if enemy_name not in self.enemies or self.enemies[enemy_name].state != "alive":
                self.mission_stats['enemies_spawned'] += 1
                self.log_event("enemy_spawned", f"{enemy_name} apparu à ({msg.x:.1f}, {msg.y:.1f})")
        
        elif msg.state == "eliminated":
            if enemy_name in self.enemies and self.enemies[enemy_name].state == "alive":
                self.mission_stats['enemies_eliminated'] += 1
                self.log_event("enemy_eliminated", f"{enemy_name} a été éliminé par C-3PO")
        
        # Mettre à jour l'ennemi
        self.enemies[enemy_name] = msg
    
    def log_event(self, event_type: str, message: str):
        """Log un événement important"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}"
        
        # Ajouter aux derniers événements (limité à 10)
        self.last_events.append(log_message)
        if len(self.last_events) > 10:
            self.last_events.pop(0)
        
        # Log avec niveau approprié
        if "error" in event_type or "failed" in event_type:
            self.get_logger().error(f'📛 {log_message}')
        elif "warning" in event_type:
            self.get_logger().warn(f'⚠️ {log_message}')
        elif "eliminated" in event_type or "combat" in event_type:
            self.get_logger().info(f'⚔️ {log_message}')
        else:
            self.get_logger().info(f'📝 {log_message}')
    
    def check_communication_health(self):
        """Vérifie la santé des communications avec les composants"""
        current_time = time.time()
        communication_issues = []
        
        for comp_name, comp_data in self.components.items():
            if comp_data['last_seen'] is not None:
                time_since_last_seen = current_time - comp_data['last_seen']
                
                if time_since_last_seen > self.communication_timeout:
                    communication_issues.append(comp_name)
                    if comp_data['status'] != 'timeout':
                        self.log_event("communication_timeout", 
                                     f"{comp_name.upper()} ne répond pas depuis {time_since_last_seen:.1f}s")
                        comp_data['status'] = 'timeout'
                else:
                    if comp_data['status'] == 'timeout':
                        self.log_event("communication_restored", 
                                     f"Communication avec {comp_name.upper()} rétablie")
                    comp_data['status'] = 'active'
        
        return communication_issues
    
    def check_mission_progress(self):
        """Vérifie la progression globale de la mission"""
        mission_duration = time.time() - self.mission_start_time
        
        # Vérifier le timeout de mission
        if mission_duration > self.mission_timeout:
            self.mission_status = "timeout"
            self.log_event("mission_timeout", f"Mission timeout après {mission_duration:.1f}s")
            return
        
        # Vérifier la complétion
        r2d2_status = self.components['r2d2'].get('status', '')
        if "complete" in r2d2_status and self.mission_status != "completed":
            self.mission_status = "completed"
            self.log_event("mission_completed", 
                         f"Mission accomplie en {mission_duration:.1f}s! "
                         f"Lettres: {self.mission_stats['letters_written']}, "
                         f"Ennemis: {self.mission_stats['enemies_eliminated']}/{self.mission_stats['enemies_spawned']}")
    
    def generate_mission_report(self) -> str:
        """Génère un rapport de mission"""
        mission_duration = time.time() - self.mission_start_time
        active_enemies = len([e for e in self.enemies.values() if e.state == "alive"])
        
        report_lines = [
            f"=== RAPPORT MISSION SIME ===",
            f"Statut: {self.mission_status.upper()}",
            f"Durée: {mission_duration:.1f}s",
            f"",
            f"📊 STATISTIQUES:",
            f"• Lettres écrites: {self.mission_stats['letters_written']}/11",
            f"• Ennemis spawnés: {self.mission_stats['enemies_spawned']}",
            f"• Ennemis éliminés: {self.mission_stats['enemies_eliminated']}",
            f"• Ennemis actifs: {active_enemies}",
            f"• Changements de mode: {self.mission_stats['mode_changes']}",
            f"",
            f"🔧 ÉTAT DES COMPOSANTS:"
        ]
        
        for comp_name, comp_data in self.components.items():
            status = comp_data.get('status', 'unknown')
            last_seen = comp_data.get('last_seen')
            
            if last_seen:
                time_since_seen = time.time() - last_seen
                status_line = f"• {comp_name.upper()}: {status} (vu il y a {time_since_seen:.1f}s)"
            else:
                status_line = f"• {comp_name.upper()}: {status} (jamais vu)"
            
            report_lines.append(status_line)
        
        report_lines.extend([
            f"",
            f"📅 DERNIERS ÉVÉNEMENTS:"
        ])
        
        for event in self.last_events[-5:]:  # 5 derniers événements
            report_lines.append(f"• {event}")
        
        return "\n".join(report_lines)
    
    def supervision_loop(self):
        """Boucle principale de surveillance"""
        # Vérifier la santé des communications
        communication_issues = self.check_communication_health()
        
        # Vérifier la progression de la mission
        self.check_mission_progress()
        
        # Log périodique du statut
        if len(communication_issues) > 0:
            self.get_logger().warn(f'⚠️ Problèmes de communication: {", ".join(communication_issues)}', 
                                 throttle_duration_sec=10.0)
        else:
            self.get_logger().info('✅ Toutes les communications sont saines', 
                                 throttle_duration_sec=15.0)
    
    def publish_mission_status(self):
        """Publie le statut de mission sur le topic"""
        report = self.generate_mission_report()
        
        msg = String()
        msg.data = report
        self.mission_publisher.publish(msg)
        
        self.get_logger().debug('📡 Rapport de mission publié', throttle_duration_sec=10.0)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        mission_supervisor = MissionSupervisor()
        rclpy.spin(mission_supervisor)
    except KeyboardInterrupt:
        # Générer un rapport final
        mission_supervisor.log_event("mission_interrupted", "Mission interrompue par l'utilisateur")
        mission_supervisor.get_logger().info(mission_supervisor.generate_mission_report())
    finally:
        mission_supervisor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# 2. Rendre le script exécutable
chmod +x mission_supervisor.py

echo ""
echo "�� ÉTAPE 6 TERMINÉE AVEC SUCCÈS !"
echo ""
echo "📊 RÉCAPITULATIF ÉTAPE 6 :"
echo "   ✅ Nœud Mission Supervisor créé : mission_supervisor.py"
echo "   ✅ Surveillance de tous les composants : R2-D2, C-3PO, Ennemis"
echo "   ✅ Détection de problèmes de communication"
echo "   ✅ Journalisation des événements importants"
echo "   ✅ Publication du statut mission sur /mission_status"
echo "   ✅ Génération de rapports détaillés"
echo "   ✅ Gestion des timeouts et anomalies"
echo ""
echo "🚀 PROCHAINES ACTIONS :"
echo "   1. Build du package"
echo "   2. Test du système complet"
echo "   3. Commit et push sur GitHub"
echo ""
echo "💡 CONSEIL : Testez avec 'ros2 run mission_sime mission_supervisor'"
EOF
