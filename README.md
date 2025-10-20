# 🐢 Mission SIME - Système Multi-Robots ROS2

## 📋 Description
Système ROS2 simulant une tortue maîtresse (R2-D2) qui écrit "MASTER SIME", une tortue suiveuse (C-3PO) qui la suit et reproduit ses mouvements, et des robots ennemis apparaissant aléatoirement.

## 🏗 Architecture

### Nœuds Principaux
- **r2d2_master**: Robot maître qui écrit et publie sa position
- **c3po_follower**: Robot suiveur qui suit R2-D2 et élimine les ennemis  
- **enemy_spawner**: Génère des robots ennemis aléatoirement
- **mission_supervisor**: Supervise l'état de la mission

### Communications
- **Topics**: /r2d2_pose, /enemy_status, /mission_status
- **Services**: /spawn_robot, /remove_robot
- **Messages personnalisés**: RobotPose.msg, EnemyStatus.msg

## 🚀 Installation
cd ~/mission_sime_ws
colcon build
source install/setup.bash

## 📁 Structure
mission_sime_ws/src/mission_sime/
├── launch/mission.launch.py
├── msg/RobotPose.msg, EnemyStatus.msg
├── srv/SpawnRobot.srv, RemoveRobot.srv
├── scripts/nœuds_python/
├── package.xml
└── CMakeLists.txt

## 🔄 Plan de Développement
- ✅ ÉTAPE 1: Configuration environnement et GitHub
- 🔲 ÉTAPE 2: Création du package et messages
- 🔲 ÉTAPE 3: Implémentation R2-D2 (maître)
- 🔲 ÉTAPE 4: Implémentation C-3PO (suiveur)
- 🔲 ÉTAPE 5: Système ennemis et supervision
- 🔲 ÉTAPE 6: Fichiers de lancement et tests
