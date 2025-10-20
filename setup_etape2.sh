#!/bin/bash

echo "🚀 DÉMARRAGE ÉTAPE 2 - Création du package et messages personnalisés"
echo "📋 Objectif: Créer le package ROS2 mission_sime avec interfaces de communication"

cd ~/mission_sime_ws/src

# 1. Création du package ROS2
echo "📦 Création du package ROS2 mission_sime..."
ros2 pkg create mission_sime \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs turtlesim std_msgs std_srvs \
  --description "Système multi-robots ROS2 pour le projet Master SIME" \
  --license "MIT"

cd mission_sime

# 2. Création de la structure des dossiers
echo "📁 Création de la structure des dossiers..."
mkdir -p launch msg srv scripts/config

# 3. Création des messages personnalisés
echo "📨 Création des messages personnalisés..."

# RobotPose.msg
cat > msg/RobotPose.msg << 'EOF'
# Message personnalisé pour la position des robots
string name
float32 x
float32 y
float32 theta
string status
EOF

# EnemyStatus.msg
cat > msg/EnemyStatus.msg << 'EOF'
# Message personnalisé pour le statut des ennemis
string name
float32 x
float32 y
string state  # alive, eliminated, spawning
EOF

# 4. Création des services personnalisés
echo "🔧 Création des services personnalisés..."

# SpawnRobot.srv
cat > srv/SpawnRobot.srv << 'EOF'
# Service pour faire apparaître un robot
string name
float32 x
float32 y
---
bool success
string message
EOF

# RemoveRobot.srv
cat > srv/RemoveRobot.srv << 'EOF'
# Service pour éliminer un robot
string name
---
bool success
string message
EOF

# 5. Mise à jour du package.xml
echo "📄 Mise à jour du package.xml..."
cat > package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypelayout="1.0"?>
<package format="3">
  <name>mission_sime</name>
  <version>0.0.0</version>
  <description>Système multi-robots ROS2 pour le projet Master SIME</description>
  <maintainer email="sabimou@ur.univ-rouen.fr">sabimou</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>turtlesim</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# 6. Mise à jour du setup.py pour inclure les messages
echo "⚙️ Configuration du setup.py..."
cat > setup.py << 'EOF'
from setuptools import setup

package_name = 'mission_sime'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mission.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sabimou',
    maintainer_email='sabimou@ur.univ-rouen.fr',
    description='Système multi-robots ROS2 pour le projet Master SIME',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2d2_master = mission_sime.r2d2_master:main',
            'c3po_follower = mission_sime.c3po_follower:main',
            'enemy_spawner = mission_sime.enemy_spawner:main',
            'mission_supervisor = mission_sime.mission_supervisor:main',
        ],
    },
)
EOF

# 7. Création du fichier de configuration des paramètres
echo "⚙️ Création des fichiers de configuration..."
cat > scripts/config/params.yaml << 'EOF'
mission_sime:
  ros__parameters:
    # Paramètres R2-D2
    r2d2_initial_x: 5.0
    r2d2_initial_y: 5.0
    r2d2_speed: 2.0
    pen_colors: ["red", "green", "blue", "yellow", "purple"]
    
    # Paramètres C-3PO
    follow_distance: 2.0
    elimination_range: 1.5
    
    # Paramètres ennemis
    enemy_spawn_rate: 5.0  # secondes
    max_enemies: 5
    
    # Paramètres mission
    mission_timeout: 300.0  # secondes
EOF

# 8. Création du module Python principal
echo "🐍 Création du module Python..."
mkdir -p mission_sime
cat > mission_sime/__init__.py << 'EOF'
# Package mission_sime - Système multi-robots ROS2
EOF

echo ""
echo "🎉 ÉTAPE 2 TERMINÉE AVEC SUCCÈS !"
echo ""
echo "📊 RÉCAPITULATIF ÉTAPE 2 :"
echo "   ✅ Package ROS2 mission_sime créé"
echo "   ✅ Messages personnalisés : RobotPose.msg, EnemyStatus.msg"
echo "   ✅ Services personnalisés : SpawnRobot.srv, RemoveRobot.srv"
echo "   ✅ Structure des dossiers : launch/, msg/, srv/, scripts/"
echo "   ✅ Configuration build : package.xml, setup.py mis à jour"
echo "   ✅ Fichier de paramètres : params.yaml créé"
echo ""
echo "🚀 PROCHAINES ACTIONS :"
echo "   1. Build du package"
echo "   2. Vérification des messages"
echo "   3. Commit et push sur GitHub"
echo ""
echo "💡 CONSEIL : Vérifiez la structure avec 'tree' ou 'find . -type f'"
