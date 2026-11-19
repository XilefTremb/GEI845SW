# Setup Simulation
Voici les étapes à suivre pour éxécuter la simulation à l'aide de WSL.

## Setup wsl
Installer ubuntu 24.04 sous WSL en suivant les étape du [guide officiel](https://learn.microsoft.com/en-us/windows/wsl/install)

ubuntu 24.04 est la version par défaut au moment de la rédaction de ce guide donc la commande ```wsl --intall``` intalle cette version.

## Installation des dépendances
1. Ouvrir un invite de commande windows
2. lancer une instance linux avec la commande ```wsl```
3. Installer ROS jazzy avec les commandes suivantes
```sh
# Locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable universe
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

# Add the ROS apt source (current official method)
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control


# Source it automatically
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
4. Installer Gazebo Harmonics avec la commande suivante
```sh
sudo apt install -y ros-jazzy-ros-gz
```
5. tester l'installation
    1. éxécutez la commande avec le terminal présentement ouvert : ```ros2 run demo_nodes_cpp talker```
    2. avec un autre terminal wsl :  ```ros2 run demo_nodes_py listener```
    3. avec un autre terminal wsl :  ```gz sim shapes.sdf```
    4. fermez les trois terminaux

## Optimisation du renderer
Si la simulation est lente et que vous avez une carte graphique vous pouvez faire l'accélération matérielle en faisant ces commandes dans une invite de commande wsl :
```sh
echo 'export GALLIUM_DRIVER=d3d12' >> ~/.bashrc
source ~/.bashrc
```
Il faudra ensuite reset le wsl en faisant les commandes 
```sh
wsl --shutdown
wsl
```

## Setup du workspace
[Simonjb] puisque j'ai inclu le src avec les modifications au git vous pouvez directement aller au #2

1. unzip le dossier "gei845_ws" dans le dossier "simulation" du projet
2. ouvrir une invite de commande windows et cd dans le dossier "gei845_ws"
    - petit truc: vous pouvez écrire "cmd" dans le chemin de fichier de l'explorateur de fichier pour ouvrir une invite de commande à cet emplacement directement
3. activer wsl avec la commande ```wsl```
    - vous devez être dans le dossier de pojet "gei845_ws" à ce point
4. installer les dépendance de projet et build les modules ROS avec ces commandes: 
```sh
#installe les dépendances de projet
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y

#build les modules
colcon build --symlink-install

#rend les modules visibles
source setup.bash
```

## Lancement automatique de la simulation
Pour lancer la simulation au démarage de gazebo, il faut simplement modifier une ligne du fichier launch pour y ajouter un argument -r.

dans le fichier ```gei845_ws\src\darwin_gazebo\launch\darwin_gazebo.launch.xml```

changer la ligne 14 pour celle-ci:
```xml
      <arg name="gz_args" value="-r empty.sdf"/>
```

## Simulation de la marche
1. Copier le fichier de commandes de joints généré dans le dossier "gei845_ws" sous le nom "dynamic_test.txt"
1. Il faut avoir 2 terminaux wsl ouvert dans le dossier de projet "gei845_ws"
2. Dans le premier terminal, démarez le controlleur de marche : 
```ros2 run darwin_control dynamic_walk```
2. Dans le second terminal, démarez la simulation : 
```ros2 launch darwin_gazebo darwin_gazebo.launch.xml```