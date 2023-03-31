# Simulation dans Blender

Pour tester la simulation du véhicule, il faut dabord modifier les varibles `front_axe` (X_AXE, Y_AXE) et `direction` (1, -1) dans le fichier `simulation/main.py` selon l'emplacement initial du véhicule. L'objet du véhicule doit être nommé `car`. Pour le contournement d'obstacles, il doit y avoir une collection nommée `obstacles` contenant un ou plusieurs obstacles.

Les ranges de détection du véhicule se trouvent dans le fichier `simulation/constants.py`. Pour démarrer la simulation, il suffit de partir le fichier `simulation/main.py` dans Blender.