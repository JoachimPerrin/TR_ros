# TP d'initiation ROS2 
## Prise en main de l'interface en ligne de commande

["Introduction Ros2"](http://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)

- Essayez de piloter la turtle ligne de commande. $ros\ topic$
   - Notez les commandes qu'il vous faut pour déplacer la turtle
   - Essayez de décrire ce qu'est un topic avec vos mots.
- Explorons les services. 
  - Essayez d'activer/de désactiver la trace écrite. 
  - Expliquez comment vous comprenez les services

- Ensuite explorez les paramètres de la tortue et listez les paramètres possibles. $ros\ param$ 
  - Trouvez comment grâce aux paramètre vous arrivez à changer des couleurs de la simulation du turtle. 
- Expliquez les différences entre topic, param et services

## C++ interfacage avec turtlesim 
Maintenant nous allons explorer la **programmation en C++** de ros2. 
Nous allons commencer par discuter avec le turtlesim en : 
* topic [CPP Topic programming](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
* services [CPP service programming](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
* param [CPP Param programming](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)

  
Tout d'abord préparez un launch file afin de pouvoir lancer les codes de manière centralisée. [launch file in ros2](http://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- Nous pouvons essayer de refaire les actions de la section précédente mais en utilisant le langage CPP. 
  - Lancer en topic des consignes de mouvement récupérer la position courante du turtle
  - configurer un paramètre qui permet d'activer la trace du crayon (paramètre boolean dans votre noeud) qui lance un service setpen sur la turtle.
  - Changer le paramètre de couleur du background (Vert par défaut) si la turtle arrive sur les bords (Rouge)  

## $action$ alors !!
En vous inspirant des tutoriels d'action [creating an action](http://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html) 
* Réalisons un générateur de trajectoire. Par exemple on envoie quelques points de passages. On fait une interpolation linéaire et on renvoie la liste des points de passage en supposant un déplacement maximum fixe qui renvoie les points avec une pause de 1/10 seconde entre chaque position de passage. 

* Ensuite, nous piloterons la tortue à partir de cette trajectoire. Faisons un algorithme qui envoie en consigne la différence entre la position actuelle et la position courante désirée.  
* On peut modifier le serveur d'action qui n'envoie le point de passage suivant en feedback seulement quand une distance est assez petite (au lieu d'un temps fixe on verifie que la distance est assez petite)
