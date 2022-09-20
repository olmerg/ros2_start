Este repositorio es una serie de tutoriales sobre ros2.

## instalación del docker

0. Copiar esta carpeta en un lugar conocido
 ```
	git clone https://github.com/olmerg/ros2_start.git
	cd ros2_start
	git submodule update --init
 ```
1. [recomendado]descargar la imagen del docker (solo la primera vez)

	docker pull osrf/ros:foxy-desktop

2. (linux)dar permisos para que pueda tener interface grafica el docker

	xhost +si:localuser:root

para **Windows** cambiar en [docker-compose](docker-compose.yml) la variable *DISPLAY=$DISPLAY* por *DISPLAY=host.docker.internal:0.0* (linea 14 por linea 15) . Adicionalmente se debe instalar [XServer for Windows](https://sourceforge.net/projects/vcxsrv/). Ayuda [video](https://www.youtube.com/watch?v=BDilFZ9C9mw)

para **MAC es necesario instalar  XQuartz y cambiar el DISPLAY igual que en windows. Ayuda [x-apps on mac](http://mamykin.com/posts/running-x-apps-on-mac-with-docker/). La ultima prueba en mac funcion la interface grafica pero no sirvio la comunicación entre nodos, en teoria deberia ser comentar la linea *network_mode: "host"*. Ayuda [ros answer](https://answers.ros.org/question/374042/nodes-cant-talk-to-host-when-running-ros2-in-docker-on-macos/) [tutorial](https://docs.ros.org/en/foxy/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)

3. ejecutar el docker

	docker compose up --build	

4. en nuevas consolas abrir lineas de comando (o realizarlo por visual studio code)

	docker exec -it utadeo-development /bin/bash

5. Probar los conceptos de ROS con [tutorial turtlesim](Introduction/Introduction.md)

6. [tutorial programación en python](src/py_basic_examples/README.md). Se recomienda crear un paquete con un nombre diferente que py_basic_examples	y realizar los 3 nodos descritos en el tutorial.

7. Ejecutar el proyecto de [lesson_urdf](https://github.com/olmerg/lesson_urdf)
	colcon build --symlink-install
	. install/setup.bash
	ros2 launch lesson_urdf view_robot_launch.py

8. Realiza tu propio urdf en un nuevo paquete.

9. Realiza sugerencias o mejoras en este repositorio creando un issue o un pull request


Algunos comandos sobre docker
		
- Para ejecutar un solo comando
	docker exec -it utadeo-development /bin/bash -c "ifconfig"

	docker exec -it utadeo-development /bin/bash -c "source /opt/ros/foxy/setup.bash;ros2 topic list"

-	para ejecutar sin el docker compose (Linux)

	docker run -it --net=host  --env="QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -e DISPLAY=$DISPLAY -v $HOME/.Xauthority:/home/.Xauthority -v ~/utadeo_ws:/home/ws osrf/ros:foxy-desktop

Realizado por olmerg para Utadeo.
