Este repositorio es una serie de tutoriales sobre ros2.

## instalación del docker

1. [recomendado]descargar la imagen del docker (solo la primera vez)

	docker pull osrf/ros:foxy-desktop

2. dar permisos para que pueda tener interface grafica el docker

	xhost +si:localuser:root

3. ejecutar el docker

	docker compose up --build	

4. en nuevas consolas abrir lineas de comando (o realizarlo por visual studio code)

	docker exec -it utadeo-development /bin/bash

5. [tutorial turtlesim](Introduction/Introduction.md)

6. [tutorial programación en python](src/py_basic_examples/README.md)	

7. Realiza sugerencias o mejoras en este repositorio creando un issue o un pull request


Algunos comandos sobre docker
		
- Para ejecutar un solo comando
	docker exec -it utadeo-development /bin/bash -c "ifconfig"

	docker exec -it utadeo-development /bin/bash -c "source /opt/ros/foxy/setup.bash;ros2 topic list"

-	para ejecutar sin el docker compose

	docker run -it --net=host  --env="QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -e DISPLAY=$DISPLAY -v $HOME/.Xauthority:/home/.Xauthority -v ~/utadeo_ws:/home/ws osrf/ros:foxy-desktop

Realizado por olmerg para Utadeo.
