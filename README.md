# Curso Robots Móviles 2023-2 FI, UNAM

Material para el curso de Robots Móviles de la Facultad de Ingeniería, UNAM, Semestre 2023-2

## Requerimientos

* Ubuntu 20.04
* ROS Noetic http://wiki.ros.org/noetic/Installation/Ubuntu

## Instalación

Nota: se asume que ya se tiene instalado Ubuntu y ROS.

* $ cd
* $ git clone https://github.com/mnegretev/Mobile-Robots-2023-2
* $ cd Mobile-Robots-2023-2
* $ ./Setup.sh
* $ cd catkin_ws
* $ catkin_make -j2 -l2

## Pruebas

Para probar que todo se instaló y compiló correctamente:

* $ cd 
* $ source Mobile-Robots-2023-2/catkin_ws/devel/setup.bash
* $ roslaunch bring_up path_planning.launch

Si todo se instaló y compiló correctamente, se debería ver un visualizador como el siguiente:

<img src="https://github.com/mnegretev/Mobile-Robots-2023-2/blob/master/Media/rviz.png" alt="RViz" width="763"/>

Un ambiente simulado como el siguiente:

<img src="https://github.com/mnegretev/Mobile-Robots-2023-2/blob/master/Media/gazebo.png" alt="Gazebo" width="798"/>

Y una GUI como la siguiente:

<img src="https://github.com/mnegretev/Mobile-Robots-2023-2/blob/master/Media/gui.png" alt="GUI" width="340"/>

## Máquina virtual

Se puede descargar una máquina virtual para [VirtualBox](https://www.virtualbox.org/wiki/Downloads) con Ubuntu y ROs ya instalado de [esta dirección.](https://drive.google.com/drive/folders/1DYhmegVFEz7VA69uncpYsL8Ck0HbaIEz?usp=sharing) <br>
Sólo es necesario  descomprimir el archivo y seguir las instrucciones del video que está en esa misma carpeta. La máquina virtual ya tiene todo instalado por lo que se puede pasar directo a la sección de pruebas.<br> 
Se recomienda configurar la máquina virtual con 4 CPUs y 4GB de RAM.<br>
Usuario: student <br>
Contraseña: pumas


## Contacto
Dr. Marco Negrete<br>
Profesor Asociado C<br>
Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
marco.negrete@ingenieria.unam.edu<br>
