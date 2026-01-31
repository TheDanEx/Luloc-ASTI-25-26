Esta carpeta contiene el algoritmo main del sigue lineas adaptado a las posiblidades de nuestro robot, y sus algoritmos conceptuales.

¿Como Funciona?
El robot contiene 8 sensores(4 pares de 2). 2 pares serán frontales ubicados casi en los extremos de la parte delantera(uno en la izquierda y otro en la derecha), y un par de sensores en cada lado. Habrán distintos casos posibles:
	- En el caso de que los dos sensores frontales detecten la linea, seguimos 	adelante.
	- En el caso de que uno de los sensores no ubique la linea correctamente, 	el robot gira levemente hacia el lado donde se encuentra la linea(si se 	sale por la derecha, gira levemente hacia la izquierda mientras sigue 	yendo de frente y viceversa).
	- En el caso de que ninguno de los 2 sensores frontales detecten la linea, 	haremos uso de los otros 2 pares de sensores, y el robot deberia girar 	inmediatamente hacia el lado en el que se encuentre la linea.


Estructura del codigo:

Definir variables y constantes para cada motor.
Definir variables y constantes para los sensores.
Setup()--> Se declaran todos los pines como salidas(pines asociados a los motores).
Loop()--> Lee y analiza el valor de los sensores y hace que el robot avance.
LecturaSensor()--> Lee y muestra el valor de los sensores.
RobotAvance()--> Pone los motores al máximo para que este se mueva hacia delante(motor izq y motor der independientes).
RobotDer()--> Configura los motores para realizar el giro a la derecha.
RobotIzq()--> Configura los motores para realizar el giro a la izquierda.
RobotParar()-→ Para los motores.


