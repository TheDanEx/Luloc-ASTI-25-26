En este algoritmo aprovecharemos los sensores ultrasonido y los sensores del siguelinea para que el robot no se salga del ring.

Estructura del codigo:
Definición pines para motores.
Definición pines sensores.
Setup()--> Calibración e inicialización del robot.
Loop()--> Analiza el estado de los sensores. Hay 3 casos:
	Si detecta linea, retrocede y gira.
	Si detecta rival, ataca.
	Si no detecta nada, gira para buscar.
Atacar()--> Activa los motores al maximo hacia delante.
Buscar()--> Gira sobre su eje hasta encontrar al rival.
Retroceder()--> Activa los motores para dar marcha atrás.
Girar()--> Hace girar el robot.
