# Gazebo
Segundo entregable para la asignatura de Simuladores de Robots del Máster de Robótica y Automatización (UC3M)

- Se ha implementado un algoritmo para que el robot sea capaz de moverse de una posición inicial hasta la meta.
- Las tareas realizadas se encuentran implementadas en el siguiente fichero [here](model_push_g9/model_push.cc)

# Extras
- Extra 1: El código se ha subido a un repositorio de [GitHub](https://github.com/lucas-rib-oli/P3_Gazebo).
- Extra 2: Se modifica el heading del robot para hacer más realista la simulación del robot Pioneer.
- Extra 3: Se ha utilizado el [script](gazebo-tools-master/greedy_path_planning.py) desarrollado en Planificación de Robots para implementar el algoritmo A<sup> * </sup>.
- Extra 4: Se ha utilizado el [script](gazebo-tools-master/greedy_path_planning.py) desarrollado en Planificación de Robots para implementar el algoritmo Breadth-first Search.
- Extra 5: Se ha utilizado el [script](gazebo-tools-master/greedy_path_planning.py) desarrollado en Planificación de Robots para implementar el algoritmo Depth-first search.
- Extra 6: Se ha utilizado el [script](gazebo-tools-master/greedy_path_planning.py) desarrollado en Planificación de Robots para implementar el algoritmo Best-first search.
- Extra 7: Se han creado imágenes sobre la trayectoria a seguir según los algoritmos de planificación ([imágenes](gazebo-tools-master/images)).
- Extra 8: Se ha añadido un sensor láser siguiendo el tutorial de gazebo ([link](http://gazebosim.org/tutorials/?tut=add_laser)).
- Extra 9: Se ha creado una lógica para llegar al punto de meta haciendo uso de la información del láser para evitar obstáculos. 
- Extra 10: Mediante una constante se puede escoger entre los diferentes métodos implementados.
- Extra 11: Cuando se utiliza el método para llegar al punto de destino usando el láser, el punto de destino está definido por unas variables constantes que se pueden modifcar.
- Extra 12: Se ha grabado un vídeo en [YouTube](https://youtu.be/IoiFscgxuR8) ejucutando el método Breadth-first Search.