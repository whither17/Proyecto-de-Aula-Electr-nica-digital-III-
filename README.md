# Descripción detallada del proyecto final 
*Black Sisyphus*

## 1. Descripción detallada del sistema:
Para el proyecto final se desarrollará un vehículo autónomo, que navegara en una región de 60 cm x 60 cm, dentro de esta región se encontraran dos tipos de cajas unas de color negro y otras de color blanco, el trabajo del robot será llevar las cajas negras al centro de la región, e ignorar las cajas blancas. Una vez se lleve una caja negra al centro el vehículo se quedará en espera hasta que este bloque sea retirado en ese momento ira en búsqueda del siguiente bloque negro en caso de que lo haya si ya no hay mas bloques negros el vehículo finalizara su trabajo.
La posición inicial del vehículo ser en el centro de la región la cual le servirá como referencia al robot para su navegación.
 
El nombre del proyecto ha sido cambiado de “Sisifo.IO robot clasificador” a: “Black Sisyphus”, el cual es un nombre más corto, y menos genérico.
## 2. Requisitos funcionales:

-	Navegación automática por odometria: el vehículo deberá estimar su posición y orientación en todo momento de su ejecución.
-	 El vehículo deberá de moverse en la región de 60 cm x 60 cm de manera sistemática para encontrar todas las cajas en la región.
-	El vehículo deberá de detectar la presencia y ubicación de las cajas por medio de un sensor ultrasónico.
-	Al aproximarse a una caja detectada el robot deberá determinar su color por medio de un sensor de línea infrarrojo.
-	El vehículo deberá de empujar las cajas detectadas como negras hasta el centro de la región de trabajo
-	Las cajas blancas deberán de ser ignoradas por el robot
-	El sistema deberá de almacenar internamente las posiciones estimadas de todas las cajas detectadas durante su barrido de la región.
-	Una vez se haya llevado una caja al centro de la región el vehículo deberá detenerse y permanecer en espera hasta que esta se retire.
-	El vehículo deberá de ser capaz de detectar cuando la caja del centro ha sido retirada.
-	Cuando no haya más cajas negras el robot debe de volver al centro y terminar su operación.

## 3. Requisitos NO funcionales:

-	El vehículo tendrá tracción 4 x 4 y el cambio de dirección se realizará cambiando la velocidad y sentido cada rueda.
-	El vehículo será ligero, no superará el kilogramo de peso y será fácil de transportar.
-	El vehículo estará bien ensamblado, evitando que se rompa al transportarlo o manipularlo.
-	Las baterías han de garantizar el funcionamiento durante todas las pruebas.

## 4. Esenario de pruebas
En un espacio plano se encerrará un espacio de 60 cm x 60 cm con el uso de cinta negra este cuadro corresponderá al área de trabajo del robot, en el centro del cuadrado con la misma cinta se marcará una cruz que coincidirá con el punto de inicio del vehículo.
Se utilizarán cajas cubicas de dimensiones de aproximadamente 4 cm x 4 cm x 4 cm, la mitad de estas cajas serán negro y la otra mitad serán blancas, asegurando el mayor contrate posible para el sensor.
Al inicio de las pruebas se colocará al robot en el centro del cuadrado, las cajas serán distribuidas aleatoriamente dentro de la arena.

-	Prueba 1: Se colocará una sola caja negra en la arena se verificará que el robot la encuentre y la clasifique como negra, para después llevarla al centro.
-	Prueba 2: Una ves la caja negra se encuentre en el centro el vehículo deberá entrar en estado de espera y deberá detectar cuando se retire la caja.
-	Prueba 3: Se colocará una sola caja blanca se verificará que el robot la encuentre la clasifique como blanca y siga su escaneo de la región
-	Prueba 4: Se colocarán 2 cajas negras y 2 blancas en la región de trabajo garantizando que ninguna caja impida el acceso al centro desde las otras cajas inicialmente se verificara que el vehículo lleve las dos cajas negras al centro ignorando las cajas blancas.
- Prueba 5: Cuando no haya más cajas negras en el campo de trabajo se verificará que el sistema termine su funcionamiento.



