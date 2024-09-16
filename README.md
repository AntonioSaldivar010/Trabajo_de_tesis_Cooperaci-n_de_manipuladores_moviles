# Trabajo de tesis de maestría: Cooperacion de manipuladores móviles con medición de fuerza usando una galga extensiométrica en el plano sagital

En este trabajo se desarrolló una estrategia para realizar sujeción y transporte de objetos con poca rigidez con dos manipuladores móviles en el plano sagital. Un solo manipulador es incapaz de transportar objetos grandes con un efector final genérico, es por ello que se requiere la cooperación entre manipuladores móviles a nivel de posición y con retroalimentación de fuerza; esto permitió que la sujeción fuera simétrica y se evitó comprometer la integridad de los robots y del objeto. Para esto, se diseñaron controladores basados en estimación de modelo, una estrategia de cooperación y un control de fuerza que usa una galga extensiométrica. La estimación del modelo cinemático (i.e., la matriz Jacobiana), se realizó usando las señales de entrada (velocidades articulares) y de salida (velocidades del efector final) de cada manipulador móvil y un sistema de captura de movimiento, véase la Figura 1. Para asegurar que la manipulación y el transporte del objeto sea segura y efectiva, se diseñó una estrategia de control con tres componentes: un control de posición, un control de coordinación y un control de fuerza, todos ellos en los efectores finales de los manipuladores móviles. El control de posición es adaptable y esta basado en la estimación del modelo; el control de coordinación usa los errores de posición del efector final de cada manipulador; el control de fuerza usa las mediciones de la galga extensiométrica y esta basado en un modelo dinámico estable y de primer orden. La medición de fuerza se hizo con la integración de la galga en el efector final de un manipulador y se calibró obteniendo la relación que generó la fuerza de un peso conocido a la deformación de la placa de aluminio donde se situó la galga. Para asegurar la convergencia de los controladores diseñados y la sintonización de sus ganancias se desarrollaron pruebas de estabilidad utilizando el método de Lyapunov. Finalmente, para validar la efectividad de la estrategia propuesta se realizaron 3 experimentos con dos manipuladores móviles de 8 gdl: (1) regulación de posición y fuerza con un manipulador móvil, (2) levantamiento de un objeto con dos manipuladores móviles coordinados y regulando posición y fuerza, y (3) realizar control de posición, coordinación de dos robots y regulación de fuerza durante el transporte del objeto con regulación a puntos. 

<p align="center">
  <img src="https://github.com/user-attachments/assets/7590e36c-02c0-4370-aff9-5c64180f7c62" alt="Figura 1: Esquema de modelo basado en datos con el sistema robótico Kuka youbot.">
  <br>Figura 1: Esquema de modelo basado en datos con el sistema robótico Kuka youbot.
</p>

La presente tesis aborda el problema de la manipulación y transporte de objetos en robótica móvil, centrándose en la implementación de un algoritmo de estimación de modelo cinemático con medición de fuerza unidimensional. Las principales contribuciones de esta investigación se pueden resumir en los siguientes puntos:

- Diseño de un control que integra regulación de posición, coordinación y fuerza, permitiendo la interacción de los robots con su entorno, del conocimiento del autor esto no se ha reportado en el estado del arte.
- Se realizó el tratamiento de señales y el diseño de un dispositivo dónde se montó la galga.
- Implementación y validación en plataforma experimental mediante una serie de experimentos que demuestran su efectividad en la manipulación de objetos.
- Análisis teórico y práctico de estabilidad y desempeño de los controladores propuestos, incluyendo pruebas físicas que respaldan los resultados obtenidos.
- La sintonización de las ganancias de los lazos de posición, coordinación y fuerza para asegurar la convergencia de los controladores basadas en el análisis de estabilidad.

## Plataforma experimental
<p align="center">
  <img src="https://github.com/user-attachments/assets/66fbc8c1-4d9d-40ad-9be9-6fafdb650349" alt="Figura 2: Estructura de la comunicación en la plataforma experimental con el sistema de cámaras Opti-track, los robots manipuladores móviles Kuka Youbot y la galga extensiométrica.">
  <br>Figura 2: Estructura de la comunicación en la plataforma experimental con el sistema de cámaras Opti-track, los robots manipuladores móviles Kuka Youbot y la galga extensiométrica.
</p>

Para realizar los experimentos, se utilizó el sistema Opti-track para la captura de movimiento con su propia computadora de escritorio, una computadora de escritorio para establecer la comunicación entre el Opti-track y los manipuladores móviles, el modelo de estos es Kuka Youbot, y el lenguaje de programación que utilizan sus programas es C++, véase la Figura 2 donde se ilustra la estructura de comunicación de la plataforma experimental.

## Sistema de captura de movimiento

<p align="center">
  <img src="https://github.com/user-attachments/assets/6e2a9a2e-e680-4d97-a3a2-dbe0e56367c9" alt="Figura 3: Software Opti-track.">
  <br>Figura 3: Software Opti-track.
</p>

Este utiliza el software Motive, el cual se empareja con las 12 cámaras Opti-track que se encargan de seguir y capturar el movimiento de marcadores retroreflectivos adheridos a personas y objetos, esto para recuperar la posición y orientación del centroide de cuerpos rígidos en línea, véase la Figura 3.

<p align="center">
  <img src="https://github.com/user-attachments/assets/4d65e090-c3f6-4e73-adec-e9ade368cf90" alt="Figura 4: Computadora encargada de enviar la posición y orientación de los objetos con markers.">
  <br>Figura 4: Computadora encargada de enviar la posición y orientación de los objetos con markers.
</p>

Este software se encuentra en la primera computadora de escritorio (ver Figura 4) que se encarga exclusivamente de realizar el seguimiento de los marcadores. Una vez que obtiene los datos de cada objeto, los publica iterativamente en la red con ROS bajo diferentes tópicos. En caso de que los marcadores lleguen a ser ocultados por otro cuerpo, se detiene la publicación de este grupo de marcadores.

## Robots manipuladores móviles

<p align="center">
  <img src="https://github.com/user-attachments/assets/57ab4cd7-1084-45fd-a71f-1c15c68a7280" alt="Figura 5: Kuka youbot.">
  <br>Figura 5: Kuka youbot.
</p>

El manipulador móvil Kuka youbot esta conformado por un brazo robótico, siendo este una cadena serial con 5 gdl de revoluta, situado sobre una base no inercial, esto es, una plataforma móvil omnidireccional, con 3 gdl, siendo 2 prismáticas y 1 de revoluta (véase la Figura 5). El acceso a los controladores de cada actuador se puede establecer via Ethernet y EtherCAT. Procesador: Intel AtomTM Dual Core D510 (1M Cache, 2 x 1.66 Ghz). Memoria RAM: 2 GB single-channel DDR2 667 MHz. Gráficos: Embedded Gen 3.5 + GFX Core, frecuencia de renderizado de 400 MHz, arriba de 224 MB de memoria compartida. Disco duro: 32 GB SSD drive. El sistema operativo que utiliza es Ubuntu 12.04 LTS.

El programa de cada robot se dedica solamente al envío de la configuración articular dada con la lectura de los encoders y a la recepción e impresión de las velocidades articulares generadas por el control de la Computadora central.

En este trabajo se nombrará a cada Kuka youbot como K1 y K2 para diferenciarlos, siendo el K1 el que tiene la tarea adicional de leer los datos que recibe en el puerto Serial de su CPU y convertir esa información en la fuerza medida por el sensor de fuerza en su efector final, en este caso, la galga extensiométrica. 

## Sensor de fuerza

<p align="center">
  <img src="https://github.com/user-attachments/assets/ca73bdeb-4757-4b19-905f-6bc314b544d4" alt="Figura 6: Estructura de la galga extensiométrica.">
  <br>Figura 6: Estructura de la galga extensiométrica.
</p>

Este sensor consta de una galga extensiométrica, una barra de aluminio y un convertidor Analógico-Digital (ADC). La galga extensiométrica es una lámina que permite medir la deformación de un objeto. Su principio de funcionamiento se basa en el efecto piezorresistivo, siendo esta una propiedad que poseen ciertos materiales de cambiar el valor nominal de su resistencia eléctrica cuando se les somete a ciertos esfuerzos mecánicos, causando así la deformación de la celda de carga, la cual es una barra de aluminio en este caso. También se hace uso de un convertidor ADC (módulo HX711), diseñado para amplificar señales de carga y convertirlas en valores digitales. 

Se diseño una base y un punto de contacto para integrar la galga extensiométrica al efector final del manipulador móvil como su herramienta para el punto de contacto de este manipulador móvil. Estas piezas fueron generadas con una impresora 3D en el laboratorio de Robótica móvil y agricultura del Cinvestav Saltillo, véase la Figura 6.

## Computadora central

<p align="center">
  <img src="https://github.com/user-attachments/assets/e6c23f95-cc59-45f6-839a-176f7fe3c98e" alt="Figura 7: Computadora encargada de comunicarse con los manipuladores móviles..">
  <br>Figura 7: Computadora encargada de comunicarse con los manipuladores móviles..
</p>

Se trata de una segunda computadora de escritorio, véase la Figura 7. Esta se encarga de suscribirse a los tópicos en los que se están publicando las posiciones y orientaciones de cada objeto y, a su vez, se encarga de iniciar y mantener la comunicación con los dos manipuladores móviles.

El programa de la Computadora central se encarga primero de asegurar la recepción de la información de los objetos de los tópicos a los que se suscribe la computadora y establece la comunicación con los robots. Este programa hace uso de multihilos para la ejecución simultánea de varios hilos dentro de un mismo programa. Cada hilo proporciona un flujo de ejecución distinto, lo que permite que diferentes elementos de un programa operen de forma paralela, mejorando así la capacidad de respuesta. Así, se crean dos hilos, uno para cada robot, y en cada hilo se realiza una transformación de los marcos referenciales del efector final y a los objetos a la base de su respectivo robot. Con esto, se procede a realizar los cálculos de estimación de la matriz Jacobiana y el control, el cual entrega velocidades articulares, las cuales son enviadas a su manipulador móvil correspondiente. 

## Comunicación por sockets

Se trata de una interfaz de programación que permite a las aplicaciones intercambiar datos entre dispositivos a tráves de una red utilizando diversos protocolos de comunicación. 

Un socket es un punto final de una comunicación bidireccional entre dos programas que se ejecutan dentro de la misma red, es decir, es una puerta de enlace que permite a una aplicación enviar y recibir datos a tráves de la red. Para establecer la comunicación entre equipos, se necesita identificar una dirección IP como el servidor (la computadora central) y otra como el cliente (los manipuladores móviles). Una vez establecida dicha conexión, el servidor y los clientes pueden intercambiar datos en tiempo real.

<p align="center">
  <img src="https://github.com/user-attachments/assets/2e1bb876-1a0e-4fcd-a221-3a486d46f480" alt="Figura 8: Diagrama de procesos de hilos dentro de la computadora central.">
  <br>Figura 8: Diagrama de procesos de hilos dentro de la computadora central.
</p>

Así es como se intercambia información dentro de cada hilo para un robot. Esta técnica permite ejecutar múltiples procesos dentro de un gran proceso, siendo útil para realizar cálculos en paralelo, mejorar la eficiencia y la capacidad de respuesta de las aplicaciones. En cuanto a la señal de fuerza de la galga, esta es enviada por sockets desde el robot K1 a la computadora central y esta la almacena en una variable que es declarada de manera global, esto para dicha variable pueda ser llamada en los dos hilos ejecutándose al mismo tiempo permitiendo así que se tenga retroalimentación de la fuerza en el control también en el robot K2, véase la Figura 8.

## Experimento N° 1

El objetivo de este experimento es hacer un primer acercamiento al agregar el lazo y la retroalimentación de fuerza al controlador para cumplir dos de los objetivos propuestos: Que el efector final alcance una posición deseada y el control de fuerza para realizar tarea de regulación de posición y fuerza. Para ello, se preparó el siguiente escenario: el manipulador móvil comenzará la ejecución con su configuración inicial a una distancia de mas de 1 metro de la esponja (la cual es el objeto de referencia) que del otro lado se encuentra contra una superficie rígida. Lo que se busca es que el control haga regulación de posición en el plano $YZ$ para alinearse al objeto y en el eje $X$ de fuerza para sostener el objeto en conjunto con la superficie rígida.

<p align="center">
  <img src="https://github.com/user-attachments/assets/920e7c73-3d24-4af3-b148-d7f61164ea92" alt="Figura 9: Captura del experimento N° 1.">
  <br>Figura 9: Captura del experimento N° 1.
</p>

En la Figura \ref{cap4_fig2} se ilustra el experimento que consiste en lo siguiente: Se da la referencia de un objeto con marcadores y se utiliza en el control en el plano $YZ$ para la retroalimentación de posición, esto para alinear el efector final con respecto al objeto. Mientras que en el eje $X$, se utiliza la retroalimentación de fuerza para hacer regulación de fuerza. Para ver el video, entre al siguiente enlace: https://youtu.be/6GFn45EOGDU.

## Experimento N° 2

<p align="center">
  <img src="https://github.com/user-attachments/assets/086a3b3c-b3da-4189-9b00-a89ceb78df87" alt="Figura 10: Captura del experimento N° 2: Elevación del objeto">
  <br>Figura 10: Captura del experimento N° 2: Elevación del objeto.
</p>

En este experimento se realizan las tareas de regulación de posición y fuerza como en el experimento anterior. Ahora, lo que nos interesa conseguir en este experimento es realizar esto ya no contra una superficie rígida, sino contra el efector final con un objeto de por medio de un segundo manipulador móvil realizando, ambos manipuladores móviles, coordinación basada en los errores de acoplamiento para levantar y sostener el objeto, así como se ilustra en la Figura 10. Para ver el video, entre al siguiente enlace: https://youtu.be/tVfLltZBB80.

## Experimento N° 3

<p align="center">
  <img src="https://github.com/user-attachments/assets/aec6d7d2-b248-4589-ac13-29ee7c7be6ef" alt="Figura 11: Captura del experimento N° 3: Inicio de la ejecución del experimento 3 con ambos Kuka Youbot en la configuración inicial.">
  <br>Figura 11: Captura del experimento N° 3: Inicio de la ejecución del experimento 3 con ambos Kuka Youbot en la configuración inicial.
</p>
<p align="center">
  <img src="https://github.com/user-attachments/assets/33dbf2bc-fbd1-4341-9d47-02030d79af02" alt="Figura 12: Captura del experimento N° 3: Etapa de levantamiento de la esponja al punto de elevación mas alto en la ejecución del experimento.">
  <br>Figura 12: Captura del experimento N° 3: Etapa de levantamiento de la esponja al punto de elevación mas alto en la ejecución del experimento.
</p>
<p align="center">
  <img src="https://github.com/user-attachments/assets/39e5f4c1-3465-4e60-a109-c62ca5aa01fe" alt="Figura 13: Captura del experimento N° 3: Transporte de la esponja desde la base inicial a la caja de cartón abierta.">
  <br>Figura 13: Captura del experimento N° 3: Transporte de la esponja desde la base inicial a la caja de cartón abierta.
</p>
<p align="center">
  <img src="https://github.com/user-attachments/assets/41e2987d-cb71-4327-9b79-7b8e621a0d36" alt="Figura 14: Captura del experimento N° 3: Depósito de la esponja en la caja.">
  <br>Figura 14: Captura del experimento N° 3: Depósito de la esponja en la caja..
</p>

Una vez que se pasa desde la configuración inicial (Figura 11) a sostener el objeto (Figura 12), se traslada a otro punto en el espacio de trabajo (Figura 13). Ya estando por encima de una caja de cartón que sirve de contenedor, se deposita la esponja, ver Figura 14, finalizando así la tarea de traslado. Para ver el video, entre al siguiente enlace: https://youtu.be/jZ-ZS3CLKuc

El códgio utilizado en la computadora central para el experimento N° 1 es: Coordination_code_with_force_FREN_1_kuka.cpp. Y para los experimentos N° 2 y N° 3 es: Coordination_code_w_force_exp_2_y_3_w_FREN.cpp. El código para el robot K1 es: codigo_kuka_coord_serial.cpp, mientras que el código de K2 es: codigo_kuka_coord.cpp.
