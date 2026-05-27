## Descripción detallada del proyecto final 
*Black Sisyphus*

Proyecto final del curso <br>
**Electrónica Digital III** 

Integrantes equipo Turing

**_Andres Felipe Agudelo Zapata_**<br>
**_Robinson Correa Morales_** 

---

## Descripción

El proyecto consiste en un robot autónomo capaz de:

- Navegar dentro de una región delimitada.
- Detectar cajas mediante sensores.
- Clasificar cajas negras y blancas.
- Expulsar únicamente las cajas negras del área de trabajo.
- Ignorar las cajas blancas.
- Generar un mapa virtual del entorno.
- Reexplorar el área periódicamente.

La arena de pruebas tiene dimensiones aproximadas de **100 cm x 70 cm** y está dividida en 4 cuadrantes mediante líneas negras de cinta aislante.

---

# Hardware utilizado

## Procesamiento
- Raspberry Pi Pico

## Sensores
- MPU-6050 (IMU)
- HC-SR04 (Ultrasónico)
- Sensores infrarrojos
- Encoders integrados en los motores N20

## Actuadores
- 2 Motores N20 con encoder
- Driver TB6612FNG

## Alimentación
- Batería Li-Po 7.4V
- Reductor MP1584

---

# Características

- Navegación autónoma
- Odometría y fusión sensorial
- Control PID
- Detección de límites
- Clasificación por color
- Mapeo periódico del entorno

---

# Arquitectura de software

El firmware utiliza una arquitectura basada en Polling + Interrupciones.

## Capas del sistema

### Drivers
Control e integración de hardware:
- TB6612FNG
- IMU
- Ultrasonido
- Encoders
- Sensores infrarrojos

### Estimación
- Odometría
- Fusión sensorial
- Estimación de posición
- Generación del mapa virtual

### Navegación y control
- Control PID
- Planeación de trayectorias
- Corrección de movimiento

### Aplicación
Implementa la lógica general del robot mediante una máquina de estados.

---

# Máquina de estados

| Estado | Función |
|---|---|
| APP_INIT | Inicialización |
| APP_EXPLORE | Exploración |
| APP_APPROACH_BOX | Aproximación a cajas |
| APP_IDENTIFY_BOX | Identificación del color |
| APP_PUSH_BOX | Expulsión de cajas negras |
| APP_CENTER | Retorno al centro |
| APP_WAIT | Espera para nuevo mapeo |

---

# Requisitos funcionales

- Navegación autónoma.
- Exploración sistemática.
- Detección de cajas.
- Clasificación por color.
- Expulsión de cajas negras.
- Ignorar cajas blancas.
- Retorno automático al centro.
- Reexploración periódica.

---

# Requisitos no funcionales

- Peso menor a 1 kg.
- Fácil transporte.
- Estructura resistente.
- Autonomía mínima de 15 minutos.
- Bajo consumo energético en espera.

---

# Pruebas

1. Detectar y expulsar una caja negra.
2. Detectar e ignorar una caja blanca.
3. Clasificar múltiples cajas negras y blancas.
4. Retornar al centro al finalizar.
5. Reiniciar el mapeo automáticamente.

---

# Estructura del proyecto

```text
├── Drivers
│   ├── Driver_TB6612FNG
│   ├── imu
│   ├── ultrasonic
│   ├── encoder
│   └── infrarrojo
│
├── Estimacion
│   └── estimacion.c/h
│
├── Navegacion
│   ├── control.c/h
│   └── navegacion.c/h
│
└── Aplicacion
    └── aplicacion.c/h



