# Control Avanzado: Péndulo Invertido Rotatorio & Ball Balancing Robot (BBR)

Este repositorio contiene el desarrollo técnico, código embebido, modelos de control y documentación para la implementación de dos sistemas de control multivariable en tiempo real, utilizando el microcontrolador **STM32F407-DISC1**.

---

Descripción General del Proyecto

El objetivo principal de este proyecto es el diseño, simulación, modelado e implementación física de dos plataformas mecatrónicas de control para la estabilización y seguimiento de trayectoria en sistemas inestables y no lineales:

1. **Péndulo Invertido Rotatorio (Rotary Inverted Pendulum):**
   * Sistema dinámico de un grado de libertad no accionado en el eje vertical (mástil) y un grado accionado en el plano horizontal (brazo).
   * Implementa una estrategia de control basada en energía para la fase de balanceo (*swing-up*) y un controlador por Espacio de Estados / Regulador Cuadrático Lineal (**LQR**) para mantener la posición de equilibrio vertical inestable.
   * Cuenta con un encoder magnético absoluto **AS5600** para realimentación angular de alta precisión y un driver **TMC2209** para el control del motor.

2. **Ball Balancing Robot (BBR):**
   * Sistema de plataforma basculante de 2 grados de libertad ($X, Y$) accionado por servos/motores, diseñado para equilibrar y posicionar una bola sobre una superficie plana.
   * Utiliza la cámara inteligente **Pixy2 CMUcam5** montada en visión cenital para la detección de color y procesamiento de posición a 60 FPS, comunicándose por SPI/UART con el microcontrolador.
   * Implementa un lazo de control **PID doble desacoplado** junto con ecuaciones de **cinemática inversa** para transformar los errores de posición en coordenadas Cartesianas en ángulos de inclinación para los brazos mecánicos.

Ambos sistemas ejecutan sus bucles de control en tiempo real sobre la tarjeta de desarrollo **STM32F407-DISC1** (ARM Cortex-M4 @ 168 MHz).

---

Integrantes del Equipo

 Proyecto 1: Péndulo Invertido Rotatorio
* **Adán Alejandro Ramos Gudiño**
* **Brayan Sinue Mata García**

Proyecto 2: Ball Balancing Robot (BBR)
* **Claudio Fabián Salas Quijas**
* **Rubén Darío Casales Ruiz**

---

Tecnologías y Hardware Utilizado

* **Microcontrolador Central:** STM32F407-DISC1 (ARM Cortex-M4)
* **Sensores:** Sensor magnético angular AS5600 | Cámara de visión artificial Pixy2 CMUcam5
* **Actuadores & Drivers:** Driver paso a paso TMC2209 | Servomotores / Motores DC
* **Entornos de Desarrollo:** PlatformIO / STM32CubeIDE / MATLAB & Simulink
* **Diseño Mecánico:** SolidWorks / Fusion 360 (Archivos STEP & STL para impresión 3D)
