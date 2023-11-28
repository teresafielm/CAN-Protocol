# CAN-Protocol
Programa de protocolo CAN en ARM Cortex M4
El planteamiento de este proyecto es la de lograr una comunicación con el protocolo CAN utilizando dos
microcontroladores TM4C1294NCPDT, se debe hacer la lectura de datos de un microcontrolador al otro
utilizando variables leídas de un sensor, esto a grandes rasgos.
A detalle lo que se realizó fue lograr una comunicación y transmisión continua de dos sensores analógicos
que se traducen en dos potenciómetros, y que al llegar a un valor determinado (que puede variar
dependiendo del sensor) encienda un led específico para cada potenciómetro, estos leds se encuentran en
la tiva que recibe los datos, la que transmite es la que tiene los sensores.
