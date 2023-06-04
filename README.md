# acuarino
Controlador de acuario basado en arduino

La finalidad de este proyecto es realizar un controlador de acuario general, de forma modular, basado en hardware arduino.

Funciones a la fecha
Informacion en patalla LCD 16x2 (via I2C)

Sensores:

Temperatura
Ph
Flujo de agua
Hora

Capacidad de manejar articulos electricos hasta 240 Volts

Moonlight funcionando de acuardo al ciclo lunar

Registro datos en intervalos programables en micro SD

Activa luces segun horario

Activa calefactor al bajar cierto grado de temperatura

Indicadores graficos de estado del sistema

Por Hacer
Usar pantalla 20x4

Codigo para uso de 2A Motor Shield para control de bomba peristaltica

Modulo para control del sitema via mando infrarojo

Webserver con los datos y control de funciones via ethernet

Ajuste y alarma para limpieza de filtro segun estimacion por flujo

Funcion de enfriamiento de acuario via chiller o ventilador

Funciones a largo plazo
Agregar funciones para reef (generador de olas, etc.)

Agregar menu (Posible limitacion por tamaño codigo > 32 KB)

El hardware utilizado a la fecha:
Arduino Uno (Atmega328P-PU)
http://www.planetaduino.com/site/wp-content/uploads/2010/10/arduino_uno.jpg

Arduino Sensor Shield v 4.0
http://www.emartee.com/Images/websites/emartee.com/arduino_sensor_shield_v4_1.png

Pantalla LCD 1602 16x2 con modulo LCD I2C
http://www.robotshop.com/Images/xbig/en/dfrobot-i2c-twi-lcd1602-module-B.jpg

http://www.robotshop.com/Images/xbig/en/dfrobot-i2c-twi-lcd1602-module-2-B.jpg

Datalogger shield (microSD + Reloj tiempo real)
http://www.olimex.cl/images/MCI-TDD-00884.jpg

BNC Shield
http://solarbotics.com/assets/images/52149/52149-img_4818_pl.JPG

2A Motor Shield
http://www.dfrobot.com/image/cache/data/DRI0009/DRI0009-500x500.jpg

Relays
http://www.dfrobot.com/image/cache/data/DFR0017/dfr0017-500x500.jpg
