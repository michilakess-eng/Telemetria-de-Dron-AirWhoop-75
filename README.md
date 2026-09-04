# Telemetria-de-Dron-AirWhoop-75
Código para servicio becario: "DroneOps".

"""
interfaz de control para el drin BetaFPV AirWhoop 75 a través del protocolo CRSF
(Crossfire) usando la Opción B: el chip FTDI USB-to-UART integrado dentro
del módulo BetaFPV ELRS Micro TX.

Gabriela Michelle Lagos Aguilar A01648192
código hecho por mi persona, inspirado en un repositorio de github (link más abajo) y con apoyo, especialmente en debugging y detalles estéticos, de Claude AI

Recap y resumen de la opción B
    El módulo BetaFPV ELRS Micro TX tiene un chip FTDI (que es USB-to-UART) en su
interior. Normalmente ese chip se usa para actualizar el firmware del
    módulo por USB-C. Con una reconfiguración de pines en el firmware ELRS
    ese mismo puerto USB-C puede redirigirse para actuar como el puerto CRSF.
    O sea, la laptop puede enviar tramas CRSF directamente sin necesidad de ningún adaptador externo.

Cadena de comunicación visualizada:
    código python
         ↓  USB-C (FTDI interno del módulo)
    BetaFPV ELRS Micro TX ;FTDI interno redirigido a CRSF
         ↓  radio 2.4 GHz ExpressLRS
    AirWhoop 75  FC G473 5IN1 AIO
         ↓  DSHOT / PWM
    Motores brushless 0802SE 23000KV

Prerequisito de hardware:
    primero, el módulo ELRS debe tener sus pines CRSF TX/RX apuntando al UART interno del USB. 
    Solo se hace una vez:
        1. Se conecta el módulo por USB-C a la PC
        2. abrir en el navegador: http://<IP_del_modulo>/hardware.html
        3. se cambia "CRSF RX pin" y "CRSF TX pin" a los valores del USB interno
           (para BetaFPV Micro 1W: pin 1 y pin 3 así)
        4. apagar la función "backpack" (comparte los mismos pines)
        5. guardar y reiniciar
    Después de esto el módulo debería aparacer como puerto serial cirutal en laptop.

Para este código es necesario:
    pip install pyserial
    
GitHub repo que me ayudó muchísimo:
https://github.com/kaack/elrs-joystick-control    
