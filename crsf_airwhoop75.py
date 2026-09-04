import serial
import serial.tools.list_ports
import time
import sys

# CONSTANTES DEL PROTOCOLO CRSF
CRSF_SYNC_BYTE= 0xC8   # Inicio de trama)device address)
CRSF_FRAMETYPE_RC_CHANNELS_PACKED= 0x16  #16 canales RC empaquetados
CRSF_NUM_CHANNELS= 16     # num de canales en el payload

#Rango de valores de canal CRSF siendo de 11 bits:
#   172-> mínimo (equivale a ~1000 µs en PWM clásico)
#   992-> centro (equivale a ~1500 µs en PWM clásico)
#   1811-> máximo (equivale a ~2000 µs en PWM clásico)

CRSF_CH_MIN=172
CRSF_CH_CENTER=992
CRSF_CH_MAX=1811

#Betaflight y ELRS convención de canales AETR:
CH_ROLL=0   #Aileron::inclinación lateral
CH_PITCH=1   #Elevator::inclinación frontal o trasera
CH_THROTTLE=2   # Throttle::acelerador (va de MIN a MAX)
CH_YAW=3   # Rudder::rotación sobre eje vertical
CH_ARM=4   # AUX1::MIN = desarmado, MAX = armado
CH_FLTMODE=5   # AUX2::modo de vuelo (acro / angle / horizon)

#velocidad del enlace serial: requerida por CRSF, no negociable
#el módulo ELRS espera exactamente 400000 baudios, formato 8N1  *
CRSF_BAUDRATE = 400_000

#la frecuencia de transmision es mínimo 50 Hz para mantener el enlace vivo
#si no llegan tramas durante ~0.5s, el receptor activa el failsafe
CRSF_LOOP_HZ  = 50



#dETECCIÓN AUTOMÁTICA DEL PUERTO USB DEL MÓDULO ELRS
def detectar_puerto_elrs() -> str | None:
    """
    se tiene que buscar en los puertos seriales disponibles uno que corresponda al
    chip FTDI interno del módlo BetaFPV ELRS Micro TX

    Cuando esté conectado por USB-C y correctamente configurado,
    el sistema operativo lo registra como un puerto serial virtual con un
    descriptor de fabricante reconocible.

    da de return nombre del puerto si se encuentra, "None" si no se detecta ningún módulo compatible
    """
    fabricantes_conocidos = [
        "stmicroelectronics",   # STM32 Virtual COM Port (más común en ELRS)
        "ftdi",                 # Chip FTDI clásico
        "silicon labs",         # CP2102/CP2104 también usado en algunos módulos
        "betafpv",
    ]

    print("Buscando modulo ELRS conectado por USB...")
    puertos = serial.tools.list_ports.comports()

    if not puertos:
        print("No se encontro ningun puerto serial.")
        return None

    for puerto in puertos:
        fabricante = (puerto.manufacturer or "").lower()
        descripcion = (puerto.description or "").lower()
        print(f"  Puerto: {puerto.device:12s} | {puerto.description}")

        for fab in fabricantes_conocidos:
            if fab in fabricante or fab in descripcion:
                print(f"  -> modulo ELRS detectado en: {puerto.device}")
                return puerto.device

    print("No se pudo detectar un modulo ELRS.")
    print("Puertos disponibles:")
    for p in puertos:
        print(f" {p.device} — {p.description}")
    return None


def abrir_conexion_usb(puerto: str) -> serial.Serial:
    """
    Abre el puerto serial del FTDI interno con los parámetros requeridos por el 
    protocolo CRSF: 400000 baudios, 8 bits de datos, sin paridad, 1 bit de stop (el formato 8N1).

    arg:
        puerto: nombre del puerto

    return:
        Objeto serial.Serial listo para escribir tramas CRSF.

    si el puerto no existe o está ocupado.:
        serial.SerialException
    """
    conexion = serial.Serial(
        port      = puerto,
        baudrate  = CRSF_BAUDRATE,
        bytesize  = serial.EIGHTBITS,
        parity    = serial.PARITY_NONE,
        stopbits  = serial.STOPBITS_ONE,
        timeout   = 1
    )
    print(f"Conexion USB abierta: {puerto} @ {CRSF_BAUDRATE} baud (8N1)")
    return conexion




# PROTOCOLO CRSF, CONSTRUCCIÓN DE TRAMAS
def crc8_dvb_s2(data: bytes) -> int:
    """
    seguridad  *
    se calcula el checksum CRC-8 con el polinomio DVB-S2 (0xD5).

    el CRC garantiza la integridad del paquete: si un solo bit cambia durante la transmisión, 
    el receptor lo detecta y descarta la trama. El polinomio 0xD5 es el estándar definido
    en la especificación CRSF;usar cualquier otro valor hace que todos los paquetes sean rechazados.

    se calcula únicamente sobre [FRAME TYPE + PAYLOAD], no sobre SYNC ni LENGTH.

    arg:
        data: bytes sobre los que calcular el checksum.

    return:
        Byte de checksum (0–255).
    """
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


def empaquetar_canales(canales: list) -> bytes:
    """
    comprime 16 canales RC (11 bits cada uno) en exactamente 22 bytes.
    
    Son 11 bits pq un canal RC necesita 2048 posiciones distintas (172 a 1811 en CRSF), lo que es
    2^11 = 2048. Con 16 bits por canal desperdiciaría 5 bits por canal

    Aquí se construye un entero de 176 bits (16x11) concatenando todos los canales con 
    desplazamientos de bits y luego se serializa en Little Endian (byte menos significativo primero),
    que es lo que espera CRSF.

    arg:
        canales: lista de exactamente 16 enteros en [CRSF_CH_MIN, CRSF_CH_MAX].

    return:
        22 bytes con los canales empaquetados.
    """
    if len(canales) != CRSF_NUM_CHANNELS:
        raise ValueError(f"Se requieren {CRSF_NUM_CHANNELS} canales, recibidos {len(canales)}")

    bits = 0
    for i in range(CRSF_NUM_CHANNELS - 1, -1, -1):
        bits = (bits << 11) | (canales[i] & 0x7FF)
        # 0x7FF = máscara de 11 bits, previene desbordamiento si el valor está fuera del rango permitido.

    return bits.to_bytes(22, byteorder='little')


def construir_trama(canales: list) -> bytes:
    """
    Ensambla la trama CRSF completa de 26 bytes:
        Byte 0: SYNC (0xC8), marca el inicio de la trama
        Byte 1: LENGTH, cantidad de bytes que siguen (tipo + payload + crc = 24)
        Byte 2: FRAME TYPE (0x16), identifica el tipo de contenido
        Bytes 3–24: PAYLOAD, 22 bytes con los 16 canales empaquetados
        Byte 25: CRC-8, checksum calculado sobre [TYPE + PAYLOAD]

    arg:
        canales: lista de 16 enteros de canal.

    return:
        26 bytes listos para escribir en el puerto serial.
    """
    payload       = empaquetar_canales(canales)
    frame_length  = len(payload) + 2              # +2: tipo (1B) + CRC (1B)
    datos_para_crc = bytes([CRSF_FRAMETYPE_RC_CHANNELS_PACKED]) + payload
    crc           = crc8_dvb_s2(datos_para_crc)

    trama = bytes([CRSF_SYNC_BYTE, frame_length]) + datos_para_crc + bytes([crc])
    assert len(trama) == 26, f"Error interno: trama de {len(trama)} bytes, se esperaban 26"
    return trama




# FUNCIONES DE CONVERSIÓN DE ESCALA
def pct_a_crsf(pct: float) -> int:
    """
    Convierte -100% … +100% a CRSF_CH_MIN … CRSF_CH_MAX, centrado en 992
    usar para roll, pitch y yaw (ejes que tienen posición neutra en el centro)
    """
    pct = max(-100.0, min(100.0, pct))
    if pct >= 0:
        return int(CRSF_CH_CENTER+(pct/100.0)*(CRSF_CH_MAX-CRSF_CH_CENTER))
    else:
        return int(CRSF_CH_CENTER+(pct/100.0)*(CRSF_CH_CENTER-CRSF_CH_MIN))


def throttle_pct_a_crsf(pct: float) -> int:
    """
    Convierte 0% … 100% a CRSF_CH_MIN … CRSF_CH_MAX
    usar para el acelerador (no tiene posición neutra — 0% = apagado)
    """
    pct = max(0.0, min(100.0, pct))
    return int(CRSF_CH_MIN+(pct/100.0)*(CRSF_CH_MAX-CRSF_CH_MIN))



# API DE CONTROL DE VUELO **
def enviar_rc(conexion: serial.Serial,
              roll_pct:     float = 0.0,
              pitch_pct:    float = 0.0,
              throttle_pct: float = 0.0,
              yaw_pct:      float = 0.0,
              armado:       bool  = False,
              modo_angle:   bool  = False) -> None:
    """
    esto construye y envía una única trama CRSF al ELRS
    parámetros en porcentaje para facilitar el uso:
        roll_pct      : -100 (izquierda)  a +100 (derecha)
        pitch_pct     : -100 (atrás)      a +100 (adelante)
        throttle_pct  :    0 (mínimo)     a +100 (máximo)
        yaw_pct       : -100 (izq / CCW)  a +100 (der / CW)
        armado        : True para armar, False para desarmar
        modo_angle    : True = modo angle (estabilizado), False = acro

    Además se tiene llamar en bucle a ≥50 Hz para mantener el enlace.
    """
    canales=[CRSF_CH_CENTER]*CRSF_NUM_CHANNELS

    canales[CH_ROLL]     = pct_a_crsf(roll_pct)
    canales[CH_PITCH]    = pct_a_crsf(pitch_pct)
    canales[CH_THROTTLE] = throttle_pct_a_crsf(throttle_pct)
    canales[CH_YAW]      = pct_a_crsf(yaw_pct)
    canales[CH_ARM]      = CRSF_CH_MAX if armado    else CRSF_CH_MIN
    canales[CH_FLTMODE]  = CRSF_CH_MAX if modo_angle else CRSF_CH_MIN

    conexion.write(construir_trama(canales))


def bucle_control(conexion: serial.Serial,
                  duracion_s: float,
                  roll_pct:     float = 0.0,
                  pitch_pct:    float = 0.0,
                  throttle_pct: float = 0.0,
                  yaw_pct:      float = 0.0,
                  armado:       bool  = True,
                  modo_angle:   bool  = False) -> None:
    """
    Envía el mismo comando de control repetidamente durante 'duracion_s' segundos a 50 Hz, manteniendo el enlace activo.

    args:
        conexion: puerto serial abierto
        duracion_s: cuántos segundos mantener este estado
        (resto): parámetros de vuelo, igual que enviar_rc()
    """
    fin=time.time()+duracion_s
    while time.time()<fin:
        enviar_rc(conexion, roll_pct, pitch_pct, throttle_pct, yaw_pct,
                  armado, modo_angle)
        time.sleep(1/CRSF_LOOP_HZ)




# SECUENCIAS COMPUESTAS
def prearm(conexion: serial.Serial, duracion_s: float = 2.0) -> None:
    """
    Envía señal neutra con el dron desarmado durante 'duracion_s' segundos
    Necesario para que el FC y el receptor se inicialicen correctamente antes de intentar armar.
    """
    print(f"Pre-arm: senal neutra {duracion_s}s (dron desarmado)...")
    bucle_control(conexion, duracion_s,
                  throttle_pct=0.0, armado=False)


def armar(conexion: serial.Serial, duracion_s: float = 2.0) -> None:
    """
    arma el dron: throttle en min + canal ARM en HIGH
    Betaflight requiere ver esta combinación de forma sostenida para armar
    no armar con throttle alto(!) los motores arrancarían en mera potencia.
    """
    print(f"Armando dron ({duracion_s}s)...")
    bucle_control(conexion, duracion_s,
                  throttle_pct=0.0, armado=True)
    print("Dron armado.")


def desarmar(conexion: serial.Serial, duracion_s: float = 2.0) -> None:
    """
    desarma el dron: throttle en min + canal ARM en LOW.
    Siempre llamar antes de cerrar el programa(*)
    """
    print(f"Desarmando dron ({duracion_s}s)...")
    bucle_control(conexion, duracion_s,
                  throttle_pct=0.0, armado=False)
    print("Dron desarmado.")


def hover(conexion: serial.Serial,
          duracion_s: float,
          throttle_pct: float = 40.0) -> None:
    """
    mantiene el dron en vuelo estacionario durante 'duracion_s' segundos.

    El porcentaje de throttle para hover depende del peso y los motores, que en el caso del
    AirWhoop 75 son como 21g entonces el punto de hover debería estar entre 30–45%, pero verdaderamente 
    esto es de calibrar experimentando.
    """
    print(f"Hover {duracion_s}s @ throttle={throttle_pct}%")
    bucle_control(conexion, duracion_s,
                  throttle_pct=throttle_pct, armado=True)


def rampa_eje(conexion: serial.Serial,
              eje:str,
              pct_inicio:float,
              pct_fin:float,
              duracion_s:float,
              throttle_pct:float=40.0)->None:
    """
    Aplica un movimiento en rampa lineal sobre un eje durante 'duracion_s'
    esta interpolación lineal evita cambios bruscos que desestabilicen el FC.

    args:
        eje: 'roll', 'pitch' o 'yaw'
        pct_inicio: valor de inicio del eje en porcentaje
        pct_fin: valor de fin del eje en porcentaje
        duracion_s: duración total de la transición
        throttle_pct: throttle constante durante el movimiento
    """
    pasos = max(1, int(duracion_s * CRSF_LOOP_HZ))
    print(f"Rampa {eje}: {pct_inicio:+.0f}% -> {pct_fin:+.0f}% en {duracion_s}s")

    for paso in range(pasos):
        t = paso / pasos
        pct = pct_inicio + t * (pct_fin - pct_inicio)

        enviar_rc(
            conexion,
            roll_pct     = pct if eje == 'roll'  else 0.0,
            pitch_pct    = pct if eje == 'pitch' else 0.0,
            yaw_pct      = pct if eje == 'yaw'   else 0.0,
            throttle_pct = throttle_pct,
            armado       = True,
        )
        time.sleep(1 / CRSF_LOOP_HZ)


def aterrizaje_gradual(conexion: serial.Serial,
                       throttle_inicial_pct: float = 40.0,
                       duracion_s: float = 3.0) -> None:
    """
    Reduce el throttle linealmente de 'throttle_inicial_pct' a 0% en 'duracion_s' segundos, 
    simulando un descenso suave antes de tocar el suelo
    """
    pasos = max(1, int(duracion_s * CRSF_LOOP_HZ))
    print(f"Aterrizaje gradual: {throttle_inicial_pct}% -> 0% en {duracion_s}s")

    for paso in range(pasos):
        t = paso / pasos
        thr = throttle_inicial_pct * (1.0 - t)
        enviar_rc(conexion, throttle_pct=thr, armado=True)
        time.sleep(1 / CRSF_LOOP_HZ)




# diagnóstico sin hardware
def test_sin_hardware() -> None:
    """
    ejecuta todas las verificaciones de protocolo sin el hardware
    permite confirmar que las tramas generadas tienen la estructura correcta antes de conectar el módulo físico
    """
    print("=" * 55)
    print("  TEST DE PROTOCOLO CRSF (sin hardware)")
    print("=" * 55)

    #Verificar estructura de una trama de reposo
    canales_reposo = [CRSF_CH_CENTER] * CRSF_NUM_CHANNELS
    canales_reposo[CH_THROTTLE] = CRSF_CH_MIN
    canales_reposo[CH_ARM]      = CRSF_CH_MIN
    trama = construir_trama(canales_reposo)

    print(f"\n[1] Trama de reposo ({len(trama)} bytes):")
    print("    " + " ".join(f"{b:02X}" for b in trama))
    print(f"    SYNC=0x{trama[0]:02X}  "
          f"LEN=0x{trama[1]:02X} ({trama[1]})  "
          f"TYPE=0x{trama[2]:02X}  "
          f"CRC=0x{trama[-1]:02X}")
    print(f"    Longitud correcta: {'bien' if len(trama) == 26 else 'ERROR'}")

    #Verificar tabla de conversión porcentaje -> CRSF
    print("\n[2] Tabla de conversion roll/pitch/yaw (centrado):")
    print(f"    {'%':>6}  ->  CRSF")
    for pct in [-100, -50, 0, 50, 100]:
        print(f"    {pct:>+5}%  ->  {pct_a_crsf(pct)}")

    print("\n[3] Tabla de conversion throttle (no centrado):")
    print(f"    {'%':>6}  ->  CRSF")
    for pct in [0, 25, 50, 75, 100]:
        print(f"    {pct:>5}%  ->  {throttle_pct_a_crsf(pct)}")

    #Verificar que el CRC cambia si un byte del payload cambia
    canales_mod = canales_reposo.copy()
    canales_mod[CH_THROTTLE] = CRSF_CH_CENTER
    trama_mod = construir_trama(canales_mod)
    crcs_distintos = trama[-1] != trama_mod[-1]
    print(f"\n[4] CRC sensible a cambios en payload: "
          f"{'bien' if crcs_distintos else 'ERROR'}")
    print(f"    CRC reposo={trama[-1]:02X}  CRC modificado={trama_mod[-1]:02X}")

    print("\nTodos los tests pasaron" if crcs_distintos and len(trama) == 26
          else "\nHay errores en el protocolo(!)")
    print("=" * 55)


#------------------------------------------------------------------------------------------
#PROGRAMA PRINCIPAL
if __name__ == "__main__":

    #primerito: Test de protocolo sin hardware
    test_sin_hardware()
    print()

    # ya hecho eso 1: Detectar el módulo ELRS conectado por USB-C
    # recordar que el módulo debe estar configurado con los pines CRSF apuntando al UART interno del USB
    puerto = detectar_puerto_elrs()

    if puerto is None:
        print("\nNo se encontro el modulo ELRS.")
        print("Opciones:")
        print("  A) Conectar el BetaFPV ELRS Micro TX por USB-C y reintentar.")
        print("  B) Especificar el puerto manualmente:")
        print("        Windows  ->  puerto = 'COM3'")
        print("        Linux    ->  puerto = '/dev/ttyUSB0'")
        print("  C) Para pruebas sin hardware en Windows: instalar com0com")
        print("        https://sourceforge.net/projects/com0com/")
        sys.exit(0)

    # 2: Abrir conexión serial con el FTDI interno
    try:
        conexion = abrir_conexion_usb(puerto)
    except serial.SerialException as e:
        print(f"Error al abrir {puerto}: {e}")
        sys.exit(1)

    #3: Secuencia de vuelo
    try:
        # 3a. Pre-arm: señal neutra para que el receptor y FC se estabilicen
        prearm(conexion, duracion_s=2.0)

        # 3b. Armar el dron (throttle min + ARM HIGH sostenido)
        armar(conexion, duracion_s=2.0)

        # 3c. Subir gradualmente el throttle hasta el punto de hover (como 40% pero depende)
        print("Subiendo throttle al punto de hover...")
        rampa_eje(conexion, eje='roll',  # usamos rampa para subir throttle suavemente
                  pct_inicio=0, pct_fin=0,
                  duracion_s=0.1, throttle_pct=0.0)
        pasos_subida = int(1.5 * CRSF_LOOP_HZ)
        for paso in range(pasos_subida):
            thr = 40.0 * (paso / pasos_subida)
            enviar_rc(conexion, throttle_pct=thr, armado=True)
            time.sleep(1 / CRSF_LOOP_HZ)

        # 3d. Hover estacionario durante 3 segundos
        hover(conexion, duracion_s=3.0, throttle_pct=40.0)

        # 3e. Movimiento de roll a la derecha +25% y regreso al centro
        rampa_eje(conexion, eje='roll',
                  pct_inicio=0, pct_fin=25,
                  duracion_s=1.0, throttle_pct=40.0)
        rampa_eje(conexion, eje='roll',
                  pct_inicio=25, pct_fin=0,
                  duracion_s=1.0, throttle_pct=40.0)

        # 3f. Hover adicional para estabilizar
        hover(conexion, duracion_s=1.0, throttle_pct=40.0)

        # 3g. Aterrizaje gradual
        aterrizaje_gradual(conexion, throttle_inicial_pct=40.0, duracion_s=3.0)

        # 3h. Desarmar
        desarmar(conexion, duracion_s=2.0)

    except KeyboardInterrupt:
        print("\n[!] Interrumpido por el usuario.")
        print("    Enviando senal de desarmado de emergencia...")
        try:
            desarmar(conexion, duracion_s=1.0)
        except Exception:
            pass

    except Exception as e:
        print(f"\n[!] Error inesperado: {e}")
        try:
            desarmar(conexion, duracion_s=1.0)
        except Exception:
            pass

    finally:
        try:
            conexion.close()
            print("Puerto serial cerrado.")
        except Exception:
            pass