# Arquitectura de Modos de Operación (`main/modes`)

## Propósito Arquitectónico
Para garantizar la **Legibilidad Extrema** y el **Desacoplamiento**, el firmware utiliza un patrón de Dispatcher para gestionar los modos de operación del robot (Idle, Teleoperación, Siguelineas, etc.). 

En lugar de tener un bucle de control gigante con múltiples condicionales, cada modo es un módulo independiente que se registra en el despachador central.

## Estructura de Directorios
```text
main/modes/
├── mode_interface.h       # Definición de la interfaz estándar
├── modes.c / modes.h      # Dispatcher y gestión de transiciones
├── idle/                  # Modo MODE_NONE (Parada de seguridad)
├── teleoperation/         # Modo MODE_REMOTE_DRIVE
├── follow_line/           # Modo MODE_AUTONOMOUS_PATH (Siguelineas)
└── calibrate/             # Modos de calibración de motores y sensores
```

## Interfaz de Modo (`mode_interface_t`)
Cada modo debe implementar esta estructura:
- `enter()`: Inicialización de recursos específicos del modo (ej. crear telemetría).
- `execute()`: Bucle principal de control (se ejecuta a 100Hz).
- `exit()`: Limpieza y parada de seguridad al salir del modo.

## Flujo de Ejecución
1. El `task_rtcontrol_cpu0` llama a `modes_execute()` en cada iteración.
2. El despachador detecta si ha habido un cambio de modo en el `state_machine`.
3. Si hay cambio:
   - Llama al `exit()` del modo anterior.
   - Llama al `enter()` del modo nuevo.
4. Llama al `execute()` del modo activo actual.

## Cómo Añadir un Modo
1. Crear una subcarpeta en `main/modes/`.
2. Implementar la `mode_interface_t`.
3. Declarar la interfaz como `extern` en `modes.c`.
4. Añadir el mapeo en el `switch` de `get_interface_for_mode`.
5. Registrar los nuevos archivos en `main/CMakeLists.txt`.
