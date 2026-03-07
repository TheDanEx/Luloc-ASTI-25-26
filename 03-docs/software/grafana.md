# Grafana

## Propósito Arquitectónico
Visualizador analítico (Frontend) de las condiciones de contorno y depuración cruda. Convierte las enormes bases de datos temporales producidas por el ESP32 en paneles humanos de diagnósticos, gráficos de lazo cerrado (PID response), y alarmas dinámicas para el investigador de hardware/software del Lurloc.

## Entorno y Dependencias
Imagen pública `grafana/grafana`. Atada al conector DataSource hacia InfluxDB (espera explícitamente a su levantamiento general por medio de `depends_on`). Su diseño recae en auto-provisionamiento (`provisioning` y `dashboards` volúmenes bind).

## Interfaces de E/S (Inputs/Outputs)
- **Hardware:** Persistencia atada local al volumen dinámico `grafana-data`.
- **Software:** Expone una pasarela HTTP/UX al mundo exterior vía `${GRAFANA_PORT}:3000` del host (típicamente 3000 o 80 en la red local).

## Flujo de Ejecución Lógico
Se acopla detrás de InfluxDB. Una vez en pie, inyecta su propio setup nativo importando sin preguntar todos los Dashboards JSON existentes de la carpeta enlazada, por lo que el usuario al autenticarse (parámetros `GF_SECURITY_ADMIN_USER` automáticos) encuentra las gráficas vivas inmediatamente, sin necesidad de conectarlas repetidamente o configuraciones iniciales con clics.

## Parámetros de Configuración y Entorno (Docker)
Interfaz estática aprovisionada transparentemente.
- `GF_SECURITY_ADMIN_USER` / `_PASSWORD`: Fija y bloquea las credenciales primarias a través de `.env` usando `${GRAFANA_ADMIN_USER}` y `${GRAFANA_ADMIN_PASSWORD}`.
- `ports`: Exportado al host anfitrión por la variable universal externa `${GRAFANA_PORT}:3000`.
- `volumes (Provisioning)`: `./05-telemetry/grafana/provisioning` que hospeda definiciones nativas YAML que instruyen a grafana la existencia irrevocable e intrínseca del InfluxDB (DataSource).
- `volumes (Dashboards)`: `./05-telemetry/grafana/dashboards` inyecta los paneles JSON crudos al directorio esperado `/var/lib/grafana/dashboards` forzando su carga al iniciar.

## Puntos Críticos y Depuración
- **Rezagos de Visualización por Polling Ciego:** Las gráficas refrescan por lo general a 1Hz o 5Hz por browser nativo. Picos intermitentes y micro-frenadas (Brake-Coasting motors) en milisegundos no siempre son visibles; Grafana es un agregador visual, si el desarrollador no mira de cerca el min/max del InfluxDB, podría subestimar ruidos de encoders.
- **Pérdida de Configuración por UI:** Cualquier edición cosmética en los paneles web SE PERDERÁ o generará bifurcaciones si no fue documentada permanentemente exportando el JSON al volumen de `dashboards`, debido al aprovisionamiento hardcodeado "As Code".
