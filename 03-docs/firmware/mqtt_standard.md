# Estándar de Comunicación MQTT - Lurloc-ASTI

Este documento establece la jerarquía de tópicos y el formato de mensajes para la comunicación MQTT del robot. El objetivo es garantizar la compatibilidad con **Telegraf**, **InfluxDB 2.7** y **Grafana**, además de facilitar la depuración remota.

## 1. Jerarquía de Tópicos (Hierarchy)

Todos los tópicos deben colgar de la raíz `robot/`.

| Categoría | Tópico | Formato | Descripción |
| :--- | :--- | :--- | :--- |
| **Telemetría** | `robot/telemetry/<M>` | ILP (Unix ns) | Datos de sensores de alta frecuencia para InfluxDB. `<M>` es el nombre de la medición (measurement). |
| **API Request** | `robot/api/request` | JSON | Canal de entrada unificado para peticiones Síncronas (GET y SET). |
| **API Response**| `robot/api/response`| JSON | Canal de salida unificado para las respuestas de la API. |
| **Eventos** | `robot/events` | JSON | Eventos críticos asíncronos (cambios de estado, errores graves, avisos). |
| **Logs** | `robot/logs/<L>` | Texto/JSON | Trazas de depuración remota. `<L>` es el nivel (`error`, `warn`, `info`, `debug`). |
| **Debug** | `robot/debug` | Libre / Raw | Sandbox para pruebas temporales y volcado de datos crudos durante el desarrollo. |
| **Configuración**| `robot/config/<S>` | JSON | Actualización de parámetros persistentes (NVS). `<S>` es el subsistema. |

---

## 2. Telemetría (ILP Standard)

Para la telemetría, se utiliza el **Influx Line Protocol (ILP)** con una precisión obligatoria de **19 dígitos (Unix Nanoseconds)**.

**Ejemplo de Payload:**
```text
odometry velIZ=1.25,posIZ=0.54 1677628800000000000
odometry velIZ=1.26,posIZ=0.55 1677628800010000000
```

- Los datos deben enviarse en **batches** cada 1000-5000ms.
- Cada muestra capturada debe llevar su propio **timestamp independiente**.

---

## 3. Logs Remotos (MQTT Logging)

El robot debe espejar (mirror) sus logs internos de `ESP_LOG` hacia tópicos MQTT para permitir la depuración sin cable serial.

- **Frecuencia:** Los logs de nivel `ERROR` y `WARN` deben enviarse instantáneamente.
- **Filtro:** Los niveles `INFO` y `DEBUG` pueden ser activados/desactivados mediante la API para evitar saturar la red.

---

## 4. API Unificada (JSON)

Para simplificar el uso desde Dashboards o scripts externos, se utiliza un único tópico de entrada `robot/api/request`. El payload debe incluir el tipo de operación (`GET` o `SET`).

**Ejemplo Request (GET):**
```json
{
  "op": "get",
  "resource": "battery"
}
```

**Ejemplo Request (SET):**
```json
{
  "op": "set",
  "action": "play_sound",
  "sound_id": 1
}
```

**Ejemplo Response:**
```json
{
  "op": "resp",
  "status": "success",
  "data": { ... }
}
```

---

## 5. Configuración de Telegraf

Telegraf debe configurarse para parsear el tópico y extraer la medición:
```toml
# Ejemplo de configuración recomendada
[[inputs.mqtt_consumer]]
  servers = ["tcp://localhost:1883"]
  topics = ["robot/telemetry/#"]
  data_format = "influx"
```
