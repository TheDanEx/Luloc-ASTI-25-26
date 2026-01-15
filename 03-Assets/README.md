# Assets (Recursos externos)

Esta carpeta contiene recursos externos y material bruto utilizado durante
el desarrollo del proyecto.

⚠️ IMPORTANTE  
Los archivos contenidos aquí **NO se usan directamente en ejecución** ni por
el software de la Raspberry Pi ni por el firmware del ESP32.

---

## Uso previsto

Los assets de esta carpeta sirven como:

- Material de origen (sonidos, imágenes, gráficos)
- Recursos para documentación y presentaciones
- Base para generar archivos embebidos en firmware

---

## Flujo correcto de uso

1. El asset se guarda aquí en formato bruto (ej. WAV, PNG).
2. Si se necesita en el ESP32:
   - Se convierte al formato adecuado (PCM, array C, etc.).
   - Se copia a la carpeta correspondiente dentro de `firmware/`.
3. El firmware utiliza **su propia copia**, no este archivo.

---

## Contenido

- `media/`: media para dashboard, documentación, sonidos
- `docs/`: diagramas, esquemas y material visual

---

## Nota

Esta carpeta no forma parte del build ni del despliegue automático.
Su contenido es únicamente de apoyo al desarrollo.
