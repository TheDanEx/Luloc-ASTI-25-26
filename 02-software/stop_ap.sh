#!/bin/bash

# Validar ejecución como root
if [ "$EUID" -ne 0 ]; then
  echo "Error: Este script debe ejecutarse con privilegios elevados. Usa sudo"
  exit 1
fi

CON_NAME="AP_Luloc"

echo "=== Eliminando Punto de Acceso ==="
# Comprobar si el perfil de conexión existe antes de intentar borrarlo
if nmcli con show "$CON_NAME" > /dev/null 2>&1; then
    nmcli con down "$CON_NAME"
    nmcli con delete "$CON_NAME"
    echo "Perfil de red '$CON_NAME' eliminado correctamente."

    # Opcional: reiniciar la radio wifi para asegurar que escanee redes cliente
    # rfkill block wifi && rfkill unblock wifi
else
    echo "El perfil '$CON_NAME' no existe. Nada que limpiar a nivel de red."
fi

echo "=== Proceso de limpieza finalizado ==="