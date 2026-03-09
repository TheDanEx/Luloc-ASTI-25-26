#!/bin/bash

# Validar ejecución como root
if [ "$EUID" -ne 0 ]; then
  echo "Error: Este script debe ejecutarse con privilegios elevados. Usa: sudo ./deploy.sh"
  exit 1
fi

# Variables de configuración del AP
SSID="Luloc PI AP"
PASSWORD="jc y picor" # Requisito: Mínimo 8 caracteres
CON_NAME="AP_Luloc"
INTERFACE="wlan0"

echo "=== Configurando infraestructura de red ==="

# Comprobar si el perfil de conexión ya existe en NetworkManager
if nmcli con show "$CON_NAME" > /dev/null 2>&1; then
    echo "El perfil '$CON_NAME' ya existe. Asegurando que esté activo..."
    nmcli con up "$CON_NAME"
else
    echo "Creando el Punto de Acceso '$SSID'..."

    # Crear el perfil con autoconexión al arranque
    nmcli con add type wifi ifname "$INTERFACE" con-name "$CON_NAME" autoconnect yes ssid "$SSID"

    # Configurar el modo AP y enrutamiento (DHCP + NAT automático)
    nmcli con modify "$CON_NAME" 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared

    # Establecer la contraseña WPA2
    nmcli con modify "$CON_NAME" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASSWORD"

    # Levantar la interfaz
    nmcli con up "$CON_NAME"

    echo "Punto de Acceso configurado con éxito."
fi

echo "=== Proceso finalizado ==="