# LiDAR D500

Cada paquete que envia el lidar por el uart corresponde a:

packet[0] = 0x54
packet[1] = 0x2C
packet[2..3] = speed
packet[4..5] = start_angle
packet[6..41] = 12 puntos (cada uno de 2 bytes)
packet[42..43] = end_angle
packet[44..45] = timestamp
packet[46] = crc

Cada paquete contiene 12 medionces entre el angulo start_angle y el angulo end_angle

## Ejemplo

I (27507) LIDAR: Paquete LiDAR completo:
I (27507) LIDAR: 54 2c ee 0d 48 84 fc 06 ad f3 06 ad eb 06 af e3
I (27517) LIDAR: 06 af dc 06 b0 d5 06 b1 ce 06 b2 c8 06 b3 c1 06
I (27517) LIDAR: b4 bb 06 b5 b6 06 b6 b1 06 b8 55 87 dd 4a 0f
