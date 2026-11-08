# Carro minisumo (ESP32) · Práctica 3

Firmware del **robot** (Micro 2). El guante está en otro proyecto:
`~/laboratorio_final_guante`. El software del PC (ROS 2): `~/ros2_ws/src/minisumo_teleop`.

```
laboratorio_final_senales/
├── main/main.c              control de ruedas (PID), odometría, goto, micro-ROS,
│                            mando directo del guante por ESP-NOW
├── components/
│   ├── MotorDriver/         puente H (PWM)
│   ├── MPU6050/             IMU del carro (I2C, filtro pasa-bajos interno)
│   └── EspNowLink/          paquete y enlace ESP-NOW (copia idéntica en el guante)
├── managed_components/      micro-ROS (lo descarga ESP-IDF, no editar)
└── archivo/                 archivos viejos (log de compilación, respaldo main.c.save)
```

## Compilar y flashear
```bash
source ~/.espressif/tools/activate_idf_v6.1.sh
cd ~/laboratorio_final_senales
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Comunicación
- **ESP-NOW (recibe del guante, 50 Hz)**: con el guante armado, el carro convierte la
  inclinación de la mano en v y ω. Prioridad: goto > guante armado > `/cmd_vel` del PC.
  Sin paquetes durante 250 ms, se detiene. Ajustes en `main.c` (sección
  "Mando directo del guante"): `GLOVE_V_MAX`, `GLOVE_INVERT_PITCH`, etc.
- **micro-ROS (WiFi) con el PC**: publica `imu/*`, `wheel_*/*`, `odom/*`, `goto_result`,
  `teleop/espnow`; recibe `cmd_vel` y `goto_cmd`.
- El guante y el carro deben quedar en el **mismo canal WiFi** (mismo router, o ambos sin
  router) y con el mismo `ESPNOW_GROUP_ID` (`idf.py menuconfig` → "Enlace ESP-NOW").

## Pines
MPU6050: SDA GPIO21, SCL GPIO22 (dirección 0x68) · Encoders: der. GPIO32, izq. GPIO33.
