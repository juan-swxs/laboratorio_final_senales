# Cambios respecto a tny-robotics/ina219-esp-idf v1.0.0

## Correcciones
- **Timeout I2C**: se pasaba `pdMS_TO_TICKS(100)` a funciones que esperan milisegundos; ahora son 100 ms reales.
- **Handle sin inicializar**: `ina219_create` pone el handle en cero, así el chequeo "falta ina219_config" es fiable.
- **Calibración**: el bit 0 del registro es nulo, se escribe un valor par, y el LSB de corriente/potencia se calcula con el valor realmente escrito (antes con el ideal).
- **`ina219_config`** valida antes de tocar el hardware: shunt/corriente > 0, `Rshunt × Imax` dentro del rango del PGA, valor de calibración dentro de 16 bits, enums válidos. El handle solo se actualiza si todo sale bien.
- Códigos de error correctos (`ESP_ERR_INVALID_ARG`, `ESP_ERR_INVALID_STATE`) y chequeo de punteros NULL en todos los getters.
- `ina219_delete` devuelve el resultado real; `ina219_create` valida la dirección (0x40..0x4F).

## Añadido
- `ina219_read_all()` + `ina219_measurement_t`: lectura completa con flag de overflow.
- `ina219_get_status()`: flags CNVR y OVF.
- `ina219_pick_gain()`: elige la ganancia del PGA con mejor resolución.
- `clock_speed = 0` usa el valor por defecto.

## Sin cambios de API
Las funciones originales siguen con la misma firma. Excepción de comportamiento: los códigos de error y las validaciones nuevas de `ina219_config`.

## Probado
Pruebas en PC contra un INA219 simulado (registros, calibración, signo, overflow, validaciones, sleep/wake, reset). **No** se ha probado en hardware real ni compilado con ESP-IDF.
