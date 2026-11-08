# INA219 driver for ESP-IDF (`i2c_master`)

Driver for the INA219 current/power monitor that uses the new ESP-IDF I2C API
(`driver/i2c_master.h`). Revised fork of
[tny-robotics/ina219-esp-idf](https://github.com/tny-robotics/ina219-esp-idf) (MIT).
Requires ESP-IDF >= 5.2. See `CHANGES.md` for what was fixed.

## Instalación en tu proyecto

Copia esta carpeta como `components/ina219/` dentro de tu proyecto:

    mi_proyecto/
    ├── components/ina219/   <- esta carpeta
    └── main/

Si `main/CMakeLists.txt` usa `REQUIRES`, agrega `ina219` ahí. Si no usa `REQUIRES`, no hace falta nada más.

También puedes subir la carpeta a tu propio repositorio de GitHub y referenciarla en
`main/idf_component.yml`:

    dependencies:
      ina219:
        git: https://github.com/TU_USUARIO/TU_REPO.git

## Uso mínimo

    ina219_handle_t ina;
    ESP_ERROR_CHECK(ina219_create(bus, INA219_DEFAULT_INFO(), &ina));

    ina219_config_t cfg = INA219_DEFAULT_CONFIG();
    cfg.shunt_resistor_ohms = 0.1f;
    cfg.max_expected_current_A = 3.2f;
    ESP_ERROR_CHECK(ina219_pick_gain(cfg.shunt_resistor_ohms, cfg.max_expected_current_A, &cfg.gain));
    ESP_ERROR_CHECK(ina219_config(&ina, cfg));

    ina219_measurement_t m;
    if (ina219_read_all(ina, &m) == ESP_OK && !m.overflow) { /* usar m.current_mA ... */ }

Ejemplo completo en `examples/ina219/`.

## Notas

- `shunt_resistor_ohms` y `max_expected_current_A` deben coincidir con tu hardware, o las
  lecturas de corriente y potencia saldrán mal escaladas.
- Tras `ina219_reset()` hay que llamar otra vez a `ina219_config()` (se pierde la calibración).
- Para un lazo de control usa pocas muestras (`INA219_ADC_12BIT_1SAMP`..`4SAMP`); con 128 la lectura se actualiza cada ~70 ms.
