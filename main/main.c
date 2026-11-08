#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "mpu6050.h"
#include "ina219.h"
#include "motor_driver.h"
#include "espnow_link.h"
#include "ina219.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/battery_state.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/discovery.h>
#include "uros_network_interfaces.h"

const char *TAG = "Robot Autonomo";

/*
 * Macros micro-ROS: IDF compila con -Werror y varias funciones rcl/rclc
 * estan marcadas warn_unused_result, por eso hay que consumir el retorno.
 *  - RCRETCHECK: si falla, loguea y devuelve false (creacion de entidades;
 *    el llamador reintenta la conexion en vez de matar la tarea)
 *  - RCSOFTCHECK: ignora el retorno (publicacion periodica, destruccion)
 */
#define RCRETCHECK(fn)                                                           \
    do {                                                                         \
        rcl_ret_t rc_ = (fn);                                                    \
        if (rc_ != RCL_RET_OK) {                                                 \
            ESP_LOGE(TAG, "rcl error %d en linea %d: %s", (int)rc_, __LINE__, #fn); \
            return false;                                                        \
        }                                                                        \
    } while (0)

#define RCSOFTCHECK(fn)                                                          \
    do {                                                                         \
        rcl_ret_t rc_ = (fn);                                                    \
        (void)rc_;                                                               \
    } while (0)

/*
 * ============================================================
 * MPU6050 (I2C)
 * ============================================================
 */
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   GPIO_NUM_21
#define I2C_MASTER_SCL_IO   GPIO_NUM_22
#define MPU6050_I2C_ADDR    MPU6050_ADDR_AD0_LOW   /* 0x68 con AD0 a GND, 0x69 con AD0 a 3V3 */
#define MPU6050_I2C_HZ      400000
/* Filtro pasa-bajos interno del MPU6050 (hardware, antes del ADC de
 * salida). 44 Hz quita la vibracion de los motores y deja pasar los
 * giros del robot (pocos Hz) con ~5 ms de retardo. */
#define MPU6050_DLPF        MPU6050_DLPF_44HZ
/* 1 si el modulo esta montado boca abajo (chip mirando al piso):
 * invierte los ejes Y y Z para que Z apunte hacia arriba. */
#define IMU_MOUNTED_UPSIDE_DOWN  0

/*
 * ============================================================
 * INA219 (I2C, mismo bus que el MPU6050)
 *  Mide tension, corriente y potencia de la bateria. Direccion
 *  0x40..0x4F segun A0/A1 (se detecta sola). Si no esta conectado
 *  el carro funciona igual.
 * ============================================================
 */
#define INA219_SHUNT_OHMS        0.1f    /* resistencia del modulo (R100 = 0.1 ohm) */
#define INA219_MAX_CURRENT_A     3.2f    /* 2 motores TT + ESP32: cabe en el rango de 320 mV */
#define INA219_I2C_HZ            400000
#define INA219_PERIOD_MS         100     /* 10 Hz */
#define INA219_LOG_PERIOD_MS     2000
/* Alimentacion del sistema: 2 x Beston BST-9V Ni-MH (250 mAh c/u) en
 * paralelo -> misma tension (7 celdas, 8.4 V nominal) y 500 mAh */
#define BATT_CAPACITY_MAH        500.0f

/*
 * ============================================================
 * Encoders
 * ============================================================
 */
#define ENCODER_RIGHT GPIO_NUM_32
#define ENCODER_LEFT  GPIO_NUM_33

/*
 * ============================================================
 * Caracteristicas ruedas
 * ============================================================
 */
/* Medidas reales del robot (antes 7.1 y 11.2 cm) */
#define WHEEL_DIAMETER_CM   4.8f
#define WHEEL_DISTANCE_CM   9.7f
#define PULSES_PER_REV      20      /* tamano de la tabla por vuelta (no tocar) */
/* Pulsos por vuelta de CADA rueda. En la prueba de la libreria, con el
 * mismo PWM la izquierda contaba ~1.5 veces mas que la derecha. Mide:
 * robot encendido sin comandos, gira cada rueda 1 vuelta exacta a mano y
 * mira "Encoders (a mano)" en el monitor. Pon aqui lo que sume cada una. */
/* Medido a mano: 40 pulsos por vuelta en la izquierda y ~40-46 en la
 * derecha. Con 20 la odometria daba el doble de la distancia real (el robot
 * creia avanzar 54 cm y avanzo 21). */
#define PULSES_PER_REV_L    40
#define PULSES_PER_REV_R    40
/* Anti-rebote del encoder. Con 5 ms (max 200 pulsos/s) y 40 pulsos por
 * vuelta se perdian pulsos por encima de ~31 rad/s: en la prueba en el aire
 * la izquierda "dejaba de acelerar" desde PWM 70. Con 2 ms aparecian pulsos
 * falsos por ruido del motor; 3 ms mide hasta ~52 rad/s. */
#define ENCODER_DEBOUNCE_US 3000

/*
 * ============================================================
 * Rango de PWM del motor (dominio 0-100, ver motor_driver.h)
 * ============================================================
 */
#define MAX_PWM   100
#define MIN_PWM  -100   /* con signo: negativo = reversa */

/* Prealimentacion del PID de rueda: pwm = signo * (OFFSET + GAIN * |w|)
 * y el PI solo corrige la diferencia, asi la rueda responde sin esperar a
 * que la integral se cargue y sin dar el tiron de despues.
 * Caracterizacion en el piso con el DRV8833 (2026-10-26, bateria ~9.7 V,
 * PWM directo 30-50 en ambos sentidos, error del ajuste <= 3.7 PWM):
 *   izquierda  w(PWM 30/40/50) = 14.2 / 23.6 / 28.9 rad/s
 *   derecha    w(PWM 30/40/50) =  9.0 / 16.8 / 22.9 rad/s
 * La izquierda va ~30-40 % mas rapido con el mismo PWM, por eso cada rueda
 * tiene su propia recta. (Con el L298N era 25 + 7 * w para las dos.) */
#define PWM_FF_OFFSET_L      11.6f
#define PWM_FF_GAIN_L        1.27f   /* PWM por rad/s */
#define PWM_FF_OFFSET_R      17.3f
#define PWM_FF_GAIN_R        1.40f
/* Con PWM 25 la derecha apenas gira (1.3-7 rad/s) y nada se mueve con 20:
 * por debajo de ~4 rad/s una rueda no gira parejo, se pide 0. Debe quedar
 * por debajo de lo que piden GLOVE_V_MIN y GLOVE_W_MIN por rueda. */
#define WHEEL_MIN_SETPOINT   4.0f    /* rad/s */
#define WHEEL_MAX_RAD_S      30.0f   /* rad/s: lo medido con PWM 50 (no se caracterizo mas arriba) */
/* Arranque desde parado medido con rampa de PWM: izq 23/24, der 25/21
 * (adelante/atras). 28 deja margen a las dos. (Con el L298N era 70.) */
#define PWM_BREAKAWAY        28.0f

/*
 * ============================================================
 * Protocolo de /goto_cmd (geometry_msgs/Twist)
 *   linear.x  = x (m)          linear.y  = y (m)
 *   linear.z  = PWM izquierda  angular.x = PWM derecha
 *   angular.y = numero de secuencia (0 = sin secuencia, compatibilidad)
 *   angular.z = modo: 0 absoluto, 1 relativo, 2 reset del cero, 3 cancelar
 *
 * Estado publicado a 20 Hz en /odom/twist (campos antes sin uso):
 *   linear.y = ultima secuencia aceptada
 *   linear.z = estado: 0 inactivo, 1 en curso, 2 completado, 3 abortado
 * ============================================================
 */
#define GOTO_MODE_ABSOLUTE  0
#define GOTO_MODE_RELATIVE  1
#define GOTO_MODE_RESET     2
#define GOTO_MODE_CANCEL    3

#define GOTO_STATE_IDLE     0
#define GOTO_STATE_ACTIVE   1
#define GOTO_STATE_DONE     2
#define GOTO_STATE_ABORTED  3

/* Si no llega cmd_vel en este tiempo (y no hay goto), se paran los motores */
#define CMD_VEL_TIMEOUT_MS  500

/*
 * ============================================================
 * Mando directo del guante por ESP-NOW (estrategia 2: inclinacion)
 *   pitch de la mano -> velocidad lineal v
 *   roll de la mano  -> velocidad angular w
 * Prioridad: goto > guante armado > cmd_vel del PC. Si el guante deja
 * de enviar GLOVE_TIMEOUT_MS o se desarma, el robot se detiene.
 * Los mismos valores que glove_teleop en el PC (pestana Ajustes).
 * ============================================================
 */
#define GLOVE_TIMEOUT_MS        250
#define GLOVE_V_MAX             0.45f   /* m/s: inclinacion completa (13.4 rad/s, PWM ~30 con el DRV8833) */
#define GLOVE_W_MAX             4.5f    /* rad/s (ruedas a ~10 rad/s con 15.3 cm entre ruedas) */
/* Al salir de la zona muerta se arranca ya en una velocidad que la rueda
 * puede mantener sin trabarse; por debajo el motor avanza a tirones por la
 * friccion estatica. DRV8833: con 0.06 / 0.8 cada rueda pedia ~1.8 rad/s,
 * menos que WHEEL_MIN_SETPOINT, y entre 8 y ~20 grados de inclinacion el
 * carro no se movia. */
#define GLOVE_V_MIN             0.14f   /* m/s (~4.2 rad/s por rueda) */
#define GLOVE_W_MIN             1.8f    /* rad/s (giro en el sitio, ~4.1 rad/s por rueda) */
/* Lazo de giro con el giroscopio del MPU6050: si el carro no gira lo que
 * pide la mano (una rueda mas floja, el piso), corrige la diferencia
 * entre ruedas. */
#define GLOVE_YAW_KP            0.6f
#define GLOVE_YAW_KI            1.5f
#define GLOVE_YAW_CORR_MAX      1.5f    /* rad/s de correccion maxima */
#define GLOVE_DEADZONE_DEG      8.0f
#define GLOVE_DEADZONE_EXIT_DEG 5.0f    /* histeresis: ya en marcha, se para solo por debajo de esto */
#define GLOVE_FULL_TILT_DEG     35.0f
#define GLOVE_EXPO              1.6f
#define GLOVE_INVERT_PITCH      0       /* 1 si inclinar hacia adelante hace retroceder */
#define GLOVE_INVERT_ROLL       0
#define GLOVE_SMOOTH_TAU_S      0.12f   /* pasa-bajos del comando */
#define GLOVE_DEBUG_LOG         1       /* 1: estado del guante en el monitor cada 1 s */
#define CAR_TELEMETRY           1       /* 1: linea CSV "T,..." a 50 Hz por el monitor (analisis) */

/* Proteccion del borde del dojo con la odometria del propio carro */
#define DOJO_GUARD_ENABLED      0       /* 1 solo en el dojo y arrancando en el centro: usa la odometria desde el encendido */
#define DOJO_RADIUS_M           2.5f    /* dojo de 5 m de diametro (igual que la app) */
#define DOJO_GUARD_MARGIN_M     0.05f
#define DOJO_GUARD_LOOKAHEAD_S  0.45f   /* lo que rueda tras pedir v = 0 */
#define DOJO_GUARD_SLOW         0.15f   /* m/s maximos cerca del borde */

/*
 * ============================================================
 * Estado global compartido entre tareas
 * ============================================================
 */

volatile int32_t encoder_count_left = 0;
volatile int32_t encoder_count_right = 0;
volatile uint32_t last_time_right = 0;
volatile uint32_t last_time_left = 0;
/* Tiempo entre los dos ultimos pulsos (us). Con 40 pulsos/vuelta, contar
 * pulsos en 20 ms da 0, 1 o 2 pulsos (velocidad muy "escalonada"); medir el
 * periodo da una velocidad nueva en cada pulso. */
volatile uint32_t encoder_period_right = 0;
volatile uint32_t encoder_period_left = 0;

/* Posicion angular de la rueda izquierda en pulsos, CON signo (segun el
 * sentido comandado). Sirve para saber en que parte de la vuelta esta la
 * rueda y compensar el punto duro que se repite cada vuelta. */
static volatile int32_t g_left_pos = 0;

/* Velocidad de rueda por periodo de pulsos, filtrada (rad/s, sin signo) */
static volatile float g_wheel_speed_l = 0.0f;
static volatile float g_wheel_speed_r = 0.0f;

typedef struct
{
    float raw_left;       /* rad/s, velocidad instantanea sin filtrar */
    float raw_right;
    float filtered_left;  /* rad/s, tras EMA propio */
    float filtered_right;
} encoder_data_t;
static volatile encoder_data_t g_encoder_data = {0};

typedef struct
{
    float raw_ax, raw_ay, raw_az;      /* m/s^2, lectura cruda acelerometro */
    float raw_gx, raw_gy, raw_gz;      /* rad/s, lectura del giroscopio */
    float lin_ax, lin_ay, lin_az;      /* m/s^2, aceleracion lineal (gravedad restada con roll/pitch estimados) */
    float roll, pitch;                 /* rad, filtro complementario (acelerometro + giroscopio) */
    float qx, qy, qz, qw;              /* quaternion de (roll, pitch, yaw del giroscopio) */
    float f_gx, f_gy, f_gz;            /* rad/s, giroscopio filtrado (sin sesgo + pasa-bajos EMA) */
} imu_data_t;
static volatile imu_data_t g_imu_data = {0};

/* Rumbo en el marco del arranque (0 = orientacion inicial, antihorario +) */
static volatile float g_yaw_gyro   = 0.0f;   /* rad, integral del giroscopio */
static volatile float g_gz_bias    = 0.0f;   /* rad/s, sesgo del giroscopio */

/* Retroceso empujando a mano. Los encoders son de un solo canal: cuentan
 * ranuras pero no saben hacia donde gira la rueda, asi que se usa el
 * sentido del ultimo comando y, sin comando, adelante. Lo unico que se
 * agrega: si nadie comanda las ruedas y el MPU6050 indica con claridad que
 * el carro va hacia atras (g_push_v = integral con fuga de la aceleracion
 * longitudinal), las ruedas cuentan en reversa. El avance no se toca.
 * g_imu_x_sign = +1 si el eje X del MPU6050 apunta al frente del robot. */
#define IMU_X_FORWARD_SIGN   1.0f
#define PUSH_V_TAU_S         0.6f    /* s: fuga del integrador (quita la deriva del acelerometro) */
#define PUSH_ACC_DEADBAND    0.12f   /* m/s^2: por debajo no se integra (ruido, sesgo) */
#define PUSH_IDLE_CYCLES     25      /* 0.5 s sin comando antes de usar el IMU (deja terminar la frenada) */
#define PUSH_BACK_V          0.06f   /* m/s: "velocidad" de empuje que cuenta como retroceso claro */
static volatile float g_push_v = 0.0f;
static volatile float g_imu_x_sign = IMU_X_FORWARD_SIGN;
static volatile bool  g_yaw_ready  = false;
static float g_gx_bias = 0.0f, g_gy_bias = 0.0f;   /* rad/s, para roll/pitch */

/* Peticiones de reset del cero (las ejecuta cada tarea duena del dato) */
static volatile bool g_reset_yaw_req  = false;
static volatile bool g_reset_odom_req = false;

typedef struct
{
    float Kp, Ki, Kd;
    float ff_offset, ff_gain;   /* prealimentacion de esta rueda */
    float integral;
    float prev_error;
    volatile float setpoint_rad_s;
    volatile float measured_rad_s;
} wheel_pid_t;

/* Kd = 0: el encoder de 40 pulsos/vuelta da una velocidad escalonada y la
 * derivada convertia cada escalon en un golpe de +-50 de PWM. */
/* Con 40 pulsos por vuelta (antes 20) la velocidad en rad/s mide la mitad
 * que antes: Kp y Ki se duplicaron para conservar el mismo ajuste.
 * Con el DRV8833 el motor da ~5.3 veces mas rad/s por PWM (1/1.33 frente a
 * 1/7 con el L298N): Kp 10 -> 2.0 y Ki 24 -> 4.5 conservan la misma
 * ganancia de lazo. El motor tarda ~150-250 ms en llegar al 63 %. */
static wheel_pid_t pid_left  = { .Kp = 2.0f, .Ki = 4.5f, .Kd = 0.0f,
                                 .ff_offset = PWM_FF_OFFSET_L, .ff_gain = PWM_FF_GAIN_L };
static wheel_pid_t pid_right = { .Kp = 2.0f, .Ki = 4.5f, .Kd = 0.0f,
                                 .ff_offset = PWM_FF_OFFSET_R, .ff_gain = PWM_FF_GAIN_R };

/* Odometria (marco del arranque: x adelante, y izquierda, theta antihorario) */
typedef struct
{
    float x, y, theta;   /* m, m, rad */
    float v, w;          /* m/s, rad/s */
} odom_t;
static volatile odom_t g_odom = {0};

/* Modo PWM directo (lazo abierto): lo activa cmd_vel o goto_task */
static volatile bool  g_pwm_mode = false;
static volatile float g_pwm_cmd_left = 0.0f;
static volatile float g_pwm_cmd_right = 0.0f;
static volatile TickType_t g_last_cmd_vel_tick = 0;
static volatile bool  g_brake = false;   /* freno activo (lo usa goto al llegar) */

/* Comando goto */
static volatile bool     g_goto_pending = false;
static volatile bool     g_goto_active = false;
static volatile bool     g_goto_abort = false;
static volatile bool     g_goto_relative = false;
static volatile float    g_goto_target_x = 0.0f;
static volatile float    g_goto_target_y = 0.0f;
static volatile float    g_goto_pwm_left = 0.0f;
static volatile float    g_goto_pwm_right = 0.0f;
static volatile uint32_t g_last_seq = 0;          /* ultima secuencia procesada (dedup) */
static volatile uint32_t g_status_seq = 0;        /* secuencia del estado publicado */
static volatile int      g_status_state = GOTO_STATE_IDLE;

typedef struct
{
    volatile bool ready;
    float arrived_x, arrived_y, target_x, target_y, error_m;
    uint32_t seq;
} goto_result_t;
static volatile goto_result_t g_goto_result = {0};

/* MPU6050 */
static mpu6050_t imu = {0};
static i2c_master_bus_handle_t s_i2c_bus = NULL;

/* INA219: ultima medicion (la escribe power_task) */
static ina219_handle_t s_ina = {0};
static bool s_ina_ok = false;
static volatile float g_batt_v = 0.0f, g_batt_ma = 0.0f, g_batt_mw = 0.0f;
static volatile float g_batt_used_mah = 0.0f;   /* carga consumida desde el arranque (integral de la corriente) */

/* Declaracion de funciones */
static void mpu6050_setup(void);
static void encoders_init(void);

static float low_pass(float prev_filtered, float new_raw, float alpha);
static float pulses_to_rad_s(int32_t delta_pulses, float window_s, int ppr);
static float clampf(float x, float lo, float hi);
static float wheel_pid_update(wheel_pid_t *pid, float dt);
static void twist_to_wheel_speeds(float v, float w, float *omega_left, float *omega_right);
static float get_yaw_odom(void);
static float wrap_pi(float a);
static bool robot_is_still(void);

void wheel_control_task(void *pvParameters);
void imu_task(void *pvParameters);
void goto_task(void *pvParameters);
void micro_ros_task(void *pvParameters);

void test_raw_motors(void);
void test_encoder_mapping(void);
void test_motor_library(void);

/* Funciones de motor_driver.c usadas en la prueba de la libreria */
esp_err_t car_turn_right(uint8_t speed_percent);
esp_err_t car_turn_left(uint8_t speed_percent);
esp_err_t car_stop(void);
esp_err_t car_brake(void);

/* ============================================================
 * MPU6050: inicializacion
 *  No necesita calibracion guardada: el sesgo del giroscopio se
 *  estima en imu_task con el robot quieto al arrancar.
 * ============================================================ */
static void mpu6050_setup(void)
{
    i2c_master_bus_handle_t i2c_master_bus = NULL;

    i2c_master_bus_config_t i2c_master_conf = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_conf, &i2c_master_bus));
    s_i2c_bus = i2c_master_bus;

    /* Escaneo del bus: que sensores responden (MPU6050 0x68/0x69,
     * INA219 0x40..0x4F) */
    {
        char found[96] = "";
        int n = 0;
        for (uint16_t a = 0x08; a < 0x78; a++)
        {
            if (i2c_master_probe(i2c_master_bus, a, 20) == ESP_OK)
            {
                n += snprintf(found + n, sizeof(found) - n, " 0x%02X", a);
                if (n >= (int)sizeof(found) - 6) break;
            }
        }
        ESP_LOGI(TAG, "Escaneo I2C (SDA=GPIO%d, SCL=GPIO%d):%s", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO,
                 found[0] ? found : " ningun dispositivo");
    }

    /* Si el MPU6050 no responde se reintenta (con motores apagados) en
     * vez de reiniciar el ESP32 en bucle. */
    for (int intento = 1; ; intento++)
    {
        esp_err_t r = mpu6050_init(&imu, i2c_master_bus, MPU6050_I2C_ADDR, MPU6050_DLPF, MPU6050_I2C_HZ);
        if (r == ESP_OK)
        {
            break;
        }
        ESP_LOGE(TAG, "MPU6050 no responde por I2C (intento %d: %s). Revisa: VCC y GND del "
                      "MPU6050, SDA->GPIO21, SCL->GPIO22, AD0 (0x68 a GND), cables flojos. Reintentando...",
                 intento, esp_err_to_name(r));
        motor_left_set(0);
        motor_right_set(0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "MPU6050 configurado (+-500 deg/s, +-4 g, DLPF %d)", (int)MPU6050_DLPF);
}

/* ============================================================
 * INA219: deteccion y configuracion (despues del MPU6050, mismo bus)
 * ============================================================ */
static void ina219_setup(void)
{
    uint16_t addr = 0;
    for (uint16_t a = 0x40; a <= 0x4F; a++)
    {
        if (i2c_master_probe(s_i2c_bus, a, 20) == ESP_OK)
        {
            addr = a;
            break;
        }
    }
    if (addr == 0)
    {
        ESP_LOGW(TAG, "INA219 no encontrado (0x40..0x4F): el carro sigue sin medir la bateria");
        return;
    }

    ina219_info_t info = { .address = addr, .clock_speed = INA219_I2C_HZ };
    esp_err_t r = ina219_create(s_i2c_bus, info, &s_ina);
    if (r == ESP_OK)
    {
        ina219_config_t cfg = INA219_DEFAULT_CONFIG();
        cfg.shunt_resistor_ohms = INA219_SHUNT_OHMS;
        cfg.max_expected_current_A = INA219_MAX_CURRENT_A;
        /* 64 muestras promediadas (34 ms por lectura): con 8 la medida
         * saltaba entre dos valores vecinos (0.1 mA, 4 mV) y con el ruido
         * de los motores; a 10 Hz de lectura sigue sobrando velocidad */
        cfg.badc_res = INA219_ADC_12BIT_64SAMP;
        cfg.sadc_res = INA219_ADC_12BIT_64SAMP;
        r = ina219_pick_gain(cfg.shunt_resistor_ohms, cfg.max_expected_current_A, &cfg.gain);
        if (r == ESP_OK)
        {
            r = ina219_config(&s_ina, cfg);
        }
    }
    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "INA219 en 0x%02X no se pudo configurar: %s", addr, esp_err_to_name(r));
        return;
    }
    s_ina_ok = true;
    ESP_LOGI(TAG, "INA219 configurado en 0x%02X (shunt %.2f ohm, hasta %.1f A)", addr,
             INA219_SHUNT_OHMS, INA219_MAX_CURRENT_A);
}

/* Lee el INA219 a 10 Hz (prioridad baja: comparte el bus con el MPU6050,
 * el driver de I2C serializa los accesos) */
/* Vigila el nFAULT del DRV8833 (ULT -> GPIO19) a 50 Hz y avisa al
 * cambiar: con PWM mandado y nFAULT=0 el driver corto las salidas. */
static void driver_fault_task(void *arg)
{
    bool prev = false;
    uint32_t count = 0;
    int64_t last_log_us = 0;
    while (1)
    {
        bool f = motor_driver_fault();
        int64_t now = esp_timer_get_time();
        if (f && !prev)
        {
            count++;
        }
        if (f != prev || (f && now - last_log_us >= 2000000))
        {
            last_log_us = now;
            if (f)
            {
                ESP_LOGE(TAG, "DRV8833 nFAULT ACTIVO (%lu veces): sobrecorriente, temperatura o bajo voltaje en VM",
                         (unsigned long)count);
            }
            else
            {
                ESP_LOGI(TAG, "DRV8833 nFAULT normal");
            }
        }
        prev = f;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void power_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_log_us = 0;
    uint32_t fails = 0;
    while (1)
    {
        ina219_measurement_t m;
        if (ina219_read_all(s_ina, &m) == ESP_OK)
        {
            g_batt_v = m.bus_voltage_V;
            if (!m.overflow)
            {
                g_batt_ma = m.current_mA;
                g_batt_mw = m.power_mW;
                /* Conteo de coulombs: mAh = mA * horas (solo descarga) */
                if (m.current_mA > 0.0f)
                {
                    g_batt_used_mah += m.current_mA * ((float)INA219_PERIOD_MS / 3600000.0f);
                }
            }
            int64_t now = esp_timer_get_time();
            if (now - last_log_us >= (int64_t)INA219_LOG_PERIOD_MS * 1000)
            {
                last_log_us = now;
                ESP_LOGI(TAG, "INA219: %.2f V  %.0f mA  %.2f W%s", m.bus_voltage_V, m.current_mA,
                         m.power_mW / 1000.0f, m.overflow ? "  (overflow: corriente fuera de rango)" : "");
            }
        }
        else if (++fails % 50 == 1)
        {
            ESP_LOGW(TAG, "INA219: fallo de lectura I2C (%lu en total)", (unsigned long)fails);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(INA219_PERIOD_MS));
    }
}

/* ============================================================
 * Encoders
 * ============================================================ */
static void IRAM_ATTR encoder_right_isr_handler(void *arg)
{
    uint32_t now = (uint32_t)esp_timer_get_time();
    uint32_t dt_us = now - last_time_right;
    if (dt_us > ENCODER_DEBOUNCE_US)
    {
        encoder_count_right++;
        encoder_period_right = dt_us;
        last_time_right = now;
    }
}

static void IRAM_ATTR encoder_left_isr_handler(void *arg)
{
    uint32_t now = (uint32_t)esp_timer_get_time();
    uint32_t dt_us = now - last_time_left;
    if (dt_us > ENCODER_DEBOUNCE_US)
    {
        encoder_count_left++;
        encoder_period_left = dt_us;
        last_time_left = now;
    }
}

static void encoders_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENCODER_RIGHT) | (1ULL << ENCODER_LEFT),
        .pull_up_en = 1,
        .pull_down_en = 0
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_RIGHT, encoder_right_isr_handler, NULL);
    gpio_isr_handler_add(ENCODER_LEFT, encoder_left_isr_handler, NULL);
}

/* ============================================================
 * Utilidades de filtrado, conversion y angulos
 * ============================================================ */
static float low_pass(float prev_filtered, float new_raw, float alpha)
{
    return alpha * new_raw + (1.0f - alpha) * prev_filtered;
}

/* Velocidad (rad/s) a partir del periodo del ultimo pulso. Si desde el
 * ultimo pulso ya paso mas tiempo que ese periodo, la rueda esta frenando
 * y se usa ese tiempo como cota; sin pulsos en 150 ms, rueda parada. */
static float period_to_rad_s(uint32_t period_us, uint32_t last_pulse_us, uint32_t now_us, int ppr)
{
    uint32_t since = now_us - last_pulse_us;
    if (period_us == 0 || since > 150000) return 0.0f;
    float p = (float)((since > period_us) ? since : period_us);
    return (2.0f * (float)M_PI / (float)ppr) / (p * 1e-6f);
}

static float pulses_to_rad_s(int32_t delta_pulses, float window_s, int ppr)
{
    float rev = (float)delta_pulses / (float)ppr;
    return (rev * 2.0f * (float)M_PI) / window_s;
}

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* Cinematica diferencial: (v lineal m/s, w angular rad/s) del
 * robot -> velocidad angular objetivo (rad/s) de cada rueda. */
static void twist_to_wheel_speeds(float v, float w, float *omega_left, float *omega_right)
{
    const float R = (WHEEL_DIAMETER_CM / 100.0f) / 2.0f;
    const float L = WHEEL_DISTANCE_CM / 100.0f;

    float v_left  = v - (w * L / 2.0f);
    float v_right = v + (w * L / 2.0f);

    *omega_left  = v_left  / R;
    *omega_right = v_right / R;
}

static float wrap_pi(float a)
{
    while (a > (float)M_PI)  a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* Yaw en el marco de la ODOMETRIA (0 = orientacion al arrancar o al
 * ultimo reset, positivo = antihorario visto desde arriba). El MPU6050
 * no tiene magnetometro: el rumbo es la integral del giroscopio. */
static float get_yaw_odom(void)
{
    return wrap_pi(g_yaw_gyro);
}

/* El robot esta quieto: no hay ningun comando de movimiento y las
 * ruedas no giran. Se usa para re-estimar el sesgo del giroscopio. */
static bool robot_is_still(void)
{
    if (g_goto_active || g_goto_pending) return false;

    bool cmd_zero;
    if (g_pwm_mode)
    {
        cmd_zero = (fabsf(g_pwm_cmd_left) < 1.0f) && (fabsf(g_pwm_cmd_right) < 1.0f);
    }
    else
    {
        cmd_zero = (fabsf(pid_left.setpoint_rad_s) < 1e-4f) && (fabsf(pid_right.setpoint_rad_s) < 1e-4f);
    }

    return cmd_zero &&
           (g_encoder_data.filtered_left < 0.3f) &&
           (g_encoder_data.filtered_right < 0.3f);
}

/* ============================================================
 * PID de velocidad por rueda (control de bajo nivel)
 * ============================================================ */
static float wheel_pid_update(wheel_pid_t *pid, float dt)
{
    float error = pid->setpoint_rad_s - pid->measured_rad_s;

    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float sp = pid->setpoint_rad_s;
    float sgn = sp > 0.0f ? 1.0f : -1.0f;
    float ff = sgn * (pid->ff_offset + pid->ff_gain * fabsf(sp));
    float output = ff + pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    /* Rueda parada con velocidad pedida: empujon de arranque. Sin esto el
     * PWM subia despacio y la rueda arrancaba de golpe al vencer la
     * friccion estatica (giro "por tramos"). */
    if (fabsf(pid->measured_rad_s) < 0.5f && fabsf(output) < PWM_BREAKAWAY)
    {
        output = sgn * PWM_BREAKAWAY;
    }

    /* Nunca PWM en contra del sentido pedido: si la rueda va rapida se
     * suelta (0), no se invierte (antes daba golpes de reversa de -65). */
    float hi = sp > 0.0f ? MAX_PWM : 0.0f;
    float lo = sp > 0.0f ? 0.0f : MIN_PWM;
    if (output > hi)
    {
        output = hi;
        pid->integral -= error * dt; /* anti-windup */
    }
    if (output < lo)
    {
        output = lo;
        pid->integral -= error * dt;
    }

    return output;
}

#define CONTROL_WINDOW_S       0.02f  /* 50 Hz */
#define ENCODER_FILTER_ALPHA   0.3f
#define WHEEL_SPEED_ALPHA      0.5f   /* filtro de la velocidad por periodo */

/* Correccion de distancia de la odometria. Calibrar con cinta metrica:
 *   ODOM_DIST_SCALE = distancia_real_medida / distancia_que_reporto_el_robot
 * (p. ej. el robot dice 0.90 m y la cinta marca 0.95 m -> 1.055) */
#define ODOM_DIST_SCALE        1.0f

/* ============================================================
 * Mando directo del guante (ESP-NOW)
 * ============================================================ */
typedef struct
{
    glove_packet_t pkt;
    int64_t t_us;          /* llegada del ultimo paquete */
    int rssi;
    uint32_t count;
} glove_rx_t;

static glove_rx_t g_glove_rx = {0};
static portMUX_TYPE g_glove_lock = portMUX_INITIALIZER_UNLOCKED;

/* Estado publicado en teleop/espnow */
static volatile bool  g_glove_active = false;   /* el guante esta conduciendo */
static volatile int   g_glove_link = 0;         /* 0 sin enlace, 1 enlace desarmado, 2 armado */
static volatile float g_glove_v = 0.0f, g_glove_w = 0.0f;
static volatile float g_glove_rate = 0.0f;      /* paquetes por segundo */
static volatile int   g_glove_rssi = 0;
static volatile int   g_glove_guard = 0;        /* 0 libre, 1 zona lenta, 2 borde: solo giro */

/* Telemetria: lo que vio glove_teleop_step en el ultimo ciclo */
static uint32_t g_tl_seq = 0, g_tl_age_ms = 0;
static uint8_t  g_tl_flags = 0;
static float    g_tl_roll = 0.0f, g_tl_pitch = 0.0f, g_tl_v_tgt = 0.0f, g_tl_w_tgt = 0.0f;
static volatile uint32_t g_mpu_fail = 0;        /* lecturas I2C fallidas del MPU6050 */

/* Llamado desde la tarea de WiFi: solo copia */
static void glove_rx_cb(const glove_packet_t *pkt, int rssi)
{
    taskENTER_CRITICAL(&g_glove_lock);
    g_glove_rx.pkt = *pkt;
    g_glove_rx.t_us = esp_timer_get_time();
    g_glove_rx.rssi = rssi;
    g_glove_rx.count++;
    taskEXIT_CRITICAL(&g_glove_lock);
}

/* Zona muerta con histeresis: arranca al pasar dz y solo vuelve a 0 por
 * debajo de dz_exit. Sin esto, con la mano cerca del borde de la zona
 * muerta el comando saltaba 0 <-> minimo varias veces por segundo y el
 * carro giraba "por tramos". Devuelve 0 o un valor con signo en [~0, 1]. */
static float tilt_shape_hyst(float x, float dz, float dz_exit, float full, float expo, bool *active)
{
    float a = fabsf(x);
    if (!*active && a > dz) *active = true;
    else if (*active && a < dz_exit) *active = false;
    if (!*active) return 0.0f;
    float n = a <= dz ? 0.0f : fminf((a - dz) / fmaxf(full - dz, 1e-6f), 1.0f);
    return copysignf(fmaxf(powf(n, expo), 1e-6f), x);
}

/* true si avanzar con (v, w) durante el tiempo de anticipacion acerca el
 * robot al borde del dojo (misma regla que glove_teleop en el PC) */
__attribute__((unused)) static bool arc_leaves_dojo(float x, float y, float th, float v, float w)
{
    const float limit = DOJO_RADIUS_M - DOJO_GUARD_MARGIN_M;
    const int steps = 12;
    const float dt = DOJO_GUARD_LOOKAHEAD_S / steps;
    float r0 = sqrtf(x * x + y * y), r = r0;
    for (int i = 0; i < steps; i++)
    {
        x += v * cosf(th) * dt;
        y += v * sinf(th) * dt;
        th += w * dt;
        r = sqrtf(x * x + y * y);
        if (r > limit && r > r0 + 0.002f) return true;
    }
    return r0 > limit && r > r0 - 0.01f;
}

/* Un ciclo del mando directo (lo llama wheel_control_task a 50 Hz) */
static void glove_teleop_step(float dt)
{
    static float v_out = 0.0f, w_out = 0.0f;
    static bool v_on = false, w_on = false;   /* histeresis de la zona muerta */
    static uint32_t last_count = 0;
    static int64_t last_rate_us = 0;

    glove_rx_t rx;
    taskENTER_CRITICAL(&g_glove_lock);
    rx = g_glove_rx;
    taskEXIT_CRITICAL(&g_glove_lock);

    int64_t now = esp_timer_get_time();
    if (now - last_rate_us >= 1000000)
    {
        g_glove_rate = (float)(rx.count - last_count) * 1e6f / (float)(now - last_rate_us);
        last_count = rx.count;
        last_rate_us = now;
    }

    bool fresh = rx.count > 0 && (now - rx.t_us) < (int64_t)GLOVE_TIMEOUT_MS * 1000;
    bool armed = fresh && (rx.pkt.flags & GLOVE_FLAG_ARMED) && (rx.pkt.flags & GLOVE_FLAG_FUSION_OK);
    g_glove_link = !fresh ? 0 : (armed ? 2 : 1);
    g_glove_rssi = rx.rssi;
    g_tl_seq = rx.pkt.seq;
    g_tl_flags = rx.pkt.flags;
    g_tl_age_ms = rx.count ? (uint32_t)((now - rx.t_us) / 1000) : 99999;
    g_tl_roll = rx.pkt.roll;
    g_tl_pitch = rx.pkt.pitch;
    g_tl_v_tgt = g_tl_w_tgt = 0.0f;

#if GLOVE_DEBUG_LOG
    {
        static int64_t last_log_us = 0;
        if (now - last_log_us >= 1000000)
        {
            last_log_us = now;
            static const char *const link_txt[] = { "SIN SENAL", "desarmado", "ARMADO" };
            if (!fresh)
            {
                ESP_LOGI(TAG, "Guante: %s (canal %u)", link_txt[0], espnow_link_channel());
            }
            else
            {
                ESP_LOGI(TAG, "Guante: %s %.0f paq/s RSSI %d | roll %.1f pitch %.1f deg -> v %.2f m/s w %.2f rad/s%s",
                         link_txt[g_glove_link], g_glove_rate, rx.rssi,
                         rx.pkt.roll * 180.0f / (float)M_PI, rx.pkt.pitch * 180.0f / (float)M_PI,
                         g_glove_v, g_glove_w, g_glove_guard == 2 ? " [BORDE]" : (g_glove_guard == 1 ? " [lento]" : ""));
            }
        }
    }
#endif

    if (!armed || g_goto_active || g_goto_pending)
    {
        if (g_glove_active)
        {
            /* Se perdio el guante o se desarmo: parar ya */
            pid_left.setpoint_rad_s = 0.0f;
            pid_right.setpoint_rad_s = 0.0f;
            ESP_LOGW(TAG, "Guante: %s, robot detenido", fresh ? "desarmado" : "sin senal ESP-NOW");
        }
        g_glove_active = false;
        g_glove_guard = 0;
        v_out = w_out = 0.0f;   /* el integral del lazo de giro se limpia al no moverse */
        v_on = w_on = false;
        g_glove_v = g_glove_w = 0.0f;
        return;
    }

    const float d2r = (float)M_PI / 180.0f;
    const float dz = GLOVE_DEADZONE_DEG * d2r, full = GLOVE_FULL_TILT_DEG * d2r;
    float pitch = GLOVE_INVERT_PITCH ? -rx.pkt.pitch : rx.pkt.pitch;
    float roll  = GLOVE_INVERT_ROLL  ? -rx.pkt.roll  : rx.pkt.roll;
    const float dz_exit = GLOVE_DEADZONE_EXIT_DEG * d2r;
    float sv = tilt_shape_hyst(pitch, dz, dz_exit, full, GLOVE_EXPO, &v_on);
    float sw = tilt_shape_hyst(roll, dz, dz_exit, full, GLOVE_EXPO, &w_on);
    float v = sv == 0.0f ? 0.0f : copysignf(GLOVE_V_MIN + (GLOVE_V_MAX - GLOVE_V_MIN) * fabsf(sv), sv);
    float w = sw == 0.0f ? 0.0f : -copysignf(GLOVE_W_MIN + (GLOVE_W_MAX - GLOVE_W_MIN) * fabsf(sw), sw);
    g_tl_v_tgt = v;
    g_tl_w_tgt = w;

    bool guard_hit = false;
    g_glove_guard = 0;
#if DOJO_GUARD_ENABLED
    if (v != 0.0f)
    {
        float x = g_odom.x, y = g_odom.y, th = g_odom.theta;
        float speed = fmaxf(fabsf(v), fabsf(v_out));   /* g_odom.v es ruidosa: disparaba el borde */
        if (arc_leaves_dojo(x, y, th, copysignf(speed, v), w))
        {
            v = 0.0f;
            guard_hit = true;
            g_glove_guard = 2;
        }
        else if (sqrtf(x * x + y * y) > DOJO_RADIUS_M - DOJO_GUARD_MARGIN_M - 0.05f)
        {
            v = clampf(v, -DOJO_GUARD_SLOW, DOJO_GUARD_SLOW);
            g_glove_guard = 1;
        }
    }
#endif

    float a = fminf(1.0f, dt / fmaxf(GLOVE_SMOOTH_TAU_S, 1e-3f));
    v_out = guard_hit ? 0.0f : v_out + a * (v - v_out);
    w_out += a * (w - w_out);
    /* La cola exponencial del pasa-bajos nunca llega a 0: al volver la
     * mano a neutro se corta, para que el carro pare de verdad */
    if (v == 0.0f && fabsf(v_out) < 0.03f) v_out = 0.0f;
    if (w == 0.0f && fabsf(w_out) < 0.15f) w_out = 0.0f;

    if (!g_glove_active)
    {
        ESP_LOGI(TAG, "Guante armado: mando directo por ESP-NOW (RSSI %d dBm)", rx.rssi);
    }
    g_glove_active = true;
    g_glove_v = v_out;
    g_glove_w = w_out;

    /* Lazo de giro: compara el giro pedido con el que mide el giroscopio */
    static float yaw_i = 0.0f;
    float w_cmd = w_out;
    bool moving = fabsf(v_out) > 0.02f || fabsf(w_out) > 0.05f;
    if (moving && g_yaw_ready)
    {
        float gz = g_imu_data.raw_gz - g_gz_bias;
        float e = w_out - gz;
        yaw_i = clampf(yaw_i + e * dt, -GLOVE_YAW_CORR_MAX / GLOVE_YAW_KI, GLOVE_YAW_CORR_MAX / GLOVE_YAW_KI);
        w_cmd += clampf(GLOVE_YAW_KP * e + GLOVE_YAW_KI * yaw_i, -GLOVE_YAW_CORR_MAX, GLOVE_YAW_CORR_MAX);
    }
    else
    {
        yaw_i = 0.0f;
    }

    float wl, wr;
    twist_to_wheel_speeds(v_out, w_cmd, &wl, &wr);
    /* Si una rueda pasa de su maximo, se escalan las dos: se conserva la
     * curva en vez de recortar solo una rueda */
    float m = fmaxf(fabsf(wl), fabsf(wr));
    if (m > WHEEL_MAX_RAD_S)
    {
        wl *= WHEEL_MAX_RAD_S / m;
        wr *= WHEEL_MAX_RAD_S / m;
    }
    pid_left.setpoint_rad_s = wl;
    pid_right.setpoint_rad_s = wr;
    g_pwm_mode = false;
    g_brake = false;
    g_last_cmd_vel_tick = xTaskGetTickCount();   /* mantiene vivo el watchdog */
}

/* ============================================================
 * Telemetria para analisis (CAR_TELEMETRY): wheel_control_task
 * deja una foto por ciclo en una cola y esta tarea de baja
 * prioridad la imprime, asi el lazo de control nunca espera a la
 * UART. Columnas (encabezado "T,"):
 *  t_ms,seq,age_ms,flags,link,rssi,guard,roll_deg,pitch_deg,
 *  v_tgt,w_tgt,v_out,w_out,sp_l,sp_r,meas_l,meas_r,pwm_l,pwm_r,
 *  x,y,th,gz,mpu_fail,dt_ms
 * ============================================================ */
typedef struct
{
    uint32_t t_ms, seq, age_ms, mpu_fail;
    uint8_t flags, link;
    int8_t rssi, guard;
    float roll, pitch, v_tgt, w_tgt, v_out, w_out;
    float sp_l, sp_r, meas_l, meas_r, pwm_l, pwm_r;
    float x, y, th, gz, dt_ms;
} car_tl_t;

static QueueHandle_t s_tl_queue = NULL;

static void telemetry_task(void *arg)
{
    car_tl_t s;
    const float r2d = 180.0f / (float)M_PI;
    while (1)
    {
        if (xQueueReceive(s_tl_queue, &s, portMAX_DELAY) == pdTRUE)
        {
            printf("T,%lu,%lu,%lu,%u,%u,%d,%d,%.1f,%.1f,%.3f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.3f,%.3f,%.3f,%.3f,%lu,%.1f\n",
                   (unsigned long)s.t_ms, (unsigned long)s.seq, (unsigned long)s.age_ms, s.flags, s.link,
                   s.rssi, s.guard, s.roll * r2d, s.pitch * r2d, s.v_tgt, s.w_tgt, s.v_out, s.w_out,
                   s.sp_l, s.sp_r, s.meas_l, s.meas_r, s.pwm_l, s.pwm_r, s.x, s.y, s.th, s.gz,
                   (unsigned long)s.mpu_fail, s.dt_ms);
        }
    }
}

/* ============================================================
 * Control de ruedas + odometria
 *  - Watchdog de cmd_vel: sin goto en curso, si no llega cmd_vel
 *    en CMD_VEL_TIMEOUT_MS se paran los motores (si se cae el
 *    WiFi el robot no sigue andando solo).
 *  - Modo velocidad: PID por rueda, parada dura si ambos setpoints
 *    son 0.
 *  - Modo PWM: PWM directo a cada motor, sin PID.
 *  - Odometria: distancia por encoders, angulo por el IMU.
 * ============================================================ */
void wheel_control_task(void *pvParameters)
{
    const float R = (WHEEL_DIAMETER_CM / 100.0f) / 2.0f;
    const float L = WHEEL_DISTANCE_CM / 100.0f;

    int32_t prev_left = 0, prev_right = 0;
    float filtered_left = 0.0f, filtered_right = 0.0f;
    float dir_l = 1.0f, dir_r = 1.0f;   /* ultimo sentido comandado */

    TickType_t last_wake = xTaskGetTickCount();
    while (1)
    {
        /* --- Reset del cero pedido por /goto_cmd modo 2 --- */
        if (g_reset_odom_req)
        {
            g_odom.x = 0.0f;
            g_odom.y = 0.0f;
            g_odom.theta = 0.0f;    
            g_reset_odom_req = false;
        }

        /* --- Mando directo del guante (ESP-NOW) --- */
        glove_teleop_step(CONTROL_WINDOW_S);

        /* --- Watchdog de cmd_vel --- */
        if (!g_goto_active && !g_goto_pending &&
            (xTaskGetTickCount() - g_last_cmd_vel_tick) > pdMS_TO_TICKS(CMD_VEL_TIMEOUT_MS))
        {
            g_pwm_cmd_left = 0.0f;
            g_pwm_cmd_right = 0.0f;
            pid_left.setpoint_rad_s = 0.0f;
            pid_right.setpoint_rad_s = 0.0f;
        }

        int32_t cur_left = encoder_count_left;
        int32_t cur_right = encoder_count_right;

        int32_t delta_left = cur_left - prev_left;
        int32_t delta_right = cur_right - prev_right;
        prev_left = cur_left;
        prev_right = cur_right;

        float raw_left  = pulses_to_rad_s(delta_left,  CONTROL_WINDOW_S, PULSES_PER_REV_L);
        float raw_right = pulses_to_rad_s(delta_right, CONTROL_WINDOW_S, PULSES_PER_REV_R);

        filtered_left  = low_pass(filtered_left,  raw_left,  ENCODER_FILTER_ALPHA);
        filtered_right = low_pass(filtered_right, raw_right, ENCODER_FILTER_ALPHA);

        /* Velocidad por periodo entre pulsos: no tiene el escalon de 15.7
         * rad/s que da contar 0 o 1 pulso en 20 ms; es la que usa el PID */
        {
            uint32_t now_us = (uint32_t)esp_timer_get_time();
            float sl = period_to_rad_s(encoder_period_left, last_time_left, now_us, PULSES_PER_REV_L);
            float sr = period_to_rad_s(encoder_period_right, last_time_right, now_us, PULSES_PER_REV_R);
            g_wheel_speed_l = low_pass(g_wheel_speed_l, sl, WHEEL_SPEED_ALPHA);
            g_wheel_speed_r = low_pass(g_wheel_speed_r, sr, WHEEL_SPEED_ALPHA);
        }

        float pwm_left, pwm_right;

        if (g_pwm_mode)
        {
            pwm_left  = g_pwm_cmd_left;
            pwm_right = g_pwm_cmd_right;

            if (pwm_left  > 0.0f) dir_l = 1.0f; else if (pwm_left  < 0.0f) dir_l = -1.0f;
            if (pwm_right > 0.0f) dir_r = 1.0f; else if (pwm_right < 0.0f) dir_r = -1.0f;

            pid_left.integral = 0.0f;
            pid_right.integral = 0.0f;
            pid_left.prev_error = 0.0f;
            pid_right.prev_error = 0.0f;
        }
        else
        {
            bool stop_cmd = (fabsf(pid_left.setpoint_rad_s) < 1e-4f) &&
                            (fabsf(pid_right.setpoint_rad_s) < 1e-4f);

            static int idle_cycles = 0;
            if (stop_cmd)
            {
                /* Nadie comanda las ruedas: si giran es que alguien mueve el
                 * carro a mano. Pasada la frenada, el sentido lo da el IMU. */
                if (idle_cycles < PUSH_IDLE_CYCLES) idle_cycles++;
                bool turning = filtered_left > 0.5f || filtered_right > 0.5f;
                if (idle_cycles >= PUSH_IDLE_CYCLES && turning)
                {
                    /* Solo el retroceso claro cambia el sentido; si no, adelante */
                    float d = (g_push_v < -PUSH_BACK_V) ? -1.0f : 1.0f;
                    dir_l = d;
                    dir_r = d;
                }
                pwm_left = 0.0f;
                pwm_right = 0.0f;
                pid_left.integral = 0.0f;
                pid_right.integral = 0.0f;
                pid_left.prev_error = 0.0f;
                pid_right.prev_error = 0.0f;
            }
            else
            {
                if (pid_left.setpoint_rad_s  > 0.0f) dir_l = 1.0f; else if (pid_left.setpoint_rad_s  < 0.0f) dir_l = -1.0f;
                if (pid_right.setpoint_rad_s > 0.0f) dir_r = 1.0f; else if (pid_right.setpoint_rad_s < 0.0f) dir_r = -1.0f;

                idle_cycles = 0;
                pid_left.measured_rad_s  = g_wheel_speed_l * dir_l;
                pid_right.measured_rad_s = g_wheel_speed_r * dir_r;

                /* Curva abierta: la rueda interior pide menos del minimo
                 * pero en el mismo sentido que la exterior. Antes se apagaba
                 * y el carro pivotaba sobre ella (pedia -1.9 rad/s de giro y
                 * giraba -2.5); ahora se sostiene en el minimo y el lazo de
                 * giro con el giroscopio corrige lo que falte de curva. */
                {
                    float sl = pid_left.setpoint_rad_s, sr = pid_right.setpoint_rad_s;
                    if (sl * sr > 0.0f && fmaxf(fabsf(sl), fabsf(sr)) >= WHEEL_MIN_SETPOINT)
                    {
                        if (fabsf(sl) < WHEEL_MIN_SETPOINT) pid_left.setpoint_rad_s  = copysignf(WHEEL_MIN_SETPOINT, sl);
                        if (fabsf(sr) < WHEEL_MIN_SETPOINT) pid_right.setpoint_rad_s = copysignf(WHEEL_MIN_SETPOINT, sr);
                    }
                }

                /* Setpoint demasiado lento para mover la rueda: 0 y sin
                 * integral (antes quedaba zumbando con PWM 11-45 sin girar) */
                if (fabsf(pid_left.setpoint_rad_s) < WHEEL_MIN_SETPOINT)
                {
                    pwm_left = 0.0f;
                    pid_left.integral = 0.0f;
                    pid_left.prev_error = 0.0f;
                }
                else
                {
                    pwm_left = wheel_pid_update(&pid_left, CONTROL_WINDOW_S);
                }
                if (fabsf(pid_right.setpoint_rad_s) < WHEEL_MIN_SETPOINT)
                {
                    pwm_right = 0.0f;
                    pid_right.integral = 0.0f;
                    pid_right.prev_error = 0.0f;
                }
                else
                {
                    pwm_right = wheel_pid_update(&pid_right, CONTROL_WINDOW_S);
                }
            }
        }

        if (g_brake)
        {
            car_brake();
        }
        else
        {
            motor_left_set((int8_t)pwm_left);
            motor_right_set((int8_t)pwm_right);
        }

        /* --- Odometria: distancia por encoders, angulo por IMU --- */
        float vl = filtered_left  * dir_l * R;   /* m/s */
        float vr = filtered_right * dir_r * R;
        float v  = 0.5f * (vl + vr) * ODOM_DIST_SCALE;
        float w  = (vr - vl) / L;

        if (g_yaw_ready)
        {
            float yaw = get_yaw_odom();
            float th_mid = g_odom.theta + 0.5f * wrap_pi(yaw - g_odom.theta);
            g_odom.x += v * cosf(th_mid) * CONTROL_WINDOW_S;
            g_odom.y += v * sinf(th_mid) * CONTROL_WINDOW_S;
            g_odom.theta = yaw;
            g_odom.w = g_imu_data.raw_gz - g_gz_bias;
        }
        else
        {
            float th_mid = g_odom.theta + 0.5f * w * CONTROL_WINDOW_S;
            g_odom.x     += v * cosf(th_mid) * CONTROL_WINDOW_S;
            g_odom.y     += v * sinf(th_mid) * CONTROL_WINDOW_S;
            g_odom.theta = wrap_pi(g_odom.theta + w * CONTROL_WINDOW_S);
            g_odom.w = w;
        }
        g_odom.v = v;

        g_encoder_data.raw_left = raw_left;
        g_encoder_data.raw_right = raw_right;
        g_encoder_data.filtered_left = filtered_left;
        g_encoder_data.filtered_right = filtered_right;

        g_left_pos += (int32_t)delta_left * (int32_t)dir_l;

#if CAR_TELEMETRY
        if (s_tl_queue != NULL)
        {
            static int64_t prev_tl_us = 0;
            int64_t tl_us = esp_timer_get_time();
            car_tl_t s = {
                .t_ms = (uint32_t)(tl_us / 1000), .seq = g_tl_seq, .age_ms = g_tl_age_ms,
                .mpu_fail = g_mpu_fail, .flags = g_tl_flags, .link = (uint8_t)g_glove_link,
                .rssi = (int8_t)g_glove_rssi, .guard = (int8_t)g_glove_guard,
                .roll = g_tl_roll, .pitch = g_tl_pitch, .v_tgt = g_tl_v_tgt, .w_tgt = g_tl_w_tgt,
                .v_out = g_glove_v, .w_out = g_glove_w,
                .sp_l = pid_left.setpoint_rad_s, .sp_r = pid_right.setpoint_rad_s,
                .meas_l = g_wheel_speed_l * dir_l, .meas_r = g_wheel_speed_r * dir_r,
                .pwm_l = g_brake ? 0.0f : pwm_left, .pwm_r = g_brake ? 0.0f : pwm_right,
                .x = g_odom.x, .y = g_odom.y, .th = g_odom.theta, .gz = g_odom.w,
                .dt_ms = prev_tl_us ? (float)(tl_us - prev_tl_us) * 1e-3f : 0.0f,
            };
            prev_tl_us = tl_us;
            xQueueSend(s_tl_queue, &s, 0);   /* si la cola esta llena se pierde la foto, no el control */
        }
#endif

        /* Prueba a mano: con el robot sin comandos, girar una rueda
         * exactamente 1 vuelta debe sumar 40 pulsos en ESA rueda. */
        {
            static int32_t shown_l = 0, shown_r = 0;
            static TickType_t last_show = 0;
            bool no_cmd = !g_goto_active && !g_goto_pending &&
                          (xTaskGetTickCount() - g_last_cmd_vel_tick) > pdMS_TO_TICKS(1000);
            if (no_cmd && (cur_left != shown_l || cur_right != shown_r) &&
                (xTaskGetTickCount() - last_show) > pdMS_TO_TICKS(500))
            {
                ESP_LOGI(TAG, "Encoders (a mano): izq=%ld der=%ld pulsos", (long)cur_left, (long)cur_right);
                shown_l = cur_left;
                shown_r = cur_right;
                last_show = xTaskGetTickCount();
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS((int)(CONTROL_WINDOW_S * 1000)));
    }
}

/* ============================================================
 * Tarea de IMU (MPU6050)
 *  - Lee acelerometro y giroscopio en una sola transaccion I2C.
 *  - Al arrancar promedia el giroscopio 1 s (robot quieto) para
 *    estimar su sesgo en los 3 ejes, y fija el cero de rumbo.
 *  - Integra gz para el rumbo. Con el robot quieto no integra el
 *    ruido y sigue afinando el sesgo, asi el rumbo no deriva.
 *  - Roll/pitch con filtro complementario: a corto plazo manda el
 *    giroscopio y a largo plazo el acelerometro (direccion de la
 *    gravedad) corrige su deriva. Con roll/pitch se resta la
 *    gravedad para obtener la aceleracion lineal (el BNO055 lo
 *    hacia dentro del chip; el MPU6050 no tiene fusion).
 * ============================================================ */
#define IMU_WINDOW_S          0.02f  /* 50 Hz */
#define GYRO_BIAS_SAMPLES     50     /* 1 s a 50 Hz */
#define GYRO_STILL_DEADBAND   0.02f  /* rad/s: por debajo, quieto = no se integra */
#define GYRO_BIAS_ALPHA       0.01f  /* EMA del sesgo con el robot quieto */
#define GYRO_FILTER_ALPHA     0.1f   /* pasa-bajos del giroscopio en /imu/filtered (corte ~0.9 Hz a 50 Hz) */
#define ACC_FILTER_ALPHA      0.1f   /* pasa-bajos de la aceleracion lineal en /imu/filtered (corte ~0.9 Hz) */
#define COMP_FILTER_TAU_S     0.5f   /* s: constante de tiempo del filtro complementario */
#define ACC_TRUST_BAND        0.15f  /* el acelerometro corrige roll/pitch solo si |a| = g +-15 % */
#define GRAVITY_M_S2          9.80665f

void imu_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    int bias_count = 0;
    float bias_acc_x = 0.0f, bias_acc_y = 0.0f, bias_acc_z = 0.0f;
    float roll = 0.0f, pitch = 0.0f;
    int64_t last_us = esp_timer_get_time();
    uint32_t i2c_fail_count = 0;
    mpu6050_data_t d;

    while (1)
    {
        esp_err_t ret = mpu6050_read(&imu, &d);

        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us = now_us;
        if (dt > 0.1f) dt = 0.1f;   /* por si una lectura I2C se atrasa */

        if (ret != ESP_OK)
        {
            /* Sin lectura este ciclo (ruido de los motores en el I2C): se
             * integra el ultimo giro valido para no perderlo mientras se
             * recupera el bus. */
            if (g_yaw_ready && !robot_is_still())
            {
                g_yaw_gyro = wrap_pi(g_yaw_gyro + (g_imu_data.raw_gz - g_gz_bias) * dt);
            }
            g_mpu_fail = ++i2c_fail_count;
            if (i2c_fail_count % 50 == 1)
            {
                ESP_LOGW(TAG, "MPU6050: fallo de lectura I2C (%lu en total)",
                         (unsigned long)i2c_fail_count);
            }
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS((int)(IMU_WINDOW_S * 1000)));
            continue;
        }

#if IMU_MOUNTED_UPSIDE_DOWN
        /* Giro de 180 grados sobre X: Z vuelve a apuntar hacia arriba */
        d.ay = -d.ay; d.az = -d.az;
        d.gy = -d.gy; d.gz = -d.gz;
#endif

        g_imu_data.raw_ax = d.ax;
        g_imu_data.raw_ay = d.ay;
        g_imu_data.raw_az = d.az;
        g_imu_data.raw_gx = d.gx;
        g_imu_data.raw_gy = d.gy;
        g_imu_data.raw_gz = d.gz;

        /* Roll/pitch que indica la gravedad (validos con el robot quieto
         * o a velocidad constante) */
        float roll_acc  = atan2f(d.ay, d.az);
        float pitch_acc = atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az));

        if (!g_yaw_ready)
        {
            /* Estimacion inicial del sesgo con el robot quieto */
            bias_acc_x += d.gx;
            bias_acc_y += d.gy;
            bias_acc_z += d.gz;
            roll  = roll_acc;
            pitch = pitch_acc;
            if (++bias_count >= GYRO_BIAS_SAMPLES)
            {
                g_gx_bias = bias_acc_x / (float)bias_count;
                g_gy_bias = bias_acc_y / (float)bias_count;
                g_gz_bias = bias_acc_z / (float)bias_count;
                g_yaw_gyro = 0.0f;
                g_odom.theta = 0.0f;
                g_yaw_ready = true;
                ESP_LOGI(TAG, "Rumbo en cero. Sesgo giroscopio=(%.4f, %.4f, %.4f) rad/s, roll=%.1f pitch=%.1f deg",
                         g_gx_bias, g_gy_bias, g_gz_bias,
                         roll * 180.0f / (float)M_PI, pitch * 180.0f / (float)M_PI);
            }
        }
        else
        {
            if (g_reset_yaw_req)
            {
                g_yaw_gyro = 0.0f;
                g_reset_yaw_req = false;
                ESP_LOGI(TAG, "Cero de rumbo y odometria reiniciado");
            }

            float gx = d.gx - g_gx_bias;
            float gy = d.gy - g_gy_bias;
            float rate = d.gz - g_gz_bias;
            bool still = robot_is_still();

            if (still && fabsf(rate) < 0.05f)
            {
                /* Afinar el sesgo solo con el robot realmente quieto */
                g_gx_bias = (1.0f - GYRO_BIAS_ALPHA) * g_gx_bias + GYRO_BIAS_ALPHA * d.gx;
                g_gy_bias = (1.0f - GYRO_BIAS_ALPHA) * g_gy_bias + GYRO_BIAS_ALPHA * d.gy;
                g_gz_bias = (1.0f - GYRO_BIAS_ALPHA) * g_gz_bias + GYRO_BIAS_ALPHA * d.gz;
            }

            if (!(still && fabsf(rate) < GYRO_STILL_DEADBAND))
            {
                g_yaw_gyro = wrap_pi(g_yaw_gyro + rate * dt);
            }

            /* Filtro complementario: prediccion con el giroscopio
             * (cinematica de angulos de Euler) ... */
            float sr = sinf(roll), cr = cosf(roll);
            float tp = tanf(pitch);
            roll  += (gx + sr * tp * gy + cr * tp * rate) * dt;
            pitch += (cr * gy - sr * rate) * dt;

            /* ... y correccion con el acelerometro, solo si mide ~1 g
             * (si el robot acelera o choca, la gravedad no es fiable) */
            float a_norm = sqrtf(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
            if (fabsf(a_norm - GRAVITY_M_S2) < ACC_TRUST_BAND * GRAVITY_M_S2)
            {
                float k = dt / (COMP_FILTER_TAU_S + dt);
                roll  = wrap_pi(roll + k * wrap_pi(roll_acc - roll));
                pitch += k * (pitch_acc - pitch);
            }
        }

        /* Aceleracion lineal = medida - gravedad expresada en el cuerpo,
         * + pasa-bajos EMA */
        float sr = sinf(roll), cr = cosf(roll);
        float sp = sinf(pitch), cp = cosf(pitch);
        float lin_x = d.ax - GRAVITY_M_S2 * (-sp);
        float lin_y = d.ay - GRAVITY_M_S2 * (sr * cp);
        float lin_z = d.az - GRAVITY_M_S2 * (cr * cp);
        g_imu_data.lin_ax += ACC_FILTER_ALPHA * (lin_x - g_imu_data.lin_ax);

        /* Velocidad de empuje (solo su signo importa): integrador con fuga */
        {
            float a_fwd = g_imu_x_sign * lin_x;
            if (fabsf(a_fwd) < PUSH_ACC_DEADBAND) a_fwd = 0.0f;
            g_push_v += (a_fwd - g_push_v / PUSH_V_TAU_S) * dt;
        }
        g_imu_data.lin_ay += ACC_FILTER_ALPHA * (lin_y - g_imu_data.lin_ay);
        g_imu_data.lin_az += ACC_FILTER_ALPHA * (lin_z - g_imu_data.lin_az);

        /* Giroscopio filtrado para /imu/filtered: sin sesgo + pasa-bajos EMA */
        g_imu_data.f_gx += GYRO_FILTER_ALPHA * ((d.gx - g_gx_bias) - g_imu_data.f_gx);
        g_imu_data.f_gy += GYRO_FILTER_ALPHA * ((d.gy - g_gy_bias) - g_imu_data.f_gy);
        g_imu_data.f_gz += GYRO_FILTER_ALPHA * ((d.gz - g_gz_bias) - g_imu_data.f_gz);

        /* Orientacion (roll, pitch, yaw) -> quaternion, convencion ZYX */
        float yaw = get_yaw_odom();
        float cr2 = cosf(0.5f * roll),  sr2 = sinf(0.5f * roll);
        float cp2 = cosf(0.5f * pitch), sp2 = sinf(0.5f * pitch);
        float cy2 = cosf(0.5f * yaw),   sy2 = sinf(0.5f * yaw);
        g_imu_data.qw = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
        g_imu_data.qx = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
        g_imu_data.qy = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
        g_imu_data.qz = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
        g_imu_data.roll  = roll;
        g_imu_data.pitch = pitch;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS((int)(IMU_WINDOW_S * 1000)));
    }
}

/* ============================================================
 * Navegacion a un punto
 *
 * Destino: absoluto (marco del cero) o RELATIVO = desplazamiento
 * (dx, dy) desde la posicion actual, expresado en los ejes del
 * mismo cero. Asi, mandar dos veces (0.9, 0) relativo avanza 1.8 m
 * en la direccion en que el robot arranco.
 *
 * Fase 1: giro en el sitio solo si el error de rumbo es grande.
 *         Si la rueda no vence la friccion, el PWM se va subiendo
 *         (refuerzo). Si aun asi no gira, se aborta en vez de
 *         quedarse 15 s intentando.
 * Fase 2: seguimiento de linea guiado por el rumbo del IMU.
 * ============================================================ */
#define GOTO_ANGLE_TOL          0.03f   /* rad (~1.7 grados) */
#define GOTO_DIST_TOL           0.02f   /* m */
#define GOTO_SKIP_TURN          0.15f   /* rad (~8.6 grados): menos que esto se corrige andando */
/*
 * Giro en el sitio en dos etapas (sirve para cualquier cuadrante):
 *  1) Giro continuo hasta entrar en la zona fina. Se corta ANTES segun
 *     la velocidad de giro (el robot sigue girando un poco al frenar).
 *  2) Freno + pausa, y ajuste fino con pulsos cortos (giro-freno-medir)
 *     hasta quedar dentro de la tolerancia. Con la friccion del robot un
 *     giro continuo lento no existe: o no se mueve o se pasa; los pulsos
 *     si dan pasos pequenos y controlados.
 *  Al terminar: freno y pausa quieto antes de avanzar.
 * Cada rueda tiene su propio valor: con el DRV8833 la derecha necesita
 * ~6 PWM mas que la izquierda para la misma velocidad (ver PWM_FF_*_R).
 * Con el L298N eran 85/72 y 92/80; con el DRV8833 eso hacia que la
 * alineacion final oscilara +-5 grados (cada pulso se pasaba).
 */
#define GOTO_TURN_PWM_L         38.0f   /* giro continuo */
#define GOTO_TURN_PWM_R         44.0f
#define GOTO_TURN_BOOST_STEP    3.0f    /* PWM extra si no gira */
#define GOTO_TURN_BOOST_MAX     12.0f
#define GOTO_TURN_STALL_MS      300
#define GOTO_TURN_FINE_ZONE     0.26f   /* rad (~15 grados): a partir de aqui, pulsos */
#define GOTO_TURN_LEAD_S        0.12f   /* s: corta antes segun la velocidad de giro */
#define GOTO_TURN_TOL           0.05f   /* rad (~3 grados): el resto lo corrige el avance */
#define GOTO_PULSE_PWM_L        42.0f   /* pulsos: fuerte para vencer la friccion */
#define GOTO_PULSE_PWM_R        48.0f
#define GOTO_PULSE_MS_PER_DEG   5.0f    /* duracion del pulso segun el error */
#define GOTO_PULSE_MIN_MS       40
#define GOTO_PULSE_MAX_MS       160
#define GOTO_PULSE_GROW_MS      20      /* si un pulso no movio el robot, el siguiente dura mas */
#define GOTO_PULSE_MAX_COUNT    20
#define GOTO_TURN_BRAKE_MS      200
#define GOTO_TURN_SETTLE_MS     250     /* espera tras frenar antes de medir */
#define GOTO_TURN_PAUSE_MS      500     /* quieto tras el giro, antes de avanzar */
#define GOTO_TURN_TIMEOUT_MS    8000

/* Alineacion final al rumbo de inicio. Pulsos mas cortos que en el giro
 * inicial: uno de 48 ms giraba ~9 grados, demasiado grueso para dejarlo
 * recto. Si un pulso corto no vence la friccion, crece de a poco. */
/* Complementa la aproximacion (GOTO_ALIGN_DIST): en los ultimos ~16 cm los
 * motores van cortados y el frenado cambiaba el rumbo de -6 a +5 grados sin
 * control; esto corrige solo ese resto. */
#define GOTO_FINAL_ALIGN        1
#define GOTO_FINAL_ALIGN_TOL    0.035f  /* rad (~2 grados) */
#define GOTO_FINAL_PULSE_MIN_MS 20
#define GOTO_FINAL_PULSE_GROW_MS 8

/*
 * Control de rumbo en cascada (GOTO_USE_WHEEL_SYNC = 1, recomendado):
 *   lazo externo  : error de rumbo (IMU) -> diferencia de velocidad de
 *                   ruedas deseada  dw = w_der - w_izq  (rad/s)
 *   lazo interno  : giroscopio -> PWM diferencial para que las ruedas
 *                   cumplan esa diferencia. Su integral ES el
 *                   "trim" de la rueda izquierda (cuanto PWM extra
 *                   necesita), se guarda en NVS y sobrevive reinicios.
 * El lazo interno reacciona en decenas de ms cuando la rueda izquierda se
 * traba, antes de que el robot llegue a girar; el externo solo corrige lo
 * que quede. Con 0 se usa el PID de rumbo directo anterior.
 */
#define GOTO_USE_WHEEL_SYNC     1

/*
 * Compensacion por vuelta de la rueda izquierda (aprendizaje iterativo):
 * la rueda tiene un punto duro que se repite cada vuelta (el robot
 * oscilaba con el mismo periodo que una vuelta de rueda). Se guarda una
 * tabla de 20 correcciones, una por pulso del encoder izquierdo: cada vez
 * que la rueda pasa por un tramo, se corrige el PWM que ese tramo necesito
 * la vuelta anterior, adelantado para compensar el retardo del motor.
 * Mejora trayecto a trayecto. Se reinicia al apagar (el angulo de la rueda
 * no se conoce tras un reinicio).
 */
#define GOTO_USE_REV_FF         0       /* desactivado: los encoders no cuentan bien (izq 122 / der 56 pulsos en el mismo trayecto) */
#define REV_FF_GAIN             0.4f    /* aprendizaje por ciclo (1.0 puede inestabilizar) */
#define REV_FF_DELAY            3       /* pulsos: retardo entre accion y efecto medido */
#define REV_FF_LEAD             2       /* pulsos: adelanto al aplicar */
#define REV_FF_MAX              15.0f   /* PWM maximo por tramo */
#define REV_FF_MIN_SPEED        3.0f    /* rad/s de rueda izquierda para aprender */

/* El lazo interno (giroscopio -> correccion -> ruedas -> giroscopio) oscilaba
 * solo: con KP*(HEADING_KD + L/R) = 14.5 PWM por rad/s y ~0.1-0.15 s de
 * retraso de los motores, la correccion iba de -30 a +30 cada ~0.2 s siguiendo
 * a gz. Se baja esa ganancia a ~5.5 y se sube HEADING_K para mantener la
 * rigidez de rumbo (~28 PWM/rad contando el integrador). */
/* Se probo 20: no mejoro el error (media 5.0 cm frente a 4.3 cm con 14),
 * subieron los vaivenes a ~19 grados y el trim dejo de ser estable (+3 a +10
 * en un mismo trayecto, porque el integrador acumula KI*K*error). */
/* DRV8833: con 14 un error de 5-8 grados sostenido 1 s solo pedia ~1 PWM y
 * el robot terminaba 3-5 cm a la izquierda (dos trayectos de 0.9 m). */
#define GOTO_SYNC_HEADING_K     24.0f   /* (rad/s de rueda) por rad de error de rumbo */
#define GOTO_SYNC_HEADING_KD    0.5f    /* (rad/s de rueda) por rad/s del giroscopio */
/* Con KP=1.5, 4 rad/s limitaba la correccion de rumbo a ~6 PWM: un error de
 * 15 grados duraba medio segundo y el robot se iba 5-10 cm de lado. Con 12
 * llega a ~18 PWM; solo actua con errores de mas de ~16 grados. */
#define GOTO_SYNC_DW_MAX        12.0f   /* rad/s, diferencia maxima pedida */
/* Con KI=20 el integrador dominaba: equivalia a Kp~100 PWM/rad de rumbo con
 * poco amortiguamiento, y el robot oscilaba +-15-30 grados (~2 Hz) en cada
 * trayecto; el trim acababa en +15, -16, +13 segun la fase de la oscilacion.
 * Ahora el integrador es lento (solo trim) y el proporcional amortigua.
 * Ojo: el integrador integra el giroscopio, y la integral del giroscopio
 * ES el rumbo; asi que KI*(HEADING_KD + L/R) actua como un Kp oculto sobre
 * el rumbo, sin amortiguamiento. Con KI=6 seguia en zigzag (+-10-20 grados
 * cada ~0.45 s).
 * KP 5 -> 3.5 -> 1.5: con 5 y 3.5 la correccion seguia yendo de tope a tope.
 * Equivalente ahora: Kp~28 PWM/rad de rumbo, ~5.5 PWM por rad/s de gz.
 * DRV8833: el motor da ~5.3 veces mas rad/s por PWM que con el L298N, asi
 * que las ganancias en PWM se dividen por ~5 para conservar el mismo lazo
 * (KP 1.5 -> 0.3, KI 2.0 -> 0.4, correccion 30 -> 8, rampa 8 -> 1.5).
 * La asimetria fija entre ruedas ya no la carga el trim: la compensa el
 * PWM base por rueda (GOTO_BASE_FF_*), el trim solo guarda el resto. */
/* 0.3/0.4 (escalado directo desde el L298N) quedaba blando: 0.6/0.8 */
#define GOTO_SYNC_KP            0.6f    /* PWM por rad/s de error de diferencia */
#define GOTO_SYNC_KI            0.8f    /* PWM por (rad/s * s) */
#define GOTO_SYNC_TRIM_MAX      8.0f    /* PWM maximo de trim */

/* PID de rumbo directo (solo si GOTO_USE_WHEEL_SYNC = 0) */
#define GOTO_HEADING_KP         25.0f
#define GOTO_HEADING_KI         20.0f
#define GOTO_HEADING_KD         5.0f
#define GOTO_HEADING_I_MAX      15.0f

#define GOTO_MAX_CORRECTION     8.0f    /* ~6 rad/s por rueda (con el L298N 30 PWM eran ~4.3) */
#define GOTO_MAX_HEADING_ERR    1.05f   /* rad (~60 grados) */
/* Con 4 la correccion llegaba tarde: el termino derivativo pedia ~8 PWM por
 * ciclo cuando gz cambiaba de +2.5 a -1.6 rad/s en 0.2 s, y ese retraso
 * sostenia un zigzag de +-10 grados cada ~0.4 s. */
#define GOTO_CORR_SLEW          1.5f    /* PWM por ciclo de 20 ms */

#define GOTO_CROSS_K            2.0f
#define GOTO_CROSS_MAX_ANGLE    0.17f

/* Con 0.15 nunca frenaba: los motores se cortan a ~0.18 m (inercia). Con
 * 0.35 baja la velocidad antes del corte, rueda menos sin control y hay mas
 * tiempo para cuadrar el rumbo. */
#define GOTO_SLOW_DIST          0.35f
/* Aproximacion final: en los ultimos GOTO_ALIGN_DIST el rumbo deseado pasa
 * de "volver a la linea" al rumbo con el que empezo el goto, para llegar
 * cuadrado (antes la correccion lateral lo dejaba apuntando hasta 10 grados). */
#define GOTO_ALIGN_DIST         0.35f
#define GOTO_COAST_K_INIT       0.08f   /* s: con freno activo la inercia es mucho menor; se reaprende sola */
#define GOTO_COAST_K_MIN        0.05f
#define GOTO_COAST_K_MAX        0.50f
#define GOTO_COAST_LEARN        0.6f    /* cuanto se ajusta k por trayecto (0.3 tardaba en converger) */
/* Piso de PWM por rueda en avance (prueba en el piso): la izquierda no
 * arranca con 60 % y la derecha no con 40 %. Por debajo de estos valores
 * una rueda se detiene y el robot gira sobre ella. */
/* Antes 68/52: eran pisos de ARRANQUE, pero el arranque ya lo resuelve el
 * empujon (GOTO_KICK_*). Con 68 la izquierda solo podia bajar 2 PWM desde la
 * base 70, la correccion se saturaba y el robot zigzagueaba; ademas un PWM
 * de 50 desde ROS se convertia en 68/52 y no cambiaba nada. */
/* 58 -> 45 -> 52: en las derivas largas la izquierda quedaba clavada en 58,
 * pero en la prueba en el aire ninguna rueda gira con PWM 40, asi que 45 esta
 * al borde de detenerse con el peso del robot. 52 deja margen a las dos. */
/* DRV8833: con 52 la izquierda ya iba a ~25 rad/s y quedaba clavada en el
 * piso todo el trayecto (el lazo no podia frenarla y el robot se fue 20
 * grados a la derecha). Pisos nuevos = ~9-10 rad/s en cada rueda segun la
 * caracterizacion, por encima del PWM de arranque (izq 24, der 25). */
#define GOTO_WHEEL_PWM_FLOOR_L  25.0f
#define GOTO_WHEEL_PWM_FLOOR_R  30.0f
#define GOTO_WHEEL_PWM_MAX_L    100.0f
#define GOTO_WHEEL_PWM_MAX_R    100.0f
/* Arranque: la friccion estatica es mayor que la de rodadura. Durante los
 * primeros ms se da un empujon para que ambas ruedas despeguen a la vez. */
#define GOTO_KICK_MS            150
#define GOTO_BRAKE_MS           300     /* freno activo al llegar */
/* Con 75 la derecha seguia parada (wR=0) mientras la izquierda ya giraba:
 * el robot se iba ~15-20 grados a la derecha en cada arranque. Con 90/90
 * aun giraba 9-15 grados a la derecha, por eso la izquierda baja a 80.
 * DRV8833: 80/90 hacia girar 18 grados a la derecha en 0.3 s. Ahora ~16
 * rad/s en las dos ruedas segun la recta de cada una (izq 33, der 40). */
#define GOTO_KICK_PWM_L         33.0f
#define GOTO_KICK_PWM_R         40.0f
/* El PWM base del goto se interpreta como el de una rueda "media" (recta
 * promedio de las dos) y se convierte al que necesita cada rueda para ir a
 * la misma velocidad: con base 35 da izq 31 / der 39. */
#define GOTO_BASE_FF_OFFSET     ((PWM_FF_OFFSET_L + PWM_FF_OFFSET_R) / 2.0f)
#define GOTO_BASE_FF_GAIN       ((PWM_FF_GAIN_L + PWM_FF_GAIN_R) / 2.0f)

#define GOTO_TIMEOUT_MS         15000
#define GOTO_STALL_CHECK_MS     1500
#define GOTO_STALL_MIN_PROGRESS 0.02f

#define GOTO_LOOP_MS            20
#define GOTO_DT                 (GOTO_LOOP_MS / 1000.0f)

static float s_learned_integral = 0.0f;

/* Trim de la rueda izquierda (PWM): positivo = la DERECHA recibe mas.
 * Se guarda en NVS para que el primer trayecto tras encender ya salga
 * recto. */
static float s_sync_trim = 0.0f;
#if GOTO_USE_REV_FF
static float s_rev_ff[PULSES_PER_REV] = {0};
#endif
static float s_sync_trim_saved = 0.0f;

/* Inercia aprendida: distancia que rueda tras cortar motores = k * v */
static float s_coast_k = GOTO_COAST_K_INIT;
static float s_coast_k_saved = GOTO_COAST_K_INIT;

/* Claves NVS nuevas ("_b"): el trim anterior se aprendio con los motores
 * en otros canales del driver y ya no sirve. */
#define NVS_KEY_TRIM   "trim_e"    /* reinicio: lo guardado se aprendio con el L298N */
#define NVS_KEY_COAST  "coast_d"   /* reinicio: con el DRV8833 cambian la velocidad y la inercia */

static bool nvs_load_float(const char *key, float *out)
{
    nvs_handle_t h;
    if (nvs_open("robot", NVS_READONLY, &h) != ESP_OK) return false;
    float v = 0.0f;
    size_t sz = sizeof(v);
    bool ok = (nvs_get_blob(h, key, &v, &sz) == ESP_OK && sz == sizeof(v) && isfinite(v));
    nvs_close(h);
    if (ok) *out = v;
    return ok;
}

static bool nvs_save_float(const char *key, float v)
{
    nvs_handle_t h;
    if (nvs_open("robot", NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = (nvs_set_blob(h, key, &v, sizeof(v)) == ESP_OK && nvs_commit(h) == ESP_OK);
    nvs_close(h);
    return ok;
}

static void sync_trim_load(void)
{
    float v;
    if (nvs_load_float(NVS_KEY_TRIM, &v))
    {
        s_sync_trim = clampf(v, -GOTO_SYNC_TRIM_MAX, GOTO_SYNC_TRIM_MAX);
        s_sync_trim_saved = s_sync_trim;
        ESP_LOGI(TAG, "Trim de ruedas cargado de NVS: %.1f PWM", s_sync_trim);
    }
    if (nvs_load_float(NVS_KEY_COAST, &v))
    {
        s_coast_k = clampf(v, GOTO_COAST_K_MIN, GOTO_COAST_K_MAX);
        s_coast_k_saved = s_coast_k;
        ESP_LOGI(TAG, "Inercia cargada de NVS: k=%.3f s", s_coast_k);
    }
}

static void sync_trim_save(void)
{
    /* no gastar flash por cambios pequenos */
    if (fabsf(s_sync_trim - s_sync_trim_saved) >= 0.5f && nvs_save_float(NVS_KEY_TRIM, s_sync_trim))
    {
        s_sync_trim_saved = s_sync_trim;
    }
    if (fabsf(s_coast_k - s_coast_k_saved) >= 0.005f && nvs_save_float(NVS_KEY_COAST, s_coast_k))
    {
        s_coast_k_saved = s_coast_k;
    }
}

static void turn_cmd(float sign, float scale)
{
    g_brake = false;
    g_pwm_cmd_left  = -sign * GOTO_TURN_PWM_L * scale;
    g_pwm_cmd_right =  sign * GOTO_TURN_PWM_R * scale;
}

static void brake_and_settle(int brake_ms, int settle_ms)
{
    g_pwm_cmd_left = 0.0f;
    g_pwm_cmd_right = 0.0f;
    g_brake = true;
    vTaskDelay(pdMS_TO_TICKS(brake_ms));
    g_brake = false;
    vTaskDelay(pdMS_TO_TICKS(settle_ms));
}

/* Ajuste fino con pulsos cortos (giro-freno-medir) hasta quedar dentro de
 * tol. Si un pulso no mueve el robot (friccion), el siguiente dura grow_ms
 * mas. Devuelve false solo si se cancela. */
static bool turn_pulses(float target_heading, float tol, int min_ms, int grow_ms, TickType_t t0)
{
    int extra_ms = 0;
    for (int n = 0; n < GOTO_PULSE_MAX_COUNT; n++)
    {
        if (g_goto_abort) return false;

        float err = wrap_pi(target_heading - get_yaw_odom());
        if (fabsf(err) < tol) break;

        float err_deg = fabsf(err) * 180.0f / (float)M_PI;
        int ms = (int)(err_deg * GOTO_PULSE_MS_PER_DEG) + extra_ms;
        if (ms < min_ms) ms = min_ms;
        if (ms > GOTO_PULSE_MAX_MS) ms = GOTO_PULSE_MAX_MS;

        float sign = (err > 0.0f) ? 1.0f : -1.0f;
        g_brake = false;
        g_pwm_cmd_left  = -sign * GOTO_PULSE_PWM_L;
        g_pwm_cmd_right =  sign * GOTO_PULSE_PWM_R;
        vTaskDelay(pdMS_TO_TICKS(ms));
        brake_and_settle(GOTO_TURN_BRAKE_MS / 2, GOTO_TURN_SETTLE_MS);

        float new_err = wrap_pi(target_heading - get_yaw_odom());
        ESP_LOGI(TAG, "Giro: pulso %d de %d ms, error %.1f -> %.1f deg",
                 n + 1, ms, err * 180.0f / (float)M_PI, new_err * 180.0f / (float)M_PI);

        /* Pulso sin efecto (friccion): el siguiente, mas largo */
        if (fabsf(new_err - err) < 0.01f) extra_ms += grow_ms;
        else if (extra_ms > 0) extra_ms -= grow_ms / 2;

        if (xTaskGetTickCount() - t0 > pdMS_TO_TICKS(GOTO_TURN_TIMEOUT_MS)) break;
    }
    return true;
}

#if GOTO_FINAL_ALIGN
/* Al llegar: deja el robot con el mismo rumbo que tenia al empezar el goto
 * (el zigzag y el frenado lo dejaban torcido hasta ~10 grados). Si no logra
 * la tolerancia no aborta: el goto ya llego al punto. */
static void goto_align_final(float target_heading)
{
    float err = wrap_pi(target_heading - get_yaw_odom());
    if (fabsf(err) < GOTO_FINAL_ALIGN_TOL) return;

    ESP_LOGI(TAG, "Alineacion final: de %.1f a %.1f deg (error %.1f deg)",
             get_yaw_odom() * 180.0f / (float)M_PI, target_heading * 180.0f / (float)M_PI,
             err * 180.0f / (float)M_PI);
    turn_pulses(target_heading, GOTO_FINAL_ALIGN_TOL, GOTO_FINAL_PULSE_MIN_MS,
                GOTO_FINAL_PULSE_GROW_MS, xTaskGetTickCount());
    brake_and_settle(GOTO_TURN_BRAKE_MS, GOTO_TURN_SETTLE_MS);

    err = wrap_pi(target_heading - get_yaw_odom());
    ESP_LOGI(TAG, "Alineacion final: error %.1f deg", err * 180.0f / (float)M_PI);
}
#endif

/* Gira en el sitio hasta target_heading. Devuelve false si se cancela
 * o si no logra girar. */
static bool goto_turn_in_place(float target_heading)
{
    TickType_t t0 = xTaskGetTickCount();
    float err = wrap_pi(target_heading - get_yaw_odom());
    ESP_LOGI(TAG, "Giro: de %.1f a %.1f deg (error %.1f deg)",
             get_yaw_odom() * 180.0f / (float)M_PI, target_heading * 180.0f / (float)M_PI,
             err * 180.0f / (float)M_PI);

    /* ---- Etapa 1: giro continuo hasta la zona fina ---- */
    if (fabsf(err) > GOTO_TURN_FINE_ZONE)
    {
        float boost = 0.0f;
        TickType_t last_check = t0;
        float err_at_check = fabsf(err);
        float sign0 = (err > 0.0f) ? 1.0f : -1.0f;

        while (1)
        {
            if (g_goto_abort) { brake_and_settle(GOTO_TURN_BRAKE_MS, 0); return false; }

            TickType_t now = xTaskGetTickCount();
            err = wrap_pi(target_heading - get_yaw_odom());
            float gz = g_imu_data.raw_gz - g_gz_bias;

            /* Cortar antes: lo que falta lo recorre por inercia. Tambien si
             * ya se paso (cambio el signo del error). */
            float lead = fabsf(gz) * GOTO_TURN_LEAD_S;
            if (fabsf(err) < GOTO_TURN_FINE_ZONE * 0.5f + lead || err * sign0 < 0.0f) break;

            if (now - last_check > pdMS_TO_TICKS(GOTO_TURN_STALL_MS))
            {
                if (fabsf(err) > err_at_check - 0.02f)
                {
                    boost = fminf(boost + GOTO_TURN_BOOST_STEP, GOTO_TURN_BOOST_MAX);
                }
                err_at_check = fabsf(err);
                last_check = now;
            }

            float sl = (GOTO_TURN_PWM_L + boost) / GOTO_TURN_PWM_L;
            turn_cmd(sign0, fminf(sl, 100.0f / GOTO_TURN_PWM_L));

            if (now - t0 > pdMS_TO_TICKS(GOTO_TURN_TIMEOUT_MS))
            {
                brake_and_settle(GOTO_TURN_BRAKE_MS, 0);
                ESP_LOGW(TAG, "Giro: no avanza (err=%.1f deg) -- abortando", err * 180.0f / (float)M_PI);
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(GOTO_LOOP_MS));
        }
        brake_and_settle(GOTO_TURN_BRAKE_MS, GOTO_TURN_SETTLE_MS);
        err = wrap_pi(target_heading - get_yaw_odom());
        ESP_LOGI(TAG, "Giro: etapa continua terminada, error %.1f deg", err * 180.0f / (float)M_PI);
    }

    /* ---- Etapa 2: pulsos cortos ---- */
    if (!turn_pulses(target_heading, GOTO_TURN_TOL, GOTO_PULSE_MIN_MS, GOTO_PULSE_GROW_MS, t0)) return false;

    err = wrap_pi(target_heading - get_yaw_odom());
    brake_and_settle(GOTO_TURN_BRAKE_MS, GOTO_TURN_PAUSE_MS);   /* quieto antes de avanzar */

    if (fabsf(err) > GOTO_SKIP_TURN)
    {
        ESP_LOGW(TAG, "Giro: no llego al angulo (error %.1f deg) -- abortando", err * 180.0f / (float)M_PI);
        return false;
    }
    ESP_LOGI(TAG, "Giro: listo, error final %.1f deg", err * 180.0f / (float)M_PI);
    return true;
}

/* Avanza sobre la recta start -> target. Devuelve false si se cancela,
 * se atasca o se agota el tiempo. */
static bool goto_follow_line(float sx, float sy, float target_x, float target_y,
                             float base_pwm_l, float base_pwm_r, float final_heading)
{
    /* PWM base -> PWM de cada rueda para la misma velocidad */
    base_pwm_l = PWM_FF_OFFSET_L + PWM_FF_GAIN_L * fmaxf((base_pwm_l - GOTO_BASE_FF_OFFSET) / GOTO_BASE_FF_GAIN, 0.0f);
    base_pwm_r = PWM_FF_OFFSET_R + PWM_FF_GAIN_R * fmaxf((base_pwm_r - GOTO_BASE_FF_OFFSET) / GOTO_BASE_FF_GAIN, 0.0f);

    float line_heading = atan2f(target_y - sy, target_x - sx);
    float ux = cosf(line_heading), uy = sinf(line_heading);
    float line_len = (target_x - sx) * ux + (target_y - sy) * uy;

    float heading_integral = s_learned_integral;
#if GOTO_USE_WHEEL_SYNC
    float prev_correction = s_sync_trim;   /* arranca ya compensado */
#else
    float prev_correction = 0.0f;
#endif
    float best_remaining = line_len;
    TickType_t t0 = xTaskGetTickCount();
    TickType_t last_progress = t0;
    int debug_counter = 0;
    bool ok = true;
    bool arrived = false;
    float along_stop = 0.0f, v_stop = 0.0f;

    while (1)
    {
        if (g_goto_abort)
        {
            ESP_LOGW(TAG, "Goto: cancelado");
            ok = false;
            break;
        }

        TickType_t now = xTaskGetTickCount();

        float px = g_odom.x - sx;
        float py = g_odom.y - sy;
        float along = px * ux + py * uy;
        float cross = -px * uy + py * ux;         /* >0: robot a la izquierda de la linea */
        float remaining = line_len - along;

        /* Corta motores antes del objetivo: lo que falta lo recorre por
         * inercia, que depende de la velocidad (k se aprende solo). */
        float v_now = fmaxf(g_odom.v, 0.0f);
        if (remaining < GOTO_DIST_TOL + s_coast_k * v_now)
        {
            arrived = true;
            along_stop = along;
            v_stop = v_now;
            break;
        }

        if (now - t0 > pdMS_TO_TICKS(GOTO_TIMEOUT_MS))
        {
            ESP_LOGW(TAG, "Goto: timeout en fase de avance (restante=%.3f m)", remaining);
            ok = false;
            break;
        }

        if (remaining < best_remaining - GOTO_STALL_MIN_PROGRESS)
        {
            best_remaining = remaining;
            last_progress = now;
        }
        else if (now - last_progress > pdMS_TO_TICKS(GOTO_STALL_CHECK_MS))
        {
            ESP_LOGW(TAG, "Goto: sin avance en %d ms (restante=%.3f m) -- abortando",
                     GOTO_STALL_CHECK_MS, remaining);
            ok = false;
            break;
        }

        float cross_corr = clampf(atanf(GOTO_CROSS_K * cross),
                                  -GOTO_CROSS_MAX_ANGLE, GOTO_CROSS_MAX_ANGLE);
        float heading_des = wrap_pi(line_heading - cross_corr);
        if (remaining < GOTO_ALIGN_DIST)
        {
            float w = clampf(remaining / GOTO_ALIGN_DIST, 0.0f, 1.0f);   /* 1 -> 0 al llegar */
            heading_des = wrap_pi(final_heading + w * wrap_pi(heading_des - final_heading));
        }

        float yaw = get_yaw_odom();
        float gz  = g_imu_data.raw_gz - g_gz_bias;
        float err = wrap_pi(heading_des - yaw);

        /* Seguridad: si el rumbo se va mas de 60 grados de la linea, algo
         * fisico fallo (rueda atascada): parar en vez de dar vueltas. */
        if (fabsf(err) > GOTO_MAX_HEADING_ERR)
        {
            ESP_LOGW(TAG, "Goto: rumbo fuera de control (err=%.1f deg) -- abortando",
                     err * 180.0f / (float)M_PI);
            ok = false;
            break;
        }

#if GOTO_USE_WHEEL_SYNC
        /* Lazo externo: rumbo -> diferencia de velocidad de ruedas deseada */
        float dw_des = GOTO_SYNC_HEADING_K * err - GOTO_SYNC_HEADING_KD * gz;
        dw_des = clampf(dw_des, -GOTO_SYNC_DW_MAX, GOTO_SYNC_DW_MAX);

        /* Lazo interno: la diferencia REAL de velocidad de ruedas se toma
         * del giroscopio (dw = gz * L / R). Los encoders de 20 ranuras daban
         * lecturas erraticas (wL=19 con wR=9 y el robot casi sin girar) y con
         * la rueda izquierda atascada al arrancar marcaban 0 y el lazo no
         * actuaba. El giroscopio mide el efecto real, sin cuantizacion. */
        /* Fijo en el valor con que se ajustaron las ganancias (L=11.2 cm,
         * D=7.1 cm). Con las medidas reales seria 4.57 y cambiaria la
         * ganancia del lazo interno que ya quedo ajustada. */
        const float L_over_R = 3.155f;
        float dw_meas = gz * L_over_R;
        float e_sync = dw_des - dw_meas;       /* >0: la derecha debe ir mas rapido */

        s_sync_trim += GOTO_SYNC_KI * e_sync * GOTO_DT;
        s_sync_trim = clampf(s_sync_trim, -GOTO_SYNC_TRIM_MAX, GOTO_SYNC_TRIM_MAX);

        float correction = s_sync_trim + GOTO_SYNC_KP * e_sync;

#if GOTO_USE_REV_FF
        {
            int32_t pos = g_left_pos;
            int sector = (int)(((pos % PULSES_PER_REV) + PULSES_PER_REV) % PULSES_PER_REV);
            if (g_encoder_data.filtered_left > REV_FF_MIN_SPEED)
            {
                int k = (sector - REV_FF_DELAY + PULSES_PER_REV) % PULSES_PER_REV;
                s_rev_ff[k] = clampf(s_rev_ff[k] + REV_FF_GAIN * e_sync, -REV_FF_MAX, REV_FF_MAX);

                /* Sin componente media: la parte constante la lleva el trim */
                float mean = 0.0f;
                for (int i = 0; i < PULSES_PER_REV; i++) mean += s_rev_ff[i];
                mean /= (float)PULSES_PER_REV;
                for (int i = 0; i < PULSES_PER_REV; i++) s_rev_ff[i] -= mean;
            }
            correction += s_rev_ff[(sector + REV_FF_LEAD) % PULSES_PER_REV];
        }
#endif
        (void)heading_integral;
#else
        heading_integral += err * GOTO_DT;
        float i_lim = GOTO_HEADING_I_MAX / GOTO_HEADING_KI;
        heading_integral = clampf(heading_integral, -i_lim, i_lim);

        float correction = GOTO_HEADING_KP * err
                         + GOTO_HEADING_KI * heading_integral
                         - GOTO_HEADING_KD * gz;
#endif
        correction = clampf(correction, -GOTO_MAX_CORRECTION, GOTO_MAX_CORRECTION);
        correction = clampf(correction,
                            prev_correction - GOTO_CORR_SLEW,
                            prev_correction + GOTO_CORR_SLEW);
        prev_correction = correction;

        float scale = clampf(remaining / GOTO_SLOW_DIST, 0.0f, 1.0f);
        float bl = GOTO_WHEEL_PWM_FLOOR_L + (fmaxf(base_pwm_l, GOTO_WHEEL_PWM_FLOOR_L) - GOTO_WHEEL_PWM_FLOOR_L) * scale;
        float br = GOTO_WHEEL_PWM_FLOOR_R + (fmaxf(base_pwm_r, GOTO_WHEEL_PWM_FLOOR_R) - GOTO_WHEEL_PWM_FLOOR_R) * scale;

        float pwm_l = bl - correction;
        float pwm_r = br + correction;

        /* Mantener la diferencia entre ruedas respetando el piso y el
         * techo de cada una */
        float deficit = fmaxf(fmaxf(GOTO_WHEEL_PWM_FLOOR_L - pwm_l, GOTO_WHEEL_PWM_FLOOR_R - pwm_r), 0.0f);
        pwm_l += deficit;
        pwm_r += deficit;
        float excess = fmaxf(fmaxf(pwm_l - GOTO_WHEEL_PWM_MAX_L, pwm_r - GOTO_WHEEL_PWM_MAX_R), 0.0f);
        pwm_l -= excess;
        pwm_r -= excess;
        pwm_l = clampf(pwm_l, GOTO_WHEEL_PWM_FLOOR_L, GOTO_WHEEL_PWM_MAX_L);
        pwm_r = clampf(pwm_r, GOTO_WHEEL_PWM_FLOOR_R, GOTO_WHEEL_PWM_MAX_R);

        if (now - t0 < pdMS_TO_TICKS(GOTO_KICK_MS))
        {
            /* El empujon lleva la correccion de rumbo (con fmaxf se perdia),
             * pero SIN el trim: un trim de +12 aprendido con PWM 50 daba
             * 70/100 en el arranque a PWM 70 y el robot salia girando
             * 15-35 grados a la izquierda. */
            float corr_fb = correction - s_sync_trim;
            pwm_l = clampf(GOTO_KICK_PWM_L - corr_fb, 0.0f, 100.0f);
            pwm_r = clampf(GOTO_KICK_PWM_R + corr_fb, 0.0f, 100.0f);
        }

        g_pwm_cmd_left  = pwm_l;
        g_pwm_cmd_right = pwm_r;

        if (++debug_counter >= 5)
        {
            debug_counter = 0;
            ESP_LOGI(TAG, "Goto avance -> rest:%.3f cross:%.3f | yaw:%.1f err:%.1f gz:%.2f | wL:%.1f wR:%.1f trim:%.1f | corr:%.1f pwmL:%.0f pwmR:%.0f",
                     remaining, cross,
                     yaw * 180.0f / (float)M_PI, err * 180.0f / (float)M_PI, gz,
                     g_wheel_speed_l, g_wheel_speed_r, s_sync_trim,
                     correction, pwm_l, pwm_r);
        }

        vTaskDelay(pdMS_TO_TICKS(GOTO_LOOP_MS));
    }

    /* Medir cuanto rodo por inercia y ajustar k para el proximo trayecto */
    if (ok && arrived)
    {
        g_pwm_cmd_left = 0.0f;
        g_pwm_cmd_right = 0.0f;
        /* Freno activo: antes rodaba libre ~22 cm y en ese tramo se
         * desviaba sin control. Frenando para en pocos cm. */
        g_brake = true;
        vTaskDelay(pdMS_TO_TICKS(GOTO_BRAKE_MS));
        g_brake = false;
        vTaskDelay(pdMS_TO_TICKS(150));
        float along_final = (g_odom.x - sx) * ux + (g_odom.y - sy) * uy;
        float coast = along_final - along_stop;
        if (v_stop > 0.1f)
        {
            float k_meas = clampf(coast / v_stop, GOTO_COAST_K_MIN, GOTO_COAST_K_MAX);
            s_coast_k = (1.0f - GOTO_COAST_LEARN) * s_coast_k + GOTO_COAST_LEARN * k_meas;
            ESP_LOGI(TAG, "Goto: corto a %.2f m/s, rodo %.3f m por inercia -> k=%.3f s (quedo a %.3f m del objetivo)",
                     v_stop, coast, s_coast_k, line_len - along_final);
        }
    }

    if (ok)
    {
#if GOTO_USE_WHEEL_SYNC
        sync_trim_save();
#if GOTO_USE_REV_FF
        {
            float mn = s_rev_ff[0], mx = s_rev_ff[0];
            for (int i = 1; i < PULSES_PER_REV; i++)
            {
                if (s_rev_ff[i] < mn) mn = s_rev_ff[i];
                if (s_rev_ff[i] > mx) mx = s_rev_ff[i];
            }
            ESP_LOGI(TAG, "Goto: compensacion por vuelta aprendida, rango %.1f a %.1f PWM", mn, mx);
        }
#endif
        ESP_LOGI(TAG, "Goto: trim de ruedas = %.1f PWM (la izquierda recibe %.1f mas que la derecha)",
                 s_sync_trim, -2.0f * s_sync_trim);
#else
        s_learned_integral = heading_integral;
        ESP_LOGI(TAG, "Goto: trim aprendido = %.1f PWM", GOTO_HEADING_KI * s_learned_integral);
#endif
    }
    return ok;
}

void goto_task(void *pvParameters)
{
    while (1)
    {
        if (g_goto_pending && g_yaw_ready)
        {
            g_goto_active = true;
            g_goto_pending = false;

            uint32_t seq = g_status_seq;
            float base_pwm_l = g_goto_pwm_left;
            float base_pwm_r = g_goto_pwm_right;

            float sx = g_odom.x, sy = g_odom.y;
            float target_x = g_goto_target_x;
            float target_y = g_goto_target_y;
            if (g_goto_relative)
            {
                target_x += sx;
                target_y += sy;
            }

            float dx0 = target_x - sx;
            float dy0 = target_y - sy;
            float line_heading = atan2f(dy0, dx0);
            float distance = sqrtf(dx0 * dx0 + dy0 * dy0);
            float err0 = wrap_pi(line_heading - get_yaw_odom());
            /* Rumbo con el que debe llegar: el de la linea (en recta, el del
             * cero). Con el rumbo de inicio se arrastraba el error: si un goto
             * terminaba en -7.8 grados, el siguiente intentaba llegar a -7.8. */
            float final_heading = line_heading;

            ESP_LOGI(TAG, "Goto #%lu (%s): desde (%.3f,%.3f) a (%.3f,%.3f) dist=%.3f m rumbo=%.1f deg yaw=%.1f deg pwm=(%.0f,%.0f)",
                     (unsigned long)seq, g_goto_relative ? "relativo" : "absoluto",
                     sx, sy, target_x, target_y, distance,
                     line_heading * 180.0f / (float)M_PI,
                     get_yaw_odom() * 180.0f / (float)M_PI,
                     base_pwm_l, base_pwm_r);

            /* Ningun cmd_vel viejo debe reactivarse al terminar */
            pid_left.setpoint_rad_s = 0.0f;
            pid_right.setpoint_rad_s = 0.0f;
            g_pwm_cmd_left = 0.0f;
            g_pwm_cmd_right = 0.0f;
            g_pwm_mode = true;

            bool ok = true;
            if (distance >= GOTO_DIST_TOL)
            {
                if (fabsf(err0) > GOTO_SKIP_TURN)
                {
                    ok = goto_turn_in_place(line_heading);   /* termina frenado y en pausa */
                }
                if (ok)
                {
                    ok = goto_follow_line(sx, sy, target_x, target_y, base_pwm_l, base_pwm_r, final_heading);
                }
#if GOTO_FINAL_ALIGN
                if (ok)
                {
                    goto_align_final(final_heading);
                }
#endif
            }

            g_pwm_cmd_left = 0.0f;
            g_pwm_cmd_right = 0.0f;
            vTaskDelay(pdMS_TO_TICKS(300));   /* deja terminar la inercia antes de medir */
            g_pwm_mode = false;

            g_goto_result.arrived_x = g_odom.x;
            g_goto_result.arrived_y = g_odom.y;
            g_goto_result.target_x = target_x;
            g_goto_result.target_y = target_y;
            g_goto_result.error_m = sqrtf(powf(target_x - g_odom.x, 2.0f) + powf(target_y - g_odom.y, 2.0f));
            g_goto_result.seq = seq;
            g_goto_result.ready = true;

            ESP_LOGI(TAG, "Goto #%lu %s: llego a (%.3f,%.3f), objetivo (%.3f,%.3f), error=%.3f m, yaw=%.1f deg",
                     (unsigned long)seq, ok ? "completado" : "ABORTADO",
                     g_odom.x, g_odom.y, target_x, target_y, g_goto_result.error_m,
                     get_yaw_odom() * 180.0f / (float)M_PI);

            g_goto_abort = false;
            g_status_state = ok ? GOTO_STATE_DONE : GOTO_STATE_ABORTED;
            g_goto_active = false;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ============================================================
 * micro-ROS: entidades
 * ============================================================ */
static rcl_allocator_t   s_allocator;
static rcl_init_options_t s_init_options;
static rclc_support_t    s_support;
static bool              s_support_ok = false;
static rcl_node_t        s_node;
static rcl_timer_t       s_timer;
static rclc_executor_t   s_executor;

static rcl_subscription_t cmd_vel_sub;
static geometry_msgs__msg__Twist cmd_vel_msg;

static rcl_subscription_t goto_sub;
static geometry_msgs__msg__Twist goto_msg;

static rcl_publisher_t imu_raw_pub, imu_filtered_pub;
static sensor_msgs__msg__Imu imu_raw_msg, imu_filtered_msg;

static rcl_publisher_t wl_raw_pub, wl_filt_pub, wr_raw_pub, wr_filt_pub;
static std_msgs__msg__Float32 wl_raw_msg, wl_filt_msg, wr_raw_msg, wr_filt_msg;

static rcl_publisher_t odom_pose_pub, odom_twist_pub;
static geometry_msgs__msg__Pose  odom_pose_msg;
static geometry_msgs__msg__Twist odom_twist_msg;

static rcl_publisher_t goto_result_pub;
static geometry_msgs__msg__Twist goto_result_msg;

/* Estado del mando directo por ESP-NOW (20 Hz):
 *   linear.x = v, angular.z = w comandadas por el guante
 *   linear.y = paquetes/s recibidos, linear.z = 0 sin enlace, 1 desarmado, 2 armado
 *   angular.x = RSSI (dBm), angular.y = 0 libre, 1 cerca del borde, 2 borde (solo giro) */
static rcl_publisher_t espnow_pub;
static geometry_msgs__msg__Twist espnow_msg;

/* Alimentacion del sistema (INA219), 10 Hz. Convencion de ROS: current < 0
 * descargando. charge = capacidad - lo consumido (Ah). El porcentaje lo
 * calcula el PC (curva de Ni-MH con la tension y la carga). */
static rcl_publisher_t battery_pub;
static sensor_msgs__msg__BatteryState battery_msg;
static char s_batt_location[] = "carro/puente_h";
static char s_batt_serial[] = "BST-9V";

static rcl_publisher_t *const s_pubs[] = {
    &imu_raw_pub, &imu_filtered_pub,
    &wl_raw_pub, &wl_filt_pub, &wr_raw_pub, &wr_filt_pub,
    &odom_pose_pub, &odom_twist_pub, &goto_result_pub, &espnow_pub, &battery_pub,
};
#define NUM_PUBS (sizeof(s_pubs) / sizeof(s_pubs[0]))

static void cmd_vel_callback(const void *msgin)
{
    if (g_goto_active || g_goto_pending) return;   /* goto tiene el control mientras corre */
    if (g_glove_active) return;                     /* el guante armado tiene prioridad sobre el PC */

    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    g_last_cmd_vel_tick = xTaskGetTickCount();

    if (msg->linear.y != 0.0 || msg->linear.z != 0.0)
    {
        g_pwm_cmd_left  = clampf((float)msg->linear.y, (float)MIN_PWM, (float)MAX_PWM);
        g_pwm_cmd_right = clampf((float)msg->linear.z, (float)MIN_PWM, (float)MAX_PWM);
        g_pwm_mode = true;
    }
    else
    {
        float omega_left, omega_right;
        twist_to_wheel_speeds((float)msg->linear.x, (float)msg->angular.z,
                              &omega_left, &omega_right);

        pid_left.setpoint_rad_s  = omega_left;
        pid_right.setpoint_rad_s = omega_right;
        g_pwm_mode = false;
    }
}

static void goto_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    uint32_t seq = (msg->angular.y > 0.0) ? (uint32_t)msg->angular.y : 0;
    int mode = (int)(msg->angular.z + 0.5);

    /* Copias repetidas del mismo comando (el sender reenvia hasta
     * recibir confirmacion): se ignoran, no se ejecutan dos veces. */
    if (seq != 0 && seq == g_last_seq) return;

    if (mode == GOTO_MODE_CANCEL)
    {
        g_last_seq = seq;
        if (g_goto_active || g_goto_pending) g_goto_abort = true;
        return;
    }

    /* Ocupado: no se acepta; el sender sigue reintentando */
    if (g_goto_active || g_goto_pending) return;

    g_last_seq = seq;

    if (mode == GOTO_MODE_RESET)
    {
        g_reset_yaw_req = true;
        g_reset_odom_req = true;
        g_status_seq = seq;
        g_status_state = GOTO_STATE_DONE;
        return;
    }

    /* Un goto con PWM 0 en ambas ruedas no es valido. Al reconectarse el
     * agente llego un mensaje todo en cero (nadie lo publico) y el robot
     * arranco un goto absoluto a (0,0) desde donde estuviera. */
    if (msg->linear.z <= 0.0 && msg->angular.x <= 0.0)
    {
        ESP_LOGW(TAG, "Goto ignorado: PWM (0,0) no es valido (seq=%lu)", (unsigned long)seq);
        return;
    }

    g_goto_target_x  = (float)msg->linear.x;
    g_goto_target_y  = (float)msg->linear.y;
    g_goto_pwm_left  = clampf((float)msg->linear.z,  0.0f, (float)MAX_PWM);
    g_goto_pwm_right = clampf((float)msg->angular.x, 0.0f, (float)MAX_PWM);
    g_goto_relative  = (mode == GOTO_MODE_RELATIVE);
    g_goto_abort = false;
    g_status_seq = seq;
    g_status_state = GOTO_STATE_ACTIVE;
    g_goto_pending = true;
}

static void publish_timer_callback(rcl_timer_t *timer, int64_t last_call_time, uintptr_t user_data)
{
    (void)last_call_time;
    (void)user_data;
    if (timer == NULL) return;

    imu_raw_msg.orientation_covariance[0] = -1.0;
    imu_raw_msg.linear_acceleration.x = g_imu_data.raw_ax;
    imu_raw_msg.linear_acceleration.y = g_imu_data.raw_ay;
    imu_raw_msg.linear_acceleration.z = g_imu_data.raw_az;
    imu_raw_msg.angular_velocity.x = g_imu_data.raw_gx;
    imu_raw_msg.angular_velocity.y = g_imu_data.raw_gy;
    imu_raw_msg.angular_velocity.z = g_imu_data.raw_gz;
    RCSOFTCHECK(rcl_publish(&imu_raw_pub, &imu_raw_msg, NULL));

    imu_filtered_msg.orientation.x = g_imu_data.qx;
    imu_filtered_msg.orientation.y = g_imu_data.qy;
    imu_filtered_msg.orientation.z = g_imu_data.qz;
    imu_filtered_msg.orientation.w = g_imu_data.qw;
    imu_filtered_msg.linear_acceleration.x = g_imu_data.lin_ax;
    imu_filtered_msg.linear_acceleration.y = g_imu_data.lin_ay;
    imu_filtered_msg.linear_acceleration.z = g_imu_data.lin_az;
    imu_filtered_msg.angular_velocity.x = g_imu_data.f_gx;   /* giroscopio filtrado */
    imu_filtered_msg.angular_velocity.y = g_imu_data.f_gy;
    imu_filtered_msg.angular_velocity.z = g_imu_data.f_gz;
    RCSOFTCHECK(rcl_publish(&imu_filtered_pub, &imu_filtered_msg, NULL));

    wl_raw_msg.data  = g_encoder_data.raw_left;
    wl_filt_msg.data = g_encoder_data.filtered_left;
    wr_raw_msg.data  = g_encoder_data.raw_right;
    wr_filt_msg.data = g_encoder_data.filtered_right;

    RCSOFTCHECK(rcl_publish(&wl_raw_pub, &wl_raw_msg, NULL));
    RCSOFTCHECK(rcl_publish(&wl_filt_pub, &wl_filt_msg, NULL));
    RCSOFTCHECK(rcl_publish(&wr_raw_pub, &wr_raw_msg, NULL));
    RCSOFTCHECK(rcl_publish(&wr_filt_pub, &wr_filt_msg, NULL));

    odom_pose_msg.position.x = g_odom.x;
    odom_pose_msg.position.y = g_odom.y;
    odom_pose_msg.position.z = 0.0;
    odom_pose_msg.orientation.x = 0.0;
    odom_pose_msg.orientation.y = 0.0;
    odom_pose_msg.orientation.z = sinf(0.5f * g_odom.theta);
    odom_pose_msg.orientation.w = cosf(0.5f * g_odom.theta);
    RCSOFTCHECK(rcl_publish(&odom_pose_pub, &odom_pose_msg, NULL));

    odom_twist_msg.linear.x  = g_odom.v;                 /* m/s */
    odom_twist_msg.angular.z = g_odom.w;                 /* rad/s */
    odom_twist_msg.linear.y  = (double)g_status_seq;     /* estado del goto */
    odom_twist_msg.linear.z  = (double)g_status_state;
    RCSOFTCHECK(rcl_publish(&odom_twist_pub, &odom_twist_msg, NULL));

    espnow_msg.linear.x  = g_glove_v;
    espnow_msg.angular.z = g_glove_w;
    espnow_msg.linear.y  = g_glove_rate;
    espnow_msg.linear.z  = (double)g_glove_link;
    espnow_msg.angular.x = (double)g_glove_rssi;
    espnow_msg.angular.y = (double)g_glove_guard;
    RCSOFTCHECK(rcl_publish(&espnow_pub, &espnow_msg, NULL));

    static int batt_div = 0;
    if (s_ina_ok && ++batt_div >= 2)   /* 20 Hz / 2 = 10 Hz */
    {
        batt_div = 0;
        battery_msg.voltage = g_batt_v;
        battery_msg.current = -g_batt_ma / 1000.0f;
        battery_msg.charge = (BATT_CAPACITY_MAH - g_batt_used_mah) / 1000.0f;
        battery_msg.capacity = BATT_CAPACITY_MAH / 1000.0f;
        battery_msg.design_capacity = BATT_CAPACITY_MAH / 1000.0f;
        battery_msg.percentage = NAN;
        battery_msg.temperature = NAN;
        battery_msg.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
        battery_msg.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
        battery_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_NIMH;
        battery_msg.present = g_batt_v > 1.0f;
        battery_msg.location.data = s_batt_location;
        battery_msg.location.size = sizeof(s_batt_location) - 1;
        battery_msg.location.capacity = sizeof(s_batt_location);
        battery_msg.serial_number.data = s_batt_serial;
        battery_msg.serial_number.size = sizeof(s_batt_serial) - 1;
        battery_msg.serial_number.capacity = sizeof(s_batt_serial);
        RCSOFTCHECK(rcl_publish(&battery_pub, &battery_msg, NULL));
    }

    if (g_goto_result.ready)
    {
        goto_result_msg.linear.x  = g_goto_result.arrived_x;
        goto_result_msg.linear.y  = g_goto_result.arrived_y;
        goto_result_msg.linear.z  = g_goto_result.error_m;
        goto_result_msg.angular.x = g_goto_result.target_x;
        goto_result_msg.angular.y = g_goto_result.target_y;
        goto_result_msg.angular.z = (double)g_goto_result.seq;
        if (rcl_publish(&goto_result_pub, &goto_result_msg, NULL) == RCL_RET_OK)
        {
            g_goto_result.ready = false;
        }
    }
}

#define ENTITY_CREATION_TIMEOUT_MS  3000

static bool create_entities(void)
{
    for (size_t i = 0; i < NUM_PUBS; i++) *s_pubs[i] = rcl_get_zero_initialized_publisher();
    cmd_vel_sub = rcl_get_zero_initialized_subscription();
    goto_sub = rcl_get_zero_initialized_subscription();
    s_node = rcl_get_zero_initialized_node();
    s_timer = rcl_get_zero_initialized_timer();
    s_executor = rclc_executor_get_zero_initialized_executor();
    s_support_ok = false;

    RCRETCHECK(rclc_support_init_with_options(&s_support, 0, NULL, &s_init_options, &s_allocator));
    s_support_ok = true;

    /* Mas margen para que el agente confirme cada entidad: sobre WiFi, un
     * paquete perdido hacia fallar la creacion y habia que reintentar todo. */
    (void)rmw_uros_set_context_entity_creation_session_timeout(
        rcl_context_get_rmw_context(&s_support.context), ENTITY_CREATION_TIMEOUT_MS);

    RCRETCHECK(rclc_node_init_default(&s_node, "robot_esp32_node", "", &s_support));

    /* Sensores a 20 Hz en "best effort": sin confirmaciones. En modo fiable,
     * cuando el WiFi retrasaba los acuses se llenaba el buffer, rcl_publish
     * fallaba y la grafica quedaba "escalonada" (el mismo valor 0.5-1 s).
     * Odometria y estado tambien van en best effort: en micro-ROS cada
     * rcl_publish fiable ESPERA el acuse del agente (hasta 1 s), y con
     * dos publicaciones fiables por ciclo el temporizador de 50 ms caia
     * a ~3 Hz y arrastraba a todos los topicos. Como se publican 20 veces
     * por segundo, perder uno no importa. Solo goto_result (un mensaje
     * unico por goto) sigue fiable. */
    RCRETCHECK(rclc_publisher_init_best_effort(&imu_raw_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/raw"));
    RCRETCHECK(rclc_publisher_init_best_effort(&imu_filtered_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/filtered"));

    RCRETCHECK(rclc_publisher_init_best_effort(&wl_raw_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_left/raw"));
    RCRETCHECK(rclc_publisher_init_best_effort(&wl_filt_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_left/filtered"));
    RCRETCHECK(rclc_publisher_init_best_effort(&wr_raw_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_right/raw"));
    RCRETCHECK(rclc_publisher_init_best_effort(&wr_filt_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheel_right/filtered"));

    RCRETCHECK(rclc_publisher_init_best_effort(&odom_pose_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose), "odom/pose"));
    RCRETCHECK(rclc_publisher_init_best_effort(&odom_twist_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "odom/twist"));

    RCRETCHECK(rclc_publisher_init_default(&goto_result_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "goto_result"));
    RCRETCHECK(rclc_publisher_init_best_effort(&espnow_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "teleop/espnow"));
    RCRETCHECK(rclc_publisher_init_best_effort(&battery_pub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "battery/state"));

    RCRETCHECK(rclc_subscription_init_default(&cmd_vel_sub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    RCRETCHECK(rclc_subscription_init_default(&goto_sub, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "goto_cmd"));

    RCRETCHECK(rclc_timer_init_default2(&s_timer, &s_support, RCL_MS_TO_NS(50),
                                        publish_timer_callback, true)); /* 20 Hz */

    RCRETCHECK(rclc_executor_init(&s_executor, &s_support.context, 3, &s_allocator));
    RCRETCHECK(rclc_executor_add_subscription(&s_executor, &cmd_vel_sub, &cmd_vel_msg,
                                              &cmd_vel_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_subscription(&s_executor, &goto_sub, &goto_msg,
                                              &goto_callback, ON_NEW_DATA));
    RCRETCHECK(rclc_executor_add_timer(&s_executor, &s_timer));

    return true;
}

static void destroy_entities(void)
{
    if (s_support_ok)
    {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&s_support.context);
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    }

    for (size_t i = 0; i < NUM_PUBS; i++) RCSOFTCHECK(rcl_publisher_fini(s_pubs[i], &s_node));
    RCSOFTCHECK(rcl_subscription_fini(&cmd_vel_sub, &s_node));
    RCSOFTCHECK(rcl_subscription_fini(&goto_sub, &s_node));
    RCSOFTCHECK(rcl_timer_fini(&s_timer));
    RCSOFTCHECK(rclc_executor_fini(&s_executor));
    RCSOFTCHECK(rcl_node_fini(&s_node));
    if (s_support_ok)
    {
        RCSOFTCHECK(rclc_support_fini(&s_support));
        s_support_ok = false;
    }
}

/* ============================================================
 * micro-ROS: maquina de estados de conexion
 *  Antes: si fallaba la creacion de UNA entidad (paso con
 *  wheel_right/filtered) o se caia el agente/WiFi, la tarea moria
 *  y el robot ya no recibia comandos hasta reiniciarlo. Ahora se
 *  detecta, se destruye todo y se reconecta sola.
 * ============================================================ */
typedef enum
{
    AGENT_WAITING,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED,
} agent_state_t;

void micro_ros_task(void *arg)
{
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    /* Sin ahorro de energia WiFi: en modo "modem sleep" (pm start,
     * type:1 en el log) la radio duerme entre beacons y cada paquete
     * puede tardar 100-300 ms. Eso hacia fallar el ping al agente y
     * la creacion de entidades ("Se perdio el agente micro-ROS"). */
    esp_err_t ps_ret = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ps_ret != ESP_OK)
    {
        ESP_LOGW(TAG, "No se pudo desactivar el ahorro de energia WiFi: %s", esp_err_to_name(ps_ret));
    }

    /* Receptor del guante. Funciona aunque el agente micro-ROS no este. */
    esp_err_t en = espnow_link_init(glove_rx_cb);
    if (en != ESP_OK)
    {
        ESP_LOGE(TAG, "ESP-NOW no inicio: %s (sin mando directo del guante)", esp_err_to_name(en));
    }

    s_allocator = rcl_get_default_allocator();
    s_init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&s_init_options, s_allocator) != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "rcl_init_options_init fallo");
        vTaskDelete(NULL);
    }

    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&s_init_options);
    if (rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                         CONFIG_MICRO_ROS_AGENT_PORT,
                                         rmw_options) != RMW_RET_OK)
    {
        ESP_LOGE(TAG, "Direccion del agente invalida");
        vTaskDelete(NULL);
    }

    agent_state_t state = AGENT_WAITING;
    TickType_t last_ping = 0;
    int wait_log = 0;
    int discover_tries = 0;
    int ping_failures = 0;

    while (1)
    {
        switch (state)
        {
        case AGENT_WAITING:
            if (rmw_uros_ping_agent_options(200, 1, rmw_options) == RMW_RET_OK)
            {
                state = AGENT_AVAILABLE;
                discover_tries = 0;
            }
            else
            {
                /* Sin respuesta en la IP configurada (~3 s): buscar el agente
                 * en la red. El router puede cambiarle la IP al PC (DHCP);
                 * el agente responde si se lanzo con --discovery 7400. */
                if (++discover_tries >= 6)
                {
                    discover_tries = 0;
                    if (rmw_uros_discover_agent(rmw_options) == RMW_RET_OK)
                    {
                        ESP_LOGI(TAG, "Agente micro-ROS encontrado en la red por descubrimiento");
                        break;
                    }
                }
                if (++wait_log >= 5)
                {
                    wait_log = 0;
                    ESP_LOGW(TAG, "Esperando al agente micro-ROS (IP configurada %s, buscando en la red)...",
                             CONFIG_MICRO_ROS_AGENT_IP);
                }
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            break;

        case AGENT_AVAILABLE:
            if (create_entities())
            {
                ESP_LOGI(TAG, "Agente conectado, entidades creadas");
                last_ping = xTaskGetTickCount();
                ping_failures = 0;
                state = AGENT_CONNECTED;
            }
            else
            {
                ESP_LOGW(TAG, "Fallo creando entidades, reintentando");
                destroy_entities();
                vTaskDelay(pdMS_TO_TICKS(500));
                state = AGENT_WAITING;
            }
            break;

        case AGENT_CONNECTED:
            if (xTaskGetTickCount() - last_ping > pdMS_TO_TICKS(1000))
            {
                last_ping = xTaskGetTickCount();
                /* Tolerante a picos de latencia: solo se declara perdido
                 * tras 5 rondas seguidas sin respuesta (~5 s). */
                /* Un solo intento corto: el ping bloquea esta tarea y con
                 * (200 ms x 3) frenaba las publicaciones hasta 0.6 s. */
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
                {
                    if (++ping_failures >= 5)
                    {
                        ESP_LOGW(TAG, "Se perdio el agente micro-ROS");
                        state = AGENT_DISCONNECTED;
                        break;
                    }
                }
                else
                {
                    ping_failures = 0;
                }
            }
            RCSOFTCHECK(rclc_executor_spin_some(&s_executor, RCL_MS_TO_NS(10)));
            vTaskDelay(pdMS_TO_TICKS(10));
            break;

        case AGENT_DISCONNECTED:
            /* El watchdog de cmd_vel para los motores; un goto en
             * curso termina solo y su resultado se publica al volver. */
            destroy_entities();
            state = AGENT_WAITING;
            break;
        }
    }
}

/* ============================================================
 * TEST DE DIAGNOSTICO: motores a PWM fijo e IGUAL en ambas ruedas
 * ============================================================ */
void test_raw_motors(void)
{
    const int8_t TEST_PWM_LEFT = 60;
    const int8_t TEST_PWM_RIGHT = 60;
    const int TEST_DURATION_STEPS = 10;

    ESP_LOGI(TAG, "=== TEST SIMETRICO: IZQUIERDA=%d, DERECHA=%d ===", TEST_PWM_LEFT, TEST_PWM_RIGHT);

    portDISABLE_INTERRUPTS();
    encoder_count_left = 0;
    encoder_count_right = 0;
    portENABLE_INTERRUPTS();

    motor_right_set(TEST_PWM_RIGHT);
    motor_left_set(TEST_PWM_LEFT);

    for (int i = 0; i < TEST_DURATION_STEPS; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_LOGI(TAG, "TEST -> t:%.1fs Pulsos L:%ld | Pulsos R:%ld",
                 (i + 1) * 0.5f, (long)encoder_count_left, (long)encoder_count_right);
    }

    motor_right_set(0);
    motor_left_set(0);
}

/* ============================================================
 * TEST DE DIAGNOSTICO: mapeo encoder <-> rueda fisica.
 * ============================================================ */
void test_encoder_mapping(void)
{
    const int8_t TEST_SPEED = 60;
    const uint32_t TEST_TIME_MS = 3000;

    portDISABLE_INTERRUPTS();
    encoder_count_left = 0;
    encoder_count_right = 0;
    portENABLE_INTERRUPTS();

    ESP_LOGI(TAG, "=== Test mapeo encoders: motor_right_set (observa que rueda gira) ===");
    motor_right_set(TEST_SPEED);
    vTaskDelay(pdMS_TO_TICKS(TEST_TIME_MS));
    motor_right_set(0);
    ESP_LOGI(TAG, "Con motor_right_set activo -> R:%ld L:%ld",
             (long)encoder_count_right, (long)encoder_count_left);

    vTaskDelay(pdMS_TO_TICKS(1000));
    portDISABLE_INTERRUPTS();
    encoder_count_left = 0;
    encoder_count_right = 0;
    portENABLE_INTERRUPTS();

    ESP_LOGI(TAG, "=== Test mapeo encoders: motor_left_set (observa que rueda gira) ===");
    motor_left_set(TEST_SPEED);
    vTaskDelay(pdMS_TO_TICKS(TEST_TIME_MS));
    motor_left_set(0);
    ESP_LOGI(TAG, "Con motor_left_set activo -> R:%ld L:%ld",
             (long)encoder_count_right, (long)encoder_count_left);

    ESP_LOGI(TAG, "=== Fin test de mapeo ===");
}

/* ============================================================
 * TEST DE LA LIBRERIA motor_driver: prueba cada funcion con el
 * robot en el aire y muestra los pulsos de cada encoder.
 * ============================================================ */
static void lib_step(const char *desc, int ms)
{
    ESP_LOGI(TAG, "%s  <-- MIRA QUE RUEDA GIRA Y HACIA DONDE", desc);
    portDISABLE_INTERRUPTS();
    encoder_count_left = 0;
    encoder_count_right = 0;
    portENABLE_INTERRUPTS();
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void lib_end_step(void)
{
    car_stop();
    vTaskDelay(pdMS_TO_TICKS(400));
    ESP_LOGI(TAG, "   -> Encoder IZQ: %ld pulsos | Encoder DER: %ld pulsos",
             (long)encoder_count_left, (long)encoder_count_right);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void test_motor_library(void)
{
    const int STEP_MS = 1500;

    for (int i = 5; i > 0; i--)
    {
        ESP_LOGW(TAG, "Prueba de la libreria en %d s -- LEVANTA EL ROBOT", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "===== motor_left_set(): rueda IZQUIERDA sola =====");
    const int8_t levels[] = { 40, 60, 80, 100 };
    for (int i = 0; i < 4; i++)
    {
        char d[64];
        snprintf(d, sizeof(d), "motor_left_set(%d)", levels[i]);
        motor_left_set(levels[i]);
        lib_step(d, STEP_MS);
        lib_end_step();
    }
    motor_left_set(-60);
    lib_step("motor_left_set(-60)  (atras)", STEP_MS);
    lib_end_step();

    ESP_LOGI(TAG, "===== motor_right_set(): rueda DERECHA sola =====");
    for (int i = 0; i < 4; i++)
    {
        char d[64];
        snprintf(d, sizeof(d), "motor_right_set(%d)", levels[i]);
        motor_right_set(levels[i]);
        lib_step(d, STEP_MS);
        lib_end_step();
    }
    motor_right_set(-60);
    lib_step("motor_right_set(-60)  (atras)", STEP_MS);
    lib_end_step();

    ESP_LOGI(TAG, "===== Ambas ruedas =====");
    motor_left_set(70);
    motor_right_set(70);
    lib_step("motor_left_set(70) + motor_right_set(70)  (adelante)", STEP_MS);
    lib_end_step();

    motor_left_set(-70);
    motor_right_set(-70);
    lib_step("motor_left_set(-70) + motor_right_set(-70)  (atras)", STEP_MS);
    lib_end_step();

    car_turn_right(70);
    lib_step("car_turn_right(70)", STEP_MS);
    lib_end_step();

    car_turn_left(70);
    lib_step("car_turn_left(70)", STEP_MS);
    lib_end_step();

    ESP_LOGI(TAG, "===== Fin de la prueba de la libreria (20 pulsos = 1 vuelta) =====");
}

/* ============================================================
 * app_main
 * ============================================================ */
void app_main(void)
{
    esp_err_t nvs_ret = nvs_flash_init();
    if ((nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES) || (nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);
    sync_trim_load();

    /* Motores apagados lo antes posible tras el arranque */
    ESP_ERROR_CHECK(motor_driver_init());
    motor_left_set(0);
    motor_right_set(0);

    mpu6050_setup();
    ina219_setup();
    encoders_init();

    const bool MODO_TEST_ENCODER_MAPPING = false;
    const bool MODO_TEST_MOTORES = false;
    const bool MODO_TEST_LIBRERIA = false;  /* true = prueba de motor_driver con el robot en el aire */

    if (MODO_TEST_LIBRERIA)
    {
        test_motor_library();
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    if (MODO_TEST_ENCODER_MAPPING)
    {
        test_encoder_mapping();
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    if (MODO_TEST_MOTORES)
    {
        test_raw_motors();
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    /* El IMU arranca primero: el robot debe estar QUIETO ~1 s
     * mientras se estima el sesgo del giroscopio. */
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
#if CAR_TELEMETRY
    s_tl_queue = xQueueCreate(32, sizeof(car_tl_t));
    xTaskCreate(telemetry_task, "telemetry_task", 3072, NULL, 1, NULL);
#endif
    xTaskCreate(wheel_control_task, "wheel_control_task", 4096, NULL, 6, NULL);
    xTaskCreate(driver_fault_task, "driver_fault_task", 2560, NULL, 2, NULL);
    xTaskCreate(goto_task, "goto_task", 4096, NULL, 5, NULL);
    if (s_ina_ok)
    {
        xTaskCreate(power_task, "power_task", 3072, NULL, 2, NULL);
    }
    xTaskCreate(micro_ros_task, "micro_ros_task", 8192, NULL, 4, NULL);
}