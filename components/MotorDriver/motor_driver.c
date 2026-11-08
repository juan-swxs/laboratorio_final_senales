#include "motor_driver.h"

#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "MOTOR_DRIVER";

/*
 * ============================================================
 * Pin configuration (DRV8833)
 *
 * El DRV8833 no tiene pines EN: el PWM va directo en las entradas.
 * Se reutilizan los mismos GPIO que iban a IN1..IN4 del L298N:
 *   Canal A: AIN1=GPIO16, AIN2=GPIO17 -> AOUT1/AOUT2 (rueda derecha)
 *   Canal B: BIN1=GPIO4,  BIN2=GPIO18 -> BOUT1/BOUT2 (rueda izquierda)
 * BIN1 iba en GPIO5, pero es pin de arranque (queda en alto al reiniciar
 * y la rueda izquierda arrancaba a fondo al flashear): se movio a GPIO4.
 * nFAULT (ULT en el modulo) -> GPIO19 (antiguo ENB), con pull-up interno:
 *   es de drenaje abierto y baja a 0 con sobrecorriente, sobretemperatura
 *   o bajo voltaje en VM.
 * nSLEEP (EEP en algunos modulos) debe ir a 3.3 V para habilitar el driver.
 *
 * Tabla del DRV8833 (xIN1 xIN2):
 *   PWM 0   -> adelante, en el tiempo apagado la rueda rueda libre
 *   0   PWM -> atras
 *   0   0   -> libre (coast)
 *   1   1   -> freno
 * Con "PWM / 0" el apagado es rueda libre, igual que el L298N con PWM en EN,
 * asi el control se comporta como antes (salvo que el DRV8833 casi no
 * tiene caida de tension: a igual PWM el motor recibe ~2 V mas).
 *
 * MOTOR_CHANNEL_SWAP_TEST
 *   0 = normal: rueda derecha en canal A, rueda izquierda en canal B
 *   1 = PRUEBA: rueda derecha en canal B, rueda izquierda en canal A
 *       (moviendo tambien los cables de los motores en las borneras)
 * ============================================================
 */
#define MOTOR_CHANNEL_SWAP_TEST  0

#define MOTOR_FAULT_GPIO       GPIO_NUM_19   /* nFAULT / ULT */

#if MOTOR_CHANNEL_SWAP_TEST
/* PRUEBA: canales intercambiados */
#define MOTOR_RIGHT_IN1_GPIO   GPIO_NUM_4    /* BIN1 */
#define MOTOR_RIGHT_IN2_GPIO   GPIO_NUM_18   /* BIN2 */
#define MOTOR_LEFT_IN3_GPIO    GPIO_NUM_16   /* AIN1 */
#define MOTOR_LEFT_IN4_GPIO    GPIO_NUM_17   /* AIN2 */
#else
/* NORMAL */
#define MOTOR_RIGHT_IN1_GPIO   GPIO_NUM_16   /* AIN1 */
#define MOTOR_RIGHT_IN2_GPIO   GPIO_NUM_17   /* AIN2 */
#define MOTOR_LEFT_IN3_GPIO    GPIO_NUM_4    /* BIN1 */
#define MOTOR_LEFT_IN4_GPIO    GPIO_NUM_18   /* BIN2 */
#endif

/*
 * ============================================================
 * LEDC (PWM) configuration: un canal por entrada del driver
 * ============================================================
 */
#define LEDC_SPEED_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_SEL        LEDC_TIMER_0
#define LEDC_FREQUENCY_HZ     1000      /* 1 kHz: mas par a PWM bajo (el DRV8833 admite hasta 50 kHz) */
#define LEDC_DUTY_RESOLUTION  LEDC_TIMER_10_BIT   /* duty range: 0 - 1023 */
#define LEDC_MAX_DUTY         ((1 << 10) - 1)

typedef struct
{
    gpio_num_t     in1_gpio;
    gpio_num_t     in2_gpio;
    ledc_channel_t in1_channel;
    ledc_channel_t in2_channel;
} motor_t;

/*
 * MOTOR_RIGHT_INVERT: 1 = el motor derecho quedo cableado al reves en
 * AOUT1/AOUT2 (al rearmar el sumo, "adelante" lo hacia girar hacia atras).
 * Se intercambian sus dos entradas en vez de mover los cables.
 */
#define MOTOR_RIGHT_INVERT  1

#if MOTOR_RIGHT_INVERT
static const motor_t MOTOR_RIGHT = {
    MOTOR_RIGHT_IN2_GPIO, MOTOR_RIGHT_IN1_GPIO, LEDC_CHANNEL_0, LEDC_CHANNEL_1
};
#else
static const motor_t MOTOR_RIGHT = {
    MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO, LEDC_CHANNEL_0, LEDC_CHANNEL_1
};
#endif

/*
 * MOTOR_LEFT_INVERT: 1 = el motor izquierdo quedo cableado al reves en
 * BOUT1/BOUT2 (con "adelante" en ambas ruedas el carro giraba a la
 * izquierda, gz = +4.2 rad/s). Se intercambian sus dos entradas en vez
 * de mover los cables.
 */
#define MOTOR_LEFT_INVERT  1

#if MOTOR_LEFT_INVERT
static const motor_t MOTOR_LEFT = {
    MOTOR_LEFT_IN4_GPIO, MOTOR_LEFT_IN3_GPIO, LEDC_CHANNEL_2, LEDC_CHANNEL_3
};
#else
static const motor_t MOTOR_LEFT = {
    MOTOR_LEFT_IN3_GPIO, MOTOR_LEFT_IN4_GPIO, LEDC_CHANNEL_2, LEDC_CHANNEL_3
};
#endif

static esp_err_t set_duty(ledc_channel_t channel, uint32_t duty)
{
    esp_err_t ret = ledc_set_duty(LEDC_SPEED_MODE, channel, duty);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ledc_update_duty(LEDC_SPEED_MODE, channel);
}

/* Duty en cada entrada del puente: in1 / in2 en 0..LEDC_MAX_DUTY */
static esp_err_t motor_write(const motor_t *m, uint32_t in1_duty, uint32_t in2_duty)
{
    esp_err_t ret = set_duty(m->in1_channel, in1_duty);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return set_duty(m->in2_channel, in2_duty);
}

/* Velocidad con signo -100..100: positivo = adelante, negativo = atras, 0 = libre */
static esp_err_t motor_set(const motor_t *m, int speed_percent)
{
    int magnitude = abs(speed_percent);
    if (magnitude > 100)
    {
        magnitude = 100;
    }

    uint32_t duty = ((uint32_t)magnitude * LEDC_MAX_DUTY) / 100;

    if (speed_percent > 0)
    {
        return motor_write(m, duty, 0);
    }
    if (speed_percent < 0)
    {
        return motor_write(m, 0, duty);
    }
    return motor_write(m, 0, 0);
}

static esp_err_t motor_channel_init(gpio_num_t gpio, ledc_channel_t channel)
{
    ledc_channel_config_t conf = {
        .gpio_num = gpio,
        .speed_mode = LEDC_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_SEL,
        .duty = 0,
        .hpoint = 0,
    };

    esp_err_t ret = ledc_channel_config(&conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC channel %d (GPIO%d): %s", channel, gpio, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t motor_driver_init(void)
{
    esp_err_t ret;

    gpio_config_t fault_conf = {
        .pin_bit_mask = 1ULL << MOTOR_FAULT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&fault_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure nFAULT GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * ============================================================
     * LEDC timer (shared by the four PWM channels)
     * ============================================================
     */
    ledc_timer_config_t ledc_timer_conf = {
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = LEDC_DUTY_RESOLUTION,
        .timer_num = LEDC_TIMER_SEL,
        .freq_hz = LEDC_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ret = ledc_timer_config(&ledc_timer_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    const motor_t *motors[] = { &MOTOR_RIGHT, &MOTOR_LEFT };
    for (int i = 0; i < 2; i++)
    {
        ret = motor_channel_init(motors[i]->in1_gpio, motors[i]->in1_channel);
        if (ret != ESP_OK)
        {
            return ret;
        }
        ret = motor_channel_init(motors[i]->in2_gpio, motors[i]->in2_channel);
        if (ret != ESP_OK)
        {
            return ret;
        }
    }

#if MOTOR_CHANNEL_SWAP_TEST
    ESP_LOGW(TAG, "PRUEBA: canales del driver INTERCAMBIADOS (izq en canal A, der en canal B)");
#endif
    ESP_LOGI(TAG, "Motor driver initialized (DRV8833)");

    return ESP_OK;
}

bool motor_driver_fault(void)
{
    return gpio_get_level(MOTOR_FAULT_GPIO) == 0;
}

esp_err_t motor_right_set(int8_t signed_speed_percent)
{
    return motor_set(&MOTOR_RIGHT, signed_speed_percent);
}

esp_err_t motor_left_set(int8_t signed_speed_percent)
{
    return motor_set(&MOTOR_LEFT, signed_speed_percent);
}

esp_err_t car_turn_right(uint8_t speed_percent)
{
    esp_err_t ret = motor_set(&MOTOR_RIGHT, speed_percent);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return motor_set(&MOTOR_LEFT, -(int)speed_percent);
}

esp_err_t car_turn_left(uint8_t speed_percent)
{
    esp_err_t ret = motor_set(&MOTOR_RIGHT, -(int)speed_percent);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return motor_set(&MOTOR_LEFT, speed_percent);
}

esp_err_t car_stop(void)
{
    esp_err_t ret = motor_right_set(0);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return motor_left_set(0);
}

/* Freno activo en ambas ruedas: xIN1=xIN2=1, el DRV8833 cortocircuita el motor. */
esp_err_t car_brake(void)
{
    esp_err_t ret = motor_write(&MOTOR_RIGHT, LEDC_MAX_DUTY, LEDC_MAX_DUTY);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return motor_write(&MOTOR_LEFT, LEDC_MAX_DUTY, LEDC_MAX_DUTY);
}
