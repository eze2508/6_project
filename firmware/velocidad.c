#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include <math.h>

#define SENSOR_PIN 27            // Pin del sensor de imanes
#define MOTOR_PIN 15            // Pin para controlar el motor (PWM)
#define BUTTON_PIN 26           // Pin para el botón de velocidad de crucero
#define POTENTIOMETER_PIN 28    // Pin analógico para el potenciómetro
#define TARGET_SPEED 20         // Velocidad objetivo en RPM para velocidad crucero
#define PULSES_PER_REV 3        // Pulsos por revolución del sensor
#define MOTOR_MAX_DUTY_CYCLE 255  // Valor máximo del ciclo de trabajo PWM
#define POT_CHANGE_THRESHOLD 0.5 // Umbral de cambio del potenciómetro para desactivar crucero
#define POT_DEAD_ZONE 0.2      // Zona muerta para el potenciómetro (para evitar oscilaciones cercanas a 0)

volatile uint32_t pulse_count = 0;
uint32_t last_pulse_time = 0;
float speed = 0.0;
bool cruise_control_active = false;

int current_duty_cycle = 0;    // Ciclo de trabajo actual
const int duty_cycle_step = 20; // Tamaño del paso para cambios suaves en el ciclo de trabajo

float last_pot_value = 0; // Almacena el último valor del potenciómetro

// Callback para la interrupción del sensor de efecto Hall (flanco descendente)
void sensor_callback(uint gpio, uint32_t events) {
    pulse_count++;
}

void calculate_speed() {
    if (pulse_count >= PULSES_PER_REV) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        float time_per_rev = (current_time - last_pulse_time) / 1000.0;  // tiempo en segundos
        speed = (60.0 / time_per_rev);  // RPM
        last_pulse_time = current_time;
        pulse_count = 0;
        printf("Velocidad calculada: %.2f RPM\n", speed);
        sleep_ms(1000);  // Pausa de 1000 ms para hacer más legible la salida
    }
}

void set_motor_speed(float target_speed) {
    int target_duty_cycle = (target_speed / TARGET_SPEED) * MOTOR_MAX_DUTY_CYCLE;
    if (target_duty_cycle > MOTOR_MAX_DUTY_CYCLE) target_duty_cycle = MOTOR_MAX_DUTY_CYCLE;
    if (target_duty_cycle < 0) target_duty_cycle = 0;

    // Evitar que el ciclo de trabajo se sume si está cerca de 0
    if (target_duty_cycle < 1) {
        target_duty_cycle = 0;
    }

    if (current_duty_cycle < target_duty_cycle) {
        current_duty_cycle += duty_cycle_step;
        if (current_duty_cycle > target_duty_cycle) current_duty_cycle = target_duty_cycle;
    } else if (current_duty_cycle > target_duty_cycle) {
        current_duty_cycle -= duty_cycle_step;
        if (current_duty_cycle < target_duty_cycle) current_duty_cycle = target_duty_cycle;
    }

    pwm_set_gpio_level(MOTOR_PIN, current_duty_cycle);
    printf("PWM enviado al motor: %d\n", current_duty_cycle);
    sleep_ms(1000);  // Pausa de 1000 ms
}

float read_potentiometer() {
    // Leer el valor crudo del ADC
    uint16_t raw_value = adc_read();
    float pot_value = (raw_value / 4095.0) * TARGET_SPEED;

    // Si el valor está en la zona muerta, asigna 0
    if (fabs(pot_value) < POT_DEAD_ZONE) {
        pot_value = 0.0;
    }

    printf("Valor del potenciómetro: %.2f\n", pot_value);
    sleep_ms(1000);  // Pausa de 1000 ms
    return pot_value;
}

int main() {
    stdio_init_all();

    // Inicializar el pin del sensor de efecto Hall
    gpio_init(SENSOR_PIN);
    gpio_set_dir(SENSOR_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, &sensor_callback);  // Flanco descendente

    // Inicializar el pin del motor para PWM
    gpio_set_function(MOTOR_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(MOTOR_PIN);
    pwm_set_wrap(slice_num, MOTOR_MAX_DUTY_CYCLE);
    pwm_set_enabled(slice_num, true);

    // Inicializar el pin del botón para el control de crucero con pull-up interno
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);  // Habilitar el resistor pull-up interno

    // Inicializar el ADC para el potenciómetro (GPIO 28)
    adc_init();
    adc_gpio_init(POTENTIOMETER_PIN);  // Inicializa el pin de lectura del ADC para el potenciómetro
    adc_select_input(2);  // Selecciona el canal del ADC correspondiente al GPIO28 (POTENTIOMETER_PIN)

    // Asegúrate de que el pin 26 no se utiliza para ADC
    gpio_set_function(SENSOR_PIN, GPIO_FUNC_NULL); // Deshabilitar cualquier función ADC para el pin 26

    bool last_button_state = true;

    while (true) {
        // Calcular la velocidad del motor
        calculate_speed();

        // Leer el estado del botón y cambiar entre modo manual y crucero
        bool current_button_state = gpio_get(BUTTON_PIN);
        if (!current_button_state && last_button_state) {  // Detecta el flanco de bajada
            cruise_control_active = !cruise_control_active;
            printf("Velocidad crucero %s\n", cruise_control_active ? "activada" : "desactivada");
            sleep_ms(200);  // Pausa para evitar rebotes del botón
        }
        last_button_state = current_button_state;

        // Leer el valor del potenciómetro
        float current_pot_value = read_potentiometer();

        // Desactivar el modo crucero si se detecta un cambio significativo en el potenciómetro
        if (cruise_control_active && fabs(current_pot_value - last_pot_value) > POT_CHANGE_THRESHOLD) {
            cruise_control_active = false;
            printf("Velocidad crucero desactivada por cambio en el potenciómetro.\n");
            sleep_ms(1000);  // Pausa de 1000 ms
        }
        last_pot_value = current_pot_value;

        // Control del motor con transición suave
        if (cruise_control_active) {
            set_motor_speed(speed);  // Usar la velocidad calculada por el sensor de efecto Hall
        } else {
            set_motor_speed(current_pot_value);  // Usar el valor del potenciómetro
        }

        sleep_ms(100);  // Esperar 100ms antes de la siguiente iteración
    }

    return 0;
}