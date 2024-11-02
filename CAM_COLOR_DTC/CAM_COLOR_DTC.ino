#define CAMERA_MODEL_AI_THINKER
#include "esp_camera.h"
#include <ESP32Servo.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "camera_pins.h"

#define CAMERA_MODEL_AI_THINKER // Define el modelo de cámara que estás usando
#include "esp_camera.h"
#include <ESP32Servo.h> // Incluye ESP32Servo en lugar de Servo
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "camera_pins.h" 

// Parámetros del color azul en HSV
uint8_t HUE_MIN = 70;
uint8_t HUE_MAX = 150;
uint8_t SAT_MIN = 100;
uint8_t SAT_MAX = 255;
uint8_t VAL_MIN = 100;
uint8_t VAL_MAX = 190;

Servo servo; // Usa la clase ESP32Servo
int servoPin = 12;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Desactivar brownout
  Serial.begin(115200);

  // Configuración de la cámara
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QQVGA;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error inicializando la cámara: 0x%x", err);
    return;
  }

  // Configurar el servomotor
  servo.attach(servoPin);
  servo.write(0);  // Posición inicial
}

void loop() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Error capturando la imagen");
    return;
  }

  int bluePixelCount = 0;
  int totalPixelCount = fb->width * fb->height;

  for (int i = 0; i < fb->len; i += 3) {
    uint8_t r = fb->buf[i];
    uint8_t g = fb->buf[i + 1];
    uint8_t b = fb->buf[i + 2];

    // Conversión de RGB a HSV
    float rNorm = r / 255.0;
    float gNorm = g / 255.0;
    float bNorm = b / 255.0;
    float cMax = max(rNorm, max(gNorm, bNorm));
    float cMin = min(rNorm, min(gNorm, bNorm));
    float delta = cMax - cMin;
    float h = 0, s = 0, v = cMax;

    if (delta != 0) {
      if (cMax == rNorm) {
        h = 60 * fmod(((gNorm - bNorm) / delta), 6);
      } else if (cMax == gNorm) {
        h = 60 * (((bNorm - rNorm) / delta) + 2);
      } else {
        h = 60 * (((rNorm - gNorm) / delta) + 4);
      }
      if (h < 0) h += 360;
      s = delta / cMax;
    } else {
      h = 0;
      s = 0;
    }


    if (h < 0) h += 360;

    // Conversión a formato de 8 bits
    uint8_t h8 = (uint8_t)(h / 2);
    uint8_t s8 = (uint8_t)(s * 255);
    uint8_t v8 = (uint8_t)(v * 255);

    // Crear máscara HSV para color azul basado en los parámetros
    if (h8 >= HUE_MIN && h8 <= HUE_MAX && s8 >= SAT_MIN && s8 <= SAT_MAX && v8 >= VAL_MIN && v8 <= VAL_MAX) {
      bluePixelCount++;
    }
  }

  float bluePercentage = (float)bluePixelCount / totalPixelCount;

  if (bluePercentage > 0.4) {  // Umbral del 40%
    Serial.println("Pelota azul detectada");
    servo.write(90);  // Activar servomotor
    delay(1000);
    servo.write(0);   // Volver a la posición inicial
  } else {
    Serial.println("Pelota no detectada");
  }

  esp_camera_fb_return(fb);
  delay(2000);
}
