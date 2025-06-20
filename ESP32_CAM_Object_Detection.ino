/*
 * ESP32-CAM Object Detection for Autonomous Robot Navigation
 * 
 * This sketch uses Object Detection v2 to detect Aruco markers and yellow objects,
 * calculates distance and angle between robot and target, and sends navigation
 * data to the ESP32-WROVER robot controller.
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <Object_Detection_v2_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include "config.h"  // You can put all the sensitive data in the config.h file

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Select camera model
#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#else
#error "Camera model not selected"
#endif

/* Constant defines */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS   320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS   240
#define EI_CAMERA_FRAME_BYTE_SIZE         3

/* Private variables */
static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// Function prototypes
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
float calculateAngle(int x1, int y1, int x2, int y2);
void sendToWrover(int x_x, int y_x, int x_robot, int y_robot, float distance, String position, float angle);

void setup() {
    Serial.begin(115200);
    
    // Print configuration for debugging
    if (DEBUG_MODE) {
        Serial.println("=== ESP32-CAM Configuration ===");
        Serial.print("WiFi SSID: ");
        Serial.println(WIFI_SSID);
        Serial.print("WROVER IP: ");
        Serial.println(WROVER_IP_ADDRESS);
        Serial.print("WROVER Port: ");
        Serial.println(WROVER_PORT);
        Serial.print("Pixels to CM: ");
        Serial.println(PIXELS_TO_CM);
        Serial.println("================================");
    }
    
    // Connect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.print("ESP32-CAM IP Address: ");
    Serial.println(WiFi.localIP());
    
    while (!Serial);
    Serial.println("Object Detection v2 for Autonomous Robot Navigation");
    
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else {
        ei_printf("Camera initialized\r\n");
    }

    ei_printf("\nStarting continuous object detection in 2 seconds...\n");
    ei_sleep(2000);
}

void loop() {
    int center_x_x = -1, center_y_x = -1;
    int center_x_robot = -1, center_y_robot = -1;

    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    if (snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        free(snapshot_buf);
        return;
    }

    ei_printf("Object Detection Results (DSP: %d ms., Classification: %d ms.): \n",
              result.timing.dsp, result.timing.classification);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                  bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);

        int center_x = bb.x + (bb.width / 2);
        int center_y = bb.y + (bb.height / 2);

        if (strcmp(bb.label, "X") == 0) {
            center_x_x = center_x;
            center_y_x = center_y;
        } else if (strcmp(bb.label, "Robot") == 0) {
            center_x_robot = center_x;
            center_y_robot = center_y;
        }
    }

    if (center_x_x != -1 && center_y_x != -1 && center_x_robot != -1 && center_y_robot != -1) {
        float distance_px = sqrt(pow(center_x_robot - center_x_x, 2) + pow(center_y_robot - center_y_x, 2));
        float distance_cm = distance_px * PIXELS_TO_CM;
        float angle = calculateAngle(center_x_robot, center_y_robot, center_x_x, center_y_x);
        
        ei_printf("Distance: %.2f px (%.2f cm), Angle: %.2f°\n", distance_px, distance_cm, angle);
        
        String position;
        position = (center_x_robot < center_x_x) ? "LEFT" : "RIGHT";
        position += (center_y_robot < center_y_x) ? "-ABOVE" : "-BELOW";
        
        sendToWrover(center_x_x, center_y_x, center_x_robot, center_y_robot, distance_cm, position, angle);
    } else {
        ei_printf("Could not detect both objects\n");
    }
#endif

    free(snapshot_buf);
}

float calculateAngle(int x1, int y1, int x2, int y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float angle_rad = atan2(dy, dx);
    float angle_deg = angle_rad * 180.0 / PI;
    if (angle_deg < 0) angle_deg += 360;
    return angle_deg;
}

void sendToWrover(int x_x, int y_x, int x_robot, int y_robot, float distance, String position, float angle) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        
        String jsonData = "{\"x_position\":" + String(x_x) + 
                         ",\"y_position\":" + String(y_x) + 
                         ",\"robot_x\":" + String(x_robot) + 
                         ",\"robot_y\":" + String(y_robot) + 
                         ",\"distance\":" + String(distance) + 
                         ",\"position\":\"" + position + "\"" +
                         ",\"angle\":" + String(angle) + "}";
        
        String url = "http://" + String(WROVER_IP_ADDRESS) + ":" + String(WROVER_PORT) + "/position";
        
        http.begin(url);
        http.addHeader("Content-Type", "application/json");
        
        int httpResponseCode = http.POST(jsonData);
        
        if (httpResponseCode > 0) {
            Serial.print("WROVER Response: ");
            Serial.println(httpResponseCode);
        } else {
            Serial.print("WROVER Error: ");
            Serial.println(httpResponseCode);
        }
        
        http.end();
    }
}

bool ei_camera_init(void) {
    if (is_initialised) return true;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }

    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        ei_printf("Camera deinit failed\n");
    }
    is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);

    if (!converted) {
        ei_printf("Conversion failed\n");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif