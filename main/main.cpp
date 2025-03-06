#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_idf_version.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "ei_device_espressif_esp32.h"
#include "ei_at_handlers.h"
#include "ei_classifier_porting.h"
#include "ei_run_impulse.h"
#include "ei_camera.h"
#include "edge-impulse-sdk/dsp/image/processing.hpp"
#include "ei_run_classifier.h"

// Kích thước ảnh đầu vào cho mô hình
#define MODEL_INPUT_WIDTH   96
#define MODEL_INPUT_HEIGHT  96
#define MODEL_INPUT_CH      3

// Độ phân giải camera (ví dụ: 320x240)
#define CAMERA_WIDTH   320
#define CAMERA_HEIGHT  240

// Mảng chứa dữ liệu ảnh chuẩn hóa
static float input_buf[MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CH] = {0};

static uint8_t *rgb888_buf = NULL;
static uint8_t *resized_buf = NULL;

// Hàm lấy dữ liệu từ bộ đệm
int get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, input_buf + offset, length * sizeof(float));
    return 0;
}

// Hàm callback của Edge Impulse cho việc trích xuất đặc trưng từ ảnh
int extract_tflite_features(ei::ei_signal_t* signal, ei::ei_matrix* output_matrix, void* config, float scale) {
    // Copy dữ liệu từ signal sang output_matrix
    int ret = signal->get_data(0, output_matrix->rows * output_matrix->cols, output_matrix->buffer);
    return ret;
}

extern "C" void app_main() {
    // Kiểm tra xem có PSRAM không và cấp phát bộ nhớ cho ảnh
    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0) {
        ei_printf("PSRAM detected! Sử dụng PSRAM để lưu ảnh.\n");
        rgb888_buf = (uint8_t *)heap_caps_malloc(CAMERA_WIDTH * CAMERA_HEIGHT * 3, MALLOC_CAP_SPIRAM);
        resized_buf = (uint8_t *)heap_caps_malloc(MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CH, MALLOC_CAP_SPIRAM);
    } else {
        ei_printf("Không có PSRAM! Sẽ sử dụng bộ nhớ RAM thông thường.\n");
        rgb888_buf = (uint8_t *)malloc(CAMERA_WIDTH * CAMERA_HEIGHT * 3);
        resized_buf = (uint8_t *)malloc(MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CH);
    }

    if (!rgb888_buf || !resized_buf) {
        ei_printf("Cấp phát bộ nhớ thất bại!\n");
        return;
    }

    // Lấy đối tượng camera và khởi tạo
    EiCamera *camera = EiCamera::get_camera();
    if (!camera->init(CAMERA_WIDTH, CAMERA_HEIGHT)) {
        ei_printf("Camera init thất bại!\n");
        return;
    }
    ei_printf("Camera đã init xong, bắt đầu vòng lặp chụp + suy luận.\n");

    while (true) {
        // Chụp ảnh RGB888
        bool capture_status = camera->ei_camera_capture_rgb888_packed_big_endian(
            rgb888_buf,
            CAMERA_WIDTH * CAMERA_HEIGHT * 3
        );
        if (!capture_status) {
            ei_printf("Chụp ảnh thất bại.\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Resize ảnh về kích thước đầu vào của mô hình (96x96)
        int resize_status = ei::image::processing::resize_image(
            rgb888_buf,
            CAMERA_WIDTH, CAMERA_HEIGHT,
            resized_buf,
            MODEL_INPUT_WIDTH,
            MODEL_INPUT_HEIGHT,
            MODEL_INPUT_CH
        );
        if (resize_status != EIDSP_OK) {
            ei_printf("Resize ảnh thất bại, code: %d\n", resize_status);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Chuẩn hóa ảnh (0..255) -> (0..1)
        size_t pixel_count = MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CH;
        for (size_t i = 0; i < pixel_count; i++) {
            input_buf[i] = resized_buf[i] / 255.0f;
        }

        // Tạo signal cho Edge Impulse
        signal_t signal;
        signal.total_length = pixel_count;
        signal.get_data = &get_data;

        // Gọi hàm phân loại
        ei_impulse_result_t result = {0};
        EI_IMPULSE_ERROR ei_status = run_classifier(&signal, &result, false);
        if (ei_status != EI_IMPULSE_OK) {
            ei_printf("run_classifier thất bại (%d)\n", ei_status);
        } else {
            // In kết quả phân loại
            ei_printf("Kết quả phân loại:\n");
            for (int ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                const char *label = result.classification[ix].label;
                if (label == NULL) {
                    label = "(null)";
                }
                ei_printf("  %s: %.5f\n", label, result.classification[ix].value);
            }

            // Nếu có anomaly thì in ra
            #if EI_CLASSIFIER_HAS_ANOMALY == 1
                ei_printf("  Anomaly: %.3f\n", result.anomaly);
            #endif
        }

        // Chờ 1 giây trước khi chụp ảnh tiếp theo
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
