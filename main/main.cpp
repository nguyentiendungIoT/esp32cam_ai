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

// Edge Impulse SDK & Porting
#include "ei_device_espressif_esp32.h"
#include "ei_at_handlers.h"
#include "ei_classifier_porting.h"
#include "ei_run_impulse.h"
#include "ei_camera.h"
#include "ei_run_classifier.h"

// Thư viện xử lý ảnh: dùng để resize ảnh
#include "edge-impulse-sdk/dsp/image/processing.hpp"

// Các file tham số mô hình
#include "model_metadata.h"
#include "model_variables.h"

// Định nghĩa alias: chuyển extract_image_features thành extract_tflite_features
#define extract_image_features extract_tflite_features

// Kích thước đầu vào của mô hình (96×96×3)
#define MODEL_INPUT_WIDTH   EI_CLASSIFIER_INPUT_WIDTH   // 96
#define MODEL_INPUT_HEIGHT  EI_CLASSIFIER_INPUT_HEIGHT  // 96
#define MODEL_INPUT_CH      3

// Độ phân giải camera: ESP32-CAM chỉ có thể chụp 169×120
#define CAMERA_WIDTH   160
#define CAMERA_HEIGHT  120

// Cấp phát tensor arena theo kích thước khuyến nghị (căn chỉnh 16-byte)
EXT_RAM_ATTR uint8_t tensor_arena[EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE] __attribute__((aligned(16))) = {0};

// Buffer chứa ảnh chuẩn hóa (int8) – dùng làm input cho mô hình
// Sau chuyển đổi, giá trị pixel sẽ nằm trong khoảng -128 đến 127
EXT_RAM_BSS_ATTR static int8_t input_buf[MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CH] = {0};

// Buffer chứa ảnh RGB888 được chụp từ camera (169×120×3 bytes)
static uint8_t *rgb888_buf = NULL;
// Buffer chứa ảnh đã resize về kích thước mô hình (96×96×3 bytes)
static uint8_t *resized_buf = NULL;

// Hàm callback: cung cấp dữ liệu từ input_buf cho mô hình Edge Impulse (kiểu int8)
int get_data(size_t offset, size_t length, int8_t *out_ptr) {
    memcpy(out_ptr, input_buf + offset, length * sizeof(int8_t));
    return 0;
}

// Hàm chuyển đổi ảnh từ uint8_t (0–255) sang int8_t (0–255 chuyển sang -128..127)
void convert_image_to_int8(const uint8_t *src, int8_t *dst, size_t length) {
    for (size_t i = 0; i < length; i++) {
        dst[i] = (int8_t)((int)src[i] - 128);
    }
}

extern "C" void app_main() {
    // Cấp phát bộ nhớ cho buffer ảnh RGB888 (kích thước: 169×120×3) và buffer ảnh đã resize (96×96×3)
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

    // Khởi tạo camera với độ phân giải 169×120
    EiCamera *camera = EiCamera::get_camera();
    if (!camera->init(CAMERA_WIDTH, CAMERA_HEIGHT)) {
        ei_printf("Camera init thất bại!\n");
        return;
    }
    ei_printf("Camera đã init xong, bắt đầu vòng lặp chụp + suy luận.\n");

    while (true) {
        // Chụp ảnh RGB888 với độ phân giải 169×120
        bool capture_status = camera->ei_camera_capture_rgb888_packed_big_endian(
            rgb888_buf,
            CAMERA_WIDTH * CAMERA_HEIGHT * 3
        );
        if (!capture_status) {
            ei_printf("Chụp ảnh thất bại.\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ei_printf("Chụp ảnh thành công.\n");

        // In 10 byte đầu của ảnh chụp để kiểm tra
        ei_printf("RGB888 (10 byte đầu): ");
        for (int i = 0; i < 10; i++) {
            ei_printf("%u ", rgb888_buf[i]);
        }
        ei_printf("\n");

        // Resize ảnh từ 169×120 xuống 96×96 sử dụng hàm resize_image của Edge Impulse
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
        ei_printf("Resize ảnh thành công về kích thước %dx%d.\n", MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT);

        // Chuyển đổi dữ liệu ảnh: từ uint8_t sang int8_t (0–255 -> -128..127)
        size_t pixel_count = MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CH;
        convert_image_to_int8(resized_buf, input_buf, pixel_count);

        // In 5 giá trị đầu của ảnh đã chuyển đổi để kiểm tra
        ei_printf("Ảnh chuẩn hóa (5 giá trị đầu): ");
        for (int i = 0; i < 5; i++) {
            ei_printf("%d ", input_buf[i]);
        }
        ei_printf("\n");

        // Tạo cấu trúc signal cho mô hình Edge Impulse
        signal_t signal;
        signal.total_length = pixel_count;
        // Ép kiểu hàm get_data phù hợp với callback int8
        signal.get_data = (int (*)(size_t, size_t, void*)) &get_data;

        // Gọi run_classifier để thực hiện suy luận
        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR ei_status = run_classifier(&signal, &result, false);
        if (ei_status != EI_IMPULSE_OK) {
            ei_printf("run_classifier() lỗi: %d\n", ei_status);
        } else {
            ei_printf("Kết quả phân loại:\n");
            for (int ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                const char *label = result.classification[ix].label;
                if (label == NULL) label = "(null)";
                ei_printf("  %s: %.5f\n", label, result.classification[ix].value);
            }
        }

        // Nghỉ 5 giây trước vòng lặp kế tiếp
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
