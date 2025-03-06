


// Định nghĩa alias: chuyển extract_image_features thành extract_tflite_features
#define extract_image_features extract_tflite_features

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


EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE
// Include các file auto-generated từ Edge Impulse
#include "model_metadata.h"
#include "model_variables.h"

// Include các file porting của Edge Impulse
#include "ei_device_espressif_esp32.h"
#include "ei_at_handlers.h"
#include "ei_classifier_porting.h"
#include "ei_run_impulse.h"
#include "ei_camera.h"

// Thư viện xử lý ảnh và suy luận của Edge Impulse
#include "edge-impulse-sdk/dsp/image/processing.hpp"
#include "ei_run_classifier.h"

// Định nghĩa kích thước ảnh từ metadata
#define MODEL_INPUT_WIDTH   EI_CLASSIFIER_INPUT_WIDTH    // 96
#define MODEL_INPUT_HEIGHT  EI_CLASSIFIER_INPUT_HEIGHT   // 96
#define MODEL_INPUT_CH      3

// Độ phân giải chụp ảnh gốc (ví dụ: 320x240)
#define CAMERA_WIDTH   320
#define CAMERA_HEIGHT  240

// Sử dụng EXT_RAM_BSS_ATTR thay cho EXT_RAM_ATTR (macro cũ đã bị deprecate)
EXT_RAM_BSS_ATTR static float input_buf[MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CH] = {0};

static uint8_t *rgb888_buf = NULL;
static uint8_t *resized_buf = NULL;

/*--------------------*/
// Thêm định nghĩa hàm extract_tflite_features
// Đây là hàm DSP đơn giản, chuyển dữ liệu từ signal sang output_matrix.
// Nếu mô hình của bạn không cần xử lý đặc trưng phức tạp, ta chỉ cần copy toàn bộ dữ liệu.
int extract_tflite_features(ei::ei_signal_t* signal, ei::ei_matrix* output_matrix, void* config, float scale) {
    // Giả sử output_matrix->rows * output_matrix->cols == signal->total_length
    int ret = signal->get_data(0, output_matrix->rows * output_matrix->cols, output_matrix->buffer);
    return ret;
}
/*--------------------*/

int get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, input_buf + offset, length * sizeof(float));
    return 0;
}

extern "C" void app_main()
{
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

    EiCamera *camera = EiCamera::get_camera();
    if (!camera->init(CAMERA_WIDTH, CAMERA_HEIGHT)) {
        ei_printf("Camera init thất bại!\n");
        return;
    }
    ei_printf("Camera đã init xong, bắt đầu vòng lặp chụp + suy luận.\n");

    while (true) {
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

        ei_printf("RGB888 (10 byte đầu): ");
        for (int i = 0; i < 10; i++) {
            ei_printf("%u ", rgb888_buf[i]);
        }
        ei_printf("\n");

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

        ei_printf("Resized (10 byte đầu): ");
        for (int i = 0; i < 10; i++) {
            ei_printf("%u ", resized_buf[i]);
        }
        ei_printf("\n");

        size_t pixel_count = MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CH;
        for (size_t i = 0; i < pixel_count; i++) {
            input_buf[i] = resized_buf[i] / 255.0f;
        }

        ei_printf("Chuẩn hóa (5 giá trị đầu): ");
        for (int i = 0; i < 5; i++) {
            ei_printf("%.3f ", input_buf[i]);
        }
        ei_printf("\n");

        signal_t signal;
        signal.total_length = pixel_count;
        signal.get_data = &get_data;

        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR ei_status = run_classifier(&signal, &result, false);
        if (ei_status != EI_IMPULSE_OK) {
            ei_printf("run_classifier() lỗi: %d\n", ei_status);
        } else {
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
            ei_printf("Object detection bounding boxes:\n");
            ei_printf("  bounding_boxes_count = %u\n", result.bounding_boxes_count);
            if (result.bounding_boxes_count == 0) {
                ei_printf("  Không phát hiện đối tượng nào.\n");
            } else {
                for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
                    ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
                    if (bb.value == 0) continue;
                    ei_printf("  %s (%.2f) [x=%u, y=%u, w=%u, h=%u]\n",
                        bb.label, bb.value,
                        bb.x, bb.y, bb.width, bb.height);
                }
            }
#else
            ei_printf("Kết quả phân loại:\n");
            for (int ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                const char *label = result.classification[ix].label;
                if (label == NULL) label = "(null)";
                ei_printf("  %s: %.5f\n", label, result.classification[ix].value);
            }
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
