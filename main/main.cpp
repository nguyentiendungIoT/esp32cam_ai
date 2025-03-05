/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
//#include "esp_spi_flash.h"
/* Include ----------------------------------------------------------------- */
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_idf_version.h"
#include <string.h>
#include <stdio.h>

#include "ei_device_espressif_esp32.h"

#include "ei_at_handlers.h"
#include "ei_classifier_porting.h"
#include "ei_run_impulse.h"

#include "ei_analogsensor.h"
#include "ei_inertial_sensor.h"
#include "ei_camera.h"

#include "esp_camera.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



#include "ei_run_classifier.h"




/// Kích thước mô hình
#define MODEL_INPUT_WIDTH   96
#define MODEL_INPUT_HEIGHT  96
#define MODEL_INPUT_CH      3




EiDeviceInfo *EiDevInfo = dynamic_cast<EiDeviceInfo *>(EiDeviceESP32::get_device());
static ATServer *at;





// Mảng float chứa ảnh chuẩn hóa (0..1)
static float input_buf[MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CH] = {0};

// Hàm callback cho Edge Impulse
int get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, input_buf + offset, length * sizeof(float));
    return 0;
}

extern "C" void app_main()
{
	// 1. Lấy đối tượng camera
	    EiCamera *camera = EiCamera::get_camera();

	    // 2. Khởi tạo camera ở độ phân giải 96x96
	    // (sẽ gọi nội bộ set_resolution, esp_camera_init v.v.)
	    if (!camera->init(MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT)) {
	        ei_printf("Camera init thất bại!\n");
	        return;
	    }

	    ei_printf("Camera đã init xong, bắt đầu vòng lặp chụp + suy luận.\n");

	    while (true) {
	        // 3. Cấp phát (tĩnh) bộ đệm RGB888 cho ảnh 96x96
	        static uint8_t rgb888_buf[MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CH];

	        // 4. Chụp ảnh RGB888
	        bool status = camera->ei_camera_capture_rgb888_packed_big_endian(
	            rgb888_buf,
	            sizeof(rgb888_buf));
	        if (!status) {
	            ei_printf("Chụp ảnh thất bại.\n");
	            vTaskDelay(pdMS_TO_TICKS(1000));
	            continue;
	        }

	        // 5. Chuẩn hóa ảnh (0..255) -> (0..1)
	        size_t pixel_count = MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CH;
	        for (size_t i = 0; i < pixel_count; i++) {
	            input_buf[i] = rgb888_buf[i] / 255.0f;
	        }

	        // 6. Tạo signal cho Edge Impulse
	        signal_t signal;
	        signal.total_length = pixel_count;
	        signal.get_data = &get_data;

	        // 7. Gọi run_classifier
	        ei_impulse_result_t result = {0};
	        EI_IMPULSE_ERROR ei_status = run_classifier(&signal, &result, false);
	        if (ei_status != EI_IMPULSE_OK) {
	            ei_printf("run_classifier thất bại (%d)\n", ei_status);
	        }
	        else {
	            // In kết quả
	            ei_printf("Kết quả phân loại:\n");
	            for (int ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
	                const char *label = result.classification[ix].label;
	                if(label == NULL) {
	                    label = "(null)";
	                }
	                ei_printf("  %s: %.5f\n", label, result.classification[ix].value);
	            }

	#if EI_CLASSIFIER_HAS_ANOMALY == 1
	            ei_printf("  Anomaly: %.3f\n", result.anomaly);
	#endif
	        }

	        // Nghỉ 1 giây
	        vTaskDelay(pdMS_TO_TICKS(5000));
	    }
}
