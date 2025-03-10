#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Các hàm inferencing và camera đã được định nghĩa ở các file khác,
// ví dụ: ei_run_impulse.h, ei_device_espressif_esp32.h, ei_camera.h, ...
#include "ei_run_impulse.h"
#include "ei_device_espressif_esp32.h"
#include "ei_camera.h"

//EI_MAX_OVERFLOW_BUFFER_COUNT


// Hàm main (app_main) sẽ gọi ei_start_impulse để khởi chạy inferencing liên tục
extern "C" void app_main(void) {
    // Thiết lập chế độ liên tục, bật debug và không sử dụng tốc độ UART tối đa.
    // Lưu ý: Các hàm như ei_start_impulse đã được triển khai đầy đủ ở các file khác.
    bool continuous_mode = true;   // Chạy liên tục
    bool debug_mode = true;        // Bật debug (in thông tin ra UART)
    bool use_max_uart_speed = false; // Không thay đổi tốc độ UART

    // Gọi hàm khởi chạy inferencing liên tục.
    // Hàm này sẽ tự động thực hiện việc khởi tạo camera, chụp ảnh, xử lý và phân loại.
    ei_start_impulse(continuous_mode, debug_mode, use_max_uart_speed);

    // Vì hàm ei_start_impulse có vòng lặp liên tục,
    // nên app_main() sẽ không kết thúc trừ khi có tín hiệu dừng từ người dùng (theo hàm ei_user_invoke_stop()).
}
