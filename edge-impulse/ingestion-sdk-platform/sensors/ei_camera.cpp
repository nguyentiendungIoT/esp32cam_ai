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

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "firmware-sdk/ei_camera_interface.h"
#include "firmware-sdk/ei_device_interface.h"
#include "firmware-sdk/ei_image_lib.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "ei_camera.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "CameraDriver";

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

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 20, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    //.fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

ei_device_snapshot_resolutions_t EiCameraESP32::resolutions[] = {
        { .width = 160, .height = 120 },
        { .width = 320, .height = 240 },
        { .width = 480, .height = 320 }
    };

EiCameraESP32::EiCameraESP32()
{
}


bool EiCameraESP32::is_camera_present(void)
{
    return true;
}

ei_device_snapshot_resolutions_t EiCameraESP32::get_min_resolution(void) {
    return resolutions[0];
}

void EiCameraESP32::get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num) {

    *res = &EiCameraESP32::resolutions[0];
    *res_num = sizeof(EiCameraESP32::resolutions) / sizeof(ei_device_snapshot_resolutions_t);

}

bool EiCameraESP32::set_resolution(const ei_device_snapshot_resolutions_t res) {

    framesize_t frame_size = FRAMESIZE_HVGA;

    switch(res.height) {

    case 96:
    frame_size = FRAMESIZE_QQVGA;
    break;

    case 120:
    frame_size = FRAMESIZE_QQVGA;
    break;

    case 160:
    frame_size = FRAMESIZE_QVGA;
    break;

    case 240:
    frame_size = FRAMESIZE_QVGA;
    break;

    case 320:
    frame_size = FRAMESIZE_HVGA;
    break;

    default:
    break;

    }
    ESP_LOGD(TAG, "frame size %d\n", frame_size);
    camera_config.frame_size = frame_size;
    return true;
}


// see README, need to close and re open for certain operations
bool EiCameraESP32::init(uint16_t width, uint16_t height)
{
    ei_device_snapshot_resolutions_t res = search_resolution(width, height);
    set_resolution(res);

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);

    if (err != ESP_OK)
    {
        ei_printf("ERR: Camera init failed\n");
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);

    // camera warm-up to avoid wrong WB
    ei_sleep(10);
    for (uint8_t i = 0; i < 7; i++) {
        camera_fb_t *fb = esp_camera_fb_get();

        if (!fb) {
            ei_printf("ERR: Camera capture failed during warm-up \n");
            return false;
    }

    esp_camera_fb_return(fb);
    }

    return true;
}

bool EiCameraESP32::deinit()
{
    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("ERR: Camera deinit failed\n");
        return false;
    }

    return true;
}

bool EiCameraESP32::ei_camera_capture_rgb888_packed_big_endian(
    uint8_t *image,
    uint32_t image_size)
{
    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("ERR: Camera capture failed\n");
        return false;
    }

    ESP_LOGD(TAG, "fb res %d %d \n", fb->width, fb->height);

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, image);
    esp_camera_fb_return(fb);

    if(!converted){
        ei_printf("ERR: Conversion failed\n");
        return false;
    }

    return true;
}


bool EiCameraESP32::ei_camera_capture_jpeg(uint8_t **image, uint32_t *image_size)
{
    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("ERR: Camera capture failed\n");
        return false;
    }

    ESP_LOGD(TAG, "fb res %d %d \n", fb->width, fb->height);

    *image = nullptr;
    *image = (uint8_t*)ei_malloc(fb->len);

    memcpy(*image, fb->buf, fb->len);
    memcpy(image_size, &fb->len, sizeof(uint32_t));

    esp_camera_fb_return(fb);

    return true;
}

bool EiCameraESP32::ei_camera_jpeg_to_rgb888(uint8_t *jpeg_image, uint32_t jpeg_image_size,
                                             uint8_t *rgb88_image)
{
    bool converted = fmt2rgb888(jpeg_image, jpeg_image_size, PIXFORMAT_JPEG, rgb88_image);

    if(!converted){
        ESP_LOGE(TAG, "ERR: Conversion failed");
        return false;
    }
    return true;
}

EiCamera *EiCamera::get_camera()
{
    static EiCameraESP32 camera;
    return &camera;
}
