#include "avatar_storage.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "spi_flash_mmap.h"

#define ENABLE_CUSTOM_AVATAR_STORAGE 1
#define AVATAR_PARTITION_LABEL "avatar"
#define AVATAR_MAGIC "XAVATAR"
#define AVATAR_MAGIC_LEN 8
#define AVATAR_HEADER_VERSION 1
#define AVATAR_HTTP_CHUNK 4096

typedef struct __attribute__((packed)) {
    char magic[AVATAR_MAGIC_LEN];
    uint32_t version;
    uint32_t width;
    uint32_t height;
    uint32_t format;      // 0x52474236 = RGB6
    uint32_t bin_size;
    uint32_t crc32;
    uint32_t reserved;
} avatar_header_t;

static const char *TAG = "avatar_storage";
static const esp_partition_t *s_partition;
static spi_flash_mmap_handle_t s_mmap_handle;
static const uint8_t *s_mapped_pixels;
static uint32_t s_version;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            uint32_t mask = -(crc & 1U);
            crc = (crc >> 1) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

static uint32_t parse_crc32(const char *text)
{
    if (!text || !*text) {
        return 0;
    }
    if (strncmp(text, "0x", 2) == 0 || strncmp(text, "0X", 2) == 0) {
        text += 2;
    }
    return (uint32_t)strtoul(text, NULL, 16);
}

static bool header_valid(const avatar_header_t *h)
{
    return h &&
           memcmp(h->magic, AVATAR_MAGIC, strlen(AVATAR_MAGIC)) == 0 &&
           h->version == AVATAR_HEADER_VERSION &&
           h->width == AVATAR_STORAGE_W &&
           h->height == AVATAR_STORAGE_H &&
           h->format == 0x52474236U &&
           h->bin_size == AVATAR_STORAGE_RGB666_SIZE;
}

static esp_err_t avatar_storage_reload(void)
{
    if (!s_partition) {
        s_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                               ESP_PARTITION_SUBTYPE_ANY,
                                               AVATAR_PARTITION_LABEL);
    }
    if (!s_partition) {
        ESP_LOGW(TAG, "avatar partition not found; use built-in avatar");
        return ESP_ERR_NOT_FOUND;
    }

    avatar_header_t header = {0};
    ESP_RETURN_ON_ERROR(esp_partition_read(s_partition, 0, &header, sizeof(header)),
                        TAG, "read avatar header");
    if (!header_valid(&header)) {
        ESP_LOGI(TAG, "no valid custom avatar; use built-in avatar");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_mmap_handle) {
        spi_flash_munmap(s_mmap_handle);
        s_mmap_handle = 0;
        s_mapped_pixels = NULL;
    }

    const void *mapped = NULL;
    size_t map_size = sizeof(header) + header.bin_size;
    ESP_RETURN_ON_ERROR(esp_partition_mmap(s_partition, 0, map_size,
                                           ESP_PARTITION_MMAP_DATA,
                                           &mapped, &s_mmap_handle),
                        TAG, "mmap avatar partition");
    s_mapped_pixels = (const uint8_t *)mapped + sizeof(header);
    s_version++;
    ESP_LOGI(TAG, "custom LCD avatar active: size=%" PRIu32 " crc=%08" PRIx32,
             header.bin_size, header.crc32);
    return ESP_OK;
}

esp_err_t avatar_storage_init(void)
{
#if !ENABLE_CUSTOM_AVATAR_STORAGE
    ESP_LOGI(TAG, "custom avatar storage disabled; use built-in avatar");
    s_mapped_pixels = NULL;
    return ESP_OK;
#endif
    s_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                           ESP_PARTITION_SUBTYPE_ANY,
                                           AVATAR_PARTITION_LABEL);
    esp_err_t ret = avatar_storage_reload();

    // 自定义头像是可选功能：分区为空、头部无效或旧固件没有该分区时，
    // 都应正常回退到固件内置动态小安，而不是把“没有自定义头像”作为
    // 启动错误交给 ESP_ERROR_CHECK_WITHOUT_ABORT 打印。
    if (ret == ESP_ERR_INVALID_STATE || ret == ESP_ERR_NOT_FOUND) {
        s_mapped_pixels = NULL;
        return ESP_OK;
    }
    return ret;
}

const uint8_t *avatar_storage_get_base(const uint8_t *fallback)
{
#if !ENABLE_CUSTOM_AVATAR_STORAGE
    return fallback;
#endif
    return s_mapped_pixels ? s_mapped_pixels : fallback;
}

bool avatar_storage_custom_active(void)
{
#if !ENABLE_CUSTOM_AVATAR_STORAGE
    return false;
#endif
    return s_mapped_pixels != NULL;
}

uint32_t avatar_storage_get_version(void)
{
#if !ENABLE_CUSTOM_AVATAR_STORAGE
    return 0;
#endif
    return s_version;
}

esp_err_t avatar_storage_download_rgb666(const char *url,
                                         size_t expected_size,
                                         uint32_t expected_crc32)
{
#if !ENABLE_CUSTOM_AVATAR_STORAGE
    (void)url;
    (void)expected_size;
    (void)expected_crc32;
    ESP_LOGW(TAG, "custom avatar download ignored; built-in animation locked");
    return ESP_ERR_NOT_SUPPORTED;
#endif
    if (!url || !*url) {
        return ESP_ERR_INVALID_ARG;
    }
    if (expected_size != AVATAR_STORAGE_RGB666_SIZE) {
        ESP_LOGE(TAG, "bad avatar size: %u", (unsigned)expected_size);
        return ESP_ERR_INVALID_SIZE;
    }
    if (!s_partition) {
        s_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                               ESP_PARTITION_SUBTYPE_ANY,
                                               AVATAR_PARTITION_LABEL);
    }
    if (!s_partition) {
        ESP_LOGE(TAG, "avatar partition not found");
        return ESP_ERR_NOT_FOUND;
    }
    if (sizeof(avatar_header_t) + expected_size > s_partition->size) {
        ESP_LOGE(TAG, "avatar partition too small: %u", (unsigned)s_partition->size);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "downloading LCD avatar: %s", url);
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 30000,
        .buffer_size = AVATAR_HTTP_CHUNK,
        .buffer_size_tx = 1024,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = esp_http_client_open(client, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "http open failed: %s", esp_err_to_name(ret));
        esp_http_client_cleanup(client);
        return ret;
    }

    int64_t content_length = esp_http_client_fetch_headers(client);
    if (content_length > 0 && (size_t)content_length != expected_size) {
        ESP_LOGW(TAG, "content length %lld != expected %u",
                 content_length, (unsigned)expected_size);
    }

    size_t erase_size = sizeof(avatar_header_t) + expected_size;
    erase_size = (erase_size + SPI_FLASH_SEC_SIZE - 1) & ~(SPI_FLASH_SEC_SIZE - 1);
    ret = esp_partition_erase_range(s_partition, 0, erase_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "erase avatar partition failed: %s", esp_err_to_name(ret));
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ret;
    }

    uint8_t *buf = malloc(AVATAR_HTTP_CHUNK);
    if (!buf) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_ERR_NO_MEM;
    }

    size_t offset = 0;
    uint32_t crc = 0;
    while (offset < expected_size) {
        int want = AVATAR_HTTP_CHUNK;
        if (expected_size - offset < (size_t)want) {
            want = (int)(expected_size - offset);
        }
        int n = esp_http_client_read(client, (char *)buf, want);
        if (n < 0) {
            ret = ESP_FAIL;
            break;
        }
        if (n == 0) {
            if (esp_http_client_is_complete_data_received(client)) {
                break;
            }
            continue;
        }
        crc = crc32_update(crc, buf, (size_t)n);
        ret = esp_partition_write(s_partition,
                                  sizeof(avatar_header_t) + offset,
                                  buf,
                                  (size_t)n);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "write avatar failed at %u: %s",
                     (unsigned)offset, esp_err_to_name(ret));
            break;
        }
        offset += (size_t)n;
    }
    free(buf);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (ret != ESP_OK) {
        return ret;
    }
    if (offset != expected_size) {
        ESP_LOGE(TAG, "download incomplete: %u/%u", (unsigned)offset, (unsigned)expected_size);
        return ESP_FAIL;
    }
    if (expected_crc32 && crc != expected_crc32) {
        ESP_LOGE(TAG, "crc mismatch: got=%08" PRIx32 " expected=%08" PRIx32,
                 crc, expected_crc32);
        return ESP_ERR_INVALID_CRC;
    }

    avatar_header_t header = {0};
    memcpy(header.magic, AVATAR_MAGIC, strlen(AVATAR_MAGIC));
    header.version = AVATAR_HEADER_VERSION;
    header.width = AVATAR_STORAGE_W;
    header.height = AVATAR_STORAGE_H;
    header.format = 0x52474236U;
    header.bin_size = (uint32_t)expected_size;
    header.crc32 = crc;
    ret = esp_partition_write(s_partition, 0, &header, sizeof(header));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "write avatar header failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "avatar downloaded OK: bytes=%u crc=%08" PRIx32,
             (unsigned)expected_size, crc);
    return avatar_storage_reload();
}

esp_err_t avatar_storage_clear_custom(void)
{
#if !ENABLE_CUSTOM_AVATAR_STORAGE
    return ESP_OK;
#endif
    if (!s_partition) {
        s_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                               ESP_PARTITION_SUBTYPE_ANY,
                                               AVATAR_PARTITION_LABEL);
    }
    if (!s_partition) {
        return ESP_ERR_NOT_FOUND;
    }

    if (s_mmap_handle) {
        spi_flash_munmap(s_mmap_handle);
        s_mmap_handle = 0;
    }
    s_mapped_pixels = NULL;

    esp_err_t ret = esp_partition_erase_range(s_partition, 0, s_partition->size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "erase custom avatar failed: %s", esp_err_to_name(ret));
        avatar_storage_reload();
        return ret;
    }

    s_version++;
    ESP_LOGI(TAG, "custom avatar cleared; built-in avatar active");
    return ESP_OK;
}

uint32_t avatar_storage_parse_crc32_text(const char *text)
{
    return parse_crc32(text);
}

