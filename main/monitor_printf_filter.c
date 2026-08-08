#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static bool is_wakenet_startup_message(const char *format)
{
    if (!format) {
        return false;
    }

    // 这些格式串来自 ESP-SR 2.4.6 的预编译 libwakenet.a。
    // 必须精确匹配，避免误伤唤醒、对话、FSR 和鼾声事件输出。
    return strcmp(format,
                  "MC Quantized wakenet9: %s, tigger:v4, mode:%d, p:%d, (%s %s)\n") == 0 ||
           strcmp(format, "%s set threshold for %d word: %f\n") == 0;
}

int __wrap_printf(const char *format, ...)
{
    if (is_wakenet_startup_message(format)) {
        return 0;
    }

    va_list args;
    va_start(args, format);
    int written = vprintf(format, args);
    va_end(args);
    return written;
}
