#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start WebSocket client, connect to cloud server.
 * Sends {"type":"text","text":"..."} messages.
 * Receives tts_audio_chunk PCM and plays through I2S.
 */
void ws_client_start(const char *uri);

/**
 * Send a text message to the cloud LLM.
 * Wraps in {"type":"text","text":"..."} JSON.
 */
bool ws_client_send_text(const char *text);

/**
 * Send a pre-built JSON string as-is (no wrapping).
 * Use for audio, control messages etc. that are already valid JSON.
 */
bool ws_client_send_raw(const char *json_str);

bool ws_client_send_binary(const uint8_t *data, int len);

bool ws_client_is_connected(void);
uint32_t ws_client_disconnected_ms(void);
bool ws_client_is_tts_active(void);
bool ws_client_is_tts_guard_active(void);
bool ws_client_consume_wake_ack(uint32_t wake_id);
void ws_client_clear_events(void);
bool ws_client_consume_turn_done(void);
bool ws_client_consume_dialog_end(void);
bool ws_client_consume_listen_once_request(void);
bool ws_client_has_listen_once_request(void);
bool ws_client_request_pillow_tilt_to_kpa(float target_kpa, const char *source);

#ifdef __cplusplus
}
#endif
