#ifndef SX1262_SECURITY_H
#define SX1262_SECURITY_H

/**
 * @file sx1262_security.h
 * @brief Optional encryption module for SX1262 LoRa driver suite
 * 
 * This module provides AES-128-CTR encryption with replay protection for
 * LoRa payloads. It is designed as a standalone, optional layer that
 * wraps payloads before transmission and unwraps them after reception.
 * 
 * Architecture:
 * @code
 *   ┌─────────────────────────────────────────────────┐
 *   │              APPLICATION LAYER                   │
 *   │  Your sensor/gateway code                       │
 *   ├─────────────────────────────────────────────────┤
 *   │              SECURITY LAYER (this module)       │
 *   │  sx1262_secure_pack() / sx1262_secure_unpack()  │
 *   ├─────────────────────────────────────────────────┤
 *   │              SX1262 DRIVER LAYER                │
 *   │  sx1262_transmit() / sx1262_receive()           │
 *   └─────────────────────────────────────────────────┘
 * @endcode
 * 
 * Features:
 *   - AES-128-CTR encryption (stream cipher, no padding)
 *   - 4-byte explicit counter for replay protection
 *   - Platform-agnostic (uses crypto HAL abstraction)
 *   - Zero overhead when disabled (compile-time option)
 *   - Designed for constrained LoRa payloads (1-251 bytes after header)
 * 
 * Packet Structure (Secured):
 * @code
 *   Byte:  0    1    2    3    4    5    ...   N
 *         [-- Counter (4B) --][-- Encrypted Payload --]
 *         ^                   ^
 *         |                   |
 *         Plaintext           AES-128-CTR encrypted
 *         (for IV derivation)
 * @endcode
 * 
 * Usage Example:
 * @code
 *   // Initialization
 *   sx1262_security_context_t sec_ctx;
 *   uint8_t key[16] = { 0x00, 0x11, 0x22, ... };  // Your 128-bit key
 *   sx1262_security_init(&sec_ctx, key, 0);       // Counter starts at 0
 *   
 *   // Transmit (Sensor side)
 *   uint8_t payload[] = {0x01, 0x02, 0x03, 0x04, 0x05};
 *   uint8_t buffer[64];
 *   memcpy(buffer, payload, sizeof(payload));
 *   
 *   uint8_t secure_len;
 *   if (sx1262_secure_pack(&sec_ctx, buffer, sizeof(payload), &secure_len) == SX1262_SEC_OK) {
 *       sx1262_transmit(buffer, secure_len, 0, NULL);
 *   }
 *   
 *   // Receive (Gateway side)
 *   uint8_t rx_buffer[64];
 *   sx1262_rx_result_t rx_result;
 *   if (sx1262_receive(rx_buffer, sizeof(rx_buffer), 5000, &rx_result) == SX1262_OK) {
 *       uint8_t payload_len;
 *       if (sx1262_secure_unpack(&sec_ctx, rx_buffer, rx_result.payload_length, &payload_len) == SX1262_SEC_OK) {
 *           // rx_buffer now contains decrypted payload
 *           // payload_len is the original payload length
 *       }
 *   }
 * @endcode
 * 
 * Security Considerations:
 *   - Key must be pre-shared securely (not transmitted over LoRa)
 *   - Counter must never repeat with same key (save to NVS on reboot)
 *   - This provides confidentiality, NOT authentication (no MIC/MAC)
 *   - For authenticated encryption, consider future AES-CCM extension
 * 
 * @author Oscar / Claude
 * @date December 2024
 * @version 1.0.0
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// COMPILE-TIME CONFIGURATION
// ============================================================================

/**
 * @brief Maximum payload size that can be secured
 * 
 * LoRa max payload is 255 bytes. With 4-byte counter header,
 * maximum application payload is 251 bytes.
 */
#define SX1262_SEC_MAX_PAYLOAD_SIZE     251

/**
 * @brief Security header size (counter)
 */
#define SX1262_SEC_HEADER_SIZE          4

/**
 * @brief AES-128 key size
 */
#define SX1262_SEC_KEY_SIZE             16

/**
 * @brief Nonce size (static portion of IV)
 * 
 * IV = [4-byte counter][12-byte nonce]
 */
#define SX1262_SEC_NONCE_SIZE           12

/**
 * @brief Full IV size for AES-CTR
 */
#define SX1262_SEC_IV_SIZE              16

/**
 * @brief Default counter window for replay protection
 * 
 * Allows for some packet reordering/loss while still detecting replays.
 * Packets with counter <= (last_rx_counter - window) are rejected.
 */
#define SX1262_SEC_DEFAULT_REPLAY_WINDOW    32

// ============================================================================
// RETURN CODES
// ============================================================================

/**
 * @brief Security module return codes
 */
typedef enum {
    SX1262_SEC_OK = 0,                      ///< Operation successful
    SX1262_SEC_ERROR_INVALID_PARAM = -1,    ///< NULL pointer or invalid parameter
    SX1262_SEC_ERROR_NOT_INITIALIZED = -2,  ///< Context not initialized
    SX1262_SEC_ERROR_BUFFER_TOO_SMALL = -3, ///< Output buffer too small
    SX1262_SEC_ERROR_PAYLOAD_TOO_LARGE = -4,///< Payload exceeds max size
    SX1262_SEC_ERROR_CRYPTO = -5,           ///< Crypto HAL operation failed
    SX1262_SEC_ERROR_REPLAY = -6,           ///< Replay attack detected (counter too old)
    SX1262_SEC_ERROR_COUNTER_OVERFLOW = -7, ///< TX counter overflow (need new key)
    SX1262_SEC_ERROR_MALFORMED = -8         ///< Received packet too short or malformed
} sx1262_security_result_t;

// ============================================================================
// STORAGE CALLBACKS (for counter persistence)
// ============================================================================

/**
 * @brief Storage callback function types
 * 
 * These callbacks enable automatic counter persistence to non-volatile storage.
 * The driver calls save_counter() after each transmission and load_counter()
 * during initialization.
 * 
 * Platform Examples:
 *   - ESP32: Use NVS (Non-Volatile Storage)
 *   - STM32: Use internal flash or external EEPROM
 *   - Arduino: Use EEPROM library
 *   - Generic: Use file system, FRAM, etc.
 */

/**
 * @brief Save counter to persistent storage
 * 
 * Called automatically after each successful TX if storage is configured.
 * 
 * @param counter  Current TX counter value to save
 * @param user_ctx User-provided context pointer (from storage config)
 * @return true on successful save, false on failure
 * 
 * Example ESP32 implementation:
 * @code
 *   bool my_save_counter(uint32_t counter, void* user_ctx) {
 *       nvs_handle_t handle;
 *       if (nvs_open("lora", NVS_READWRITE, &handle) != ESP_OK) return false;
 *       esp_err_t err = nvs_set_u32(handle, "tx_counter", counter);
 *       nvs_commit(handle);
 *       nvs_close(handle);
 *       return (err == ESP_OK);
 *   }
 * @endcode
 */
typedef bool (*sx1262_security_save_counter_fn)(uint32_t counter, void* user_ctx);

/**
 * @brief Load counter from persistent storage
 * 
 * Called during initialization if storage is configured.
 * 
 * @param counter  Pointer to receive loaded counter value
 * @param user_ctx User-provided context pointer (from storage config)
 * @return true if counter was loaded, false if not found or error
 * 
 * Example ESP32 implementation:
 * @code
 *   bool my_load_counter(uint32_t* counter, void* user_ctx) {
 *       nvs_handle_t handle;
 *       if (nvs_open("lora", NVS_READONLY, &handle) != ESP_OK) return false;
 *       esp_err_t err = nvs_get_u32(handle, "tx_counter", counter);
 *       nvs_close(handle);
 *       return (err == ESP_OK);
 *   }
 * @endcode
 */
typedef bool (*sx1262_security_load_counter_fn)(uint32_t* counter, void* user_ctx);

/**
 * @brief Storage configuration structure
 * 
 * Provides callbacks for automatic counter persistence.
 * When configured, the security module will:
 *   - Load counter from storage during init (with safety margin)
 *   - Save counter to storage after each TX
 */
typedef struct {
    sx1262_security_save_counter_fn save;   ///< Save callback (required)
    sx1262_security_load_counter_fn load;   ///< Load callback (required)
    void* user_ctx;                          ///< User context passed to callbacks
    uint32_t save_interval;                  ///< Save every N transmissions (0 = every TX)
    uint32_t safety_margin;                  ///< Add this to loaded counter (default: 100)
} sx1262_security_storage_t;

// ============================================================================
// SECURITY CONTEXT
// ============================================================================

/**
 * @brief Security context structure
 * 
 * Holds all state needed for encryption/decryption operations.
 * Each node should have its own context instance.
 * 
 * @note For bidirectional communication, you may need separate contexts
 *       for TX and RX if using different keys or counter spaces.
 */
typedef struct {
    // Configuration
    bool initialized;                           ///< Context has been initialized
    bool enabled;                               ///< Security is active (can be disabled at runtime)
    
    // Key material
    uint8_t key[SX1262_SEC_KEY_SIZE];          ///< AES-128 encryption key
    uint8_t nonce[SX1262_SEC_NONCE_SIZE];      ///< Static nonce (unique per device/network)
    
    // Counter management
    uint32_t tx_counter;                        ///< Outgoing message counter (increments each TX)
    uint32_t rx_counter_last;                   ///< Last valid received counter
    uint32_t replay_window;                     ///< Replay protection window size
    
    // Storage callbacks (optional, for auto-persistence)
    sx1262_security_storage_t storage;          ///< Storage configuration
    bool storage_configured;                    ///< Storage callbacks are valid
    uint32_t tx_since_last_save;               ///< TX count since last storage save
    
    // Statistics
    uint32_t packets_encrypted;                 ///< Total packets encrypted
    uint32_t packets_decrypted;                 ///< Total packets successfully decrypted
    uint32_t replay_rejections;                 ///< Packets rejected due to replay detection
    uint32_t storage_saves;                     ///< Successful storage saves
    uint32_t storage_failures;                  ///< Failed storage operations
    
} sx1262_security_context_t;

// ============================================================================
// INITIALIZATION
// ============================================================================

/**
 * @brief Initialize security context
 * 
 * Sets up the security context with the provided key and initial counter.
 * Must be called before any pack/unpack operations.
 * 
 * @param ctx              Pointer to security context structure
 * @param key              AES-128 key (exactly 16 bytes, will be copied)
 * @param initial_counter  Starting value for TX counter
 *                         - Use 0 for new deployments
 *                         - Load from NVS for persistence across reboots
 * 
 * @return SX1262_SEC_OK on success
 * @return SX1262_SEC_ERROR_INVALID_PARAM if ctx or key is NULL
 * 
 * @note The key is copied into the context. The original can be cleared.
 * @note After init, security is ENABLED by default.
 * 
 * @warning This function does NOT provide automatic counter persistence!
 *          For production use, prefer sx1262_security_init_with_storage()
 *          or manually save/restore the counter using NVS.
 * 
 * Example:
 * @code
 *   sx1262_security_context_t sec;
 *   uint8_t key[16] = {0x2B, 0x7E, 0x15, 0x16, ...};  // Your pre-shared key
 *   
 *   // Fresh start
 *   sx1262_security_init(&sec, key, 0);
 *   
 *   // Or restore from NVS (manual approach)
 *   uint32_t saved_counter = nvs_read_counter();
 *   sx1262_security_init(&sec, key, saved_counter + 100);  // +100 safety margin
 * @endcode
 */
sx1262_security_result_t sx1262_security_init(
    sx1262_security_context_t* ctx,
    const uint8_t* key,
    uint32_t initial_counter
);

/**
 * @brief Initialize security context with automatic storage (RECOMMENDED)
 * 
 * This is the recommended initialization method for production deployments.
 * It configures automatic counter persistence, ensuring replay protection
 * survives device reboots.
 * 
 * On initialization:
 *   1. Calls load() to retrieve saved counter
 *   2. Adds safety_margin to loaded counter (default: 100)
 *   3. If load fails, uses fallback_counter
 * 
 * After each TX (or every save_interval TXs):
 *   1. Calls save() with current counter
 *   2. Tracks save failures in statistics
 * 
 * @param ctx              Pointer to security context structure
 * @param key              AES-128 key (exactly 16 bytes, will be copied)
 * @param storage          Storage configuration with callbacks
 * @param fallback_counter Counter to use if load() fails (typically 0)
 * 
 * @return SX1262_SEC_OK on success
 * @return SX1262_SEC_ERROR_INVALID_PARAM if ctx, key, or storage callbacks are NULL
 * 
 * Example (ESP32 with NVS):
 * @code
 *   // Define storage callbacks
 *   bool save_counter(uint32_t counter, void* ctx) {
 *       nvs_handle_t h;
 *       if (nvs_open("lora", NVS_READWRITE, &h) != ESP_OK) return false;
 *       nvs_set_u32(h, "counter", counter);
 *       nvs_commit(h);
 *       nvs_close(h);
 *       return true;
 *   }
 *   
 *   bool load_counter(uint32_t* counter, void* ctx) {
 *       nvs_handle_t h;
 *       if (nvs_open("lora", NVS_READONLY, &h) != ESP_OK) return false;
 *       esp_err_t err = nvs_get_u32(h, "counter", counter);
 *       nvs_close(h);
 *       return (err == ESP_OK);
 *   }
 *   
 *   // Initialize with storage
 *   sx1262_security_storage_t storage = {
 *       .save = save_counter,
 *       .load = load_counter,
 *       .user_ctx = NULL,
 *       .save_interval = 0,    // Save every TX (safest)
 *       .safety_margin = 100   // Skip 100 counters on load
 *   };
 *   
 *   sx1262_security_init_with_storage(&sec, key, &storage, 0);
 * @endcode
 */
sx1262_security_result_t sx1262_security_init_with_storage(
    sx1262_security_context_t* ctx,
    const uint8_t* key,
    const sx1262_security_storage_t* storage,
    uint32_t fallback_counter
);

/**
 * @brief Initialize security context with custom nonce
 * 
 * Extended initialization that allows setting a custom static nonce.
 * The nonce should be unique per device or network segment.
 * 
 * @param ctx              Pointer to security context structure
 * @param key              AES-128 key (exactly 16 bytes)
 * @param nonce            Static nonce (exactly 12 bytes)
 * @param initial_counter  Starting value for TX counter
 * 
 * @return SX1262_SEC_OK on success
 * @return SX1262_SEC_ERROR_INVALID_PARAM if any pointer is NULL
 * 
 * @note Use this when you have multiple networks or need nonce separation
 */
sx1262_security_result_t sx1262_security_init_with_nonce(
    sx1262_security_context_t* ctx,
    const uint8_t* key,
    const uint8_t* nonce,
    uint32_t initial_counter
);

/**
 * @brief Deinitialize security context
 * 
 * Securely clears all sensitive data from the context.
 * 
 * @param ctx  Pointer to security context structure
 * 
 * @note Always call this before discarding a context to prevent key leakage
 */
void sx1262_security_deinit(sx1262_security_context_t* ctx);

// ============================================================================
// ENABLE/DISABLE
// ============================================================================

/**
 * @brief Enable security processing
 * 
 * When enabled, pack() encrypts and unpack() decrypts.
 * 
 * @param ctx  Pointer to security context
 */
void sx1262_security_enable(sx1262_security_context_t* ctx);

/**
 * @brief Disable security processing
 * 
 * When disabled, pack() and unpack() become pass-through
 * (no encryption/decryption, no counter added).
 * 
 * @param ctx  Pointer to security context
 * 
 * @note Useful for debugging or mixed secure/insecure networks
 */
void sx1262_security_disable(sx1262_security_context_t* ctx);

/**
 * @brief Check if security is enabled
 * 
 * @param ctx  Pointer to security context
 * @return true if security is enabled
 * @return false if disabled or context not initialized
 */
bool sx1262_security_is_enabled(const sx1262_security_context_t* ctx);

// ============================================================================
// PACK / UNPACK OPERATIONS
// ============================================================================

/**
 * @brief Pack (encrypt) a payload for transmission
 * 
 * Prepends the counter and encrypts the payload in-place.
 * 
 * Before: buffer = [Payload (N bytes)]
 * After:  buffer = [Counter (4B)][Encrypted Payload (N bytes)]
 * 
 * @param ctx           Pointer to initialized security context
 * @param buffer        Buffer containing payload, must have room for 4 extra bytes at front
 *                      On success, contains secured packet ready for transmission
 * @param payload_len   Length of the original payload (1-251 bytes)
 * @param out_total_len Pointer to receive total secured packet length (payload_len + 4)
 * 
 * @return SX1262_SEC_OK on success
 * @return SX1262_SEC_ERROR_NOT_INITIALIZED if context not initialized
 * @return SX1262_SEC_ERROR_INVALID_PARAM if buffer or out_total_len is NULL
 * @return SX1262_SEC_ERROR_PAYLOAD_TOO_LARGE if payload_len > 251
 * @return SX1262_SEC_ERROR_COUNTER_OVERFLOW if TX counter would overflow
 * @return SX1262_SEC_ERROR_CRYPTO if encryption failed
 * 
 * @note The TX counter is incremented on successful encryption
 * @note If security is disabled, just copies payload (no encryption/counter)
 * 
 * @warning The buffer must have at least (payload_len + SX1262_SEC_HEADER_SIZE) bytes capacity
 * 
 * Example:
 * @code
 *   uint8_t buffer[64];  // Must be larger than payload
 *   uint8_t payload[] = {0x01, 0x02, 0x03};
 *   
 *   // Copy payload to buffer (leaving room at front for header)
 *   memcpy(buffer, payload, sizeof(payload));
 *   
 *   uint8_t secure_len;
 *   sx1262_secure_pack(&sec_ctx, buffer, sizeof(payload), &secure_len);
 *   // secure_len == 7 (3 payload + 4 header)
 *   // buffer now ready for sx1262_transmit()
 * @endcode
 */
sx1262_security_result_t sx1262_secure_pack(
    sx1262_security_context_t* ctx,
    uint8_t* buffer,
    uint8_t payload_len,
    uint8_t* out_total_len
);

/**
 * @brief Unpack (decrypt) a received secured payload
 * 
 * Verifies counter for replay protection and decrypts payload in-place.
 * 
 * Before: buffer = [Counter (4B)][Encrypted Payload (N bytes)]
 * After:  buffer = [Decrypted Payload (N bytes)][garbage (4B)]
 * 
 * @param ctx             Pointer to initialized security context
 * @param buffer          Buffer containing received secured packet
 *                        On success, contains decrypted payload at start
 * @param total_len       Total received packet length (including 4-byte counter)
 * @param out_payload_len Pointer to receive original payload length (total_len - 4)
 * 
 * @return SX1262_SEC_OK on success
 * @return SX1262_SEC_ERROR_NOT_INITIALIZED if context not initialized
 * @return SX1262_SEC_ERROR_INVALID_PARAM if buffer or out_payload_len is NULL
 * @return SX1262_SEC_ERROR_MALFORMED if total_len < 5 (minimum: 4B counter + 1B payload)
 * @return SX1262_SEC_ERROR_REPLAY if counter indicates replay attack
 * @return SX1262_SEC_ERROR_CRYPTO if decryption failed
 * 
 * @note The RX counter is updated on successful decryption
 * @note If security is disabled, just shifts buffer (no decryption)
 * 
 * Example:
 * @code
 *   uint8_t rx_buffer[64];
 *   sx1262_rx_result_t rx_result;
 *   
 *   if (sx1262_receive(rx_buffer, sizeof(rx_buffer), 5000, &rx_result) == SX1262_OK) {
 *       uint8_t payload_len;
 *       if (sx1262_secure_unpack(&sec_ctx, rx_buffer, rx_result.payload_length, &payload_len) == SX1262_SEC_OK) {
 *           // Process decrypted payload in rx_buffer[0..payload_len-1]
 *       }
 *   }
 * @endcode
 */
sx1262_security_result_t sx1262_secure_unpack(
    sx1262_security_context_t* ctx,
    uint8_t* buffer,
    uint8_t total_len,
    uint8_t* out_payload_len
);

// ============================================================================
// COUNTER MANAGEMENT
// ============================================================================

/**
 * @brief Get current TX counter value
 * 
 * Use this to save the counter to persistent storage (NVS).
 * 
 * @param ctx  Pointer to security context
 * @return Current TX counter value, or 0 if ctx is NULL
 * 
 * Example:
 * @code
 *   // Save counter periodically or before sleep
 *   uint32_t counter = sx1262_security_get_tx_counter(&sec_ctx);
 *   nvs_write("lora_counter", counter);
 * @endcode
 */
uint32_t sx1262_security_get_tx_counter(const sx1262_security_context_t* ctx);

/**
 * @brief Set TX counter value
 * 
 * Use this to restore counter from persistent storage, or to
 * manually advance the counter (e.g., after detecting desync).
 * 
 * @param ctx      Pointer to security context
 * @param counter  New counter value
 * 
 * @return SX1262_SEC_OK on success
 * @return SX1262_SEC_ERROR_INVALID_PARAM if ctx is NULL
 * 
 * @warning Setting counter backwards can enable replay attacks!
 *          Only set to a value >= current counter.
 */
sx1262_security_result_t sx1262_security_set_tx_counter(
    sx1262_security_context_t* ctx,
    uint32_t counter
);

/**
 * @brief Set replay protection window size
 * 
 * Controls how many packets behind the latest can be accepted.
 * Larger windows tolerate more packet loss/reordering but
 * are more vulnerable to delayed replay attacks.
 * 
 * @param ctx    Pointer to security context
 * @param window Window size (default: 32)
 * 
 * @return SX1262_SEC_OK on success
 * @return SX1262_SEC_ERROR_INVALID_PARAM if ctx is NULL or window is 0
 */
sx1262_security_result_t sx1262_security_set_replay_window(
    sx1262_security_context_t* ctx,
    uint32_t window
);

// ============================================================================
// STATISTICS & DIAGNOSTICS
// ============================================================================

/**
 * @brief Security statistics structure
 */
typedef struct {
    uint32_t packets_encrypted;     ///< Total successful encryptions
    uint32_t packets_decrypted;     ///< Total successful decryptions
    uint32_t replay_rejections;     ///< Packets rejected as replays
    uint32_t tx_counter;            ///< Current TX counter
    uint32_t rx_counter_last;       ///< Last valid RX counter
    uint32_t storage_saves;         ///< Successful storage saves
    uint32_t storage_failures;      ///< Failed storage operations
    bool enabled;                   ///< Current enabled state
    bool storage_configured;        ///< Storage callbacks configured
} sx1262_security_stats_t;

/**
 * @brief Get security statistics
 * 
 * @param ctx   Pointer to security context
 * @param stats Pointer to statistics structure to fill
 * 
 * @return SX1262_SEC_OK on success
 * @return SX1262_SEC_ERROR_INVALID_PARAM if ctx or stats is NULL
 */
sx1262_security_result_t sx1262_security_get_stats(
    const sx1262_security_context_t* ctx,
    sx1262_security_stats_t* stats
);

/**
 * @brief Reset security statistics
 * 
 * Clears packet counts but preserves counters and keys.
 * 
 * @param ctx  Pointer to security context
 */
void sx1262_security_reset_stats(sx1262_security_context_t* ctx);

/**
 * @brief Get human-readable error description
 * 
 * @param result  Security result code
 * @return Null-terminated error description string
 */
const char* sx1262_security_error_to_string(sx1262_security_result_t result);

// ============================================================================
// STORAGE MANAGEMENT
// ============================================================================

/**
 * @brief Force immediate save of counter to storage
 * 
 * Useful before entering deep sleep or when you want to ensure
 * the counter is persisted immediately.
 * 
 * @param ctx  Pointer to security context
 * 
 * @return SX1262_SEC_OK on success
 * @return SX1262_SEC_ERROR_NOT_INITIALIZED if context not initialized
 * @return SX1262_SEC_ERROR_INVALID_PARAM if storage not configured
 * @return SX1262_SEC_ERROR_CRYPTO if save callback returned false
 * 
 * Example:
 * @code
 *   // Before entering deep sleep
 *   sx1262_security_force_save(&sec_ctx);
 *   esp_deep_sleep_start();
 * @endcode
 */
sx1262_security_result_t sx1262_security_force_save(sx1262_security_context_t* ctx);

/**
 * @brief Check if storage is configured
 * 
 * @param ctx  Pointer to security context
 * @return true if storage callbacks are configured
 */
bool sx1262_security_has_storage(const sx1262_security_context_t* ctx);

#ifdef __cplusplus
}
#endif

#endif // SX1262_SECURITY_H
