/**
 * @file sx1262_security.cpp
 * @brief Platform-agnostic security module implementation for SX1262 driver suite
 * 
 * This file implements the security layer for LoRa payload encryption.
 * It is completely platform-agnostic - all hardware-specific crypto
 * operations are delegated to the crypto HAL.
 * 
 * Implementation Details:
 * 
 * Encryption (pack):
 *   1. Validate inputs and check counter won't overflow
 *   2. Shift payload right by 4 bytes to make room for counter
 *   3. Write TX counter to first 4 bytes (little-endian)
 *   4. Build IV: [counter (4B)][nonce (12B)]
 *   5. Encrypt payload portion using AES-128-CTR
 *   6. Increment TX counter
 * 
 * Decryption (unpack):
 *   1. Validate inputs (minimum packet size)
 *   2. Extract counter from first 4 bytes
 *   3. Check counter against replay window
 *   4. Build IV from counter + nonce
 *   5. Decrypt payload portion
 *   6. Shift decrypted payload to buffer start
 *   7. Update RX counter tracking
 * 
 * @author Oscar / Claude
 * @date December 2024
 * @version 1.0.0
 */

#include "sx1262_security.h"
#include "sx1262_crypto_hal.h"
#include "config.h"

#include <string.h>

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

/**
 * @brief Build the 16-byte IV from counter and nonce
 * 
 * IV Structure: [Counter (4B little-endian)][Nonce (12B)]
 * 
 * @param counter  32-bit counter value
 * @param nonce    12-byte static nonce
 * @param iv_out   16-byte output buffer for IV
 */
static void build_iv(uint32_t counter, const uint8_t* nonce, uint8_t* iv_out) {
    // Counter in little-endian (LSB first)
    iv_out[0] = (uint8_t)(counter & 0xFF);
    iv_out[1] = (uint8_t)((counter >> 8) & 0xFF);
    iv_out[2] = (uint8_t)((counter >> 16) & 0xFF);
    iv_out[3] = (uint8_t)((counter >> 24) & 0xFF);
    
    // Static nonce
    memcpy(iv_out + 4, nonce, SX1262_SEC_NONCE_SIZE);
}

/**
 * @brief Extract counter from buffer (little-endian)
 * 
 * @param buffer  Buffer containing counter at start
 * @return 32-bit counter value
 */
static uint32_t extract_counter(const uint8_t* buffer) {
    return (uint32_t)buffer[0] |
           ((uint32_t)buffer[1] << 8) |
           ((uint32_t)buffer[2] << 16) |
           ((uint32_t)buffer[3] << 24);
}

/**
 * @brief Write counter to buffer (little-endian)
 * 
 * @param buffer   Buffer to write counter to
 * @param counter  32-bit counter value
 */
static void write_counter(uint8_t* buffer, uint32_t counter) {
    buffer[0] = (uint8_t)(counter & 0xFF);
    buffer[1] = (uint8_t)((counter >> 8) & 0xFF);
    buffer[2] = (uint8_t)((counter >> 16) & 0xFF);
    buffer[3] = (uint8_t)((counter >> 24) & 0xFF);
}

/**
 * @brief Check if received counter is valid (not a replay)
 * 
 * A counter is valid if it's greater than (last_counter - window).
 * This allows for some packet loss and reordering while still
 * detecting replay attacks.
 * 
 * @param rx_counter       Received counter value
 * @param last_rx_counter  Last accepted counter value
 * @param window           Replay window size
 * @return true if counter is valid (not a replay)
 */
static bool is_valid_counter(uint32_t rx_counter, uint32_t last_rx_counter, uint32_t window) {
    // First packet ever - always accept
    if (last_rx_counter == 0) {
        return true;
    }
    
    // Counter must be greater than (last - window)
    // Handle underflow carefully
    uint32_t min_valid;
    if (last_rx_counter >= window) {
        min_valid = last_rx_counter - window;
    } else {
        min_valid = 0;
    }
    
    // Accept if counter > min_valid
    // Also accept if counter > last (normal progression)
    return (rx_counter > min_valid) || (rx_counter > last_rx_counter);
}

// ============================================================================
// DEFAULT NONCE
// ============================================================================

/**
 * Default nonce used when not explicitly provided.
 * In production, each device/network should have a unique nonce.
 * 
 * This is a 12-byte value that, combined with the 4-byte counter,
 * forms the 16-byte IV for AES-CTR.
 */
static const uint8_t DEFAULT_NONCE[SX1262_SEC_NONCE_SIZE] = {
    0x4C, 0x6F, 0x52, 0x61,  // "LoRa" ASCII
    0x53, 0x58, 0x31, 0x32,  // "SX12" ASCII
    0x36, 0x32, 0x00, 0x00   // "62" + padding
};

// ============================================================================
// INITIALIZATION
// ============================================================================

sx1262_security_result_t sx1262_security_init(
    sx1262_security_context_t* ctx,
    const uint8_t* key,
    uint32_t initial_counter
) {
    return sx1262_security_init_with_nonce(ctx, key, DEFAULT_NONCE, initial_counter);
}

sx1262_security_result_t sx1262_security_init_with_storage(
    sx1262_security_context_t* ctx,
    const uint8_t* key,
    const sx1262_security_storage_t* storage,
    uint32_t fallback_counter
) {
    // Validate parameters
    if (ctx == NULL || key == NULL || storage == NULL) {
        SX1262_LOG_ERROR("[SECURITY] Init failed: NULL parameter\n");
        return SX1262_SEC_ERROR_INVALID_PARAM;
    }
    
    if (storage->save == NULL || storage->load == NULL) {
        SX1262_LOG_ERROR("[SECURITY] Init failed: storage callbacks are NULL\n");
        return SX1262_SEC_ERROR_INVALID_PARAM;
    }
    
    // Try to load counter from storage
    uint32_t loaded_counter = 0;
    uint32_t initial_counter = fallback_counter;
    uint32_t safety_margin = (storage->safety_margin > 0) ? storage->safety_margin : 100;
    
    if (storage->load(&loaded_counter, storage->user_ctx)) {
        // Successfully loaded - add safety margin
        initial_counter = loaded_counter + safety_margin;
        SX1262_LOG_INFO("[SECURITY] Loaded counter=%lu, using %lu (+%lu margin)\n",
                       (unsigned long)loaded_counter,
                       (unsigned long)initial_counter,
                       (unsigned long)safety_margin);
    } else {
        SX1262_LOG_WARN("[SECURITY] Failed to load counter, using fallback=%lu\n",
                       (unsigned long)fallback_counter);
    }
    
    // Initialize with the determined counter
    sx1262_security_result_t result = sx1262_security_init_with_nonce(
        ctx, key, DEFAULT_NONCE, initial_counter
    );
    
    if (result != SX1262_SEC_OK) {
        return result;
    }
    
    // Configure storage callbacks
    ctx->storage.save = storage->save;
    ctx->storage.load = storage->load;
    ctx->storage.user_ctx = storage->user_ctx;
    ctx->storage.save_interval = storage->save_interval;
    ctx->storage.safety_margin = safety_margin;
    ctx->storage_configured = true;
    ctx->tx_since_last_save = 0;
    
    // Immediately save the new counter (with safety margin applied)
    if (!storage->save(initial_counter, storage->user_ctx)) {
        SX1262_LOG_WARN("[SECURITY] Initial counter save failed\n");
        ctx->storage_failures++;
    } else {
        ctx->storage_saves++;
        SX1262_LOG_INFO("[SECURITY] Initial counter saved: %lu\n", 
                       (unsigned long)initial_counter);
    }
    
    SX1262_LOG_INFO("[SECURITY] Initialized with storage (interval=%lu)\n",
                   (unsigned long)ctx->storage.save_interval);
    
    return SX1262_SEC_OK;
}

sx1262_security_result_t sx1262_security_init_with_nonce(
    sx1262_security_context_t* ctx,
    const uint8_t* key,
    const uint8_t* nonce,
    uint32_t initial_counter
) {
    // Validate parameters
    if (ctx == NULL || key == NULL || nonce == NULL) {
        SX1262_LOG_ERROR("[SECURITY] Init failed: NULL parameter\n");
        return SX1262_SEC_ERROR_INVALID_PARAM;
    }
    
    // Initialize crypto HAL if not already done
    crypto_hal_result_t crypto_res = crypto_hal_init();
    if (crypto_res != CRYPTO_HAL_OK) {
        SX1262_LOG_ERROR("[SECURITY] Crypto HAL init failed: %d\n", crypto_res);
        return SX1262_SEC_ERROR_CRYPTO;
    }
    
    // Clear context first
    memset(ctx, 0, sizeof(sx1262_security_context_t));
    
    // Copy key and nonce
    memcpy(ctx->key, key, SX1262_SEC_KEY_SIZE);
    memcpy(ctx->nonce, nonce, SX1262_SEC_NONCE_SIZE);
    
    // Initialize counters
    ctx->tx_counter = initial_counter;
    ctx->rx_counter_last = 0;
    ctx->replay_window = SX1262_SEC_DEFAULT_REPLAY_WINDOW;
    
    // Storage not configured by default (use init_with_storage for that)
    ctx->storage_configured = false;
    ctx->tx_since_last_save = 0;
    ctx->storage_saves = 0;
    ctx->storage_failures = 0;
    
    // Mark as initialized and enabled
    ctx->initialized = true;
    ctx->enabled = true;
    
    SX1262_LOG_INFO("[SECURITY] Initialized with counter=%lu, using %s\n", 
                   (unsigned long)initial_counter,
                   crypto_hal_get_implementation_name());
    
    return SX1262_SEC_OK;
}

void sx1262_security_deinit(sx1262_security_context_t* ctx) {
    if (ctx == NULL) {
        return;
    }
    
    // Securely clear sensitive data
    crypto_hal_secure_zero(ctx->key, SX1262_SEC_KEY_SIZE);
    crypto_hal_secure_zero(ctx->nonce, SX1262_SEC_NONCE_SIZE);
    
    // Clear entire context
    memset(ctx, 0, sizeof(sx1262_security_context_t));
    
    SX1262_LOG_INFO("[SECURITY] Context deinitialized\n");
}

// ============================================================================
// ENABLE/DISABLE
// ============================================================================

void sx1262_security_enable(sx1262_security_context_t* ctx) {
    if (ctx != NULL && ctx->initialized) {
        ctx->enabled = true;
        SX1262_LOG_INFO("[SECURITY] Enabled\n");
    }
}

void sx1262_security_disable(sx1262_security_context_t* ctx) {
    if (ctx != NULL && ctx->initialized) {
        ctx->enabled = false;
        SX1262_LOG_INFO("[SECURITY] Disabled\n");
    }
}

bool sx1262_security_is_enabled(const sx1262_security_context_t* ctx) {
    if (ctx == NULL || !ctx->initialized) {
        return false;
    }
    return ctx->enabled;
}

// ============================================================================
// PACK (ENCRYPT FOR TX)
// ============================================================================

sx1262_security_result_t sx1262_secure_pack(
    sx1262_security_context_t* ctx,
    uint8_t* buffer,
    uint8_t payload_len,
    uint8_t* out_total_len
) {
    // Validate parameters
    if (ctx == NULL || buffer == NULL || out_total_len == NULL) {
        SX1262_LOG_ERROR("[SECURITY] Pack failed: NULL parameter\n");
        return SX1262_SEC_ERROR_INVALID_PARAM;
    }
    
    if (!ctx->initialized) {
        SX1262_LOG_ERROR("[SECURITY] Pack failed: not initialized\n");
        return SX1262_SEC_ERROR_NOT_INITIALIZED;
    }
    
    // Validate payload size
    if (payload_len == 0) {
        SX1262_LOG_ERROR("[SECURITY] Pack failed: zero payload\n");
        return SX1262_SEC_ERROR_INVALID_PARAM;
    }
    
    if (payload_len > SX1262_SEC_MAX_PAYLOAD_SIZE) {
        SX1262_LOG_ERROR("[SECURITY] Pack failed: payload too large (%d > %d)\n",
                        payload_len, SX1262_SEC_MAX_PAYLOAD_SIZE);
        return SX1262_SEC_ERROR_PAYLOAD_TOO_LARGE;
    }
    
    // If security is disabled, just return as-is
    if (!ctx->enabled) {
        *out_total_len = payload_len;
        SX1262_LOG_DEBUG("[SECURITY] Pack bypassed (disabled)\n");
        return SX1262_SEC_OK;
    }
    
    // Check for counter overflow
    if (ctx->tx_counter == 0xFFFFFFFF) {
        SX1262_LOG_ERROR("[SECURITY] TX counter overflow - need new key!\n");
        return SX1262_SEC_ERROR_COUNTER_OVERFLOW;
    }
    
    // Step 1: Shift payload right by 4 bytes to make room for counter
    // Work backwards to avoid overwriting
    for (int i = payload_len - 1; i >= 0; i--) {
        buffer[i + SX1262_SEC_HEADER_SIZE] = buffer[i];
    }
    
    // Step 2: Write counter to first 4 bytes
    write_counter(buffer, ctx->tx_counter);
    
    // Step 3: Build IV
    uint8_t iv[SX1262_SEC_IV_SIZE];
    build_iv(ctx->tx_counter, ctx->nonce, iv);
    
    // Step 4: Encrypt payload portion (bytes 4 to end)
    crypto_hal_result_t crypto_res = crypto_hal_aes128_ctr_encrypt(
        ctx->key,
        iv,
        buffer + SX1262_SEC_HEADER_SIZE,  // Plaintext starts after counter
        buffer + SX1262_SEC_HEADER_SIZE,  // Encrypt in-place
        payload_len
    );
    
    // Clear IV from stack
    crypto_hal_secure_zero(iv, sizeof(iv));
    
    if (crypto_res != CRYPTO_HAL_OK) {
        SX1262_LOG_ERROR("[SECURITY] Encryption failed: %d\n", crypto_res);
        return SX1262_SEC_ERROR_CRYPTO;
    }
    
    // Step 5: Update counter and stats
    ctx->tx_counter++;
    ctx->packets_encrypted++;
    
    // Step 6: Auto-save to storage if configured
    if (ctx->storage_configured) {
        ctx->tx_since_last_save++;
        
        // Save if interval reached (0 = every TX)
        bool should_save = (ctx->storage.save_interval == 0) ||
                           (ctx->tx_since_last_save >= ctx->storage.save_interval);
        
        if (should_save) {
            if (ctx->storage.save(ctx->tx_counter, ctx->storage.user_ctx)) {
                ctx->storage_saves++;
                ctx->tx_since_last_save = 0;
                SX1262_LOG_DEBUG("[SECURITY] Counter saved: %lu\n", 
                                (unsigned long)ctx->tx_counter);
            } else {
                ctx->storage_failures++;
                SX1262_LOG_WARN("[SECURITY] Counter save failed (failures=%lu)\n",
                               (unsigned long)ctx->storage_failures);
            }
        }
    }
    
    // Set output length
    *out_total_len = payload_len + SX1262_SEC_HEADER_SIZE;
    
    SX1262_LOG_DEBUG("[SECURITY] Packed %d bytes -> %d bytes (counter=%lu)\n",
                    payload_len, *out_total_len, 
                    (unsigned long)(ctx->tx_counter - 1));
    
    return SX1262_SEC_OK;
}

// ============================================================================
// UNPACK (DECRYPT RECEIVED)
// ============================================================================

sx1262_security_result_t sx1262_secure_unpack(
    sx1262_security_context_t* ctx,
    uint8_t* buffer,
    uint8_t total_len,
    uint8_t* out_payload_len
) {
    // Validate parameters
    if (ctx == NULL || buffer == NULL || out_payload_len == NULL) {
        SX1262_LOG_ERROR("[SECURITY] Unpack failed: NULL parameter\n");
        return SX1262_SEC_ERROR_INVALID_PARAM;
    }
    
    if (!ctx->initialized) {
        SX1262_LOG_ERROR("[SECURITY] Unpack failed: not initialized\n");
        return SX1262_SEC_ERROR_NOT_INITIALIZED;
    }
    
    // If security is disabled, just return as-is
    if (!ctx->enabled) {
        *out_payload_len = total_len;
        SX1262_LOG_DEBUG("[SECURITY] Unpack bypassed (disabled)\n");
        return SX1262_SEC_OK;
    }
    
    // Validate minimum packet size: 4 bytes counter + 1 byte payload minimum
    if (total_len < (SX1262_SEC_HEADER_SIZE + 1)) {
        SX1262_LOG_ERROR("[SECURITY] Unpack failed: packet too short (%d bytes)\n", total_len);
        return SX1262_SEC_ERROR_MALFORMED;
    }
    
    uint8_t payload_len = total_len - SX1262_SEC_HEADER_SIZE;
    
    // Step 1: Extract counter from first 4 bytes
    uint32_t rx_counter = extract_counter(buffer);
    
    // Step 2: Check for replay attack
    if (!is_valid_counter(rx_counter, ctx->rx_counter_last, ctx->replay_window)) {
        SX1262_LOG_WARN("[SECURITY] Replay detected! rx=%lu, last=%lu, window=%lu\n",
                       (unsigned long)rx_counter,
                       (unsigned long)ctx->rx_counter_last,
                       (unsigned long)ctx->replay_window);
        ctx->replay_rejections++;
        return SX1262_SEC_ERROR_REPLAY;
    }
    
    // Step 3: Build IV from received counter
    uint8_t iv[SX1262_SEC_IV_SIZE];
    build_iv(rx_counter, ctx->nonce, iv);
    
    // Step 4: Decrypt payload portion (bytes 4 to end) in-place
    crypto_hal_result_t crypto_res = crypto_hal_aes128_ctr_decrypt(
        ctx->key,
        iv,
        buffer + SX1262_SEC_HEADER_SIZE,  // Ciphertext starts after counter
        buffer + SX1262_SEC_HEADER_SIZE,  // Decrypt in-place
        payload_len
    );
    
    // Clear IV from stack
    crypto_hal_secure_zero(iv, sizeof(iv));
    
    if (crypto_res != CRYPTO_HAL_OK) {
        SX1262_LOG_ERROR("[SECURITY] Decryption failed: %d\n", crypto_res);
        return SX1262_SEC_ERROR_CRYPTO;
    }
    
    // Step 5: Shift decrypted payload to buffer start
    for (uint8_t i = 0; i < payload_len; i++) {
        buffer[i] = buffer[i + SX1262_SEC_HEADER_SIZE];
    }
    
    // Step 6: Update RX counter tracking (only if counter advances)
    if (rx_counter > ctx->rx_counter_last) {
        ctx->rx_counter_last = rx_counter;
    }
    ctx->packets_decrypted++;
    
    // Set output length
    *out_payload_len = payload_len;
    
    SX1262_LOG_DEBUG("[SECURITY] Unpacked %d bytes -> %d bytes (counter=%lu)\n",
                    total_len, payload_len, (unsigned long)rx_counter);
    
    return SX1262_SEC_OK;
}

// ============================================================================
// COUNTER MANAGEMENT
// ============================================================================

uint32_t sx1262_security_get_tx_counter(const sx1262_security_context_t* ctx) {
    if (ctx == NULL || !ctx->initialized) {
        return 0;
    }
    return ctx->tx_counter;
}

sx1262_security_result_t sx1262_security_set_tx_counter(
    sx1262_security_context_t* ctx,
    uint32_t counter
) {
    if (ctx == NULL) {
        return SX1262_SEC_ERROR_INVALID_PARAM;
    }
    
    if (!ctx->initialized) {
        return SX1262_SEC_ERROR_NOT_INITIALIZED;
    }
    
    // Warning if setting backwards (potential security issue)
    if (counter < ctx->tx_counter) {
        SX1262_LOG_WARN("[SECURITY] Setting TX counter backwards: %lu -> %lu\n",
                       (unsigned long)ctx->tx_counter, (unsigned long)counter);
    }
    
    ctx->tx_counter = counter;
    
    SX1262_LOG_INFO("[SECURITY] TX counter set to %lu\n", (unsigned long)counter);
    
    return SX1262_SEC_OK;
}

sx1262_security_result_t sx1262_security_set_replay_window(
    sx1262_security_context_t* ctx,
    uint32_t window
) {
    if (ctx == NULL) {
        return SX1262_SEC_ERROR_INVALID_PARAM;
    }
    
    if (window == 0) {
        SX1262_LOG_ERROR("[SECURITY] Replay window cannot be 0\n");
        return SX1262_SEC_ERROR_INVALID_PARAM;
    }
    
    ctx->replay_window = window;
    
    SX1262_LOG_INFO("[SECURITY] Replay window set to %lu\n", (unsigned long)window);
    
    return SX1262_SEC_OK;
}

// ============================================================================
// STATISTICS & DIAGNOSTICS
// ============================================================================

sx1262_security_result_t sx1262_security_get_stats(
    const sx1262_security_context_t* ctx,
    sx1262_security_stats_t* stats
) {
    if (ctx == NULL || stats == NULL) {
        return SX1262_SEC_ERROR_INVALID_PARAM;
    }
    
    stats->packets_encrypted = ctx->packets_encrypted;
    stats->packets_decrypted = ctx->packets_decrypted;
    stats->replay_rejections = ctx->replay_rejections;
    stats->tx_counter = ctx->tx_counter;
    stats->rx_counter_last = ctx->rx_counter_last;
    stats->storage_saves = ctx->storage_saves;
    stats->storage_failures = ctx->storage_failures;
    stats->enabled = ctx->enabled;
    stats->storage_configured = ctx->storage_configured;
    
    return SX1262_SEC_OK;
}

void sx1262_security_reset_stats(sx1262_security_context_t* ctx) {
    if (ctx == NULL) {
        return;
    }
    
    ctx->packets_encrypted = 0;
    ctx->packets_decrypted = 0;
    ctx->replay_rejections = 0;
    
    SX1262_LOG_INFO("[SECURITY] Statistics reset\n");
}

const char* sx1262_security_error_to_string(sx1262_security_result_t result) {
    switch (result) {
        case SX1262_SEC_OK:                      return "OK";
        case SX1262_SEC_ERROR_INVALID_PARAM:     return "INVALID_PARAM";
        case SX1262_SEC_ERROR_NOT_INITIALIZED:   return "NOT_INITIALIZED";
        case SX1262_SEC_ERROR_BUFFER_TOO_SMALL:  return "BUFFER_TOO_SMALL";
        case SX1262_SEC_ERROR_PAYLOAD_TOO_LARGE: return "PAYLOAD_TOO_LARGE";
        case SX1262_SEC_ERROR_CRYPTO:            return "CRYPTO_ERROR";
        case SX1262_SEC_ERROR_REPLAY:            return "REPLAY_DETECTED";
        case SX1262_SEC_ERROR_COUNTER_OVERFLOW:  return "COUNTER_OVERFLOW";
        case SX1262_SEC_ERROR_MALFORMED:         return "MALFORMED_PACKET";
        default:                                  return "UNKNOWN_ERROR";
    }
}

// ============================================================================
// STORAGE MANAGEMENT
// ============================================================================

sx1262_security_result_t sx1262_security_force_save(sx1262_security_context_t* ctx) {
    if (ctx == NULL) {
        return SX1262_SEC_ERROR_INVALID_PARAM;
    }
    
    if (!ctx->initialized) {
        return SX1262_SEC_ERROR_NOT_INITIALIZED;
    }
    
    if (!ctx->storage_configured) {
        SX1262_LOG_WARN("[SECURITY] Force save called but storage not configured\n");
        return SX1262_SEC_ERROR_INVALID_PARAM;
    }
    
    if (ctx->storage.save(ctx->tx_counter, ctx->storage.user_ctx)) {
        ctx->storage_saves++;
        ctx->tx_since_last_save = 0;
        SX1262_LOG_INFO("[SECURITY] Force save complete: counter=%lu\n",
                       (unsigned long)ctx->tx_counter);
        return SX1262_SEC_OK;
    } else {
        ctx->storage_failures++;
        SX1262_LOG_ERROR("[SECURITY] Force save failed\n");
        return SX1262_SEC_ERROR_CRYPTO;  // Reusing error code for storage failure
    }
}

bool sx1262_security_has_storage(const sx1262_security_context_t* ctx) {
    if (ctx == NULL || !ctx->initialized) {
        return false;
    }
    return ctx->storage_configured;
}
