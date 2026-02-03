/**
 * @file sx1262_crypto_hal_esp32.cpp
 * @brief ESP32-S3 Hardware AES implementation for SX1262 security module
 * 
 * This file implements the crypto HAL interface using the ESP32's
 * hardware AES accelerator for efficient encryption/decryption.
 * 
 * ESP32-S3 Crypto Capabilities:
 *   - Hardware AES-128/192/256 accelerator
 *   - DMA support for large buffers
 *   - Constant-time operations (side-channel resistant)
 * 
 * This implementation uses the ESP-IDF mbedtls wrapper which automatically
 * routes to hardware acceleration when available.
 * 
 * @note Compile this file ONLY for ESP32 targets. For other platforms,
 *       compile sx1262_crypto_hal_generic.cpp instead.
 * 
 * @author Oscar / Claude
 * @date December 2024
 * @version 1.0.0
 */

#include "sx1262_crypto_hal.h"
#include "config.h"

// ESP-IDF / Arduino ESP32 includes
#include <mbedtls/aes.h>
#include <string.h>

// ============================================================================
// MODULE STATE
// ============================================================================

static bool g_crypto_initialized = false;

// ============================================================================
// INITIALIZATION
// ============================================================================

crypto_hal_result_t crypto_hal_init(void) {
    // mbedtls doesn't require explicit global initialization on ESP32
    // The hardware is always available
    
    if (g_crypto_initialized) {
        // Already initialized - idempotent
        return CRYPTO_HAL_OK;
    }
    
    SX1262_LOG_INFO("[CRYPTO] Initializing ESP32 hardware AES\n");
    
    g_crypto_initialized = true;
    
    SX1262_LOG_INFO("[CRYPTO] ESP32 HW-AES ready\n");
    
    return CRYPTO_HAL_OK;
}

void crypto_hal_deinit(void) {
    if (!g_crypto_initialized) {
        return;
    }
    
    SX1262_LOG_INFO("[CRYPTO] Deinitializing ESP32 hardware AES\n");
    
    g_crypto_initialized = false;
}

bool crypto_hal_is_initialized(void) {
    return g_crypto_initialized;
}

// ============================================================================
// AES-128-CTR IMPLEMENTATION
// ============================================================================

/**
 * @brief Internal CTR mode implementation using mbedtls
 * 
 * mbedtls provides AES-CTR through mbedtls_aes_crypt_ctr() which handles
 * counter increment automatically. On ESP32, this routes to hardware.
 */
static crypto_hal_result_t aes128_ctr_crypt(
    const uint8_t* key,
    const uint8_t* iv,
    const uint8_t* input,
    uint8_t* output,
    size_t len
) {
    mbedtls_aes_context ctx;
    int ret;
    
    // Working copy of IV/nonce counter (mbedtls modifies it during operation)
    uint8_t nonce_counter[CRYPTO_HAL_AES128_IV_SIZE];
    uint8_t stream_block[CRYPTO_HAL_AES128_BLOCK_SIZE];
    size_t nc_offset = 0;
    
    // Copy IV to working buffer
    memcpy(nonce_counter, iv, CRYPTO_HAL_AES128_IV_SIZE);
    memset(stream_block, 0, sizeof(stream_block));
    
    // Initialize AES context
    mbedtls_aes_init(&ctx);
    
    // Set encryption key (CTR mode always uses encrypt direction for keystream)
    ret = mbedtls_aes_setkey_enc(&ctx, key, 128);
    if (ret != 0) {
        SX1262_LOG_ERROR("[CRYPTO] AES setkey failed: %d\n", ret);
        mbedtls_aes_free(&ctx);
        return CRYPTO_HAL_ERROR_ENCRYPT;
    }
    
    // Perform CTR encryption/decryption
    // On ESP32, this automatically uses hardware acceleration
    ret = mbedtls_aes_crypt_ctr(
        &ctx,
        len,
        &nc_offset,
        nonce_counter,
        stream_block,
        input,
        output
    );
    
    // Clean up
    mbedtls_aes_free(&ctx);
    
    // Securely clear sensitive working data
    crypto_hal_secure_zero(stream_block, sizeof(stream_block));
    
    if (ret != 0) {
        SX1262_LOG_ERROR("[CRYPTO] AES-CTR operation failed: %d\n", ret);
        return CRYPTO_HAL_ERROR_ENCRYPT;
    }
    
    return CRYPTO_HAL_OK;
}

crypto_hal_result_t crypto_hal_aes128_ctr_encrypt(
    const uint8_t* key,
    const uint8_t* iv,
    const uint8_t* plaintext,
    uint8_t* ciphertext,
    size_t len
) {
    // Validate initialization
    if (!g_crypto_initialized) {
        SX1262_LOG_ERROR("[CRYPTO] Not initialized\n");
        return CRYPTO_HAL_ERROR_NOT_INITIALIZED;
    }
    
    // Validate parameters
    if (key == NULL || iv == NULL || plaintext == NULL || ciphertext == NULL) {
        SX1262_LOG_ERROR("[CRYPTO] NULL parameter in encrypt\n");
        return CRYPTO_HAL_ERROR_INVALID_PARAM;
    }
    
    if (len == 0) {
        SX1262_LOG_ERROR("[CRYPTO] Zero length in encrypt\n");
        return CRYPTO_HAL_ERROR_INVALID_PARAM;
    }
    
    SX1262_LOG_DEBUG("[CRYPTO] Encrypting %u bytes\n", (unsigned int)len);
    
    return aes128_ctr_crypt(key, iv, plaintext, ciphertext, len);
}

crypto_hal_result_t crypto_hal_aes128_ctr_decrypt(
    const uint8_t* key,
    const uint8_t* iv,
    const uint8_t* ciphertext,
    uint8_t* plaintext,
    size_t len
) {
    // Validate initialization
    if (!g_crypto_initialized) {
        SX1262_LOG_ERROR("[CRYPTO] Not initialized\n");
        return CRYPTO_HAL_ERROR_NOT_INITIALIZED;
    }
    
    // Validate parameters
    if (key == NULL || iv == NULL || ciphertext == NULL || plaintext == NULL) {
        SX1262_LOG_ERROR("[CRYPTO] NULL parameter in decrypt\n");
        return CRYPTO_HAL_ERROR_INVALID_PARAM;
    }
    
    if (len == 0) {
        SX1262_LOG_ERROR("[CRYPTO] Zero length in decrypt\n");
        return CRYPTO_HAL_ERROR_INVALID_PARAM;
    }
    
    SX1262_LOG_DEBUG("[CRYPTO] Decrypting %u bytes\n", (unsigned int)len);
    
    // CTR mode: encryption and decryption are identical operations
    return aes128_ctr_crypt(key, iv, ciphertext, plaintext, len);
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void crypto_hal_secure_zero(void* buffer, size_t len) {
    if (buffer == NULL || len == 0) {
        return;
    }
    
    // Use volatile to prevent compiler optimization
    volatile uint8_t* p = (volatile uint8_t*)buffer;
    while (len--) {
        *p++ = 0;
    }
    
    // Memory barrier to ensure writes complete
    // On ESP32, this is handled by the volatile keyword
}

const char* crypto_hal_get_implementation_name(void) {
    return "ESP32-HW-AES (mbedtls)";
}
