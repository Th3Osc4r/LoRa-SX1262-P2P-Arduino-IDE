/**
 * @file sx1262_crypto_hal_esp32.cpp
 * @brief ESP32-S3 Hardware AES implementation for SX1262 security module
 * 
 * This file implements the crypto HAL interface using the ESP32's
 * hardware AES accelerator for efficient encryption/decryption.
 * 
 * ESP32-S3 Crypto Capabilities:
 *   - Hardware AES-128/192/256 accelerator
 *   - Hardware-accelerated AES-CCM (via mbedtls)
 *   - DMA support for large buffers
 *   - Constant-time operations (side-channel resistant)
 * 
 * This implementation uses the ESP-IDF mbedtls wrapper which automatically
 * routes to hardware acceleration when available.
 * 
 * Supported modes:
 *   - AES-128-CTR (legacy, retained for backward compatibility)
 *   - AES-128-CCM (authenticated encryption, recommended)
 * 
 * @note Compile this file ONLY for ESP32 targets. For other platforms,
 *       compile sx1262_crypto_hal_generic.cpp instead.
 * 
 * @author Oscar / Claude
 * @date December 2024 (CCM extension: February 2025)
 * @version 1.1.0
 */

#include "sx1262_crypto_hal.h"
#include "config.h"

// ESP-IDF / Arduino ESP32 includes
#include <mbedtls/aes.h>
#include <mbedtls/ccm.h>
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
// AES-128-CCM IMPLEMENTATION
// ============================================================================

crypto_hal_result_t crypto_hal_aes128_ccm_encrypt(
    const uint8_t* key,
    const uint8_t* nonce,
    const uint8_t* aad,
    size_t aad_len,
    const uint8_t* plaintext,
    uint8_t* ciphertext,
    size_t len,
    uint8_t* tag,
    size_t tag_len
) {
    // Validate initialization
    if (!g_crypto_initialized) {
        SX1262_LOG_ERROR("[CRYPTO] Not initialized\n");
        return CRYPTO_HAL_ERROR_NOT_INITIALIZED;
    }
    
    // Validate required parameters
    if (key == NULL || nonce == NULL || plaintext == NULL || 
        ciphertext == NULL || tag == NULL) {
        SX1262_LOG_ERROR("[CRYPTO] NULL parameter in CCM encrypt\n");
        return CRYPTO_HAL_ERROR_INVALID_PARAM;
    }
    
    if (len == 0) {
        SX1262_LOG_ERROR("[CRYPTO] Zero length in CCM encrypt\n");
        return CRYPTO_HAL_ERROR_INVALID_PARAM;
    }
    
    // Validate tag length (CCM supports 4, 6, 8, 10, 12, 14, 16)
    if (tag_len < 4 || tag_len > 16 || (tag_len % 2) != 0) {
        SX1262_LOG_ERROR("[CRYPTO] Invalid CCM tag length: %u\n", (unsigned int)tag_len);
        return CRYPTO_HAL_ERROR_INVALID_PARAM;
    }
    
    // AAD can be NULL only if aad_len is 0
    if (aad == NULL && aad_len > 0) {
        SX1262_LOG_ERROR("[CRYPTO] NULL AAD with non-zero length\n");
        return CRYPTO_HAL_ERROR_INVALID_PARAM;
    }
    
    SX1262_LOG_DEBUG("[CRYPTO] CCM encrypting %u bytes (tag=%u, aad=%u)\n",
                    (unsigned int)len, (unsigned int)tag_len, (unsigned int)aad_len);
    
    mbedtls_ccm_context ctx;
    mbedtls_ccm_init(&ctx);
    
    int ret = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, 128);
    if (ret != 0) {
        SX1262_LOG_ERROR("[CRYPTO] CCM setkey failed: %d\n", ret);
        mbedtls_ccm_free(&ctx);
        return CRYPTO_HAL_ERROR_ENCRYPT;
    }
    
    // mbedtls_ccm_encrypt_and_tag performs CCM encryption + MAC generation
    // On ESP32, AES operations are hardware-accelerated
    ret = mbedtls_ccm_encrypt_and_tag(
        &ctx,
        len,                                     // Payload length
        nonce, CRYPTO_HAL_AES128_CCM_NONCE_SIZE, // 12-byte nonce
        aad, aad_len,                            // Additional authenticated data
        plaintext,                               // Input plaintext
        ciphertext,                              // Output ciphertext
        tag, tag_len                             // Output authentication tag
    );
    
    mbedtls_ccm_free(&ctx);
    
    if (ret != 0) {
        SX1262_LOG_ERROR("[CRYPTO] CCM encrypt failed: %d\n", ret);
        return CRYPTO_HAL_ERROR_ENCRYPT;
    }
    
    return CRYPTO_HAL_OK;
}

crypto_hal_result_t crypto_hal_aes128_ccm_decrypt(
    const uint8_t* key,
    const uint8_t* nonce,
    const uint8_t* aad,
    size_t aad_len,
    const uint8_t* ciphertext,
    uint8_t* plaintext,
    size_t len,
    const uint8_t* tag,
    size_t tag_len
) {
    // Validate initialization
    if (!g_crypto_initialized) {
        SX1262_LOG_ERROR("[CRYPTO] Not initialized\n");
        return CRYPTO_HAL_ERROR_NOT_INITIALIZED;
    }
    
    // Validate required parameters
    if (key == NULL || nonce == NULL || ciphertext == NULL || 
        plaintext == NULL || tag == NULL) {
        SX1262_LOG_ERROR("[CRYPTO] NULL parameter in CCM decrypt\n");
        return CRYPTO_HAL_ERROR_INVALID_PARAM;
    }
    
    if (len == 0) {
        SX1262_LOG_ERROR("[CRYPTO] Zero length in CCM decrypt\n");
        return CRYPTO_HAL_ERROR_INVALID_PARAM;
    }
    
    // Validate tag length
    if (tag_len < 4 || tag_len > 16 || (tag_len % 2) != 0) {
        SX1262_LOG_ERROR("[CRYPTO] Invalid CCM tag length: %u\n", (unsigned int)tag_len);
        return CRYPTO_HAL_ERROR_INVALID_PARAM;
    }
    
    // AAD can be NULL only if aad_len is 0
    if (aad == NULL && aad_len > 0) {
        SX1262_LOG_ERROR("[CRYPTO] NULL AAD with non-zero length\n");
        return CRYPTO_HAL_ERROR_INVALID_PARAM;
    }
    
    SX1262_LOG_DEBUG("[CRYPTO] CCM decrypting %u bytes (tag=%u, aad=%u)\n",
                    (unsigned int)len, (unsigned int)tag_len, (unsigned int)aad_len);
    
    mbedtls_ccm_context ctx;
    mbedtls_ccm_init(&ctx);
    
    int ret = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, 128);
    if (ret != 0) {
        SX1262_LOG_ERROR("[CRYPTO] CCM setkey failed: %d\n", ret);
        mbedtls_ccm_free(&ctx);
        return CRYPTO_HAL_ERROR_DECRYPT;
    }
    
    // mbedtls_ccm_auth_decrypt atomically decrypts AND verifies tag
    // Returns MBEDTLS_ERR_CCM_AUTH_FAILED (-0x000F) if tag mismatch
    // On auth failure, plaintext buffer is zeroed by mbedtls
    ret = mbedtls_ccm_auth_decrypt(
        &ctx,
        len,                                     // Ciphertext length
        nonce, CRYPTO_HAL_AES128_CCM_NONCE_SIZE, // 12-byte nonce
        aad, aad_len,                            // Additional authenticated data
        ciphertext,                              // Input ciphertext
        plaintext,                               // Output plaintext
        tag, tag_len                             // Tag to verify
    );
    
    mbedtls_ccm_free(&ctx);
    
    if (ret == MBEDTLS_ERR_CCM_AUTH_FAILED) {
        SX1262_LOG_WARN("[CRYPTO] CCM authentication FAILED - data tampered!\n");
        // Ensure plaintext is zeroed (mbedtls should do this, but belt-and-suspenders)
        crypto_hal_secure_zero(plaintext, len);
        return CRYPTO_HAL_ERROR_AUTH_FAILED;
    }
    
    if (ret != 0) {
        SX1262_LOG_ERROR("[CRYPTO] CCM decrypt failed: %d\n", ret);
        crypto_hal_secure_zero(plaintext, len);
        return CRYPTO_HAL_ERROR_DECRYPT;
    }
    
    return CRYPTO_HAL_OK;
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
    return "ESP32-HW-AES (mbedtls, CTR+CCM)";
}
