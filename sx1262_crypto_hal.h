#ifndef SX1262_CRYPTO_HAL_H
#define SX1262_CRYPTO_HAL_H

/**
 * @file sx1262_crypto_hal.h
 * @brief Platform-agnostic cryptographic HAL interface for SX1262 security module
 * 
 * This header defines the abstract interface for cryptographic operations.
 * Platform-specific implementations must provide these functions:
 * 
 *   - ESP32-S3:  sx1262_crypto_hal_esp32.cpp  (Hardware AES accelerator)
 *   - STM32:     sx1262_crypto_hal_stm32.cpp  (Hardware crypto peripheral)
 *   - Generic:   sx1262_crypto_hal_generic.cpp (Software AES, e.g., tiny-AES-c)
 * 
 * The security layer (sx1262_security.cpp) calls these functions without
 * knowing which platform implementation is linked.
 * 
 * @note Only ONE platform implementation should be compiled/linked per build.
 * 
 * Security Design:
 *   - AES-128-CTR mode for stream encryption (counter-based, no padding needed)
 *   - 16-byte key, 16-byte IV (constructed from 4-byte counter + 12-byte nonce)
 *   - Suitable for small LoRa payloads (1-255 bytes)
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
// RETURN CODES
// ============================================================================

/**
 * @brief Crypto HAL return codes
 */
typedef enum {
    CRYPTO_HAL_OK = 0,                    ///< Operation successful
    CRYPTO_HAL_ERROR_INVALID_PARAM = -1,  ///< NULL pointer or invalid parameter
    CRYPTO_HAL_ERROR_INIT = -2,           ///< Initialization failed
    CRYPTO_HAL_ERROR_NOT_INITIALIZED = -3,///< HAL not initialized
    CRYPTO_HAL_ERROR_ENCRYPT = -4,        ///< Encryption operation failed
    CRYPTO_HAL_ERROR_DECRYPT = -5,        ///< Decryption operation failed
    CRYPTO_HAL_ERROR_KEY_SIZE = -6,       ///< Invalid key size
    CRYPTO_HAL_ERROR_PLATFORM = -7        ///< Platform-specific error
} crypto_hal_result_t;

// ============================================================================
// CONSTANTS
// ============================================================================

#define CRYPTO_HAL_AES128_KEY_SIZE   16   ///< AES-128 key size in bytes
#define CRYPTO_HAL_AES128_BLOCK_SIZE 16   ///< AES block size in bytes
#define CRYPTO_HAL_AES128_IV_SIZE    16   ///< IV/Nonce size for CTR mode

// ============================================================================
// INITIALIZATION
// ============================================================================

/**
 * @brief Initialize the cryptographic HAL
 * 
 * Platform-specific initialization of crypto hardware or software library.
 * Must be called before any encrypt/decrypt operations.
 * 
 * ESP32: Initializes hardware AES peripheral
 * Generic: Initializes software AES tables
 * 
 * @return CRYPTO_HAL_OK on success
 * @return CRYPTO_HAL_ERROR_INIT if initialization failed
 * 
 * @note Safe to call multiple times (idempotent)
 */
crypto_hal_result_t crypto_hal_init(void);

/**
 * @brief Deinitialize the cryptographic HAL
 * 
 * Releases any resources allocated by crypto_hal_init().
 * 
 * @note Safe to call even if not initialized
 */
void crypto_hal_deinit(void);

/**
 * @brief Check if crypto HAL is initialized
 * 
 * @return true if initialized and ready for operations
 * @return false if not initialized
 */
bool crypto_hal_is_initialized(void);

// ============================================================================
// AES-128-CTR OPERATIONS
// ============================================================================

/**
 * @brief Encrypt data using AES-128-CTR mode
 * 
 * Encrypts plaintext using AES-128 in Counter (CTR) mode. CTR mode turns
 * AES into a stream cipher, so no padding is required and the output
 * length equals input length.
 * 
 * CTR Mode Operation:
 *   1. IV is used as initial counter value
 *   2. Counter is encrypted with AES to produce keystream
 *   3. Plaintext XORed with keystream produces ciphertext
 *   4. Counter incremented for next block
 * 
 * @param key        AES-128 key (exactly 16 bytes)
 * @param iv         Initialization vector / nonce (exactly 16 bytes)
 *                   For replay protection, construct as: [4-byte counter][12-byte static nonce]
 * @param plaintext  Input data to encrypt
 * @param ciphertext Output buffer for encrypted data (can be same as plaintext for in-place)
 * @param len        Length of data in bytes (1-255 for LoRa payloads)
 * 
 * @return CRYPTO_HAL_OK on success
 * @return CRYPTO_HAL_ERROR_NOT_INITIALIZED if crypto_hal_init() not called
 * @return CRYPTO_HAL_ERROR_INVALID_PARAM if any pointer is NULL or len is 0
 * @return CRYPTO_HAL_ERROR_ENCRYPT if encryption failed
 * 
 * @note In CTR mode, encryption and decryption are the same operation (XOR with keystream)
 * @note Thread safety depends on platform implementation
 * 
 * Example:
 * @code
 *   uint8_t key[16] = { ... };
 *   uint8_t iv[16];
 *   
 *   // Build IV from counter + static nonce
 *   uint32_t counter = 12345;
 *   memcpy(iv, &counter, 4);  // Little-endian counter
 *   memcpy(iv + 4, static_nonce, 12);
 *   
 *   uint8_t plaintext[] = "Hello LoRa!";
 *   uint8_t ciphertext[sizeof(plaintext)];
 *   
 *   crypto_hal_aes128_ctr_encrypt(key, iv, plaintext, ciphertext, sizeof(plaintext));
 * @endcode
 */
crypto_hal_result_t crypto_hal_aes128_ctr_encrypt(
    const uint8_t* key,
    const uint8_t* iv,
    const uint8_t* plaintext,
    uint8_t* ciphertext,
    size_t len
);

/**
 * @brief Decrypt data using AES-128-CTR mode
 * 
 * Decrypts ciphertext using AES-128 in Counter (CTR) mode.
 * 
 * @param key        AES-128 key (exactly 16 bytes, same as used for encryption)
 * @param iv         Initialization vector / nonce (exactly 16 bytes, same as used for encryption)
 * @param ciphertext Input encrypted data
 * @param plaintext  Output buffer for decrypted data (can be same as ciphertext for in-place)
 * @param len        Length of data in bytes
 * 
 * @return CRYPTO_HAL_OK on success
 * @return CRYPTO_HAL_ERROR_NOT_INITIALIZED if crypto_hal_init() not called
 * @return CRYPTO_HAL_ERROR_INVALID_PARAM if any pointer is NULL or len is 0
 * @return CRYPTO_HAL_ERROR_DECRYPT if decryption failed
 * 
 * @note In CTR mode, this is mathematically identical to encryption
 */
crypto_hal_result_t crypto_hal_aes128_ctr_decrypt(
    const uint8_t* key,
    const uint8_t* iv,
    const uint8_t* ciphertext,
    uint8_t* plaintext,
    size_t len
);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Securely zero a memory buffer
 * 
 * Clears sensitive data (keys, plaintext) from memory in a way that
 * won't be optimized away by the compiler.
 * 
 * @param buffer Pointer to buffer to clear
 * @param len    Length of buffer in bytes
 * 
 * @note Always use this instead of memset() for sensitive data
 */
void crypto_hal_secure_zero(void* buffer, size_t len);

/**
 * @brief Get platform-specific crypto implementation name
 * 
 * Returns a string identifying the crypto backend for debugging.
 * 
 * @return Null-terminated string (e.g., "ESP32-HW-AES", "Software-TinyAES")
 */
const char* crypto_hal_get_implementation_name(void);

#ifdef __cplusplus
}
#endif

#endif // SX1262_CRYPTO_HAL_H
