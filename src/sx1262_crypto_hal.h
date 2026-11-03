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
 *   - AES-128-CCM mode for authenticated encryption (AEAD)
 *   - AES-128-CTR mode retained for backward compatibility
 *   - 16-byte key, 12-byte nonce (CCM standard)
 *   - 4-byte MAC tag (configurable) for message integrity
 *   - Suitable for small LoRa payloads (1-255 bytes)
 * 
 * @author Oscar / Claude
 * @date December 2024 (CCM extension: February 2025)
 * @version 1.1.0
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
    CRYPTO_HAL_ERROR_PLATFORM = -7,       ///< Platform-specific error
    CRYPTO_HAL_ERROR_AUTH_FAILED = -8     ///< CCM authentication tag verification failed
} crypto_hal_result_t;

// ============================================================================
// CONSTANTS
// ============================================================================

#define CRYPTO_HAL_AES128_KEY_SIZE   16   ///< AES-128 key size in bytes
#define CRYPTO_HAL_AES128_BLOCK_SIZE 16   ///< AES block size in bytes
#define CRYPTO_HAL_AES128_IV_SIZE    16   ///< IV/Nonce size for CTR mode
#define CRYPTO_HAL_AES128_CCM_NONCE_SIZE 12 ///< Nonce size for CCM mode (7-13 bytes, 12 standard)

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
// AES-128-CCM OPERATIONS (Authenticated Encryption with Associated Data)
// ============================================================================

/**
 * @brief Encrypt and authenticate data using AES-128-CCM mode
 * 
 * AES-CCM provides both confidentiality (encryption) and authenticity
 * (MAC tag) in a single operation. This is an AEAD (Authenticated
 * Encryption with Associated Data) mode.
 * 
 * CCM Mode Operation:
 *   1. CBC-MAC computed over AAD + plaintext to produce auth tag
 *   2. CTR mode encrypts the plaintext
 *   3. CTR mode encrypts the CBC-MAC to produce the final tag
 * 
 * @param key        AES-128 key (exactly 16 bytes)
 * @param nonce      Nonce (exactly 12 bytes for this implementation)
 *                   Must be unique for each encryption with the same key
 * @param aad        Additional Authenticated Data (authenticated but NOT encrypted)
 *                   Typically the counter bytes. Can be NULL if aad_len is 0.
 * @param aad_len    Length of AAD in bytes
 * @param plaintext  Input data to encrypt
 * @param ciphertext Output buffer for encrypted data (same length as plaintext)
 *                   Can be same as plaintext for in-place encryption
 * @param len        Length of plaintext/ciphertext in bytes (1-255 for LoRa)
 * @param tag        Output buffer for authentication tag
 * @param tag_len    Tag length in bytes (4, 6, 8, 10, 12, 14, or 16)
 *                   4 bytes recommended for LoRa (low overhead, adequate security)
 * 
 * @return CRYPTO_HAL_OK on success
 * @return CRYPTO_HAL_ERROR_NOT_INITIALIZED if crypto_hal_init() not called
 * @return CRYPTO_HAL_ERROR_INVALID_PARAM if any required pointer is NULL or len is 0
 * @return CRYPTO_HAL_ERROR_ENCRYPT if encryption failed
 * 
 * @note Thread safety depends on platform implementation
 * 
 * Example:
 * @code
 *   uint8_t key[16] = { ... };
 *   uint8_t nonce[12] = { ... };
 *   uint8_t counter_bytes[4];     // Used as AAD
 *   
 *   uint8_t plaintext[] = "Hello LoRa!";
 *   uint8_t ciphertext[sizeof(plaintext)];
 *   uint8_t tag[4];               // 4-byte MAC
 *   
 *   crypto_hal_aes128_ccm_encrypt(key, nonce, counter_bytes, 4,
 *                                  plaintext, ciphertext, sizeof(plaintext),
 *                                  tag, 4);
 * @endcode
 */
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
);

/**
 * @brief Decrypt and verify data using AES-128-CCM mode
 * 
 * Decrypts ciphertext and verifies the authentication tag atomically.
 * If the tag does not match, decryption is considered failed and the
 * output buffer contents are undefined (implementations should zero it).
 * 
 * @param key        AES-128 key (exactly 16 bytes, same as used for encryption)
 * @param nonce      Nonce (exactly 12 bytes, same as used for encryption)
 * @param aad        Additional Authenticated Data (same as used for encryption)
 *                   Can be NULL if aad_len is 0.
 * @param aad_len    Length of AAD in bytes
 * @param ciphertext Input encrypted data
 * @param plaintext  Output buffer for decrypted data (can be same as ciphertext)
 * @param len        Length of ciphertext/plaintext in bytes
 * @param tag        Authentication tag to verify
 * @param tag_len    Tag length in bytes (must match encryption tag_len)
 * 
 * @return CRYPTO_HAL_OK on success (tag verified, data authentic)
 * @return CRYPTO_HAL_ERROR_NOT_INITIALIZED if crypto_hal_init() not called
 * @return CRYPTO_HAL_ERROR_INVALID_PARAM if any required pointer is NULL or len is 0
 * @return CRYPTO_HAL_ERROR_AUTH_FAILED if tag verification failed (tampered data!)
 * @return CRYPTO_HAL_ERROR_DECRYPT if decryption failed for other reasons
 * 
 * @warning If CRYPTO_HAL_ERROR_AUTH_FAILED is returned, the plaintext buffer
 *          contents must NOT be used — the data has been tampered with.
 */
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
