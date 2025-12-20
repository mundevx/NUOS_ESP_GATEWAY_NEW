

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_zigbee_type.h"
#include "esp_zigbee_core.h"
#include "driver/uart.h"
#include "mbedtls/base64.h"
#include "mbedtls/aes.h"
#include "esp_log.h"

#include "nuos_nvs.h"
#include "common.h"
#include "XBee.h"
#include "zigbee_2_uart.h"
#include "frame_enc_dec.h"




static const char *TAG                                          = "ESP_ZB_ENC_DEC";

XBee xbee                                                       = XBee();
ZBRxResponse rx                                                 = ZBRxResponse();
ZBTxStatusResponse txStatus                                     = ZBTxStatusResponse();
// AtCommandResponse atResponse                                    = AtCommandResponse();
// RemoteAtCommandResponse remoteAtResponse                        = RemoteAtCommandResponse();
// Rx16Response rx16                                               = Rx16Response();


#define XBEE_ADDR_H                                             0x0013A200
#define XBEE_ADDR_L                                             0x421B379C

uint32_t V_ROUTER_ADDR_H_HEX = XBEE_ADDR_H, V_ROUTER_ADDR_L_HEX = XBEE_ADDR_L;

extern node_module_t device_array[MAX_DEVICES];
extern uint16_t devices_count;

// Encryption key (first 16 bytes, ignore the null terminator)
static const unsigned char C_ENCRYPTION_KEY_2[16] = {
    0x66, 0x61, 0x6c, 0x63, 0x6f, 0x6e, 0x4c, 0x44,
    0x41, 0x43, 0x4f, 0x44, 0x45, 0x2d, 0x2a, 0x2d
    // Note: Original had 17 bytes with null terminator, but AES needs 16 bytes
};

#ifdef __cplusplus
extern "C" {
#endif
extern void nuos_zb_leave_device(node_module_t * module_remove );
#ifdef __cplusplus
}
#endif

void send_xbee_data_over_uart(const char* mdata, uint16_t len, esp_zb_ieee_addr_t ieee_addr, int16_t V_XbeeMsmgNextFrameId);


/**
 * @brief Decrypts AES-encrypted base64 data
 * @param V_RECEIVED_DATA_STR Base64 encoded encrypted string
 * @return Decrypted string (caller must free) or NULL on failure
 */
char* Fn_DECRYPT_DATA(const char* V_RECEIVED_DATA_STR) {
    // Constants
    const uint8_t C_ENCRYPT_DECRYPT_WORD_LENGTH = 16;
    const uint8_t C_WORD_BASE64_LENGTH = 24;
    
    printf("Decrypting data: ");
    printf(" %s\n", V_RECEIVED_DATA_STR);
    
    // Get input length
    size_t V_DATA_TO_DECRYPT_BUFFER_BYTE_LENGTH = strlen(V_RECEIVED_DATA_STR);
    //printf("Data to decrypt length: %zu\n", V_DATA_TO_DECRYPT_BUFFER_BYTE_LENGTH);
    
    // Check if input length is valid (must be multiple of 24)
    if (V_DATA_TO_DECRYPT_BUFFER_BYTE_LENGTH % C_WORD_BASE64_LENGTH != 0) {
        printf("Error: Input length %zu is not multiple of %d\n", 
               V_DATA_TO_DECRYPT_BUFFER_BYTE_LENGTH, C_WORD_BASE64_LENGTH);
        return NULL;
    }
    
    // Compute number of 24-character base64 blocks
    uint16_t V_NO_OF_WORDS_BASE64_IN_RESULT_INT = V_DATA_TO_DECRYPT_BUFFER_BYTE_LENGTH / C_WORD_BASE64_LENGTH;
    //printf("Number of blocks to process: %u\n", V_NO_OF_WORDS_BASE64_IN_RESULT_INT);
    
    // Allocate buffer for concatenated decrypted data
    size_t total_decrypted_size = V_NO_OF_WORDS_BASE64_IN_RESULT_INT * C_ENCRYPT_DECRYPT_WORD_LENGTH + 1;
    char* V_DECRYPTED_DATA_WITH_LENGTH_STR = (char*)calloc(total_decrypted_size, sizeof(char));
    if (!V_DECRYPTED_DATA_WITH_LENGTH_STR) {
        printf("Failed to allocate memory for decrypted data\n");
        return NULL;
    }
    
    // Initialize AES context
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_dec(&aes, C_ENCRYPTION_KEY_2, 128);
    
    size_t total_decrypted_len = 0;
    
    // Process each 24-character base64 block
    for (uint8_t V_IDX_DATA_TO_DECRYPT_WORD = 0; 
         V_IDX_DATA_TO_DECRYPT_WORD < V_NO_OF_WORDS_BASE64_IN_RESULT_INT; 
         V_IDX_DATA_TO_DECRYPT_WORD++) {
        
        // Extract 24-character base64 block
        char V_ENCRYPTED_DATA_WORD_BASE64[C_WORD_BASE64_LENGTH + 1];
        memcpy(V_ENCRYPTED_DATA_WORD_BASE64, 
               V_RECEIVED_DATA_STR + (V_IDX_DATA_TO_DECRYPT_WORD * C_WORD_BASE64_LENGTH), 
               C_WORD_BASE64_LENGTH);
        V_ENCRYPTED_DATA_WORD_BASE64[C_WORD_BASE64_LENGTH] = '\0';
        
        //printf("Block %d base64: %s\n", V_IDX_DATA_TO_DECRYPT_WORD, V_ENCRYPTED_DATA_WORD_BASE64);
        
        // Base64 decode
        unsigned char V_ENCRYPTED_DATA_WORD_DECODED[C_ENCRYPT_DECRYPT_WORD_LENGTH];
        size_t decoded_len;
        
        int ret = mbedtls_base64_decode(V_ENCRYPTED_DATA_WORD_DECODED, 
                                        sizeof(V_ENCRYPTED_DATA_WORD_DECODED),
                                        &decoded_len,
                                        (const unsigned char*)V_ENCRYPTED_DATA_WORD_BASE64,
                                        C_WORD_BASE64_LENGTH);
        
        if (ret != 0 || decoded_len != C_ENCRYPT_DECRYPT_WORD_LENGTH) {
            printf("Base64 decode failed: %d (expected 16 bytes, got %zu)\n", ret, decoded_len);
            mbedtls_aes_free(&aes);
            free(V_DECRYPTED_DATA_WITH_LENGTH_STR);
            return NULL;
        }
        
        // Decrypt the block (ECB mode)
        unsigned char V_DECRYPTED_DATA_WORD[C_ENCRYPT_DECRYPT_WORD_LENGTH];
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, 
                              V_ENCRYPTED_DATA_WORD_DECODED, 
                              V_DECRYPTED_DATA_WORD);
        
        // Append to result string
        memcpy(V_DECRYPTED_DATA_WITH_LENGTH_STR + total_decrypted_len,
               V_DECRYPTED_DATA_WORD,
               C_ENCRYPT_DECRYPT_WORD_LENGTH);
        
        total_decrypted_len += C_ENCRYPT_DECRYPT_WORD_LENGTH;
        
        // printf("Block %d decrypted: ", V_IDX_DATA_TO_DECRYPT_WORD);
        // for (int i = 0; i < C_ENCRYPT_DECRYPT_WORD_LENGTH; i++) {
        //     printf("%c", V_DECRYPTED_DATA_WORD[i] >= 32 ? V_DECRYPTED_DATA_WORD[i] : '.');
        // }
        // printf("\n");
    }
    
    mbedtls_aes_free(&aes);
    
    // Null terminate the concatenated string
    V_DECRYPTED_DATA_WITH_LENGTH_STR[total_decrypted_len] = '\0';
    
    //printf("Full decrypted with length prefix: %s\n", V_DECRYPTED_DATA_WITH_LENGTH_STR);
    
    // Find '(' to separate length from data
    char* paren_ptr = strchr(V_DECRYPTED_DATA_WITH_LENGTH_STR, '(');
    if (!paren_ptr) {
        printf("Error: No '(' found in decrypted data\n");
        free(V_DECRYPTED_DATA_WITH_LENGTH_STR);
        return NULL;
    }
    
    // Extract length (everything before '(')
    size_t length_str_len = paren_ptr - V_DECRYPTED_DATA_WITH_LENGTH_STR;
    if (length_str_len == 0 || length_str_len > 10) { // Reasonable length check
        printf("Error: Invalid length string\n");
        free(V_DECRYPTED_DATA_WITH_LENGTH_STR);
        return NULL;
    }
    
    // Create null-terminated length string
    char length_str[11] = {0};
    strncpy(length_str, V_DECRYPTED_DATA_WITH_LENGTH_STR, length_str_len);
    length_str[length_str_len] = '\0';
    
    //printf("Length string: %s\n", length_str);
    
    // Convert length to integer
    uint16_t V_DECRYPTED_DATA_LENGTH_INT = atoi(length_str);
    //printf("Data length: %u\n", V_DECRYPTED_DATA_LENGTH_INT);
    
    // Calculate where data starts (after '(')
    char* data_start = paren_ptr + 1;
    
    // Check if we have enough data
    size_t data_available = strlen(data_start);
    if (V_DECRYPTED_DATA_LENGTH_INT > data_available) {
        printf("Error: Data length %u exceeds available data %zu\n", 
               V_DECRYPTED_DATA_LENGTH_INT, data_available);
        free(V_DECRYPTED_DATA_WITH_LENGTH_STR);
        return NULL;
    }
    
    // Allocate final result buffer
    char* V_DECRYPTED_DATA_STR = (char*)malloc(V_DECRYPTED_DATA_LENGTH_INT + 1);
    if (!V_DECRYPTED_DATA_STR) {
        printf("Failed to allocate memory for final result\n");
        free(V_DECRYPTED_DATA_WITH_LENGTH_STR);
        return NULL;
    }
    
    // Copy only the specified length
    memcpy(V_DECRYPTED_DATA_STR, data_start, V_DECRYPTED_DATA_LENGTH_INT);
    V_DECRYPTED_DATA_STR[V_DECRYPTED_DATA_LENGTH_INT] = '\0';
    
    printf("Final decrypted data: %s\n", V_DECRYPTED_DATA_STR);
    //printf("Final length: %u\n", V_DECRYPTED_DATA_LENGTH_INT);
    
    // Free intermediate buffer
    free(V_DECRYPTED_DATA_WITH_LENGTH_STR);
    
    return V_DECRYPTED_DATA_STR;
}

/**
 * @brief Wrapper function for easier use with char arrays
 * @param encrypted_data Base64 encrypted data
 * @param decrypted_output Buffer to store decrypted data (must be pre-allocated)
 * @param output_size Size of output buffer
 * @return true on success, false on failure
 */
bool Fn_DECRYPT_DATA_TO_BUFFER(const char* encrypted_data, char* decrypted_output, size_t output_size) {
    char* decrypted = Fn_DECRYPT_DATA(encrypted_data);
    if (!decrypted) {
        return false;
    }
    
    size_t decrypted_len = strlen(decrypted);
    if (decrypted_len >= output_size) {
        free(decrypted);
        return false;
    }
    
    strcpy(decrypted_output, decrypted);
    free(decrypted);
    return true;
}

/**
 * @brief Helper function to free decrypted data
 * @param data Pointer to decrypted data returned by Fn_DECRYPT_DATA
 */
void free_decrypted_data(char* data) {
    if (data) {
        free(data);
    }
}

char* Fn_ENCRYPT_DATA(const char* V_RECEIVED_DATA_STR) {
    printf("Encrypting data : %s\n", V_RECEIVED_DATA_STR);
    
    // Constants - MUST MATCH DECRYPTION FUNCTION
    const uint8_t C_ENCRYPT_DECRYPT_WORD_LENGTH = 16;
    const uint8_t C_WORD_BASE64_LENGTH = 24;
    
    // Step 1: Get data length
    size_t data_len = strlen(V_RECEIVED_DATA_STR);
    
    // Step 2: Create length string
    char len_str[12];
    snprintf(len_str, sizeof(len_str), "%zu", data_len);
    size_t len_str_len = strlen(len_str);
    
    // Step 3: Create formatted string: "length(data"
    // Example: for "ctrl/1*3@2,255!" (13 chars) -> "13(ctrl/1*3@2,255!"
    size_t formatted_len = len_str_len + 1 + data_len; // length + '(' + data
    
    // Step 4: Calculate number of 16-byte blocks needed
    size_t num_blocks = (formatted_len + C_ENCRYPT_DECRYPT_WORD_LENGTH - 1) / C_ENCRYPT_DECRYPT_WORD_LENGTH;
    size_t padded_len = num_blocks * C_ENCRYPT_DECRYPT_WORD_LENGTH;
    
    // Step 5: Create padded buffer
    unsigned char* plaintext_buffer = (unsigned char*)calloc(padded_len, 1);
    if (!plaintext_buffer) {
        printf("Failed to allocate memory for plaintext buffer\n");
        return NULL;
    }
    
    // Step 6: Format the data: "length(data"
    size_t offset = 0;
    memcpy(plaintext_buffer, len_str, len_str_len);
    offset += len_str_len;
    plaintext_buffer[offset] = '(';
    offset += 1;
    memcpy(plaintext_buffer + offset, V_RECEIVED_DATA_STR, data_len);
    // Rest is already zero (from calloc)
    // DO NOT write to plaintext_buffer[padded_len] - it's out of bounds!
    
    printf("Formatted data (hex, %zu bytes): ", padded_len);
    //plaintext_buffer[padded_len] = 0; //this is added to avoid overflow in some cases
    for (size_t i = 0; i < padded_len; i++) {
        printf("%02x ", plaintext_buffer[i]);
    }
    printf("\n");
    
    // Step 7: Initialize AES context
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, C_ENCRYPTION_KEY_2, 128);
    
    // Step 8: Encrypt each block
    unsigned char* encrypted_buffer = (unsigned char*)malloc(padded_len);
    if (!encrypted_buffer) {
        printf("Failed to allocate memory for encrypted buffer\n");
        mbedtls_aes_free(&aes);
        free(plaintext_buffer);
        return NULL;
    }
    
    for (size_t block = 0; block < num_blocks; block++) {
        unsigned char* input_block = plaintext_buffer + (block * C_ENCRYPT_DECRYPT_WORD_LENGTH);
        unsigned char* output_block = encrypted_buffer + (block * C_ENCRYPT_DECRYPT_WORD_LENGTH);
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, input_block, output_block);
    }
    
    mbedtls_aes_free(&aes);
    
    printf("Encrypted (hex): ");
    for (size_t i = 0; i < padded_len; i++) {
        printf("%02x ", encrypted_buffer[i]);
    }
    printf("\n");
    
    // Step 9: Base64 encode each 16-byte block separately and concatenate
    size_t total_base64_len = num_blocks * C_WORD_BASE64_LENGTH;
    char* full_base64_output = (char*)malloc(total_base64_len + 1);
    if (!full_base64_output) {
        printf("Failed to allocate memory for base64 output\n");
        free(plaintext_buffer);
        free(encrypted_buffer);
        return NULL;
    }
    
    for (size_t block = 0; block < num_blocks; block++) {
        unsigned char* encrypted_block = encrypted_buffer + (block * C_ENCRYPT_DECRYPT_WORD_LENGTH);
        char base64_block[C_WORD_BASE64_LENGTH + 1];
        
        size_t base64_len;
        int ret = mbedtls_base64_encode((unsigned char*)base64_block, C_WORD_BASE64_LENGTH + 1, 
                                        &base64_len, encrypted_block, C_ENCRYPT_DECRYPT_WORD_LENGTH);
        
        if (ret != 0 || base64_len != C_WORD_BASE64_LENGTH) {
            printf("Base64 encode error in block %zu: %d (got %zu, expected %d)\n", 
                   block, ret, base64_len, C_WORD_BASE64_LENGTH);
            free(plaintext_buffer);
            free(encrypted_buffer);
            free(full_base64_output);
            return NULL;
        }
        
        base64_block[C_WORD_BASE64_LENGTH] = '\0';
        memcpy(full_base64_output + (block * C_WORD_BASE64_LENGTH), base64_block, C_WORD_BASE64_LENGTH);
    }
    
    full_base64_output[total_base64_len] = '\0';
    
    // Clean up
    free(plaintext_buffer);
    free(encrypted_buffer);
    
    return full_base64_output;
}



/**
 * @brief Wrapper function for easier use
 * @param plaintext_data Data to encrypt
 * @param output_buffer Buffer to store result (must be pre-allocated)
 * @param buffer_size Size of output buffer
 * @return true on success, false on failure
 */
bool Fn_ENCRYPT_DATA_TO_BUFFER(const char* plaintext_data, char* output_buffer, size_t buffer_size) {
    char* encrypted = Fn_ENCRYPT_DATA(plaintext_data);
    if (!encrypted) {
        return false;
    }
    
    size_t encrypted_len = strlen(encrypted);
    if (encrypted_len >= buffer_size) {
        free(encrypted);
        return false;
    }
    
    strcpy(output_buffer, encrypted);
    free(encrypted);
    return true;
}

/**
 * @brief Helper function to free encrypted data
 * @param data Pointer to encrypted data returned by Fn_ENCRYPT_DATA
 */
void free_encrypted_data(char* data) {
    if (data) {
        free(data);
    }
}


/////////////////////////////////////////////////////////////////////////////////

void convert_xbee_to_zigbee_addr(const XBeeAddress64 &xbee_addr , esp_zb_ieee_addr_t zigbee_addr) {
    XBeeAddress64& non_const_xbee = const_cast<XBeeAddress64&>(xbee_addr);
    uint32_t msb = non_const_xbee.getMsb();
    uint32_t lsb = non_const_xbee.getLsb();
    
    // Create temporary 32-bit variables in network byte order
    uint32_t msb_be = __builtin_bswap32(msb);  // Convert to big-endian if needed
    uint32_t lsb_be = __builtin_bswap32(lsb);
    
    // Copy to output array
    memcpy(&zigbee_addr[4], &msb_be, 4);
    memcpy(&zigbee_addr[0], &lsb_be, 4);
    
    // printf("Converted XBee to Zigbee (memcpy): ");
    // for (int i = 0; i < 8; i++) {
    //     printf("%02x", zigbee_addr[i]);
    // }
    // printf("\n");
}


// Helper function to convert esp_zb_ieee_addr_t to XBeeAddress64
void convert_zigbee_to_xbee_addr(const esp_zb_ieee_addr_t zigbee_addr, 
                                 uint32_t* msb, uint32_t* lsb) {
    // Convert big-endian 8-byte array to two 32-bit integers
    *msb = (zigbee_addr[0] << 24) | (zigbee_addr[1] << 16) | 
           (zigbee_addr[2] << 8) | zigbee_addr[3];
    
    *lsb = (zigbee_addr[4] << 24) | (zigbee_addr[5] << 16) | 
           (zigbee_addr[6] << 8) | zigbee_addr[7];
}

static void build_ctrl_string(node_module_t *node, int start_idx, int idx, char *out, size_t out_size)
{
    int len = 0;
    if (start_idx == 0) {
        // Full array form
        len = snprintf(out, out_size, "ctrl/%d*%u@0,[", node->room_id, node->module_id);
        for (int i = 0; i < MAX_DEVICES_IN_MODULE; i++) {
            int val;
            if (i < node->ep_counts) {
                // real endpoint present
                val = node->endpoint[i].device_t.level;   // adjust field as needed
            } else {
                // empty slot
                val = -255;
            }
            len += snprintf(out + len, out_size - len,
                            (i == MAX_DEVICES_IN_MODULE - 1) ? "%d" : "%d,",
                            val);
        }
        snprintf(out + len, out_size - len, "]!");
    } else {
        // Single endpoint form: "addc/room_id*module_id@idx,255!"
        // clamp index into 1..MAX_DEVICES_IN_MODULE
        if (start_idx > MAX_DEVICES_IN_MODULE) start_idx = MAX_DEVICES_IN_MODULE;
        snprintf(out, out_size, "ctrl/%d*%u@%d,%d!",
                 node->room_id, node->module_id, node->endpoint[idx].ep, node->endpoint[idx].device_t.level);
    }
}

extern "C" void send_module_unconfigured_over_uart(esp_zb_ieee_addr_t ieee_addr) {
    const char* unconfigured_str = "cntr/255*255@0,-255!";
    // printf(unconfigured_str);
    send_xbee_data_over_uart(unconfigured_str, strlen(unconfigured_str), ieee_addr, -1);
    start_timer_10s(ieee_addr);
}

extern "C" void send_module_remove_device_on_uart(uint8_t room_id, uint8_t module_id, esp_zb_ieee_addr_t ieee_addr) {
    //addc/2*1@0,[255,255,-255,-255,-255,-255]!
    char del_str[50];
    snprintf(del_str, sizeof(del_str), "delc/%d*%d@0,-255!", room_id, module_id);
    //printf(del_str);
    send_xbee_data_over_uart(del_str, strlen(del_str), ieee_addr, -1);
}


extern "C" void send_module_ctrl_over_uart(node_module_t *node, int start_idx, int idx)
{
    char tx_buf[128];  // enough for your format
    build_ctrl_string(node, start_idx, idx, tx_buf, sizeof(tx_buf));
    // printf("==========>UART TX: %s  len:%d\n", tx_buf, strlen(tx_buf));
    send_xbee_data_over_uart(tx_buf, strlen(tx_buf), node->ieee_addr, -1);
}

void send_xbee_data_over_uart(const char* mdata, uint16_t len, esp_zb_ieee_addr_t ieee_addr, int16_t V_XbeeMsmgNextFrameId)
{
    // mdata should be PLAINTEXT response, e.g., "ctrl/1*3@2,255!"
    // Encrypt the plaintext response
    char* V_SEND_ENCRYPTED_DATA_TO_MODULE_STR = Fn_ENCRYPT_DATA(mdata);
    
    if (V_SEND_ENCRYPTED_DATA_TO_MODULE_STR) {
        printf("\nEncrypted result: %s\n\n", V_SEND_ENCRYPTED_DATA_TO_MODULE_STR);
        
        int encrypted_len = strlen(V_SEND_ENCRYPTED_DATA_TO_MODULE_STR);
        char V_SEND_DATA_TO_MODULE_CHAR[encrypted_len + 1];
        memset(V_SEND_DATA_TO_MODULE_CHAR, 0, sizeof(V_SEND_DATA_TO_MODULE_CHAR));
        strncpy(V_SEND_DATA_TO_MODULE_CHAR, V_SEND_ENCRYPTED_DATA_TO_MODULE_STR, encrypted_len);
        
        // Copy to payload
        uint8_t payload[encrypted_len];
        memset(payload, 0, sizeof(payload));
        for(uint8_t V_IDX_PAYLOAD_ARR = 0; V_IDX_PAYLOAD_ARR < encrypted_len; V_IDX_PAYLOAD_ARR++){
            payload[V_IDX_PAYLOAD_ARR] = V_SEND_DATA_TO_MODULE_CHAR[V_IDX_PAYLOAD_ARR];
        }
        
        // Get XBee address
        convert_zigbee_to_xbee_addr(ieee_addr, &V_ROUTER_ADDR_L_HEX, &V_ROUTER_ADDR_H_HEX);
        printf("ZB_DEVICE_ADDR_H_HEX: %08lx", V_ROUTER_ADDR_H_HEX);
        printf("  ZB_DEVICE_ADDR_L_HEX: %08lx\n", V_ROUTER_ADDR_L_HEX);
        
        if(V_ROUTER_ADDR_H_HEX != 0 && V_ROUTER_ADDR_L_HEX != 0) {
            XBeeAddress64 addr64 = XBeeAddress64(V_ROUTER_ADDR_H_HEX, V_ROUTER_ADDR_L_HEX);
            ZBTxRequest zbTx = ZBTxRequest(addr64, payload, encrypted_len);
            if(V_XbeeMsmgNextFrameId < 0) {
                V_XbeeMsmgNextFrameId = xbee.getNextFrameId();
            }
            zbTx.setFrameId((uint8_t)V_XbeeMsmgNextFrameId);
            xbee.send(zbTx);
            printf("Sending %d bytes to XBee\n", encrypted_len);
        } else {
            printf("Zb device address is 0 or room/module id out of range\n");
        }
        // Free the encrypted string
        free(V_SEND_ENCRYPTED_DATA_TO_MODULE_STR);
    }
}





typedef struct CtrlCommand_s {
  char command[20];      // "ctrl", "addc", "removec"
  uint8_t roomId;      // 1
  uint8_t moduleId;    // 1
  uint8_t deviceId;    // 1 or 0
  int8_t value;        // 0-255 or array of 6 values [-255..255]
  bool isArray;        // false for single, true for array
  int16_t values[6];    // for array mode
}CtrlCommand;

CtrlCommand parseCtrlMsg(const char* msg) {
    CtrlCommand cmd = { {0}, 0, 0, 0, 0, false, {0} };
    
    // Extract command (before first /)
    char temp[128];
    strncpy(temp, msg, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    
    char* token = strtok(temp, "/");
    if (token) {
        strncpy(cmd.command, token, sizeof(cmd.command) - 1);
        cmd.command[sizeof(cmd.command) - 1] = '\0';
    }
    
    // Extract room/module/device (split by *)
    token = strtok(NULL, "*");
    if (token) cmd.roomId = (uint8_t)atoi(token);
    
    token = strtok(NULL, "@");
    if (token) cmd.moduleId = (uint8_t)atoi(token);
    
    token = strtok(NULL, ",[!");
    if (token) cmd.deviceId = (uint8_t)atoi(token);
    
    // Check if array or single value
    char* next = strstr(msg, "[");
    if (next) {
        cmd.isArray = true;
        // Parse array [0,0,0,0,-255,-255]
        char* arrayStart = strchr(msg, '[');
        if (arrayStart) {
            arrayStart++; // skip [
            for (int i = 0; i < 6; i++) {
                // Skip whitespace/numbers
                while (*arrayStart && (*arrayStart < '0' || *arrayStart > '9') && *arrayStart != '-' && *arrayStart != '+') {
                    arrayStart++;
                }
                if (!*arrayStart) break;
                
                cmd.values[i] = (int8_t)atoi(arrayStart);
                arrayStart = strchr(arrayStart, ',');
                if (arrayStart) {
                    arrayStart++;
                } else {
                    break;
                }
            }
        }
    } else {
        cmd.isArray = false;
        // Single value before !
        char* valStart = strstr(msg, ",");
        if (valStart) {
            valStart++; // skip ,
            // Skip to number
            while (*valStart && (*valStart < '0' || *valStart > '9') && *valStart != '-' && *valStart != '+') {
                valStart++;
            }
            cmd.value = (int8_t)atoi(valStart);
        }
    }
    
    return cmd;
}

void process_uart_data(const char* data, uint16_t len, esp_zb_ieee_addr_t ieee_addr, uint8_t* original_frame, uint8_t original_frame_length) 
{

    int didx[6] = {0}; 
    int value[6] = {0};
    int V_XbeeMsmgNextFrameId = data[4];
    CtrlCommand cmd = parseCtrlMsg(data);
    
    // // Debug output
    // printf("Cmd:  %s ", cmd.command);
    // printf(" Room: %d", cmd.roomId);
    // printf(" Mod: %d", cmd.moduleId);
    // printf(" Dev:%d\n", cmd.deviceId);
    //stat/1*1@0,-255!
    // Forward parsed data 

    //edtd/1*3@2,1!
    if(cmd.command[0] == 'e' && cmd.command[1] == 'd' && cmd.command[2] == 't' && cmd.command[3] == 'd'){

        uart_send_bytes_to_gateway(original_frame, original_frame_length);
        for(int i=0; i<devices_count; i++){
            if(device_array[i].room_id == cmd.roomId && device_array[i].module_id == cmd.moduleId){
                //ESP_LOGI(TAG, "Addresses are equal");
                for(int j=0; j<device_array[i].ep_counts; j++){
                    if(device_array[i].endpoint[j].ep == cmd.deviceId){
                        if(cmd.value == 2) device_array[i].endpoint[j].dev_type = 257;
                        else if(cmd.value == 6) device_array[i].endpoint[j].dev_type = 769;
                        else if(cmd.value == 0) device_array[i].endpoint[j].dev_type = 256; 

                        device_array[i].endpoint[j].device_t.level = cmd.value;
                        if(device_array[i].endpoint[j].cluster_counts > 0){
                            for(int cl=0; cl<device_array[i].endpoint[j].cluster_counts; cl++){
                                // if(device_array[i].endpoint[j].cluster_id[cl] != ESP_ZB_ZCL_CLUSTER_ID_ON_OFF){
                                //     if(cmd.value == 6){
                                //         device_array[i].endpoint[j].cluster_id[cl] = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL;
                                //     } else if(cmd.value == 2){
                                //         device_array[i].endpoint[j].cluster_id[cl] = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
                                //     }
                                // }
                                // if(device_array[i].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL){
                                //     device_array[i].endpoint[j].cluster_id[cl] = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL;
                                // }else if(device_array[i].endpoint[j].cluster_id[cl] == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL){
                                //     device_array[i].endpoint[j].cluster_id[cl] = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL;
                                // }
                                save_or_update_light_bulb_device_to_nvs(&device_array[i]);
                                break;
                            }
                        }                  
                        //ESP_LOGI(TAG, "Device found, updating level to %d", cmd.value);
                        break;
                    }
                }
                break;
            }  
        }

    }else if(cmd.command[0] == 'a' && cmd.command[1] == 'd' && cmd.command[2] == 'd' && cmd.command[3] == 'd'){
        printf("Sending response back to gateway : %s\n" , data);
        uart_send_bytes_to_gateway(original_frame, original_frame_length);
        stop_timer_10s();
    } else if(cmd.command[0] == 'a' && cmd.command[1] == 'd' && cmd.command[2] == 'd' && cmd.command[3] == 'c'){
        //printf("Sending response back to gateway : %s\n" , data);
        uint8_t nindex = 255;
        for(int i=0; i<devices_count; i++){
            //ESP_LOGI(TAG, "Addresses are equal");
            if(device_array[i].room_id == cmd.roomId && device_array[i].module_id == cmd.moduleId){
                //ESP_LOGI(TAG, "Device already exists in NVS, updating...");
            } else {
                //ESP_LOGI(TAG, "Device with same address but different room/module ID, updating...");
                if (memcmp(device_array[i].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t)) == 0) {
                    nindex = i;
                    break;
                } else {
                    //ESP_LOGI(TAG, "Addresses are different");
                }
            }  
        }
        printf("\n\n--->Sending bytes to gateway back: \n");
        for(int i = 0; i < original_frame_length; i++) {
            // Process each byte
            printf(" %02X", original_frame[i]);
        }
        printf("\n");
        //send_xbee_data_over_uart(data, strlen(data), ieee_addr, V_XbeeMsmgNextFrameId);
        uart_send_bytes_to_gateway(original_frame, original_frame_length);
        stop_timer_10s();
        if(nindex == 255){
            return;
        }
        //printf("After save to NVS devices_count:%d  nindex:%d\n", devices_count, nindex);

        device_array[nindex].module_id = cmd.moduleId;
        device_array[nindex].room_id = cmd.roomId;

        if(save_or_update_light_bulb_device_to_nvs(&device_array[nindex]) == ESP_OK){
    		memset(device_array, 0, sizeof(device_array));
    		devices_count = 0;
    		read_all_devices_from_nvs(NULL);
    	}
    // delc/1*1@0,-255!
    } else if(cmd.command[0] == 'd' && cmd.command[1] == 'e' && cmd.command[2] == 'l' && cmd.command[3] == 'c'){
        
        node_module_t * module_remove = find_device_by_room_module_id(cmd.roomId, cmd.moduleId);
        if(module_remove != NULL){
            printf("nuos_zb_leave_device of module_id:%d did:%d\n", module_remove->room_id, module_remove->module_id);
            nuos_zb_leave_device(module_remove);
        }else{
            printf("Module to remove not found!!\n");
        }
        send_xbee_data_over_uart(data, strlen(data), ieee_addr, V_XbeeMsmgNextFrameId);
    } else if(cmd.command[0] == 'c' && cmd.command[1] == 't' && cmd.command[2] == 'r' && cmd.command[3] == 'l'){
        send_xbee_data_over_uart(data, strlen(data), ieee_addr, V_XbeeMsmgNextFrameId);
        //printf("Found ctrl Command!!\n");
        if(cmd.deviceId == 0){
            // Array mode
            didx[0] = 1;
            value[0] = cmd.values[0];
            value[1] = cmd.values[1];
            value[2] = cmd.values[2];
            value[3] = cmd.values[3];
            value[4] = cmd.values[4];
            value[5] = cmd.values[5];
        } else {
            // Single value mode
            didx[0] = cmd.deviceId;
            value[0] = cmd.value;
        }
        //printf("roomId:%d   moduleId:%d   deviceId:%d\n", cmd.roomId, cmd.moduleId, didx[0]);
        find_and_execute_ctrl_command(cmd.roomId, cmd.moduleId, didx, value, cmd.isArray);

    }else if(cmd.command[0] == 'c' && cmd.command[1] == 'n' && cmd.command[2] == 't' && cmd.command[3] == 'r'){
        // unconfigured command
        //send_xbee_data_over_uart(data, strlen(data), ieee_addr, V_XbeeMsmgNextFrameId);
        //uart_send_bytes_to_gateway( original_frame, original_frame_length);
    }
}

void find_and_execute_ctrl_command(uint8_t room_id, uint8_t module_id, int* didx, int *val, bool isArray) {
    uint8_t total_counts = 0;
    uint8_t level = 0;
    uint8_t nodeindex=0, epindex=0;
    uint16_t node_type = 0;
    uint16_t color = 0;
    uint8_t fan_speed = 0;
    bool is_fan = false;
    bool is_thermostat = false;

    node_module_t * module = find_device_by_room_module_id(room_id, module_id);

    if(module != NULL){

        if (didx[0] == 0) {
            // Array mode - process cmd.values[]
            // For simplicity, just use the first value for now
            for(int j=0; j<module->ep_counts;j++){
                didx[j] = module->endpoint[j].ep;
            }
            total_counts = module->ep_counts;
        } else {
            // Single value mode
            level = val[0];
            total_counts = 1;
        }

        for(int i = 0; i < total_counts; i++){
            uint8_t did = didx[i];
            if (didx[i] > 0 && didx[i] != -255) {
                if (find_device_by_dev_id(module_id, did, &nodeindex, &epindex)) {
                    //printf("find_device_by_dev_id success!!\n");

                    if (node_type == 0) {
                        node_type = find_device_type(nodeindex, epindex);
                    }
                    if (didx[i] == 0) {
                        level = val[i];
                    }else{
                        level = val[0];
                    }
                    // printf("nodeindex:%d, level:%d epindex:%d \n", nodeindex, level, epindex);
                    switch (node_type) {
                        case 4:
                        case 0:
                            if (get_total_endpoints(nodeindex) <= 2) {
                                nuos_update_curtain_cluster(nodeindex, epindex, level);
                            } else {
                                printf("Scene pressed!!\n");
                            }
                            break;
                        case 256:
                        case 257:
                            if (level == 0 || level == 255) {
                                nuos_update_onoff_cluster(nodeindex, epindex, level ? true : false);
                            } else {                                
                                nuos_update_level_cluster(nodeindex, epindex, fan_speed);
                            }
                            break;
                        case 269:
                            if (color > 0) {
                                nuos_update_color_hue(nodeindex, epindex, color);
                            } else {
                                if (level == 0 || level == 255) {
                                    nuos_update_onoff_cluster(nodeindex, epindex, level ? true : false);
                                } else {
                                    nuos_update_level_cluster(nodeindex, epindex, level);
                                }
                            }
                            break;
                        case 268:  // color temperature
                            if (color > 0) {
                                nuos_update_color_temperature(nodeindex, epindex, color);
                            } else {
                                if (level == 0 || level == 255) {
                                    nuos_update_onoff_cluster(nodeindex, epindex, level ? true : false);
                                } else {
                                    nuos_update_level_cluster(nodeindex, epindex, level);
                                }
                            }
                            break;
                        case 769:
                            is_fan = is_fan_control(nodeindex, epindex);
                            if (is_fan) {
                                
                                if (level == 0) fan_speed = 0;
                                if(level == 142) fan_speed = 191; 
                                else if(level == 102) fan_speed = 128; 
                                else if(level == 80) fan_speed = 64;
                                else if(level == 255) fan_speed = 254;                                    
                                zigbee_cluster_fan_control_attr_update(nodeindex, epindex, fan_speed);
                            } else {
                                is_thermostat = is_thermostat_control(nodeindex, epindex);
                                if(is_thermostat) {
                                    // For thermostat, level represents temperature setpoint
                                     zigbee_cluster_thermostat_attr_update(nodeindex, epindex, level);
                                }else{
                                    
                                    if (level == 0) fan_speed = 0;
                                    if(level == 142) fan_speed = 191; 
                                    else if(level == 102) fan_speed = 128; 
                                    else if(level == 80) fan_speed = 64;
                                    else if(level == 255) fan_speed = 254;    
                                    nuos_update_level_cluster(nodeindex, epindex, fan_speed);                                                    
                                }
                               
                            }
                            
                            break;
                        default:
                            break;
                    }
                } else {
                    printf("did not found!!\n");
                }
            }
        }
    } else {
        printf("Device not found for ctrl command!!\n");
    }
}


void process_decrypt_uart_receiver_data(uint8_t* data, uint16_t len) {
// Store byte in XBee's buffer
    memcpy(xbee.serial_read_data, (const uint8_t*)data, len);
    xbee.serial_read_data[len] = '\0'; // Null terminate for single byte
    xbee.serial_read_data_counter = 0;
    xbee.serial_data_available = true;

    // Process the byte through XBee parser
    xbee.readPacket();
    // Check if we have a complete response
    if(xbee.getResponse().isAvailable()) {
        printf("XBee Response Available. API ID: 0x%02X\n", xbee.getResponse().getApiId());
        if(xbee.getResponse().getApiId() == ZB_TX_REQUEST) {
            ZBRxResponse rx;
            xbee.getResponse().getZBRxResponse(rx);

            //printf("Rx_len: %d\n", rx.getDataLength()-1);
            // Process received data
            char received_data[rx.getDataLength()-1 + 1];
            for(int i = 0; i < rx.getDataLength()-1; i++) {
                received_data[i] = rx.getData(1+i);
                printf(" %02x", received_data[i]);
            }
            printf("\n");
            received_data[rx.getDataLength()-1] = '\0';

            //char encrypted_buffer[256];
            char* decrypted = Fn_DECRYPT_DATA(received_data);
            if (decrypted) {
                printf( "decrypted:%s\n", decrypted);
                esp_zb_ieee_addr_t zigbee_addr;
                convert_xbee_to_zigbee_addr(rx.getRemoteAddress64(), zigbee_addr);
                process_uart_data((const char*)decrypted, strlen(decrypted), zigbee_addr, data, len);
                free(decrypted);
            }
        }
    } else {
        printf("No complete XBee response available yet.\n");
    }
}