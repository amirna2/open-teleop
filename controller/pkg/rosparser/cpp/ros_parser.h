/**
 * ros_parser.h
 * C interface for ROS2 message parsing, designed to be called from Go via CGO.
 */

#ifndef ROS_PARSER_H
#define ROS_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Error codes returned by the parser
 */
typedef enum {
    ROS_PARSER_SUCCESS = 0,
    ROS_PARSER_ERROR_INIT_FAILED = 1,
    ROS_PARSER_ERROR_INVALID_MESSAGE = 2,
    ROS_PARSER_ERROR_UNSUPPORTED_TYPE = 3,
    ROS_PARSER_ERROR_SERIALIZATION = 4,
    ROS_PARSER_ERROR_MEMORY = 5,
    ROS_PARSER_ERROR_UNKNOWN = 99
} RosParserError;

/**
 * Initialize the ROS2 parser.
 * Must be called before any other functions.
 * 
 * @return ROS_PARSER_SUCCESS on success, error code otherwise.
 */
int RosParser_Initialize();

/**
 * Shutdown the ROS2 parser.
 * Should be called when the application exits.
 */
void RosParser_Shutdown();

/**
 * Parse a ROS2 message and convert it to a JSON representation.
 * 
 * @param message_type The ROS2 message type (e.g., "sensor_msgs/msg/Image")
 * @param message_data The serialized ROS2 message data
 * @param message_size Size of the message data in bytes
 * @param json_output Pointer to store the JSON output (caller must free with RosParser_FreeString)
 * @param error_msg Pointer to store error message (caller must free with RosParser_FreeString)
 * @return ROS_PARSER_SUCCESS on success, error code otherwise
 */
int RosParser_ParseToJson(
    const char* message_type,
    const unsigned char* message_data,
    int message_size,
    char** json_output,
    char** error_msg
);

/**
 * Process a ROS2 image message and extract metadata and raw data.
 * Specifically designed for sensor_msgs/msg/Image and sensor_msgs/msg/CompressedImage.
 * 
 * @param message_type The ROS2 message type (e.g., "sensor_msgs/msg/Image")
 * @param message_data The serialized ROS2 message data
 * @param message_size Size of the message data in bytes
 * @param metadata_json Pointer to store JSON metadata (caller must free with RosParser_FreeString)
 * @param raw_data Pointer to store raw image data (caller must copy, will be freed internally)
 * @param raw_data_size Pointer to store size of raw data in bytes
 * @param error_msg Pointer to store error message (caller must free with RosParser_FreeString)
 * @return ROS_PARSER_SUCCESS on success, error code otherwise
 */
int RosParser_ExtractImageData(
    const char* message_type,
    const unsigned char* message_data,
    int message_size,
    char** metadata_json,
    unsigned char** raw_data,
    int* raw_data_size,
    char** error_msg
);

/**
 * Free a string allocated by the parser.
 * 
 * @param str The string to free
 */
void RosParser_FreeString(char* str);

/**
 * Get error message for the given error code.
 * 
 * @param error_code The error code
 * @return String representation of the error (static string, do not free)
 */
const char* RosParser_GetErrorString(int error_code);

#ifdef __cplusplus
}
#endif

#endif /* ROS_PARSER_H */ 