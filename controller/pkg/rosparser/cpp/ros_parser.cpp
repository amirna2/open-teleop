/**
 * ros_parser.cpp
 * Implementation of the ROS2 message parser C interface.
 */

#include "ros_parser.h"
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include "json.hpp"  // Use our local copy
#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <cstring>

// Use JSON for modern C++
using json = nlohmann::json;

// Global ROS2 context
static bool g_is_initialized = false;
static std::mutex g_mutex;

// Map to store error messages
static std::unordered_map<int, std::string> g_error_strings = {
    {ROS_PARSER_SUCCESS, "Success"},
    {ROS_PARSER_ERROR_INIT_FAILED, "ROS2 initialization failed"},
    {ROS_PARSER_ERROR_INVALID_MESSAGE, "Invalid message data or format"},
    {ROS_PARSER_ERROR_UNSUPPORTED_TYPE, "Unsupported message type"},
    {ROS_PARSER_ERROR_SERIALIZATION, "Message serialization error"},
    {ROS_PARSER_ERROR_MEMORY, "Memory allocation error"},
    {ROS_PARSER_ERROR_UNKNOWN, "Unknown error"}
};

// Helper struct to manage message type support
struct MessageTypeSupport {
    std::string message_type;
    std::shared_ptr<rcpputils::SharedLibrary> library;
    const rosidl_message_type_support_t* type_support;
    
    MessageTypeSupport() : type_support(nullptr) {}
};

// Cache for message type supports
static std::unordered_map<std::string, MessageTypeSupport> g_type_support_cache;

// Helper function to allocate a string to be returned to Go
char* allocate_string(const std::string& str) {
    char* result = (char*)malloc(str.length() + 1);
    if (result) {
        strcpy(result, str.c_str());
    }
    return result;
}

// Helper function to get type support for a ROS2 message type
bool get_message_type_support(const std::string& message_type, MessageTypeSupport& type_support) {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    // Check if we already have this type in cache
    auto it = g_type_support_cache.find(message_type);
    if (it != g_type_support_cache.end()) {
        type_support = it->second;
        return type_support.type_support != nullptr;
    }
    
    // Parse the message type string (e.g., "sensor_msgs/msg/Image")
    auto last_slash = message_type.find_last_of('/');
    if (last_slash == std::string::npos || last_slash == 0) {
        return false;
    }
    
    auto second_last_slash = message_type.find_last_of('/', last_slash - 1);
    if (second_last_slash == std::string::npos) {
        return false;
    }
    
    std::string package_name = message_type.substr(0, second_last_slash);
    std::string msg_dir = message_type.substr(second_last_slash + 1, last_slash - second_last_slash - 1);
    std::string msg_name = message_type.substr(last_slash + 1);
    
    // Construct the library names
    std::string lib_name = package_name + "__" + msg_dir + "__" + msg_name;
    std::string lib_path = "lib" + lib_name + "__rosidl_typesupport_introspection_cpp.so";
    
    try {
        // Load the library
        type_support.library = std::make_shared<rcpputils::SharedLibrary>(lib_path);
        
        // Get the type support symbol
        std::string symbol_name = 
            "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" +
            package_name + "__" + msg_dir + "__" + msg_name;
        
        using GetMsgTypeSupport = const rosidl_message_type_support_t* (*)();
        GetMsgTypeSupport get_ts = 
            reinterpret_cast<GetMsgTypeSupport>(
                type_support.library->get_symbol(symbol_name.c_str()));
        
        if (!get_ts) {
            return false;
        }
        
        type_support.type_support = get_ts();
        type_support.message_type = message_type;
        
        // Add to cache
        g_type_support_cache[message_type] = type_support;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error loading type support for " << message_type << ": " << e.what() << std::endl;
        return false;
    }
}

// Helper function to convert a ROS message field to JSON
void field_to_json(
    const rosidl_typesupport_introspection_cpp::MessageMember& member,
    const uint8_t* data,
    json& j_field) 
{
    using namespace rosidl_typesupport_introspection_cpp;
    
    // Handle array types
    if (member.is_array_) {
        j_field = json::array();
        
        // Calculate array size
        size_t array_size = member.array_size_;
        if (member.is_upper_bound_ && member.array_size_ == 0) {
            // Dynamic array: first element is array size
            array_size = *reinterpret_cast<const size_t*>(data);
            data += sizeof(size_t);
        }
        
        // Process each array element
        for (size_t i = 0; i < array_size; i++) {
            const uint8_t* element_data = data;
            if (member.array_size_ == 0 || member.is_upper_bound_) {
                // For dynamic arrays, data points to the elements
                element_data = *reinterpret_cast<const uint8_t* const*>(data);
                element_data += i * member.size_function(nullptr);
            } else {
                // For fixed arrays, data points directly to an array
                element_data += i * member.size_function(nullptr);
            }
            
            // Recursively process the element
            json element;
            // TODO: Process based on type
            // For now, just add a placeholder
            element = "array_element_placeholder";
            j_field.push_back(element);
        }
    } else {
        // Handle scalar types
        switch (member.type_id_) {
            case ROS_TYPE_BOOL:
                j_field = *reinterpret_cast<const bool*>(data);
                break;
            case ROS_TYPE_BYTE:
            case ROS_TYPE_UINT8:
                j_field = *reinterpret_cast<const uint8_t*>(data);
                break;
            case ROS_TYPE_CHAR:
            case ROS_TYPE_INT8:
                j_field = *reinterpret_cast<const int8_t*>(data);
                break;
            case ROS_TYPE_FLOAT32:
                j_field = *reinterpret_cast<const float*>(data);
                break;
            case ROS_TYPE_FLOAT64:
                j_field = *reinterpret_cast<const double*>(data);
                break;
            case ROS_TYPE_INT16:
                j_field = *reinterpret_cast<const int16_t*>(data);
                break;
            case ROS_TYPE_UINT16:
                j_field = *reinterpret_cast<const uint16_t*>(data);
                break;
            case ROS_TYPE_INT32:
                j_field = *reinterpret_cast<const int32_t*>(data);
                break;
            case ROS_TYPE_UINT32:
                j_field = *reinterpret_cast<const uint32_t*>(data);
                break;
            case ROS_TYPE_INT64:
                j_field = *reinterpret_cast<const int64_t*>(data);
                break;
            case ROS_TYPE_UINT64:
                j_field = *reinterpret_cast<const uint64_t*>(data);
                break;
            case ROS_TYPE_STRING:
                {
                    const rosidl_runtime_c__String* str = 
                        reinterpret_cast<const rosidl_runtime_c__String*>(data);
                    j_field = std::string(str->data, str->size);
                }
                break;
            case ROS_TYPE_MESSAGE:
                {
                    // Nested message - recursively process
                    j_field = json::object();
                    // TODO: Implement nested message processing
                }
                break;
            default:
                j_field = nullptr;
                break;
        }
    }
}

// Helper function to convert a ROS message to JSON
bool message_to_json(
    const rosidl_message_type_support_t* type_support,
    const void* message_data, 
    json& j_msg) 
{
    // Get message members from type support
    const rosidl_typesupport_introspection_cpp::MessageMembers* members = 
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
            type_support->data);
    
    // Create a JSON object for the message
    j_msg = json::object();
    
    // Process each field in the message
    for (size_t i = 0; i < members->member_count_; i++) {
        auto& member = members->members_[i];
        const uint8_t* field_data = 
            static_cast<const uint8_t*>(message_data) + member.offset_;
        
        // Convert field to JSON
        json field_value;
        field_to_json(member, field_data, field_value);
        
        // Add to message JSON
        j_msg[member.name_] = field_value;
    }
    
    return true;
}

// C interface implementation

extern "C" {

int RosParser_Initialize() {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    if (g_is_initialized) {
        return ROS_PARSER_SUCCESS;
    }
    
    try {
        // Initialize ROS2 context with a minimal executor
        rclcpp::init(0, nullptr);
        g_is_initialized = true;
        return ROS_PARSER_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize ROS2: " << e.what() << std::endl;
        return ROS_PARSER_ERROR_INIT_FAILED;
    }
}

void RosParser_Shutdown() {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    if (g_is_initialized) {
        // Clean up type support cache
        g_type_support_cache.clear();
        
        // Shutdown ROS2
        rclcpp::shutdown();
        g_is_initialized = false;
    }
}

int RosParser_ParseToJson(
    const char* message_type,
    const unsigned char* message_data,
    int message_size,
    char** json_output,
    char** error_msg) 
{
    // Check initialization
    if (!g_is_initialized) {
        if (error_msg) {
            *error_msg = allocate_string("ROS Parser not initialized");
        }
        return ROS_PARSER_ERROR_INIT_FAILED;
    }
    
    // Validate inputs
    if (!message_type || !message_data || message_size <= 0) {
        if (error_msg) {
            *error_msg = allocate_string("Invalid input parameters");
        }
        return ROS_PARSER_ERROR_INVALID_MESSAGE;
    }
    
    try {
        // Get type support
        MessageTypeSupport type_support;
        if (!get_message_type_support(message_type, type_support)) {
            std::string err = "Unsupported message type: " + std::string(message_type);
            if (error_msg) {
                *error_msg = allocate_string(err);
            }
            return ROS_PARSER_ERROR_UNSUPPORTED_TYPE;
        }
        
        // TODO: Deserialize the message data
        // For the initial implementation, we'll just return a placeholder
        json result = {
            {"message_type", message_type},
            {"data_size", message_size},
            {"note", "Actual deserialization not implemented yet - this is a placeholder"}
        };
        
        // Output the JSON as a string
        std::string json_str = result.dump();
        *json_output = allocate_string(json_str);
        
        if (!*json_output) {
            if (error_msg) {
                *error_msg = allocate_string("Memory allocation failed");
            }
            return ROS_PARSER_ERROR_MEMORY;
        }
        
        return ROS_PARSER_SUCCESS;
    } catch (const std::exception& e) {
        if (error_msg) {
            *error_msg = allocate_string(e.what());
        }
        return ROS_PARSER_ERROR_UNKNOWN;
    }
}

void RosParser_FreeString(char* str) {
    if (str) {
        free(str);
    }
}

const char* RosParser_GetErrorString(int error_code) {
    static std::string unknown_error = "Unknown error code";
    
    auto it = g_error_strings.find(error_code);
    if (it != g_error_strings.end()) {
        return it->second.c_str();
    }
    
    return unknown_error.c_str();
}

} // extern "C" 