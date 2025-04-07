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
#include <dlfcn.h> // Include for dlerror
#include <cstdlib> // Include for getenv
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rmw/rmw.h> // For rmw_deserialize
#include <rmw/serialized_message.h>
#include <rcutils/shared_library.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rcutils/error_handling.h>
#include <rosbag2_cpp/typesupport_helpers.hpp> // USE THIS

// Use JSON for modern C++
using json = nlohmann::json;

// Global ROS2 context
static bool g_is_initialized = false;
static std::mutex g_mutex;

// Global flag for debug logging
static bool g_debug_logging_enabled = false;

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

// Helper function to check environment variable
bool check_debug_env()
{
    const char* env_var = std::getenv("ROS_PARSER_DEBUG");
    if (env_var == nullptr) {
        return false;
    }
    std::string env_val = env_var;
    std::transform(env_val.begin(), env_val.end(), env_val.begin(), ::tolower);
    return (env_val == "1" || env_val == "true");
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
        std::cerr << "Invalid message type format (no slashes): " << message_type << std::endl;
        return false;
    }
    
    auto second_last_slash = message_type.find_last_of('/', last_slash - 1);
    if (second_last_slash == std::string::npos) {
         std::cerr << "Invalid message type format (only one slash): " << message_type << std::endl;
        return false;
    }
    
    std::string package_name = message_type.substr(0, second_last_slash);
    std::string msg_dir = message_type.substr(second_last_slash + 1, last_slash - second_last_slash - 1);
    std::string msg_name = message_type.substr(last_slash + 1);
    
    // ROS 2 uses package-level libraries
    std::string lib_path = "lib" + package_name + "__rosidl_typesupport_introspection_cpp.so";
    
    try {
        // Log environment variable
        const char* ld_path = std::getenv("LD_LIBRARY_PATH");
        std::cerr << "LD_LIBRARY_PATH: " << (ld_path ? ld_path : "Not Set") << std::endl;

        // Load the library
        std::cerr << "Attempting to load library: " << lib_path << std::endl;
        // Use a raw dlopen for more direct error checking first
        void* lib_handle = dlopen(lib_path.c_str(), RTLD_LAZY);
        if (!lib_handle) {
            std::cerr << "dlopen failed for " << lib_path << ": " << dlerror() << std::endl;
            return false;
        }
        std::cerr << "dlopen succeeded for " << lib_path << std::endl;
        // Now create the shared library object (assuming it takes ownership or we manage handle elsewhere)
        // For now, we proceed knowing dlopen worked, but SharedLibrary might still fail
        // We'll keep the SharedLibrary logic for now, but dlopen check gives us earlier info
        type_support.library = std::make_shared<rcpputils::SharedLibrary>(lib_path);
        dlerror(); // Clear previous dlerror after SharedLibrary creation
        std::cerr << "SharedLibrary object created for " << lib_path << std::endl;

        // Get the type support symbol
        std::string symbol_name =
            "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" +
            package_name + "__" + msg_dir + "__" + msg_name;

        std::cerr << "Looking for symbol: " << symbol_name << std::endl;

        using GetMsgTypeSupport = const rosidl_message_type_support_t* (*)();
        dlerror(); // Clear dlerror before attempting to get the symbol
        void* raw_symbol = dlsym(lib_handle, symbol_name.c_str()); // Use dlsym directly with handle
        const char* dlsym_error = dlerror(); // Check for errors immediately after

        GetMsgTypeSupport get_ts = reinterpret_cast<GetMsgTypeSupport>(raw_symbol);

        if (!get_ts) {
            std::cerr << "Symbol not found: " << symbol_name << std::endl;
            if (dlsym_error) {
                 std::cerr << "dlsym error: " << dlsym_error << std::endl;
            } else {
                 std::cerr << "dlsym returned null, but dlerror is null." << std::endl;
            }
            dlclose(lib_handle); // Close the handle if symbol lookup failed
            return false;
        } else {
             std::cerr << "Symbol found successfully: " << symbol_name << std::endl;
        }

        type_support.type_support = get_ts();
        type_support.message_type = message_type;

        // Add to cache - Note: library handle needs proper management if we don't rely on SharedLibrary
        g_type_support_cache[message_type] = type_support;
        // dlclose(lib_handle); // Don't close yet if we cache the type_support

        return true;
    // Simplified catch block
    } catch (const std::exception& e) {
        std::cerr << "Exception loading type support for " << message_type << ": " << e.what() << std::endl;
        // Also print dlerror if available, might catch errors from SharedLibrary constructor
        const char* dlopen_error = dlerror();
        if (dlopen_error) {
            std::cerr << "dlerror() after exception: " << dlopen_error << std::endl;
        }
        return false;
    }
}

// Forward declarations for recursive calls
static bool message_to_json(
    const rosidl_message_type_support_t* type_support,
    const void* message_data, 
    json& j_msg); 

static bool field_to_json(
    const rosidl_typesupport_introspection_cpp::MessageMember& member,
    const uint8_t* field_data_ptr,
    json& j_field);

// Forward declarations
static bool message_to_json_internal(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members,
    const void* message_data,
    json& j_msg);

// Helper function to convert a ROS message field to JSON
bool field_to_json(
    const rosidl_typesupport_introspection_cpp::MessageMember& member,
    const uint8_t* field_data_ptr,
    json& j_field)
{
    using namespace rosidl_typesupport_introspection_cpp;
    if (g_debug_logging_enabled) {
        std::cerr << "  [field_to_json] Processing field. TypeID: " << static_cast<int>(member.type_id_)
                  << ", IsArray: " << member.is_array_ << std::endl;
    }

    if (member.is_array_) {
        if (g_debug_logging_enabled) std::cerr << "    [field_to_json] Handling array..." << std::endl;
        j_field = json::array();
        size_t array_size = member.array_size_;
        const uint8_t* array_data_ptr = field_data_ptr;

        // Define a generic sequence structure based on common rosidl_runtime_c layout
        // We'll cast the field_data_ptr to this to access size and data generically
        typedef struct GenericSequence {
            void * data; // Points to array of actual type T
            size_t size;
            size_t capacity;
        } GenericSequence;

        // Handle dynamic arrays (std::vector or similar)
        if (member.is_upper_bound_ || member.array_size_ == 0) {
            if (g_debug_logging_enabled) std::cerr << "      [field_to_json] Dynamic array detected." << std::endl;
            const GenericSequence* sequence = reinterpret_cast<const GenericSequence*>(field_data_ptr);
            array_size = sequence->size;
            array_data_ptr = static_cast<const uint8_t*>(sequence->data); // Point to the actual data buffer
            if (g_debug_logging_enabled) std::cerr << "          [field_to_json] Dynamic Size: " << array_size << std::endl;

            // Now, handle based on the specific element type ID
            switch (member.type_id_) {
                case ROS_TYPE_OCTET:
                case ROS_TYPE_UINT8: {
                    if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic uint8 array." << std::endl;
                    const uint8_t* data_typed = reinterpret_cast<const uint8_t*>(array_data_ptr);
                    for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                    return true;
                }
                case ROS_TYPE_FLOAT32: {
                    if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic float32 array." << std::endl;
                    const float* data_typed = reinterpret_cast<const float*>(array_data_ptr);
                    for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                    return true;
                }
                case ROS_TYPE_FLOAT64: {
                    if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic float64 array." << std::endl;
                    const double* data_typed = reinterpret_cast<const double*>(array_data_ptr);
                    for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                    return true;
                }
                 case ROS_TYPE_CHAR:
                 case ROS_TYPE_INT8: {
                     if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic int8 array." << std::endl;
                     const int8_t* data_typed = reinterpret_cast<const int8_t*>(array_data_ptr);
                     for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                     return true;
                 }
                 case ROS_TYPE_INT16: {
                     if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic int16 array." << std::endl;
                     const int16_t* data_typed = reinterpret_cast<const int16_t*>(array_data_ptr);
                     for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                     return true;
                 }
                 case ROS_TYPE_UINT16: {
                     if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic uint16 array." << std::endl;
                     const uint16_t* data_typed = reinterpret_cast<const uint16_t*>(array_data_ptr);
                     for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                     return true;
                 }
                 case ROS_TYPE_INT32: {
                     if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic int32 array." << std::endl;
                     const int32_t* data_typed = reinterpret_cast<const int32_t*>(array_data_ptr);
                     for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                     return true;
                 }
                 case ROS_TYPE_UINT32: {
                     if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic uint32 array." << std::endl;
                     const uint32_t* data_typed = reinterpret_cast<const uint32_t*>(array_data_ptr);
                     for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                     return true;
                 }
                 case ROS_TYPE_INT64: {
                     if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic int64 array." << std::endl;
                     const int64_t* data_typed = reinterpret_cast<const int64_t*>(array_data_ptr);
                     for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                     return true;
                 }
                 case ROS_TYPE_UINT64: {
                     if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic uint64 array." << std::endl;
                     const uint64_t* data_typed = reinterpret_cast<const uint64_t*>(array_data_ptr);
                     for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                     return true;
                 }
                 case ROS_TYPE_BOOL: {
                     if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic bool array." << std::endl;
                     // Assuming bool sequence also follows the generic layout
                     const bool* data_typed = reinterpret_cast<const bool*>(array_data_ptr);
                     for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                     return true;
                 }
                 // Add cases for STRING, WSTRING, MESSAGE if needed later
                 default: {
                     if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Dynamic array handling not implemented for primitive type_id: " << (int)member.type_id_ << std::endl;
                     // Return empty array, but indicate success to avoid blocking rest of message
                     return true; 
                 }
            }
        } else { // Handle fixed-size arrays or bounded arrays (where data is contiguous)
            if (g_debug_logging_enabled) std::cerr << "      [field_to_json] Fixed/Bounded array detected. Declared size: " << array_size << std::endl;
            size_t element_size = 0;
            if (member.type_id_ == ROS_TYPE_MESSAGE) {
                 if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Nested message array." << std::endl;
                 const rosidl_message_type_support_t* nested_ts = 
                     static_cast<const rosidl_message_type_support_t*>(member.members_->data);
                 const MessageMembers* nested_members = 
                     static_cast<const MessageMembers*>(nested_ts->data);
                  element_size = nested_members->size_of_; 
                 if (g_debug_logging_enabled) std::cerr << "          [field_to_json] Nested element size: " << element_size << std::endl;
            } else {
                if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Primitive array." << std::endl;
                switch (member.type_id_) {
                    case ROS_TYPE_BOOL: element_size = sizeof(bool); break;
                    case ROS_TYPE_OCTET: element_size = sizeof(uint8_t); break;
                    case ROS_TYPE_UINT8: element_size = sizeof(uint8_t); break;
                    case ROS_TYPE_CHAR: element_size = sizeof(char); break; // Assuming char = int8
                    case ROS_TYPE_INT8: element_size = sizeof(int8_t); break;
                    case ROS_TYPE_FLOAT32: element_size = sizeof(float); break;
                    case ROS_TYPE_FLOAT64: element_size = sizeof(double); break;
                    case ROS_TYPE_INT16: element_size = sizeof(int16_t); break;
                    case ROS_TYPE_UINT16: element_size = sizeof(uint16_t); break;
                    case ROS_TYPE_INT32: element_size = sizeof(int32_t); break;
                    case ROS_TYPE_UINT32: element_size = sizeof(uint32_t); break;
                    case ROS_TYPE_INT64: element_size = sizeof(int64_t); break;
                    case ROS_TYPE_UINT64: element_size = sizeof(uint64_t); break;
                    case ROS_TYPE_STRING: element_size = sizeof(std::string); break; 
                    default: 
                       if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Cannot determine element size for fixed array type_id: " << (int)member.type_id_ << std::endl;
                       return false;
                }
                 if (g_debug_logging_enabled) std::cerr << "          [field_to_json] Primitive element size: " << element_size << std::endl;
            }

            if (element_size == 0) {
                if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Calculated element size is 0 for fixed array type_id: " << (int)member.type_id_ << std::endl;
                return false;
            }

            for (size_t i = 0; i < array_size; ++i) {
                if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Processing array element " << i << "..." << std::endl;
                json element_json;
                const uint8_t* current_element_ptr = array_data_ptr + (i * element_size);
                MessageMember element_member = member; 
                element_member.is_array_ = false;      
                element_member.array_size_ = 0;
                element_member.is_upper_bound_ = false;
                
                if (!field_to_json(element_member, current_element_ptr, element_json)) {
                    if (g_debug_logging_enabled) std::cerr << "        [field_to_json] WARNING: Failed to convert array element " << i << std::endl;
                    j_field.push_back(nullptr);
                } else {
                    j_field.push_back(element_json);
                }
            }
             if (g_debug_logging_enabled) std::cerr << "    [field_to_json] Fixed/Bounded Array handled." << std::endl;
            return true;
        }
    } else { // Handle scalar types
        if (g_debug_logging_enabled) std::cerr << "    [field_to_json] Handling scalar type_id: " << static_cast<int>(member.type_id_) << std::endl;
        switch (member.type_id_) {
            case ROS_TYPE_BOOL:
                j_field = *reinterpret_cast<const bool*>(field_data_ptr);
                break;
            case ROS_TYPE_OCTET: // Often used for bytes
            case ROS_TYPE_UINT8:
                j_field = *reinterpret_cast<const uint8_t*>(field_data_ptr);
                break;
            case ROS_TYPE_CHAR: // Treat as number
            case ROS_TYPE_INT8:
                j_field = *reinterpret_cast<const int8_t*>(field_data_ptr);
                break;
            case ROS_TYPE_FLOAT32:
                j_field = *reinterpret_cast<const float*>(field_data_ptr);
                break;
            case ROS_TYPE_FLOAT64:
                j_field = *reinterpret_cast<const double*>(field_data_ptr);
                break;
            case ROS_TYPE_INT16:
                j_field = *reinterpret_cast<const int16_t*>(field_data_ptr);
                break;
            case ROS_TYPE_UINT16:
                j_field = *reinterpret_cast<const uint16_t*>(field_data_ptr);
                break;
            case ROS_TYPE_INT32:
                j_field = *reinterpret_cast<const int32_t*>(field_data_ptr);
                break;
            case ROS_TYPE_UINT32:
                j_field = *reinterpret_cast<const uint32_t*>(field_data_ptr);
                break;
            case ROS_TYPE_INT64:
                j_field = *reinterpret_cast<const int64_t*>(field_data_ptr);
                break;
            case ROS_TYPE_UINT64:
                j_field = *reinterpret_cast<const uint64_t*>(field_data_ptr);
                break;
            case ROS_TYPE_STRING:
                {
                    if (g_debug_logging_enabled) std::cerr << "      [field_to_json] Processing std::string..." << std::endl;
                    const auto* cpp_str_ptr = reinterpret_cast<const std::string*>(field_data_ptr);
                    // Add check for null pointer just in case, although unlikely for struct members
                    if (cpp_str_ptr) { 
                        j_field = *cpp_str_ptr;
                        if (g_debug_logging_enabled && cpp_str_ptr) std::cerr << "        [field_to_json] String value: \"" << *cpp_str_ptr << "\"" << std::endl;
                    } else {
                        if (g_debug_logging_enabled) std::cerr << "        [field_to_json] WARNING: String pointer was null." << std::endl;
                        j_field = nullptr;
                    }
                }
                break;
            case ROS_TYPE_MESSAGE:
                {
                    if (g_debug_logging_enabled) std::cerr << "      [field_to_json] Processing nested message..." << std::endl;
                    // Get the MessageMembers struct for the nested type
                    // First, check if the nested type support handle itself is valid
                    const rosidl_message_type_support_t* nested_type_support_handle = member.members_;
                    if (!nested_type_support_handle) {
                         if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Nested message type support handle (member.members_) is null." << std::endl;
                         return false;
                    }
                    // Now, safely get the members data pointer
                    const MessageMembers* nested_members = 
                        static_cast<const MessageMembers*>(nested_type_support_handle->data);
                    if (!nested_members) {
                         if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Failed to get MessageMembers from nested type support handle data." << std::endl;
                         return false;
                    }
                    // Recursively call the internal helper with nested members and field data pointer
                    if (!message_to_json_internal(nested_members, field_data_ptr, j_field)) {
                         if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Failed to process nested message field via internal helper." << std::endl;
                         return false;
                    }
                    if (g_debug_logging_enabled) std::cerr << "      [field_to_json] Nested message processed." << std::endl;
                }
                break;
            default:
                 if (g_debug_logging_enabled) std::cerr << "    [field_to_json] ERROR: Unsupported scalar type_id: " << static_cast<int>(member.type_id_) << std::endl;
                j_field = nullptr; 
                return false; 
        }
         if (g_debug_logging_enabled) std::cerr << "    [field_to_json] Scalar processed." << std::endl;
        return true;
    }
    // Should not be reached if all paths return
    return false; 
}

// NEW Internal recursive helper function
bool message_to_json_internal(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members,
    const void* message_data,
    json& j_msg)
{
    using namespace rosidl_typesupport_introspection_cpp;
    if (g_debug_logging_enabled) std::cerr << "[message_to_json_internal] Converting... Data Ptr: " << message_data << std::endl;

    if (!members || !message_data) {
         if (g_debug_logging_enabled) std::cerr << "[message_to_json_internal] ERROR: Invalid members or message_data pointer." << std::endl;
         return false;
    }

    j_msg = json::object();

    for (uint32_t i = 0; i < members->member_count_; ++i) {
        const auto& member = members->members_[i];
        const uint8_t* field_data_ptr =
            static_cast<const uint8_t*>(message_data) + member.offset_;

        if (g_debug_logging_enabled) {
             std::cerr << "  [internal] Processing Member: " << member.name_
                       << ", Offset: " << member.offset_
                       << ", TypeID: " << static_cast<int>(member.type_id_)
                       << ", IsArray: " << member.is_array_
                       << std::endl;
        }

        json field_value;
        // Call the original field_to_json which handles primitives, arrays, and calls back here for nested messages
        if (field_to_json(member, field_data_ptr, field_value)) {
             if (g_debug_logging_enabled) std::cerr << "    [internal] Field '" << member.name_ << "' converted successfully." << std::endl;
            j_msg[member.name_] = field_value;
        } else {
             if (g_debug_logging_enabled) std::cerr << "  [internal] WARNING: Failed to convert field '" << member.name_ << "' to JSON. Setting to null." << std::endl;
            j_msg[member.name_] = nullptr;
        }
    }
    if (g_debug_logging_enabled) std::cerr << "[message_to_json_internal] Conversion complete. Data Ptr: " << message_data << std::endl;
    return true;
}

// Original top-level function (now just gets members and calls internal helper)
bool message_to_json(
    const rosidl_message_type_support_t* type_support,
    const void* message_data, // Pointer to the C++ message object
    json& j_msg)
{
    if (g_debug_logging_enabled) std::cerr << "[message_to_json - TOP LEVEL] Converting message... Data Ptr: " << message_data << std::endl;
    if (!type_support || !type_support->data) {
         if (g_debug_logging_enabled) std::cerr << "[message_to_json - TOP LEVEL] ERROR: Invalid type support handle provided." << std::endl;
        return false;
    }
    if (!message_data) {
         if (g_debug_logging_enabled) std::cerr << "[message_to_json - TOP LEVEL] ERROR: Invalid message data pointer (null)." << std::endl;
         return false;
    }

    const rosidl_typesupport_introspection_cpp::MessageMembers* members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(type_support->data);

    if (!members) {
         if (g_debug_logging_enabled) std::cerr << "[message_to_json - TOP LEVEL] ERROR: Failed to cast type_support->data to MessageMembers." << std::endl;
        return false;
    }
    
    return message_to_json_internal(members, message_data, j_msg);
}

// C interface implementation

extern "C" {

int RosParser_Initialize() {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    if (g_is_initialized) {
        return ROS_PARSER_SUCCESS;
    }
    
    // Check debug environment variable
    g_debug_logging_enabled = check_debug_env();
    std::cout << "[RosParser_Initialize] Debug logging is " << (g_debug_logging_enabled ? "ENABLED" : "DISABLED") 
              << " (set ROS_PARSER_DEBUG=1 or true to enable)." << std::endl;

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
    const unsigned char* message_data, // This is RAW CDR data
    int message_size,
    char** json_output,
    char** error_msg)
{
    // Add log at the start
    std::cerr << "[RosParser] Parsing message type: " << (message_type ? message_type : "NULL") << std::endl;

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

    // Define handles and library pointers
    const rosidl_message_type_support_t* introspection_ts = nullptr;
    const rosidl_message_type_support_t* rmw_ts = nullptr;
    const rosidl_typesupport_introspection_cpp::MessageMembers* members = nullptr;
    std::shared_ptr<rcpputils::SharedLibrary> rmw_library = nullptr; // Use SharedLibrary from rosbag2 helpers
    void* cpp_message_object = nullptr;
    json result_json; 
    std::string json_str; 

    try {
        // 1. Get INTROSPECTION type support handle (needed for structure, init/fini, JSON conversion)
        MessageTypeSupport introspection_support_wrapper;
        if (!get_message_type_support(message_type, introspection_support_wrapper)) {
            std::string err = "Unsupported message type (introspection): " + std::string(message_type);
            if (error_msg) *error_msg = allocate_string(err);
            return ROS_PARSER_ERROR_UNSUPPORTED_TYPE;
        }
        introspection_ts = introspection_support_wrapper.type_support;
        members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(introspection_ts->data);
        if (!members) {
             if (error_msg) *error_msg = allocate_string("Failed to get message members from introspection type support");
             return ROS_PARSER_ERROR_SERIALIZATION;
        }
        if (g_debug_logging_enabled) std::cerr << "Using Introspection Type Support Identifier: " << introspection_ts->typesupport_identifier << std::endl;

        // 2. Get RMW-specific type support handle using rosbag2_cpp helpers
        const std::string rmw_identifier = "rosidl_typesupport_fastrtps_cpp"; // Target RMW
        try {
             if (g_debug_logging_enabled) std::cerr << "Attempting to load RMW library for type: " << message_type 
                       << " with identifier: " << rmw_identifier << std::endl;
             rmw_library = rosbag2_cpp::get_typesupport_library(message_type, rmw_identifier);
             if (g_debug_logging_enabled) std::cerr << "Attempting to get RMW handle..." << std::endl;
             rmw_ts = rosbag2_cpp::get_typesupport_handle(message_type, rmw_identifier, rmw_library);
             if (g_debug_logging_enabled) std::cerr << "Using RMW Type Support Identifier: " << rmw_ts->typesupport_identifier << std::endl;
        } catch (const std::exception& e) {
             std::string err = "Failed to get RMW typesupport ('" + rmw_identifier + "') for " + std::string(message_type) + ": " + e.what();
             if (error_msg) *error_msg = allocate_string(err);
             // rmw_library shared_ptr will clean up automatically if partially loaded
             return ROS_PARSER_ERROR_UNSUPPORTED_TYPE;
        }

        // *** DESERIALIZATION STEP using rclcpp::SerializationBase ***
        // 3. Prepare serialized message view
        rclcpp::SerializedMessage serialized_msg_view(message_size);
        memcpy(serialized_msg_view.get_rcl_serialized_message().buffer, message_data, message_size);
        serialized_msg_view.get_rcl_serialized_message().buffer_length = message_size;
        serialized_msg_view.get_rcl_serialized_message().buffer_capacity = message_size;

        // 4. Allocate and initialize message object using INTROSPECTION members
        cpp_message_object = malloc(members->size_of_);
        if (!cpp_message_object) {
             if (error_msg) *error_msg = allocate_string("Memory allocation failed for message object");
             goto cleanup_rmw_library; // Use goto for cleanup - library cleans itself via shared_ptr
        }
        members->init_function(cpp_message_object, rosidl_runtime_cpp::MessageInitialization::ZERO);

        // 5. Deserialize using SerializationBase with RMW handle
        try {
            rclcpp::SerializationBase serialization_base(rmw_ts); // Use RMW handle
            serialization_base.deserialize_message(&serialized_msg_view, cpp_message_object);
            if (g_debug_logging_enabled) std::cerr << "rclcpp Deserialization successful using RMW handle for type: " << message_type << std::endl;
            // Add unconditional log for successful deserialization
            std::cerr << "[RosParser] Deserialization successful for type: " << message_type << std::endl;
        } catch (const std::exception& deserialize_err) {
            std::string err = "rclcpp::SerializationBase::deserialize_message failed: " + std::string(deserialize_err.what());
            if (error_msg) *error_msg = allocate_string(err);
            goto cleanup_cpp_object; 
        }

        // *** PARSING TO JSON (using introspection handle) ***
        if (!message_to_json(introspection_ts, cpp_message_object, result_json)) {
             std::string err = "Failed to convert deserialized message '" + std::string(message_type) + "' to JSON.";
             if (error_msg) *error_msg = allocate_string(err);
             goto cleanup_cpp_object;
        }

        // 7. Convert JSON to string for output
        json_str = result_json.dump();
        *json_output = allocate_string(json_str);
        if (!*json_output) {
            if (error_msg) *error_msg = allocate_string("Memory allocation failed for JSON output");
            goto cleanup_cpp_object;
        }

        // Add unconditional log for successful JSON conversion
        std::cerr << "[RosParser] JSON conversion successful. Result size: " << json_str.length() << std::endl;

        // Success path cleanup
        members->fini_function(cpp_message_object);
        free(cpp_message_object);
        cpp_message_object = nullptr; 
        // rmw_library shared_ptr cleans up automatically
        // Add unconditional log just before returning success
        std::cerr << "[RosParser] ParseToJson completed successfully for type: " << message_type << std::endl;
        return ROS_PARSER_SUCCESS;

    // Cleanup blocks using goto
    cleanup_cpp_object:
        if (cpp_message_object) {
            members->fini_function(cpp_message_object);
            free(cpp_message_object);
        }
    cleanup_rmw_library: // Label kept for structure, but library uses shared_ptr
        // No explicit unload needed for shared_ptr library handle
        return ROS_PARSER_ERROR_SERIALIZATION; 

    } catch (const std::exception& e) {
        if (error_msg) *error_msg = allocate_string(e.what());
        // Ensure cleanup happens even on generic exceptions
        if (cpp_message_object && members) { members->fini_function(cpp_message_object); free(cpp_message_object); }
        // rmw_library shared_ptr cleans up automatically
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