/**
 * ros_parser.cpp
 * Implementation of the ROS2 message parser C interface.
 */

#include "ros_parser.h"
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include "json.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <cstring>
#include <dlfcn.h>
#include <cstdlib>
#include <cmath> // For std::isnan and std::signbit
#include <cfloat> // For FLT_MAX
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rcutils/shared_library.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rcutils/error_handling.h>
#include <rosbag2_cpp/typesupport_helpers.hpp>

using json = nlohmann::json;

static bool g_is_initialized = false;
static std::mutex g_mutex;

static bool g_debug_logging_enabled = true;

static std::unordered_map<int, std::string> g_error_strings = {
    {ROS_PARSER_SUCCESS, "Success"},
    {ROS_PARSER_ERROR_INIT_FAILED, "ROS2 initialization failed"},
    {ROS_PARSER_ERROR_INVALID_MESSAGE, "Invalid message data or format"},
    {ROS_PARSER_ERROR_UNSUPPORTED_TYPE, "Unsupported message type"},
    {ROS_PARSER_ERROR_SERIALIZATION, "Message serialization error"},
    {ROS_PARSER_ERROR_MEMORY, "Memory allocation error"},
    {ROS_PARSER_ERROR_UNKNOWN, "Unknown error"}
};

struct MessageTypeSupport {
    std::string message_type;
    std::shared_ptr<rcpputils::SharedLibrary> library;
    const rosidl_message_type_support_t* type_support;

    MessageTypeSupport() : type_support(nullptr) {}
};

static std::unordered_map<std::string, MessageTypeSupport> g_type_support_cache;

char* allocate_string(const std::string& str) {
    char* result = (char*)malloc(str.length() + 1);
    if (result) {
        strcpy(result, str.c_str());
    }
    return result;
}

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

bool get_message_type_support(const std::string& message_type, MessageTypeSupport& type_support) {
    std::lock_guard<std::mutex> lock(g_mutex);

    auto it = g_type_support_cache.find(message_type);
    if (it != g_type_support_cache.end()) {
        type_support = it->second;
        return type_support.type_support != nullptr;
    }

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

    std::string lib_path = "lib" + package_name + "__rosidl_typesupport_introspection_cpp.so";

    try {
        const char* ld_path = std::getenv("LD_LIBRARY_PATH");
        std::cerr << "LD_LIBRARY_PATH: " << (ld_path ? ld_path : "Not Set") << std::endl;

        std::cerr << "Attempting to load library: " << lib_path << std::endl;
        void* lib_handle = dlopen(lib_path.c_str(), RTLD_LAZY);
        if (!lib_handle) {
            std::cerr << "dlopen failed for " << lib_path << ": " << dlerror() << std::endl;
            return false;
        }
        std::cerr << "dlopen succeeded for " << lib_path << std::endl;
        type_support.library = std::make_shared<rcpputils::SharedLibrary>(lib_path);
        dlerror();
        std::cerr << "SharedLibrary object created for " << lib_path << std::endl;

        std::string symbol_name =
            "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" +
            package_name + "__" + msg_dir + "__" + msg_name;

        std::cerr << "Looking for symbol: " << symbol_name << std::endl;

        using GetMsgTypeSupport = const rosidl_message_type_support_t* (*)();
        dlerror();
        void* raw_symbol = dlsym(lib_handle, symbol_name.c_str());
        const char* dlsym_error = dlerror();

        GetMsgTypeSupport get_ts = reinterpret_cast<GetMsgTypeSupport>(raw_symbol);

        if (!get_ts) {
            std::cerr << "Symbol not found: " << symbol_name << std::endl;
            if (dlsym_error) {
                 std::cerr << "dlsym error: " << dlsym_error << std::endl;
            } else {
                 std::cerr << "dlsym returned null, but dlerror is null." << std::endl;
            }
            dlclose(lib_handle);
            return false;
        } else {
             std::cerr << "Symbol found successfully: " << symbol_name << std::endl;
        }

        type_support.type_support = get_ts();
        type_support.message_type = message_type;

        g_type_support_cache[message_type] = type_support;

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Exception loading type support for " << message_type << ": " << e.what() << std::endl;
        const char* dlopen_error = dlerror();
        if (dlopen_error) {
            std::cerr << "dlerror() after exception: " << dlopen_error << std::endl;
        }
        return false;
    }
}

static bool message_to_json(
    const rosidl_message_type_support_t* type_support,
    const void* message_data,
    json& j_msg);

static bool field_to_json(
    const rosidl_typesupport_introspection_cpp::MessageMember& member,
    const uint8_t* field_data_ptr,
    json& j_field);

static bool message_to_json_internal(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members,
    const void* message_data,
    json& j_msg);

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

    // Check for null pointer before any processing
    if (!field_data_ptr) {
        if (g_debug_logging_enabled) {
            std::cerr << "  [field_to_json] WARNING: Null field data pointer for type ID: " 
                      << static_cast<int>(member.type_id_) << std::endl;
        }
        j_field = nullptr;
        return false;
    }

    if (member.is_array_) {
        if (g_debug_logging_enabled) std::cerr << "    [field_to_json] Handling array..." << std::endl;
        j_field = json::array();
        size_t array_size = member.array_size_;
        const uint8_t* array_data_ptr = field_data_ptr;

        typedef struct GenericSequence {
            void * data;
            size_t size;
            size_t capacity;
        } GenericSequence;

        if (member.is_upper_bound_ || member.array_size_ == 0) {
            if (g_debug_logging_enabled) std::cerr << "      [field_to_json] Dynamic array detected." << std::endl;
            const GenericSequence* sequence = reinterpret_cast<const GenericSequence*>(field_data_ptr);
            
            // Safety check 1: Ensure sequence is valid
            if (!sequence) {
                if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Null sequence pointer." << std::endl;
                return false;
            }
            
            // Safety check 2: Ensure data pointer is valid
            if (!sequence->data) {
                if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Null data pointer in sequence." << std::endl;
                return false;
            }
            
            // Safety check 3: Ensure size is reasonable
            constexpr size_t MAX_SAFE_ARRAY_SIZE = 1000000; // 1 million elements should be plenty for any reasonable message
            if (sequence->size > MAX_SAFE_ARRAY_SIZE) {
                if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Array size too large: " << sequence->size 
                                                      << " (max: " << MAX_SAFE_ARRAY_SIZE << ")" << std::endl;
                // Set a reasonable upper bound for processing
                array_size = MAX_SAFE_ARRAY_SIZE;
                if (g_debug_logging_enabled) std::cerr << "      [field_to_json] WARNING: Limiting to " << MAX_SAFE_ARRAY_SIZE << " elements" << std::endl;
            } else {
                array_size = sequence->size;
            }
            
            array_data_ptr = static_cast<const uint8_t*>(sequence->data);
            if (g_debug_logging_enabled) std::cerr << "          [field_to_json] Dynamic Size: " << array_size << std::endl;

            // Check array_data_ptr again after getting from sequence
            if (!array_data_ptr) {
                if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Array data pointer is null after casting." << std::endl;
                return false;
            }

            switch (member.type_id_) {
                case ROS_TYPE_OCTET:
                case ROS_TYPE_UINT8: {
                    if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic uint8 array." << std::endl;
                    const uint8_t* data_typed = reinterpret_cast<const uint8_t*>(array_data_ptr);
                    try {
                        for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                    } catch (const std::exception& e) {
                        if (g_debug_logging_enabled) std::cerr << "        [field_to_json] ERROR in uint8 array processing: " 
                                                              << e.what() << std::endl;
                        return false;
                    }
                    return true;
                }
                case ROS_TYPE_FLOAT32: {
                    if (g_debug_logging_enabled) std::cerr << "        [field_to_json] Dynamic float32 array." << std::endl;
                    const float* data_typed = reinterpret_cast<const float*>(array_data_ptr);
                    // Additional safety check for float arrays (LaserScan ranges)
                    try {
                        // Validate float data by processing in chunks with additional sanity checks
                        constexpr size_t CHUNK_SIZE = 1000; // Process in smaller chunks
                        size_t remaining = array_size;
                        size_t offset = 0;
                        
                        // Note: JSON library doesn't support reserve() like std::vector
                        if (g_debug_logging_enabled && array_size > 10000) {
                            std::cerr << "        [field_to_json] Processing large array of size: " << array_size << std::endl;
                        }
                        
                        while (remaining > 0) {
                            size_t current_chunk = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
                            if (g_debug_logging_enabled && array_size > CHUNK_SIZE) 
                                std::cerr << "          [field_to_json] Processing chunk " << offset << " to " 
                                          << (offset + current_chunk) << " of " << array_size << std::endl;
                            
                            for (size_t i = 0; i < current_chunk; ++i) {
                                // Check for NaN and Inf before adding to JSON
                                float value = data_typed[offset + i];
                                if (std::isnan(value)) {
                                    value = 0.0f; // Replace NaN with 0
                                    if (g_debug_logging_enabled) std::cerr << "          [field_to_json] WARNING: NaN found at index " 
                                                                          << (offset + i) << ", replacing with 0" << std::endl;
                                } else if (std::isinf(value)) {
                                    value = std::signbit(value) ? -FLT_MAX : FLT_MAX; // Replace Inf with max float
                                    if (g_debug_logging_enabled) std::cerr << "          [field_to_json] WARNING: Inf found at index " 
                                                                          << (offset + i) << ", replacing with max value" << std::endl;
                                }
                                
                                // Add to JSON array
                                j_field.push_back(value);
                            }
                            
                            // Move to next chunk
                            offset += current_chunk;
                            remaining -= current_chunk;
                        }
                    } catch (const std::exception& e) {
                        if (g_debug_logging_enabled) std::cerr << "        [field_to_json] ERROR in float32 array processing: " 
                                                              << e.what() << std::endl;
                        return false;
                    }
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
                     const bool* data_typed = reinterpret_cast<const bool*>(array_data_ptr);
                     for (size_t i = 0; i < array_size; ++i) j_field.push_back(data_typed[i]);
                     return true;
                 }
                 default: {
                     if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Dynamic array handling not implemented for primitive type_id: " << (int)member.type_id_ << std::endl;
                     return true;
                 }
            }
        } else {
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
                    case ROS_TYPE_CHAR: element_size = sizeof(char); break;
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
    } else {
        if (g_debug_logging_enabled) std::cerr << "    [field_to_json] Handling scalar type_id: " << static_cast<int>(member.type_id_) << std::endl;
        switch (member.type_id_) {
            case ROS_TYPE_BOOL:
                j_field = *reinterpret_cast<const bool*>(field_data_ptr);
                break;
            case ROS_TYPE_OCTET:
            case ROS_TYPE_UINT8:
                j_field = *reinterpret_cast<const uint8_t*>(field_data_ptr);
                break;
            case ROS_TYPE_CHAR:
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
                    const rosidl_message_type_support_t* nested_type_support_handle = member.members_;
                    if (!nested_type_support_handle) {
                         if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Nested message type support handle (member.members_) is null." << std::endl;
                         return false;
                    }
                    const MessageMembers* nested_members =
                        static_cast<const MessageMembers*>(nested_type_support_handle->data);
                    if (!nested_members) {
                         if (g_debug_logging_enabled) std::cerr << "      [field_to_json] ERROR: Failed to get MessageMembers from nested type support handle data." << std::endl;
                         return false;
                    }
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
    return false;
}

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

    try {
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
            if (field_to_json(member, field_data_ptr, field_value)) {
                 if (g_debug_logging_enabled) std::cerr << "    [internal] Field '" << member.name_ << "' converted successfully." << std::endl;
                j_msg[member.name_] = field_value;
            } else {
                 if (g_debug_logging_enabled) std::cerr << "  [internal] WARNING: Failed to convert field '" << member.name_ << "' to JSON. Setting to null." << std::endl;
                j_msg[member.name_] = nullptr;
            }
        }
    } catch (const std::exception& e) {
        if (g_debug_logging_enabled) std::cerr << "[message_to_json_internal] EXCEPTION during message conversion: " << e.what() << std::endl;
        return false;
    }
    
    if (g_debug_logging_enabled) std::cerr << "[message_to_json_internal] Conversion complete. Data Ptr: " << message_data << std::endl;
    return true;
}

bool message_to_json(
    const rosidl_message_type_support_t* type_support,
    const void* message_data,
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

extern "C" {

int RosParser_Initialize() {
    std::lock_guard<std::mutex> lock(g_mutex);

    if (g_is_initialized) {
        return ROS_PARSER_SUCCESS;
    }

    g_debug_logging_enabled = check_debug_env();
    std::cout << "[RosParser_Initialize] Debug logging is " << (g_debug_logging_enabled ? "ENABLED" : "DISABLED")
              << " (set ROS_PARSER_DEBUG=1 or true to enable)." << std::endl;

    try {
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
        g_type_support_cache.clear();

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
    if (!g_is_initialized) {
        if (error_msg) {
            *error_msg = allocate_string("ROS Parser not initialized");
        }
        std::cerr << "[RosParser] ERROR: Parser not initialized. Aborting parse." << std::endl;
        return ROS_PARSER_ERROR_INIT_FAILED;
    }

    if (!message_type || !message_data || message_size <= 0) {
        if (error_msg) {
            *error_msg = allocate_string("Invalid input parameters");
        }
        std::cerr << "[RosParser] ERROR: Invalid input parameters (message_type=" << (message_type ? message_type : "null")
                  << ", message_data=" << (message_data ? "provided" : "null")
                  << ", message_size=" << message_size << ")." << std::endl;
        return ROS_PARSER_ERROR_INVALID_MESSAGE;
    }

    std::cerr << "[RosParser] INFO: Parsing message type: " << message_type << std::endl;

    const rosidl_message_type_support_t* introspection_ts = nullptr;
    const rosidl_message_type_support_t* rmw_ts = nullptr;
    const rosidl_typesupport_introspection_cpp::MessageMembers* members = nullptr;
    std::shared_ptr<rcpputils::SharedLibrary> rmw_library = nullptr;
    void* cpp_message_object = nullptr;
    json result_json;
    std::string json_str;

    try {
        MessageTypeSupport introspection_support_wrapper;
        if (!get_message_type_support(message_type, introspection_support_wrapper)) {
            std::string err = "Unsupported message type (introspection): " + std::string(message_type);
            if (error_msg) *error_msg = allocate_string(err);
            std::cerr << "[RosParser] ERROR: Failed to get introspection typesupport." << std::endl;
            return ROS_PARSER_ERROR_UNSUPPORTED_TYPE;
        }
        introspection_ts = introspection_support_wrapper.type_support;
        std::cerr << "[RosParser] INFO: Introspection typesupport loaded." << std::endl;

        members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(introspection_ts->data);
        if (!members) {
            if (error_msg) *error_msg = allocate_string("Failed to get message members from introspection type support");
            std::cerr << "[RosParser] ERROR: Failed to extract MessageMembers from introspection typesupport." << std::endl;
            return ROS_PARSER_ERROR_SERIALIZATION;
        }
        
        if (g_debug_logging_enabled) std::cerr << "Using Introspection Type Support Identifier: " << introspection_ts->typesupport_identifier << std::endl;

        // Get the RMW implementation from environment variable or use a fallback
        std::string rmw_identifier = "rosidl_typesupport_fastrtps_cpp"; // Default fallback
        const char* rmw_impl = std::getenv("RMW_IMPLEMENTATION");
        
        // Convert environment variable to typesupport library name if set
        if (rmw_impl) {
            std::string rmw_impl_str(rmw_impl);
            if (rmw_impl_str == "rmw_fastrtps_cpp") {
                rmw_identifier = "rosidl_typesupport_fastrtps_cpp";
            } else if (rmw_impl_str == "rmw_cyclonedds_cpp") {
                rmw_identifier = "rosidl_typesupport_introspection_cpp";
            } else {
                std::cerr << "[RosParser] WARNING: Unknown RMW_IMPLEMENTATION '" << rmw_impl_str 
                          << "', falling back to " << rmw_identifier << std::endl;
            }
        }

        try {
            rmw_library = rosbag2_cpp::get_typesupport_library(message_type, rmw_identifier);
            rmw_ts = rosbag2_cpp::get_typesupport_handle(message_type, rmw_identifier, rmw_library);
            std::cerr << "[RosParser] INFO: RMW typesupport loaded (" << rmw_identifier << ")." << std::endl;
        } catch (const std::exception& e) {
            std::string err = "Failed to get RMW typesupport ('" + rmw_identifier + "') for " + std::string(message_type) + ": " + e.what();
            if (error_msg) *error_msg = allocate_string(err);
            std::cerr << "[RosParser] ERROR: " << err << std::endl;
            return ROS_PARSER_ERROR_UNSUPPORTED_TYPE;
        }

        rclcpp::SerializedMessage serialized_msg_view(message_size);
        memcpy(serialized_msg_view.get_rcl_serialized_message().buffer, message_data, message_size);
        serialized_msg_view.get_rcl_serialized_message().buffer_length = message_size;
        serialized_msg_view.get_rcl_serialized_message().buffer_capacity = message_size;

        cpp_message_object = malloc(members->size_of_);
        if (!cpp_message_object) {
            std::string err = "Memory allocation failed for message object (size: " + std::to_string(members->size_of_) + ")";
            if (error_msg) *error_msg = allocate_string(err);
            std::cerr << "[RosParser] ERROR: " << err << std::endl;
            goto cleanup_rmw_library;
        }
        members->init_function(cpp_message_object, rosidl_runtime_cpp::MessageInitialization::ZERO);

        try {
            std::cerr << "[RosParser] INFO: Deserializing " << message_type << "..." << std::endl;
            rclcpp::SerializationBase serialization_base(rmw_ts);
            serialization_base.deserialize_message(&serialized_msg_view, cpp_message_object);
            std::cerr << "[RosParser] INFO: Deserialization successful." << std::endl;
        } catch (const std::exception& deserialize_err) {
            std::string err = "rclcpp::SerializationBase::deserialize_message failed: " + std::string(deserialize_err.what());
            if (error_msg) *error_msg = allocate_string(err);
            std::cerr << "[RosParser] ERROR: " << err << std::endl;
            goto cleanup_cpp_object;
        }

        if (!message_to_json(introspection_ts, cpp_message_object, result_json)) {
            std::string err = "Failed to convert deserialized message '" + std::string(message_type) + "' to JSON.";
            if (error_msg) *error_msg = allocate_string(err);
            std::cerr << "[RosParser] ERROR: " << err << std::endl;
            goto cleanup_cpp_object;
        }

        try {
            // Special handling for LaserScan messages which can be large
            if (std::string(message_type) == "sensor_msgs/msg/LaserScan") {
                std::cerr << "[RosParser] INFO: Processing LaserScan message, ensuring compact JSON format" << std::endl;
                // Use a compact representation to minimize memory usage
                json_str = result_json.dump(-1);  // Minimal output without whitespace
            } else {
                json_str = result_json.dump();
            }
            std::cerr << "[RosParser] INFO: JSON conversion successful (string size: " << json_str.length() << ")." << std::endl;
            
            // Log a truncated version for large JSONs
            if (json_str.length() > 1000) {
                std::cerr << "[RosParser] JSON_RESULT: " << json_str.substr(0, 500) << "... [truncated] ..." 
                         << json_str.substr(json_str.length() - 500) << std::endl;
            } else {
                std::cerr << "[RosParser] JSON_RESULT: " << json_str << std::endl;
            }
        } catch (const json::exception& json_ex) {
            std::string err = "Failed to dump JSON object to string: " + std::string(json_ex.what());
            if (error_msg) *error_msg = allocate_string(err);
            std::cerr << "[RosParser] ERROR: " << err << std::endl;
            goto cleanup_cpp_object;
        }

        *json_output = allocate_string(json_str);
        if (!*json_output) {
            std::string err = "Memory allocation failed for JSON output";
            if (error_msg) *error_msg = allocate_string(err);
            std::cerr << "[RosParser] ERROR: " << err << std::endl;
            goto cleanup_cpp_object;
        }

        members->fini_function(cpp_message_object);
        free(cpp_message_object);
        cpp_message_object = nullptr;
        std::cerr << "[RosParser] INFO: ParseToJson returning SUCCESS." << std::endl;
        return ROS_PARSER_SUCCESS;

    cleanup_cpp_object:
        if (cpp_message_object) {
            members->fini_function(cpp_message_object);
            free(cpp_message_object);
        }
    cleanup_rmw_library:
        std::cerr << "[RosParser] ERROR: ParseToJson returning FAILURE (check preceding error)." << std::endl;
        return ROS_PARSER_ERROR_SERIALIZATION;

    } catch (const std::exception& e) {
        std::string err = "Unhandled exception in RosParser_ParseToJson: " + std::string(e.what());
        if (error_msg) *error_msg = allocate_string(err);
        std::cerr << "[RosParser] FATAL: " << err << std::endl;
        if (cpp_message_object && members) { members->fini_function(cpp_message_object); free(cpp_message_object); }
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

int RosParser_ExtractImageData(
    const char* message_type,
    const unsigned char* message_data,
    int message_size,
    char** metadata_json,
    unsigned char** raw_data,
    int* raw_data_size,
    char** error_msg)
{
    if (!g_is_initialized) {
        if (error_msg) {
            *error_msg = allocate_string("ROS Parser not initialized");
        }
        std::cerr << "[RosParser] ERROR: Parser not initialized. Aborting image extraction." << std::endl;
        return ROS_PARSER_ERROR_INIT_FAILED;
    }

    if (!message_type || !message_data || message_size <= 0) {
        if (error_msg) {
            *error_msg = allocate_string("Invalid input parameters");
        }
        std::cerr << "[RosParser] ERROR: Invalid input parameters for image extraction." << std::endl;
        return ROS_PARSER_ERROR_INVALID_MESSAGE;
    }

    // Check if the message type is supported
    std::string msg_type_str(message_type);
    bool is_compressed = false;
    
    if (msg_type_str != "sensor_msgs/msg/Image" && msg_type_str != "sensor_msgs/msg/CompressedImage") {
        if (error_msg) {
            *error_msg = allocate_string("Message type is not a supported image type");
        }
        std::cerr << "[RosParser] ERROR: Message type '" << message_type 
                  << "' is not a supported image type." << std::endl;
        return ROS_PARSER_ERROR_UNSUPPORTED_TYPE;
    }
    
    std::cerr << "[RosParser] INFO: Extracting " << msg_type_str << " data." << std::endl;
    is_compressed = (msg_type_str == "sensor_msgs/msg/CompressedImage");
    
    // Variables needed for ROS message processing
    const rosidl_message_type_support_t* introspection_ts = nullptr;
    const rosidl_message_type_support_t* rmw_ts = nullptr;
    const rosidl_typesupport_introspection_cpp::MessageMembers* members = nullptr;
    std::shared_ptr<rcpputils::SharedLibrary> rmw_library = nullptr;
    void* cpp_message_object = nullptr;
    
    try {
        // Step 1: Get typesupport and deserialize the message (similar to ParseToJson)
        MessageTypeSupport introspection_support_wrapper;
        if (!get_message_type_support(message_type, introspection_support_wrapper)) {
            std::string err = "Unsupported message type (introspection): " + std::string(message_type);
            if (error_msg) *error_msg = allocate_string(err.c_str());
            std::cerr << "[RosParser] ERROR: Failed to get introspection typesupport for image." << std::endl;
            return ROS_PARSER_ERROR_UNSUPPORTED_TYPE;
        }
        introspection_ts = introspection_support_wrapper.type_support;

        members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(introspection_ts->data);
        if (!members) {
            if (error_msg) *error_msg = allocate_string("Failed to get message members from introspection type support");
            std::cerr << "[RosParser] ERROR: Failed to extract MessageMembers for image." << std::endl;
            return ROS_PARSER_ERROR_SERIALIZATION;
        }

        // Get the RMW implementation (same as ParseToJson)
        std::string rmw_identifier = "rosidl_typesupport_fastrtps_cpp"; // Default fallback
        const char* rmw_impl = std::getenv("RMW_IMPLEMENTATION");
        
        // Convert environment variable to typesupport library name if set
        if (rmw_impl) {
            std::string rmw_impl_str(rmw_impl);
            if (rmw_impl_str == "rmw_fastrtps_cpp") {
                rmw_identifier = "rosidl_typesupport_fastrtps_cpp";
            } else if (rmw_impl_str == "rmw_cyclonedds_cpp") {
                rmw_identifier = "rosidl_typesupport_introspection_cpp";
            } else {
                std::cerr << "[RosParser] WARNING: Unknown RMW_IMPLEMENTATION '" << rmw_impl_str 
                          << "', falling back to " << rmw_identifier << std::endl;
            }
        }

        try {
            rmw_library = rosbag2_cpp::get_typesupport_library(message_type, rmw_identifier);
            rmw_ts = rosbag2_cpp::get_typesupport_handle(message_type, rmw_identifier, rmw_library);
        } catch (const std::exception& e) {
            std::string err = "Failed to get RMW typesupport for image: " + std::string(e.what());
            if (error_msg) *error_msg = allocate_string(err.c_str());
            std::cerr << "[RosParser] ERROR: " << err << std::endl;
            return ROS_PARSER_ERROR_UNSUPPORTED_TYPE;
        }

        // Prepare serialized message view
        rclcpp::SerializedMessage serialized_msg_view(message_size);
        memcpy(serialized_msg_view.get_rcl_serialized_message().buffer, message_data, message_size);
        serialized_msg_view.get_rcl_serialized_message().buffer_length = message_size;
        serialized_msg_view.get_rcl_serialized_message().buffer_capacity = message_size;

        // Allocate and initialize message object
        cpp_message_object = malloc(members->size_of_);
        if (!cpp_message_object) {
            std::string err = "Memory allocation failed for image message object";
            if (error_msg) *error_msg = allocate_string(err.c_str());
            std::cerr << "[RosParser] ERROR: " << err << std::endl;
            return ROS_PARSER_ERROR_MEMORY;
        }
        members->init_function(cpp_message_object, rosidl_runtime_cpp::MessageInitialization::ZERO);

        // Deserialize the message
        try {
            std::cerr << "[RosParser] INFO: Deserializing " << msg_type_str << "..." << std::endl;
            rclcpp::SerializationBase serialization_base(rmw_ts);
            serialization_base.deserialize_message(&serialized_msg_view, cpp_message_object);
            std::cerr << "[RosParser] INFO: Deserialization successful." << std::endl;
        } catch (const std::exception& deserialize_err) {
            std::string err = "Failed to deserialize image message: " + std::string(deserialize_err.what());
            if (error_msg) *error_msg = allocate_string(err.c_str());
            std::cerr << "[RosParser] ERROR: " << err << std::endl;
            members->fini_function(cpp_message_object);
            free(cpp_message_object);
            return ROS_PARSER_ERROR_SERIALIZATION;
        }

        // Step 2: Extract metadata as JSON and raw data pointer
        json metadata;
        const uint8_t* data_ptr = nullptr;
        int data_size = 0;
        
        using namespace rosidl_typesupport_introspection_cpp;
        
        // Process based on message type
        if (is_compressed) {
            // Handle CompressedImage
            // Extract header, format, and data fields
            for (uint32_t i = 0; i < members->member_count_; ++i) {
                const auto& member = members->members_[i];
                const uint8_t* field_data_ptr = static_cast<const uint8_t*>(cpp_message_object) + member.offset_;
                
                std::string member_name(member.name_);
                
                if (member_name == "header") {
                    // Extract header info (timestamp, frame_id)
                    json header_json;
                    const MessageMembers* header_members = 
                        static_cast<const MessageMembers*>(member.members_->data);
                    
                    message_to_json_internal(header_members, field_data_ptr, header_json);
                    metadata["header"] = header_json;
                }
                else if (member_name == "format") {
                    // Extract format (string)
                    const auto* format_ptr = reinterpret_cast<const std::string*>(field_data_ptr);
                    if (format_ptr) {
                        metadata["format"] = *format_ptr;
                    }
                }
                else if (member_name == "data") {
                    // This is the raw compressed image data (bytes)
                    std::cerr << "[RosParser] INFO: Found data field in CompressedImage" << std::endl;
                    
                    // For CompressedImage message, data is std::vector<uint8_t>
                    try {
                        const std::vector<uint8_t>* image_data_vec = reinterpret_cast<const std::vector<uint8_t>*>(field_data_ptr);
                        if (image_data_vec) {
                            // Use vector's size() and data() methods
                            data_size = image_data_vec->size();
                            data_ptr = image_data_vec->data();
                            metadata["data_size"] = data_size;
                        } else {
                            std::cerr << "[RosParser] ERROR: CompressedImage data vector pointer is null" << std::endl;
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "[RosParser] ERROR: Exception accessing vector data: " << e.what() << std::endl;
                        data_ptr = nullptr;
                        data_size = 0;
                    }
                }
            }
        }
        else {
            // Handle regular Image
            // Extract header, height, width, encoding, step, data fields
            for (uint32_t i = 0; i < members->member_count_; ++i) {
                const auto& member = members->members_[i];
                const uint8_t* field_data_ptr = static_cast<const uint8_t*>(cpp_message_object) + member.offset_;
                
                std::string member_name(member.name_);
                
                if (member_name == "header") {
                    // Extract header info (timestamp, frame_id)
                    json header_json;
                    const MessageMembers* header_members = 
                        static_cast<const MessageMembers*>(member.members_->data);
                    
                    message_to_json_internal(header_members, field_data_ptr, header_json);
                    metadata["header"] = header_json;
                }
                else if (member_name == "height") {
                    metadata["height"] = *reinterpret_cast<const uint32_t*>(field_data_ptr);
                }
                else if (member_name == "width") {
                    metadata["width"] = *reinterpret_cast<const uint32_t*>(field_data_ptr);
                }
                else if (member_name == "encoding") {
                    const auto* encoding_ptr = reinterpret_cast<const std::string*>(field_data_ptr);
                    if (encoding_ptr) {
                        metadata["encoding"] = *encoding_ptr;
                    }
                }
                else if (member_name == "is_bigendian") {
                    metadata["is_bigendian"] = *reinterpret_cast<const uint8_t*>(field_data_ptr);
                }
                else if (member_name == "step") {
                    metadata["step"] = *reinterpret_cast<const uint32_t*>(field_data_ptr);
                }
                else if (member_name == "data") {
                    // This is the raw image data (bytes)
                    std::cerr << "[RosParser] INFO: Found data field in regular Image" << std::endl;
                    
                    // For Image message, data is std::vector<uint8_t>
                    try {
                        const std::vector<uint8_t>* image_data_vec = reinterpret_cast<const std::vector<uint8_t>*>(field_data_ptr);
                        if (image_data_vec) {
                            // Use vector's size() and data() methods
                            data_size = image_data_vec->size();
                            data_ptr = image_data_vec->data();
                            metadata["data_size"] = data_size;
                        } else {
                            std::cerr << "[RosParser] ERROR: Image data vector pointer is null" << std::endl;
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "[RosParser] ERROR: Exception accessing vector data: " << e.what() << std::endl;
                        data_ptr = nullptr;
                        data_size = 0;
                    }
                }
            }
        }
        
        // Validate that we found the image data
        std::cerr << "[RosParser] INFO: Checking image data - data_ptr: " << (data_ptr ? "valid" : "NULL") 
                  << ", data_size: " << data_size << std::endl;
                 
        if (!data_ptr || data_size <= 0) {
            std::string err = "Failed to extract raw image data";
            if (error_msg) *error_msg = allocate_string(err.c_str());
            std::cerr << "[RosParser] ERROR: " << err << std::endl;
            members->fini_function(cpp_message_object);
            free(cpp_message_object);
            return ROS_PARSER_ERROR_SERIALIZATION;
        }
        
        // Step 3: Convert metadata to JSON string and copy raw data
        try {
            // Convert metadata to JSON string
            std::string metadata_str = metadata.dump();
            *metadata_json = allocate_string(metadata_str);
            if (!*metadata_json) {
                std::string err = "Memory allocation failed for metadata JSON";
                if (error_msg) *error_msg = allocate_string(err.c_str());
                std::cerr << "[RosParser] ERROR: " << err << std::endl;
                members->fini_function(cpp_message_object);
                free(cpp_message_object);
                return ROS_PARSER_ERROR_MEMORY;
            }
            
            // Allocate and copy raw data
            // Note: We need to make a copy since the original data will be freed
            // when we free the message object
            unsigned char* raw_data_copy = static_cast<unsigned char*>(malloc(data_size));
            if (!raw_data_copy) {
                std::string err = "Memory allocation failed for raw image data";
                if (error_msg) *error_msg = allocate_string(err.c_str());
                std::cerr << "[RosParser] ERROR: " << err << std::endl;
                free(*metadata_json);
                *metadata_json = nullptr;
                members->fini_function(cpp_message_object);
                free(cpp_message_object);
                return ROS_PARSER_ERROR_MEMORY;
            }
            
            // Copy the data
            memcpy(raw_data_copy, data_ptr, data_size);
            
            // Set output parameters
            *raw_data = raw_data_copy;
            *raw_data_size = data_size;
            
            std::cerr << "[RosParser] INFO: Successfully extracted " 
                      << (is_compressed ? "compressed " : "") 
                      << "image data (" << data_size << " bytes)" << std::endl;
            
        } catch (const std::exception& json_ex) {
            std::string err = "Failed to convert metadata to JSON: " + std::string(json_ex.what());
            if (error_msg) *error_msg = allocate_string(err.c_str());
            std::cerr << "[RosParser] ERROR: " << err << std::endl;
            members->fini_function(cpp_message_object);
            free(cpp_message_object);
            return ROS_PARSER_ERROR_SERIALIZATION;
        }
        
        // Cleanup message object
        members->fini_function(cpp_message_object);
        free(cpp_message_object);
        
        return ROS_PARSER_SUCCESS;
        
    } catch (const std::exception& e) {
        std::string err = "Unhandled exception in RosParser_ExtractImageData: " + std::string(e.what());
        if (error_msg) *error_msg = allocate_string(err.c_str());
        std::cerr << "[RosParser] FATAL: " << err << std::endl;
        if (cpp_message_object && members) { 
            members->fini_function(cpp_message_object); 
            free(cpp_message_object); 
        }
        return ROS_PARSER_ERROR_UNKNOWN;
    }
}

} // extern "C" 