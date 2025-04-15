// Package rosparser provides a wrapper around the C++ ROS2 message parser.
package rosparser

/*
#cgo CFLAGS: -I${SRCDIR}/cpp
#cgo LDFLAGS: -L${SRCDIR}/cpp/build/lib -lros_parser -Wl,-rpath,${SRCDIR}/cpp/build/lib
#include "cpp/ros_parser.h"
#include <stdlib.h>

// Explicitly declare C functions that might not be visible to the Go compiler
int RosParser_ExtractImageData(
    const char* message_type,
    const unsigned char* message_data,
    int message_size,
    char** metadata_json,
    unsigned char** raw_data,
    int* raw_data_size,
    char** error_msg
);
*/
import "C"
import (
	"encoding/json"
	"fmt"
	"sync"
	"unsafe"

	customlog "github.com/open-teleop/controller/pkg/log"
)

var (
	// logger instance
	logger customlog.Logger
)

// SetLogger sets the logger for the rosparser package
func SetLogger(l customlog.Logger) {
	logger = l
}

// Error represents an error from the ROS parser.
type Error struct {
	Code    int
	Message string
}

// Error returns the error message.
func (e *Error) Error() string {
	return fmt.Sprintf("ROS Parser error %d: %s", e.Code, e.Message)
}

// Constants for error codes
const (
	Success            = C.ROS_PARSER_SUCCESS
	ErrorInitFailed    = C.ROS_PARSER_ERROR_INIT_FAILED
	ErrorInvalidMsg    = C.ROS_PARSER_ERROR_INVALID_MESSAGE
	ErrorUnsupported   = C.ROS_PARSER_ERROR_UNSUPPORTED_TYPE
	ErrorSerialization = C.ROS_PARSER_ERROR_SERIALIZATION
	ErrorMemory        = C.ROS_PARSER_ERROR_MEMORY
	ErrorUnknown       = C.ROS_PARSER_ERROR_UNKNOWN
)

var (
	// mu protects the initialization state
	mu sync.Mutex

	// initialized tracks whether the parser has been initialized
	initialized bool
)

// Initialize initializes the ROS2 parser.
// This must be called before any other functions.
func Initialize() error {
	mu.Lock()
	defer mu.Unlock()

	if initialized {
		if logger != nil {
			logger.Debugf("ROS Parser already initialized")
		}
		return nil
	}

	if logger != nil {
		logger.Infof("Initializing ROS Parser")
	}

	result := C.RosParser_Initialize()
	if result != Success {
		errStr := C.RosParser_GetErrorString(C.int(result))
		errMsg := C.GoString(errStr)
		if logger != nil {
			logger.Errorf("Failed to initialize ROS Parser: %s", errMsg)
		}
		return &Error{
			Code:    int(result),
			Message: errMsg,
		}
	}

	initialized = true
	if logger != nil {
		logger.Infof("ROS Parser initialized successfully")
	}
	return nil
}

// Shutdown shuts down the ROS2 parser.
// This should be called when the application exits.
func Shutdown() {
	mu.Lock()
	defer mu.Unlock()

	if initialized {
		if logger != nil {
			logger.Infof("Shutting down ROS Parser")
		}
		C.RosParser_Shutdown()
		initialized = false
		if logger != nil {
			logger.Infof("ROS Parser shutdown completed")
		}
	} else if logger != nil {
		logger.Debugf("ROS Parser shutdown called but not initialized")
	}
}

// ParseToJSON parses a ROS2 message and returns its JSON representation.
func ParseToJSON(messageType string, messageData []byte) (map[string]interface{}, error) {
	mu.Lock()
	defer mu.Unlock()

	if !initialized {
		if logger != nil {
			logger.Errorf("ROS Parser not initialized")
		}
		return nil, &Error{
			Code:    ErrorInitFailed,
			Message: "ROS Parser not initialized",
		}
	}

	if logger != nil {
		logger.Debugf("Parsing ROS message of type: %s (size: %d bytes)", messageType, len(messageData))
	}

	// Convert Go string to C string
	cMessageType := C.CString(messageType)
	defer C.free(unsafe.Pointer(cMessageType))

	// Prepare the message data
	var cMessageData *C.uchar
	if len(messageData) > 0 {
		cMessageData = (*C.uchar)(unsafe.Pointer(&messageData[0]))
	} else {
		if logger != nil {
			logger.Errorf("Empty message data provided")
		}
		return nil, &Error{
			Code:    ErrorInvalidMsg,
			Message: "Empty message data",
		}
	}

	// Prepare output parameters
	var jsonOutput *C.char
	var errorMsg *C.char

	// Call the C function
	result := C.RosParser_ParseToJson(
		cMessageType,
		cMessageData,
		C.int(len(messageData)),
		&jsonOutput,
		&errorMsg,
	)

	// Check for errors
	if result != Success {
		var errString string
		if errorMsg != nil {
			errString = C.GoString(errorMsg)
			C.RosParser_FreeString(errorMsg)
		} else {
			errString = C.GoString(C.RosParser_GetErrorString(result))
		}
		if logger != nil {
			logger.Errorf("Failed to parse ROS message: %s", errString)
		}
		return nil, &Error{
			Code:    int(result),
			Message: errString,
		}
	}

	// Convert JSON string to Go map
	jsonStr := C.GoString(jsonOutput)
	C.RosParser_FreeString(jsonOutput)

	var result_map map[string]interface{}
	if err := json.Unmarshal([]byte(jsonStr), &result_map); err != nil {
		if logger != nil {
			logger.Errorf("Failed to parse JSON result: %v", err)
		}
		return nil, fmt.Errorf("failed to parse JSON result: %w", err)
	}

	if logger != nil {
		if len(jsonStr) > 60 {
			logger.Infof("Successfully parsed ROS message to JSON %s...", jsonStr[:60])
		} else {
			logger.Infof("Successfully parsed ROS message to JSON %s", jsonStr)
		}
	}
	return result_map, nil
}

// ImageData represents a processed image from a ROS message
type ImageData struct {
	// Metadata contains image properties (width, height, encoding, etc.)
	Metadata map[string]interface{}

	// RawData contains the unprocessed binary image data
	RawData []byte
}

// ExtractImageData parses a ROS2 image message and returns both metadata and raw data.
// This function is specifically designed for sensor_msgs/msg/Image and sensor_msgs/msg/CompressedImage.
func ExtractImageData(messageType string, messageData []byte) (*ImageData, error) {
	mu.Lock()
	defer mu.Unlock()

	if !initialized {
		if logger != nil {
			logger.Errorf("ROS Parser not initialized")
		}
		return nil, &Error{
			Code:    ErrorInitFailed,
			Message: "ROS Parser not initialized",
		}
	}

	// Check if this is a supported image type
	if messageType != "sensor_msgs/msg/Image" && messageType != "sensor_msgs/msg/CompressedImage" {
		errMsg := fmt.Sprintf("Unsupported image type: %s", messageType)
		if logger != nil {
			logger.Errorf(errMsg)
		}
		return nil, &Error{
			Code:    ErrorUnsupported,
			Message: errMsg,
		}
	}

	if logger != nil {
		logger.Debugf("Extracting image data from ROS message of type: %s (size: %d bytes)",
			messageType, len(messageData))
	}

	// Convert Go string to C string
	cMessageType := C.CString(messageType)
	defer C.free(unsafe.Pointer(cMessageType))

	// Prepare the message data
	var cMessageData *C.uchar
	if len(messageData) > 0 {
		cMessageData = (*C.uchar)(unsafe.Pointer(&messageData[0]))
	} else {
		if logger != nil {
			logger.Errorf("Empty message data provided")
		}
		return nil, &Error{
			Code:    ErrorInvalidMsg,
			Message: "Empty message data",
		}
	}

	// Prepare output parameters
	var metadataJSON *C.char
	var rawData *C.uchar
	var rawDataSize C.int
	var errorMsg *C.char

	// Call the C function
	result := C.RosParser_ExtractImageData(
		cMessageType,
		cMessageData,
		C.int(len(messageData)),
		&metadataJSON,
		&rawData,
		&rawDataSize,
		&errorMsg,
	)

	// Check for errors
	if result != Success {
		var errString string
		if errorMsg != nil {
			errString = C.GoString(errorMsg)
			C.RosParser_FreeString(errorMsg)
		} else {
			errString = C.GoString(C.RosParser_GetErrorString(result))
		}
		if logger != nil {
			logger.Errorf("Failed to extract image data: %s", errString)
		}
		return nil, &Error{
			Code:    int(result),
			Message: errString,
		}
	}

	// Process the results
	metadataStr := C.GoString(metadataJSON)
	C.RosParser_FreeString(metadataJSON)

	// Create a Go copy of the raw data
	rawDataSize_int := int(rawDataSize)
	goRawData := make([]byte, rawDataSize_int)
	if rawDataSize_int > 0 {
		// Convert C pointer to Go slice with correct type
		rawDataSlice := (*[1 << 30]byte)(unsafe.Pointer(rawData))[:rawDataSize_int:rawDataSize_int]
		copy(goRawData, rawDataSlice)
		// Free the C memory
		C.free(unsafe.Pointer(rawData))
	}

	// Parse the metadata JSON
	var metadata map[string]interface{}
	if err := json.Unmarshal([]byte(metadataStr), &metadata); err != nil {
		if logger != nil {
			logger.Errorf("Failed to parse metadata JSON: %v", err)
		}
		return nil, fmt.Errorf("failed to parse metadata JSON: %w", err)
	}

	if logger != nil {
		logger.Infof("Successfully extracted image data: %d bytes with metadata", rawDataSize_int)
	}

	return &ImageData{
		Metadata: metadata,
		RawData:  goRawData,
	}, nil
}
