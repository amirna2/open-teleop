// Package rosparser provides a wrapper around the C++ ROS2 message parser.
package rosparser

/*
#cgo CFLAGS: -I${SRCDIR}/cpp
#cgo LDFLAGS: -L${SRCDIR}/cpp/build/lib -lros_parser -Wl,-rpath,${SRCDIR}/cpp/build/lib
#include "cpp/ros_parser.h"
#include <stdlib.h>
*/
import "C"
import (
	"encoding/json"
	"fmt"
	"sync"
	"unsafe"
)

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
		return nil
	}

	result := C.RosParser_Initialize()
	if result != Success {
		errStr := C.RosParser_GetErrorString(C.int(result))
		return &Error{
			Code:    int(result),
			Message: C.GoString(errStr),
		}
	}

	initialized = true
	return nil
}

// Shutdown shuts down the ROS2 parser.
// This should be called when the application exits.
func Shutdown() {
	mu.Lock()
	defer mu.Unlock()

	if initialized {
		C.RosParser_Shutdown()
		initialized = false
	}
}

// ParseToJSON parses a ROS2 message and returns its JSON representation.
func ParseToJSON(messageType string, messageData []byte) (map[string]interface{}, error) {
	mu.Lock()
	defer mu.Unlock()

	if !initialized {
		return nil, &Error{
			Code:    ErrorInitFailed,
			Message: "ROS Parser not initialized",
		}
	}

	// Convert Go string to C string
	cMessageType := C.CString(messageType)
	defer C.free(unsafe.Pointer(cMessageType))

	// Prepare the message data
	var cMessageData *C.uchar
	if len(messageData) > 0 {
		cMessageData = (*C.uchar)(unsafe.Pointer(&messageData[0]))
	} else {
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
		return nil, fmt.Errorf("failed to parse JSON result: %w", err)
	}

	return result_map, nil
}
