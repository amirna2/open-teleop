package log

// Logger defines a standard interface for logging.
// This allows decoupling from specific logging libraries.
type Logger interface {
	Debugf(format string, args ...interface{})
	Infof(format string, args ...interface{})
	Warnf(format string, args ...interface{})
	Errorf(format string, args ...interface{})
	Fatalf(format string, args ...interface{})

	// Add more methods as needed, e.g., for structured logging
	// WithField(key string, value interface{}) Logger
	// WithFields(fields map[string]interface{}) Logger
}
