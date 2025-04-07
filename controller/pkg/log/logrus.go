package log

import (
	"bytes"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"sort"
	"strings"

	"github.com/sirupsen/logrus"
)

// Ensure logrusLogger implements the Logger interface
var _ Logger = (*logrusLogger)(nil)

// logrusLogger wraps logrus to satisfy the Logger interface
type logrusLogger struct {
	entry *logrus.Entry
}

// NewLogrusLogger creates and configures a new logger instance using logrus.
// It logs to both console and a file (logDir/controller.log).
func NewLogrusLogger(logLevel string, logDir string) (Logger, error) {
	l := logrus.New()

	// Set Log Level
	level, err := logrus.ParseLevel(logLevel)
	if err != nil {
		level = logrus.InfoLevel // Default to Info if parsing fails
	}
	l.SetLevel(level)

	// Set Formatter to our custom SimpleFormatter
	l.SetFormatter(&SimpleFormatter{
		TimestampFormat: "2006/01/02 15:04:05.000000", // Added microseconds
	})

	// Set Console Output
	consoleWriter := os.Stdout

	// Set File Output
	if logDir != "" {
		err := os.MkdirAll(logDir, 0755)
		if err != nil {
			return nil, fmt.Errorf("failed to create log directory '%s': %w", logDir, err)
		}
		logFilePath := filepath.Join(logDir, "controller.log")
		logFile, err := os.OpenFile(logFilePath, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0666)
		if err != nil {
			return nil, fmt.Errorf("failed to open log file '%s': %w", logFilePath, err)
		}
		// Use MultiWriter to log to both console and file
		l.SetOutput(io.MultiWriter(consoleWriter, logFile))
	} else {
		// Only log to console if logDir is empty
		l.SetOutput(consoleWriter)
	}

	return &logrusLogger{entry: logrus.NewEntry(l)}, nil
}

// --- Interface Method Implementations ---

func (l *logrusLogger) Debugf(format string, args ...interface{}) {
	l.entry.Debugf(format, args...)
}

func (l *logrusLogger) Infof(format string, args ...interface{}) {
	l.entry.Infof(format, args...)
}

func (l *logrusLogger) Warnf(format string, args ...interface{}) {
	l.entry.Warnf(format, args...)
}

func (l *logrusLogger) Errorf(format string, args ...interface{}) {
	l.entry.Errorf(format, args...)
}

func (l *logrusLogger) Fatalf(format string, args ...interface{}) {
	l.entry.Fatalf(format, args...)
}

// Example for adding structured logging later:
// func (l *logrusLogger) WithField(key string, value interface{}) Logger {
// 	return &logrusLogger{entry: l.entry.WithField(key, value)}
// }
// func (l *logrusLogger) WithFields(fields map[string]interface{}) Logger {
// 	return &logrusLogger{entry: l.entry.WithFields(logrus.Fields(fields))}
// }

// --- Custom Formatter Implementation ---

// SimpleFormatter formats logs in a more concise way, similar to standard log
// Example: 2025/04/06 17:30:00 [INF] Log message here key1=value1 key2=value2
type SimpleFormatter struct {
	TimestampFormat string
}

// Format implements the logrus.Formatter interface
func (f *SimpleFormatter) Format(entry *logrus.Entry) ([]byte, error) {
	var b *bytes.Buffer
	if entry.Buffer != nil {
		b = entry.Buffer
	} else {
		b = &bytes.Buffer{}
	}

	timestampFormat := f.TimestampFormat
	if timestampFormat == "" {
		timestampFormat = "2006/01/02 15:04:05.000000" // Default format with microseconds
	}

	// Timestamp
	b.WriteString(entry.Time.Format(timestampFormat))
	b.WriteString(" ")

	// Level (e.g., [INF], [WRN])
	level := strings.ToUpper(entry.Level.String())
	if len(level) > 3 {
		level = level[:3] // Truncate (e.g., WARNING -> WAR)
	}
	fmt.Fprintf(b, "[%s] ", level)

	// Message
	b.WriteString(entry.Message)

	// Fields (optional, appended at the end)
	if len(entry.Data) > 0 {
		keys := make([]string, 0, len(entry.Data))
		for k := range entry.Data {
			keys = append(keys, k)
		}
		sort.Strings(keys) // Sort for consistent output
		for _, k := range keys {
			b.WriteString(" ")
			fmt.Fprintf(b, "%s=%v", k, entry.Data[k])
		}
	}

	b.WriteByte('\n')
	return b.Bytes(), nil
}
