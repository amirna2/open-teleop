# Build stage
FROM golang:1.24-alpine AS builder

WORKDIR /app

# Copy go mod and sum files
COPY go.mod go.sum* ./

# Download dependencies
RUN go mod download

# Copy source code
COPY . .

# Build the application
RUN CGO_ENABLED=0 GOOS=linux go build -o controller ./cmd/controller

# Final stage
FROM alpine:latest

WORKDIR /app

# Install runtime dependencies
RUN apk --no-cache add ca-certificates

# Copy binary from builder stage
COPY --from=builder /app/controller .

# Set environment variables
ENV PORT=8080
ENV WEBRTC_PORT=8888

# Expose ports
EXPOSE ${PORT}
EXPOSE ${WEBRTC_PORT}

# Run the binary
CMD ["./controller"] 