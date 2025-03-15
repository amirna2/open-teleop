# Build stage
FROM golang:1.24-alpine AS builder

WORKDIR /app

# Install build dependencies including ZeroMQ
RUN apk add --no-cache gcc musl-dev zeromq-dev

# Copy go mod and sum files
COPY go.mod go.sum* ./

# Download dependencies
RUN go mod download

# Copy source code
COPY . .

# Build the application with CGO enabled for ZeroMQ
RUN CGO_ENABLED=1 GOOS=linux go build -o controller ./cmd/controller

# Final stage
FROM alpine:latest

WORKDIR /app

# Install runtime dependencies
RUN apk --no-cache add ca-certificates zeromq

# Copy binary from builder stage
COPY --from=builder /app/controller .

# Set environment variables
ENV PORT=8080
ENV WEBRTC_PORT=8888

# Expose ports
EXPOSE ${PORT}
EXPOSE ${WEBRTC_PORT}
EXPOSE 5555  # ZeroMQ port

# Run the binary
CMD ["./controller"] 