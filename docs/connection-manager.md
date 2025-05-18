# **ConnectionManager Component – Design & Responsibilities**

## **1. Overview**
The `ConnectionManager` is a core controller component responsible for managing all client connections, subscriptions, and data delivery policies. It acts as the central hub for routing processed data (e.g., video frames, telemetry, events) to clients over various protocols (WebSocket, WebRTC, etc.), ensuring robust, scalable, and secure communication.

---

## **2. Key Responsibilities**

### **A. Client Connection Lifecycle**
- Track all active client connections (across all supported protocols).
- Handle client registration, deregistration, and resource cleanup.
- Monitor connection health (timeouts, heartbeats, etc.).

### **B. Subscription Management**
- Maintain a mapping of which clients are subscribed to which topics/streams.
- Support dynamic subscription/unsubscription at runtime.
- Enforce per-topic or per-client subscription limits if needed.

### **C. Data Routing**
- Route outbound messages (e.g., video frames, telemetry) to the correct set of clients based on their subscriptions and protocol.
- Support multiple delivery protocols (WebSocket, WebRTC, etc.) and route accordingly.
- Optionally, implement fan-out optimizations (e.g., batching, multicasting).

### **D. Delivery Policies**
- Implement delivery guarantees (best-effort, reliable, QoS, etc.).
- Handle slow or unresponsive clients (e.g., drop frames, backpressure, disconnect).
- Support rate limiting or prioritization per client/topic.

### **E. Integration Points**
- Interface with the `SessionManager` for authentication/authorization.
- Integrate with the `TopicRegistry` for topic validation and metadata.
- Provide hooks for metrics, logging, and monitoring.

---

## **3. High-Level API/Interface**

```go
type ConnectionManager interface {
    // Client lifecycle
    RegisterClient(clientID string, conn Connection, protocols []string) error
    UnregisterClient(clientID string) error
    GetActiveClients() []ClientInfo

    // Subscription management
    Subscribe(clientID, topic string) error
    Unsubscribe(clientID, topic string) error
    GetClientsForTopic(topic string) []Connection

    // Data routing
    RouteData(topic string, data []byte) error

    // Delivery policies
    SetDeliveryPolicy(topic string, policy DeliveryPolicy)
    GetDeliveryPolicy(topic string) DeliveryPolicy

    // Monitoring
    GetConnectionStats() ConnectionStats
}
```

---

## **4. Data Structures**

- **ClientInfo:** Metadata about each client (ID, protocol, connection state, subscriptions, etc.).
- **Connection:** Abstract interface for protocol-specific connections (WebSocket, WebRTC, etc.).
- **DeliveryPolicy:** Enum or struct defining delivery guarantees (e.g., best-effort, reliable, max rate).
- **ConnectionStats:** Metrics for monitoring (active clients, dropped messages, etc.).

---

## **5. Example Flow**

1. **Client connects** via WebSocket.
2. **ConnectionManager** registers the client, assigns a unique ID, and tracks the connection.
3. **Client subscribes** to a topic (e.g., `/video/main_camera`).
4. **ConnectionManager** updates the subscription map.
5. **A new video frame arrives** for `/video/main_camera`.
6. **ConnectionManager** routes the frame to all subscribed clients, applying delivery policies.
7. **Client disconnects** or times out; ConnectionManager cleans up resources.

---

## **6. Extensibility & Future Features**
- **Protocol-agnostic:** Easily add support for new protocols (e.g., gRPC, HTTP/2 push).
- **Pluggable delivery policies:** Per-topic or per-client customization.
- **Integration with SessionManager:** Enforce permissions, track user sessions.
- **Metrics & monitoring:** Expose stats for observability and debugging.
- **Admin APIs:** For live connection management, diagnostics, and control.

---

## **7. Why Centralize?**
- **Consistency:** One place to manage all connections and subscriptions.
- **Security:** Easier to enforce access control and audit connections.
- **Scalability:** Enables optimizations (e.g., multicasting, batching) and resource management.
- **Maintainability:** Decouples connection logic from business/data logic.

---

## **8. Open Questions / TODOs**
- What are the minimal delivery policies needed for v1?
- How should we handle protocol-specific quirks (e.g., WebRTC ICE restarts)?
- What metrics are most valuable for operations?
- How will we handle authentication/authorization handoff from SessionManager?
