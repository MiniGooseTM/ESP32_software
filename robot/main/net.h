/* WiFi access point + web interface (HTTP page and WebSocket) on core 0 */
#pragma once

/* Start the access point, the HTTP server and the task that pushes logs and
   telemetry to connected browsers. NVS must already be initialised. */
void net_start(void);
