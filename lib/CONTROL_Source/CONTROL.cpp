/////////////////////////////////////////////
//           Wifi RemoteXY Include         //
/////////////////////////////////////////////

#define REMOTEXY_MODE__ESP32CORE_WIFI_POINT
#include <WiFi.h>
#include <RemoteXY.h>

// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "ESP_Drone"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = // 70 bytes
    {255, 5, 0, 0, 0, 63, 0, 16, 31, 1, 4, 0, 50, 24, 8, 51, 119, 26, 1, 0,
     4, 45, 12, 12, 2, 31, 115, 111, 108, 0, 1, 0, 18, 31, 12, 12, 119, 31, 105, 108,
     101, 114, 105, 0, 1, 0, 32, 45, 12, 12, 2, 31, 115, 97, 196, 159, 0, 1, 0, 18,
     60, 12, 12, 119, 31, 103, 101, 114, 105, 0};

// this structure defines all the variables and events of your control interface
struct
{

    // input variables
    int8_t thrust; // =0..100 slider position
    uint8_t left;  // =1 if button pressed, else =0
    uint8_t up;    // =1 if button pressed, else =0
    uint8_t right; // =1 if button pressed, else =0
    uint8_t down;  // =1 if button pressed, else =0

    // other variable
    uint8_t connect_flag; // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////