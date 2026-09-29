// ================== OMPM v68 - Dual Mode Energy Monitor ==================
// Last change 25/05/2026 - stability patch v2: measurement-safe SD throttling, boot steps, reset reason, daily restart
// Hardware target:
//   - ESP32 controller
//   - 6 physical PZEM004T modules mapped as M0 to M5
//   - ST7735 TFT display
//   - SD card logging
//   - Optional WiFi, web server, MQTT, OTA, and NTP support
//
// Operating modes:
//   - Mode 1: M0 is the physical aggregate meter. M1-M5 are individual loads.
//             M6 is not used.
//   - Mode 2: M0-M5 are individual meters. M6 is calculated as the sum of M0-M5.
//
// Measurement scheduler:
//   - Reads one PZEM module per scheduler turn instead of scanning all modules at once.
//   - Avoids blocking delays between PZEM modules.
//   - Reads current, power, and power factor per channel.
//   - Uses M0 as the shared voltage and frequency reference.
//   - With 6 modules and a 70 ms scheduler interval, one full scan takes about 420 ms.
//   - Effective refresh rate per physical channel is about 2.4 Hz while the screen is active.
//
// Display and logging:
//   - TFT refresh is decoupled from PZEM reads to reduce flicker and bus contention.
//   - SD logging stores one consolidated sample per configured logging interval.
//   - Screen saver turns the backlight off after inactivity while keeping measurements running.
//
// Maintenance notes:
//   - All user-facing labels and comments should remain in English.
//   - Debug output through Serial is disabled by commenting lines with the prefix:
//     SERIAL_DISABLED:
//   - Serial2 must remain enabled because it is used to communicate with the PZEM modules.
//

//Qué cambia esta versión:
//Mantiene la frecuencia original
//La SD se usa al arrancar solo para leer configuración
//Después de WiFi + NTP crea ficheros nuevos por sesión
//El número cambia en cada arranque usando la hora NTP. Así, si un CSV viejo queda raro, corrupto o demasiado grande, no se toca al siguiente arranque.
//No escanea CSV antiguos al final del setup()
//Esto es importante. Si la tarjeta tiene ficheros antiguos problemáticos, el firmware no debe recorrerlos al arrancar.
//Elimina flush() en cada muestra
//Se mantiene close(), que ya confirma la escritura. El flush() explícito en cada línea añade castigo a la FAT sin darte mucha ventaja en este caso.
 //Dual-SPI + simple current calibration: UP/DOWN ref current, OK apply, long OK/# save-next.//

//escribe temperatura y humedad en los csv


// Last reviewed: 2026-05-06   13:04
//TRAJETA DE 8GB FORMATEADA EN FAT32

#include <WiFi.h>
#include <Arduino.h>
#include <PZEM004Tv30.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include "esp_system.h"

// ================== ST7735 SLEEP COMMANDS ==================
#ifndef ST7735_SLPIN
#define ST7735_SLPIN 0x10  // Sleep in
#endif
#ifndef ST7735_SLPOUT
#define ST7735_SLPOUT 0x11  // Sleep out
#endif
#ifndef ST7735_DISPOFF
#define ST7735_DISPOFF 0x28  // Display off
#endif
#ifndef ST7735_DISPON
#define ST7735_DISPON 0x29  // Display on
#endif

// ================== STRUCTURES ==================
struct Button {
  int pin;
  bool lastState;
  unsigned long lastChange;
};

// ================== PROTOTYPES ==================
void initLayout();
void showMenu();
void showMessage(const char* msg);
void bootStep(const char* msg);

void showCalSoft();
void handleCalSoftWithFlags(bool upPressed, bool downPressed, bool menuPressed);
void applySimpleCurrentCalibration();
void saveCalAndNextChannel();
void showPzemAddrSel();
void showPzemAddrConfirm();
void showSDmem();
void showWifiInfo();
void showPzemStatus();
void showRefreshConfig();


void showCurrentGraph();
void showPowerGraph();
void showCurrentHist();
void showPowerHist();
void showSavingCalib();
void showCalibSavedOK();
void showCurrentCalib();
void showWebServerIP();

void updateOTAProgress(int progress);
void updateDisplay();

void handleMenuWithFlags(bool upPressed, bool downPressed, bool menuPressed);
void executeCalibrationSubmenu();
void executeGraphsSubmenu();
void handleCalibrationMenuWithFlags(bool upPressed, bool downPressed, bool menuPressed);
void handleGraphsMenuWithFlags(bool upPressed, bool downPressed, bool menuPressed);

void initCalFactors();
void saveCalibration();
void loadCalibration();
void resetCalibrationDefaults();
void updateCalSoftValue();
float readCalibrationValue(int type, int channel);
void verifyAddresses();
void forceSDWrite();
void logRestartToSD();
void safeRestart();
void logSystemEvent(const char* eventName);
void logMemory(const char* location);
String getHistoricalDataRange(long startTimeSec, long endTimeSec);

void writeSDDeferred();
void closeLogFiles();
bool openLogFiles();
bool openLogFile(uint8_t meter);
bool writeCSVRowToLog(uint8_t meter, long long timestamp);
void flushLogFiles(bool force = false);
void calculateAggregate();


bool initWiFi();
bool loadConfig();
String getIPWithPort();
String getURLWithPort();
String getFormattedTime();

void initOTA();
void handleOTAStatus();

void diagnoseCSV();
void createSessionCsvFileNames();

void initWebServer();
void handleRoot();
void handleDownload();
void handleView();
void handleRealtime();
void handleSDInfo();
void handleRestart();
void handleGraphs();
void handleGraphData();
void handleExport();
void handleExportCSV();
void handleExportAll();
void handleConfig();
void handleTest();
void handleAdmin();
void handleDiagnostics();


void handleRebootLog();
void handleEventsLog();
void handleLogsIndex();
void handleClearLog();

void handleOTAWeb();

bool buttonFallingEdge(Button& b, unsigned long debounceMs = 60);

// ========== MQTT PROTOTYPES ==========
void publishMqttStatus();
void onMqttMessage(char* topic, byte* payload, unsigned int length);
// ==================================================

void checkScreenSaver();
void registerUserInteraction(const char* source = "unknown");
void updateDHT11();
bool readDHT11Raw(int pin, float& tempC, float& hum);
void readFastPZEMChannel(uint8_t idx, unsigned long now);
void showIRCalibrationMenu();
void showIRGraphsMenu();
void handleIRShortcutCode(uint32_t code);
void updateSharedVoltageFrequency(unsigned long now);
bool processIRRemote(bool& upPressed, bool& downPressed, bool& menuPressed);
const char* irKeyName(uint32_t code);

// ==================  WEB PAGES IN PROGMEM ==================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>OMPM - Energy Monitor</title>
  <style>
    body { font-family: 'Segoe UI', Arial, sans-serif; margin: 0; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; }
    .container { max-width: 1200px; margin: 0 auto; }
    .header { background: rgba(255, 255, 255, 0.95); padding: 30px; border-radius: 15px; box-shadow: 0 10px 30px rgba(0,0,0,0.2); margin-bottom: 30px; text-align: center; }
    .header h1 { color: #2c3e50; margin: 0; font-size: 2.2em; }
    .header p { color: #7f8c8d; font-size: 1.1em; }
    .stats-row { display: flex; justify-content: space-between; background: rgba(255, 255, 255, 0.95); border-radius: 15px; padding: 25px; margin: 25px 0; box-shadow: 0 5px 15px rgba(0,0,0,0.1); }
    .stat-item { text-align: center; flex: 1; }
    .stat-value { font-size: 2.2em; font-weight: bold; margin-bottom: 5px; }
    .stat-label { color: #7f8c8d; font-size: 0.9em; text-transform: uppercase; letter-spacing: 1px; }
    .dashboard-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 25px; margin-bottom: 30px; }
    .card { background: white; padding: 25px; border-radius: 15px; box-shadow: 0 5px 15px rgba(0,0,0,0.1); transition: transform 0.3s; }
    .card:hover { transform: translateY(-5px); box-shadow: 0 10px 25px rgba(0,0,0,0.15); }
    .card h2 { color: #2c3e50; margin-top: 0; border-bottom: 2px solid #3498db; padding-bottom: 10px; }
    .btn { display: inline-block; padding: 12px 25px; background: #3498db; color: white; text-decoration: none; border-radius: 25px; margin: 5px; border: none; cursor: pointer; font-weight: bold; transition: all 0.3s; }
    .btn:hover { transform: scale(1.05); box-shadow: 0 5px 15px rgba(0,0,0,0.2); }
    .btn-green { background: #2ecc71; }
    .btn-red { background: #e74c3c; }
    .btn-purple { background: #9b59b6; }
    .btn-orange { background: #e67e22; }
    .feature-list { list-style: none; padding: 0; }
    .feature-list li { padding: 10px 0; border-bottom: 1px solid #eee; display: flex; align-items: center; }
    .feature-list li:before { content: '✓'; color: #2ecc71; font-weight: bold; margin-right: 10px; }
    .status-badge { display: inline-block; padding: 5px 10px; border-radius: 20px; font-size: 0.8em; margin-left: 10px; background: #2ecc71; color: white; }
    .quick-access { background: white; border-radius: 15px; padding: 25px; margin: 30px 0; box-shadow: 0 5px 15px rgba(0,0,0,0.1); }
    .quick-buttons { display: flex; flex-wrap: wrap; justify-content: center; gap: 15px; }
    .quick-btn { display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100px; height: 100px; background: linear-gradient(135deg, #3498db, #2980b9); color: white; border-radius: 15px; text-decoration: none; transition: all 0.3s; padding: 15px; text-align: center; }
    .quick-btn:hover { transform: translateY(-5px); box-shadow: 0 10px 20px rgba(0,0,0,0.2); }
    .quick-btn i { font-size: 2em; margin-bottom: 8px; }
    .footer { text-align: center; color: white; margin-top: 40px; opacity: 0.8; }
    @media (max-width: 768px) { .stats-row { flex-wrap: wrap; } .stat-item { flex: 0 0 50%; margin-bottom: 15px; } .quick-btn { width: 80px; height: 80px; } }
  </style>
  <link rel='stylesheet' href='https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css'>
</head>
<body>
  <div class='container'>
    <div class='header'>
      <h1><i class='fas fa-bolt'></i> OMPM - Energy Monitor v68 </h1>
      <p>Version Jun/2026 | Port: __PORT__ | IP: __IP__</p>
      <div style='margin-top:10px;'>
        <span class='status-badge'><i class='fas fa-wifi'></i> OTA: __OTA_PORT__</span>
        <a href='/ota/update' style='color:#3498db; margin-left:10px;'>Activate OTA</a>
      </div>
    </div>

    <div class='stats-row'>
      <div class='stat-item'>
        <div class='stat-value' style='color:#3498db;'><i class='fas fa-clock'></i> __TIME__</div>
        <div class='stat-label'>System Time</div>
      </div>
      <div class='stat-item'>
        <div class='stat-value' style='color:__POWER_COLOR__;'><i class='fas fa-bolt'></i> __POWER__ W</div>
        <div class='stat-label'>Total Power</div>
      </div>
      <div class='stat-item'>
        <div class='stat-value' style='color:__VOLTAGE_COLOR__;'><i class='fas fa-plug'></i> __VOLTAGE__ V</div>
        <div class='stat-label'>Average Voltage</div>
      </div>
      <div class='stat-item'>
        <div class='stat-value' style='color:__METERS_COLOR__;'><i class='fas fa-microchip'></i> __METERS__/6</div>
        <div class='stat-label'>Active Meters</div>
      </div>
    </div>

    <div class='dashboard-grid'>
      <div class='card'>
        <h2><i class='fas fa-download'></i> Download Data</h2>
        <p>CSV files for each meter:</p>
        <div class='feature-list'>
          __DOWNLOAD_LINKS__
        </div>
        <a href='/export' class='btn btn-green'><i class='fas fa-file-export'></i> Advanced Export</a>
      </div>

      <div class='card'>
        <h2><i class='fas fa-eye'></i> View Data</h2>
        <p>File contents:</p>
        <div class='feature-list'>
          __VIEW_LINKS__
        </div>
        <a href='/realtime' class='btn'><i class='fas fa-chart-line'></i> Real Time</a>
      </div>

      <div class='card'>
        <h2><i class='fas fa-chart-bar'></i> Graphs</h2>
        <p>Interactive consumption analysis:</p>
        <div class='feature-list'>
          <li><i class='fas fa-chart-line'></i> Real-time graphs</li>
          <li><i class='fas fa-history'></i> Hourly history</li>
          <li><i class='fas fa-balance-scale'></i> Meter comparison</li>
        </div>
        <a href='/graphs' class='btn btn-purple'><i class='fas fa-chart-area'></i> View Graphs</a>
      </div>

      <div class='card'>
        <h2><i class='fas fa-hdd'></i> System Status</h2>
        <p>Storage and diagnostics:</p>
        <div class='feature-list'>
          <li><i class='fas fa-database'></i> Total SD: __SD_TOTAL__ MB</li>
          <li><i class='fas fa-hdd'></i> Used: __SD_USED__ MB</li>
          <li><i class='fas fa-hdd'></i> Free: __SD_FREE__ MB</li>
          <li><i class='fas fa-percentage'></i> Usage: __SD_PERCENT__%</li>
        </div>
        <a href='/sdinfo' class='btn btn-orange'><i class='fas fa-info-circle'></i> SD Details</a>
      </div>
   <div class='card'>
        <h2><i class='fas fa-history'></i> System Logs</h2>
        <p>Diagnostic information:</p>
        <div class='feature-list'>
          <li><i class='fas fa-redo-alt'></i> Reboot history</li>
          <li><i class='fas fa-exclamation-triangle'></i> System events</li>
          <li><i class='fas fa-chart-line'></i> Error tracking</li>
        </div>
        <a href='/logs' class='btn' style='background:#f39c12;'><i class='fas fa-scroll'></i> View Logs</a>
      </div>
      <div class='card'>
        <h2><i class='fas fa-tachometer-alt'></i> Live Panel</h2>
        <p>Current meter status:</p>
        <div class='feature-list'>
          __METER_STATUS__
        </div>
        <a href='/realtime' class='btn'><i class='fas fa-tachometer-alt'></i> View Full Panel</a>
      </div>

      <div class='card'>
        <h2><i class='fas fa-cogs'></i> Administration</h2>
        <p>Advanced tools:</p>
        <div class='feature-list'>
          <li><i class='fas fa-stethoscope'></i> System diagnostics</li>
          <li><i class='fas fa-wifi'></i> Connectivity tests</li>
          <li><i class='fas fa-redo'></i> Controlled restart</li>
        </div>
        <a href='/admin' class='btn btn-red'><i class='fas fa-user-shield'></i> Admin Panel</a>
        <a href='/restart' class='btn' style='background:#95a5a6;' onclick='return confirm("Restart system?")'><i class='fas fa-power-off'></i> Restart</a>
      </div>
    </div>

    <div class='quick-access'>
      <h3 style='text-align:center;'><i class='fas fa-rocket'></i> Quick Access</h3>
      <div class='quick-buttons'>
        <a href='/graphs' class='quick-btn' style='background:linear-gradient(135deg,#9b59b6,#8e44ad);'>
          <i class='fas fa-chart-area'></i>
          <span>Graphs</span>
        </a>
        <a href='/realtime' class='quick-btn' style='background:linear-gradient(135deg,#3498db,#2980b9);'>
          <i class='fas fa-tachometer-alt'></i>
          <span>Real Time</span>
        </a>
        <a href='/export' class='quick-btn' style='background:linear-gradient(135deg,#2ecc71,#27ae60);'>
          <i class='fas fa-file-export'></i>
          <span>Export</span>
        </a>
        <a href='/sdinfo' class='quick-btn' style='background:linear-gradient(135deg,#e67e22,#d35400);'>
          <i class='fas fa-hdd'></i>
          <span>System</span>
        </a>
        <a href='/admin' class='quick-btn' style='background:linear-gradient(135deg,#e74c3c,#c0392b);'>
          <i class='fas fa-user-shield'></i>
          <span>Admin</span>
        </a>
        <a href='/' class='quick-btn' style='background:linear-gradient(135deg,#1abc9c,#16a085);'>
          <i class='fas fa-home'></i>
          <span>Home</span>
        </a>
      </div>
    </div>

   // In the INDEX_HTML footer, add:
    <div class='footer'>
      <p><i class='fas fa-bolt'></i> OMPM v68 - Energy Monitoring System | <i class='fas fa-clock'></i> __TIME__ | <i class='fas fa-microchip'></i> Free: __FREE_HEAP__ KB</p>
      <p style='font-size:0.8em; margin-top:5px;'>
        <a href='/logs' style='color:#f39c12; text-decoration:none;'>📋 Logs</a> | 
        <a href='/admin' style='color:#e74c3c; text-decoration:none;'>⚙️ Admin</a>
      </p>
    </div>
</body>
</html>
)rawliteral";

const char REALTIME_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <meta http-equiv='refresh' content='5'>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>Real Time - OMPM</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }
    .container { max-width: 1200px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
    h1 { color: #2c3e50; }
    table { width: 100%; border-collapse: collapse; margin: 20px 0; }
    th { background: #2c3e50; color: white; padding: 12px; }
    td { padding: 10px; border-bottom: 1px solid #ddd; text-align: center; }
    .voltage { color: #3498db; font-weight: bold; }
    .current { color: #e67e22; font-weight: bold; }
    .power { color: #27ae60; font-weight: bold; }
    .badge { display: inline-block; padding: 3px 8px; border-radius: 12px; color: white; font-size: 0.8em; }
    .badge-active { background: #2ecc71; }
    .badge-standby { background: #f39c12; }
    .badge-off { background: #e74c3c; }
    .btn { display: inline-block; padding: 10px 20px; background: #3498db; color: white; text-decoration: none; border-radius: 5px; }
    .footer { margin-top: 20px; text-align: center; }
  </style>
</head>
<body>
  <div class='container'>
    <h1>⚡ Real Time - OMPM v68</h1>
    <p>Last update: __TIME__</p>
    <table>
      <tr>
        <th>Meter</th>
        <th>Status</th>
        <th>Voltage (V)</th>
        <th>Current (A)</th>
        <th>Power (W)</th>
        <th>Frequency (Hz)</th>
        <th>PF</th>
        <th>CSV</th>
      </tr>
      __TABLE_ROWS__
    </table>
    <h2>📊 Totals</h2>
    <p><strong>Total Power:</strong> __TOTAL_POWER__ W</p>
    <p><strong>Average Voltage:</strong> __AVG_VOLTAGE__ V</p>
    <p><strong>Active Meters:</strong> __ACTIVE_METERS__/6</p>
    <div class='footer'>
      <a href='/' class='btn'>🏠 Back</a>
      <a href='/graphs' class='btn' style='background:#9b59b6;'>📈 Graphs</a>
      <a href='/export' class='btn' style='background:#2ecc71;'>📥 Export</a>
    </div>
    <p><small>Auto-refresh every 5 seconds</small></p>
  </div>
</body>
</html>
)rawliteral";

const char ADMIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>Administration - OMPM</title>
  <style>
    body { font-family: Arial; margin: 20px; background: #f5f5f5; }
    .container { max-width: 800px; margin: 0 auto; }
    .card { background: white; padding: 25px; border-radius: 10px; margin: 20px 0; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
    h1 { color: #2c3e50; }
    .btn { display: inline-block; padding: 12px 25px; background: #3498db; color: white; text-decoration: none; border-radius: 5px; margin: 5px; }
    .btn-green { background: #2ecc71; }
    .btn-red { background: #e74c3c; }
    .btn-orange { background: #e67e22; }
    .info { background: #f8f9fa; padding: 15px; border-left: 4px solid #3498db; margin: 10px 0; }
  </style>
</head>
<body>
  <div class='container'>
    <h1>⚙️ Administration Panel</h1>
    
    <div class='card'>
      <h2>📱 OTA Update</h2>
      <div class='info'>
        <p><strong>Hostname:</strong> __HOSTNAME__</p>
        <p><strong>IP:</strong> __IP__</p>
        <p><strong>OTA Port:</strong> __OTA_PORT__</p>
        <p><strong>Status:</strong> <span style='color:#2ecc71;'>✅ READY</span></p>
      </div>
      <a href='/ota/update' class='btn btn-green'>🔄 Activate OTA Mode</a>
      <a href='/ota' class='btn'>ℹ️ Instructions</a>
    </div>
    
    <div class='card'>
      <h2>🔄 System</h2>
      <a href='/restart' class='btn btn-red' onclick='return confirm("Restart system?")'>🔁 Restart ESP32</a>
      <a href='/sdinfo' class='btn'>💾 SD Status</a>
    </div>
    
    <div class='card'>
      <h2>🔧 Tools</h2>
      <a href='/diagnostics' class='btn btn-orange'>📋 Diagnostics</a>
      <a href='/test' class='btn'>✅ Test</a>
       <a href='/logs' class='btn' style='background:#f39c12;'>📋 System Logs</a>
    </div>
    
    <p><a href='/'>🏠 Back to Dashboard</a></p>
  </div>
</body>
</html>
)rawliteral";

const char EXPORT_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <title>Export Data - OMPM</title>
  <style>
    body { font-family: Arial; margin: 20px; background: #f5f5f5; }
    .container { max-width: 800px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; }
    h1 { color: #2c3e50; }
    .btn { display: inline-block; padding: 12px 25px; background: #3498db; color: white; text-decoration: none; border-radius: 5px; margin: 5px; }
    .btn-green { background: #2ecc71; }
    .meter-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin: 20px 0; }
    .meter-item { background: #f8f9fa; padding: 15px; border-radius: 8px; text-align: center; }
  </style>
</head>
<body>
  <div class='container'>
    <h1>📥 Export Data</h1>
    
    <h2>🚀 Quick Export</h2>
    <a href='/exportall' class='btn btn-green'>📥 All meters (CSV)</a>
    
    <h2>📊 Individual Meters</h2>
    <div class='meter-grid'>
      __METER_BUTTONS__
    </div>
    
    <p><a href='/'>🏠 Back</a></p>
  </div>
</body>
</html>
)rawliteral";

const char SDINFO_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <title>SD Info - OMPM</title>
  <style>
    body { font-family: Arial; margin: 20px; background: #f5f5f5; }
    .container { max-width: 800px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; }
    h1 { color: #2c3e50; }
    .info { background: #f8f9fa; padding: 20px; border-radius: 8px; margin: 20px 0; }
    .progress { background: #e0e0e0; height: 20px; border-radius: 10px; margin: 10px 0; overflow: hidden; }
    .progress-bar { background: #3498db; height: 100%; width: __PERCENT__%; }
    .file-list { background: #f8f9fa; padding: 20px; border-radius: 8px; }
  </style>
</head>
<body>
  <div class='container'>
    <h1>💾 SD Card Information</h1>
    
    <div class='info'>
      <h2>Storage</h2>
      <p><strong>Total:</strong> __TOTAL__ MB</p>
      <p><strong>Used:</strong> __USED__ MB (__PERCENT__%)</p>
      <p><strong>Free:</strong> __FREE__ MB</p>
      <div class='progress'>
        <div class='progress-bar'></div>
      </div>
    </div>
    
    <div class='file-list'>
      <h2>Files</h2>
      <ul>
        __FILE_LIST__
      </ul>
    </div>
    
    <p><a href='/'>🏠 Back</a></p>
  </div>
</body>
</html>
)rawliteral";

const char OTA_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <meta http-equiv='refresh' content='5;url=/'>
  <title>OTA - OMPM</title>
  <style>
    body { font-family: Arial; margin: 20px; background: #f5f5f5; }
    .container { max-width: 600px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; text-align: center; }
    .success { color: #2ecc71; font-size: 1.5em; }
    .info { background: #3498db; color: white; padding: 20px; border-radius: 10px; margin: 20px 0; text-align: left; }
  </style>
</head>
<body>
  <div class='container'>
    <h1 class='success'>✅ OTA Mode Activated</h1>
    <div class='info'>
      <h2>📱 Instructions:</h2>
      <ol>
        <li>Arduino IDE: Tools → Port → Network Ports</li>
        <li>Select: <strong>__HOSTNAME__ at __IP__</strong></li>
        <li>Upload firmware (→)</li>
      </ol>
    </div>
    <p><strong>IP:</strong> __IP__</p>
    <p><strong>Port:</strong> __OTA_PORT__</p>
    <p><a href='/' style='color:#3498db;'>Back to home</a></p>
  </div>
</body>
</html>
)rawliteral";

const char OTAWEB_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <title>OTA Info - OMPM</title>
  <style>
    body { font-family: Arial; margin: 20px; background: #f5f5f5; }
    .container { max-width: 600px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; }
    .info { background: #f8f9fa; padding: 20px; border-radius: 8px; }
  </style>
</head>
<body>
  <div class='container'>
    <h1>📱 OTA Information</h1>
    <div class='info'>
      <p><strong>Hostname:</strong> __HOSTNAME__</p>
      <p><strong>IP:</strong> __IP__</p>
      <p><strong>OTA Port:</strong> 3232</p>
      <p><strong>Free memory:</strong> __HEAP__ bytes</p>
      <p><strong>Status:</strong> <span style='color:#2ecc71;'>✅ ACTIVE</span></p>
    </div>
    <p><a href='/admin'>⚙️ Back to Admin</a> | <a href='/'>🏠 Home</a></p>
  </div>
</body>
</html>
)rawliteral";

// ========== SCREEN SAVER ==========
bool screenSaverActive = false;
unsigned long lastUserInteraction = 0;
const unsigned long SCREENSAVER_TIMEOUT = 240000;   // 4 minutes
const unsigned long SCREENSAVER_COOLDOWN = 120000;  // 2 minutes minimum on time
unsigned long screenOnTime = 0;
bool screenForcedOn = false;

// ========== INTERNAL LED ==========
#define LED_BUILTIN 2  // GPIO2 is connected to the internal LED.

// ================== GLOBAL VARIABLES ==================
String wifiSSID = "", wifiPASS = "", wifiHostname = "ESP32";
bool wifiDHCP = true;
#define DEBUG_AP_SSID "OMPM_DEBUG"
#define DEBUG_AP_PASS "12345678"
IPAddress ipLocal, ipGateway, ipSubnet, ipDNS;
int webPort = 80;

// ==========  MQTT ==========
WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);
unsigned long lastMqttPublish = 0;
bool mqttEnabled = false;  // Set to false by default
String mqttServer = "";
int mqttPort = 1883;
String mqttUser = "";
String mqttPassword = "";
String mqttTopic = "ompm/status";
const unsigned long MQTT_INTERVAL = 30000;  // 30 seconds
// ==================================================

// ---------- AGGREGATE CONFIGURATION ----------
bool aggregateEnabled = false;   // true = M6 enabled, false = no M6
bool physicalAggregate = false;  // true: M0 is physical aggregate, false: M6 is calculated sum
String aggregateName = "TOTAL";
String modeName = "MODE: 6 CHANNELS";

// ---------- OTA ----------
bool otaEnabled = true;
int otaPort = 3232;
unsigned long otaLastProgress = 0;
bool otaUpdating = false;

// ---------- WEB SERVER ----------
WebServer server(webPort);
bool serverStarted = false;
bool forceFullRedraw = true;
bool needsDisplayUpdate = true;
unsigned long lastDisplayRenderTime = 0;

// ---------- HARDWARE ----------
#define SD_CS 21
#define TFT_CS 26
#define TFT_DC 27
#define TFT_RST 15  // PCB: TFT RES/RESET is GPIO15
#define BUTTON_UP_PIN 32
#define BUTTON_DOWN_PIN 33
#define BUTTON_MENU_PIN 4
#define TFT_BL_CTRL 22
#define DHT_PIN 25

SPIClass spiSD(VSPI);   // SD bus:  SCK=18, MISO=19, MOSI=23, CS=21
SPIClass spiTFT(HSPI);  // TFT bus: SCK=14, MOSI=13, CS=26, DC=27, RST=15
Adafruit_ST7735 tft = Adafruit_ST7735(&spiTFT, TFT_CS, TFT_DC, TFT_RST);


void bootStep(const char* msg) {
  Serial.print("[BOOT] ");
  Serial.println(msg);

  // Only call after TFT has been initialized with tft.initR().
  tft.fillRect(0, 92, 160, 28, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 100);
  tft.print("BOOT: ");
  tft.println(msg);
}


// ---------- SAFE SPI / SD HELPERS ----------
void deselectSPIDevices() {
  pinMode(TFT_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
  delay(2);
}

bool beginSDWithRetries() {
  // Conservative SDHC/industrial-card initialization.
  // Same PCB pins as before: CS=21 SCK=18 MISO=19 MOSI=23.
  const uint32_t freqs[] = {400000, 1000000, 2000000, 4000000, 100000, 200000};

  Serial.println("[SD] SAFE_INIT_DUAL_SPI");
  Serial.println("[SD] pins CS=21 SCK=18 MISO=19 MOSI=23");
  Serial.print("[SD] SD_CS value=");
  Serial.println(SD_CS);

  SD.end();
  delay(200);

  deselectSPIDevices();
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  delay(100);

  spiSD.begin(18, 19, 23, SD_CS);
  delay(100);
  deselectSPIDevices();
  delay(50);

  for (uint8_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); i++) {
    Serial.print("[SD] BEGIN f=");
    Serial.println(freqs[i]);

    SD.end();
    delay(200);

    deselectSPIDevices();
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    delay(80);

    spiSD.begin(18, 19, 23, SD_CS);
    delay(80);
    deselectSPIDevices();
    delay(30);

    bool ok = SD.begin(SD_CS, spiSD, freqs[i]);
    Serial.print("[SD] OK=");
    Serial.println(ok ? "1" : "0");

    if (ok) {
      uint8_t type = SD.cardType();
      Serial.print("[SD] TYPE=");
      if (type == CARD_NONE) Serial.println("NONE");
      else if (type == CARD_MMC) Serial.println("MMC");
      else if (type == CARD_SD) Serial.println("SDSC");
      else if (type == CARD_SDHC) Serial.println("SDHC/SDXC");
      else Serial.println("UNKNOWN");

      Serial.print("[SD] MB=");
      Serial.println((uint32_t)(SD.cardSize() / (1024 * 1024)));
      Serial.print("[SD] config=");
      Serial.println(SD.exists("/config.json") ? "YES" : "NO");
      Serial.print("[SD] calib=");
      Serial.println(SD.exists("/calib.json") ? "YES" : "NO");
      return true;
    }
  }

  Serial.println("[SD] FAIL_ALL");
  return false;
}


// ---------- PZEM - 6 PHYSICAL MODULES (INDIVIDUAL) ----------
uint8_t addr[6] = { 0x10, 0x60, 0x50, 0x40, 0x30, 0x20 };

// One PZEM object per physical module.
PZEM004Tv30 pzem0(Serial2, 16, 17, addr[0]);  // M0
PZEM004Tv30 pzem1(Serial2, 16, 17, addr[1]);  // M1
PZEM004Tv30 pzem2(Serial2, 16, 17, addr[2]);  // M2
PZEM004Tv30 pzem3(Serial2, 16, 17, addr[3]);  // M3
PZEM004Tv30 pzem4(Serial2, 16, 17, addr[4]);  // M4
PZEM004Tv30 pzem5(Serial2, 16, 17, addr[5]);  // M5

// Pointer array used by generic loops and helper functions.
PZEM004Tv30* pzem[6] = { &pzem0, &pzem1, &pzem2, &pzem3, &pzem4, &pzem5 };

// ---------- MEASUREMENT ARRAYS: M0-M5 PHYSICAL, M6 OPTIONAL CALCULATED AGGREGATE ----------
float current[7];
float power[7];
float voltage[7];
float frequencyArr[7];
float pfArr[7];

float calV[7] = { 0.9995, 0.9995, 0.9995, 0.9995, 0.9995, 0.9995, 1.0 };
float calI[7] = { 0.934, 0.934, 0.934, 0.934, 0.934, 0.934, 1.0 };
float calP[7] = { 0.928, 0.928, 0.928, 0.928, 0.928, 0.928, 1.0 };

int errorCount[7] = { 0 };
int readCount[7] = { 0 };
unsigned long lastUpdateTime[7] = { 0 };

bool sdWritePending[7] = { false };
float lastSDValues[7][5] = { 0 };
float lastSDCurrent[7] = { 0 };
float lastSDPower[7] = { 0 };
float lastSDFrequency[7] = { 0 };
float lastSDPF[7] = { 0 };

float ambientTempC = NAN;
float ambientHum = NAN;
unsigned long lastDHTRead = 0;
const unsigned long DHT_READ_INTERVAL = 5000;

float prevVoltage[7] = { 0 };
float prevCurrent[7] = { 0 };
float prevPower[7] = { 0 };

float voltageArr[7];
float currentArr[7];
float powerArr[7];

#define LOG_FILE_COUNT 6  // 0.csv = aggregate, 1.csv-5.csv = physical meters. Do not create 6.csv.
String fileName[LOG_FILE_COUNT] = {
  "/0.csv", "/1.csv", "/2.csv", "/3.csv", "/4.csv", "/5.csv"
};

// ---------- STABLE SD CSV LOGGING ----------
// Keep only ONE SD file descriptor open at a time. The ESP32 VFS/FAT layer has
// a small open-file limit; keeping 0.csv..5.csv open can trigger:
//   vfs_fat: open: no free file descriptors
// This logger opens one CSV, writes one row, flushes/closes it, then returns
// control to the RS485/PZEM scheduler. Boot-time CSV scanning remains disabled.
#define ENABLE_AUTO_SD_RETRY 0
#define CSV_HEADER "timestamp,VLN,A,W,f,PF,T,H"
#define SD_ROW_WRITE_GAP_MS 50UL
#define LOG_ROTATE_SIZE_BYTES (2UL * 1024UL * 1024UL)
#define LOG_ROTATE_KEEP 20

// NILM build: keep the original high-resolution logging cadence.
// Stability is improved by creating a fresh CSV set on every boot, not by
// reducing measurement/logging frequency.
#define ENABLE_SESSION_CSV_FILES 1

unsigned long lastLogFlush = 0;
uint8_t sdWriteCursor = 0;

// Stability patch:
// - Mark CSV rows for logging at a controlled interval instead of after every PZEM read.
// - Check CSV rotation periodically instead of on every row.
const unsigned long PER_METER_LOG_INTERVAL = 15000UL;
const unsigned long ROTATE_CHECK_INTERVAL = 60000UL;
unsigned long lastMeterLogWrite[LOG_FILE_COUNT] = { 0 };
unsigned long lastRotateCheck[LOG_FILE_COUNT] = { 0 };

// ========== SCHEDULED RESTART ==========
const int RESTART_HOUR = 4;    // Scheduled restart hour. 4 means 04:00 local time.
const int RESTART_MINUTE = 0;  // Scheduled restart minute.
const bool DAILY_RESTART_ENABLED = false;  // Recovery: disable scheduled restart during stability diagnosis

#define IR_PIN 34
IRrecv irrecv(IR_PIN);
decode_results irResults;
uint32_t lastIRCode = 0;
unsigned long lastIRTime = 0;
const unsigned long IR_DEBOUNCE_MS = 180;
const unsigned long RESTART_CHECK_INTERVAL = 30000;  // Check the scheduled restart condition every 30 seconds.
unsigned long LAST_CHECK_RESTART = 0;
bool RESTART_PENDING = false;  // Prevents repeated restart attempts during the same time window.

// ---------- CONSTANTS ----------
#define PZEM_COUNT 6
#define SD_WRITE_INTERVAL 1000
#define HIST_SIZE 60

const float CURRENT_THRESHOLD = 0.0;  // Do not suppress low-current readings; keep raw/calibrated PZEM current
const float POWER_THRESHOLD = 0.5;
const float VOLTAGE_THRESHOLD = 1.0;
const float FREQUENCY_THRESHOLD = 0.1;
const float PF_THRESHOLD = 0.01;

// ---------- STATUS FLAGS AND TIMERS ----------
float avgVoltage = 0.0;
unsigned long lastSDWrite = 0;
bool sdOK = false;


const unsigned long PZEM_READ_INTERVAL = 250;  // Active scheduler interval. Reads one PZEM channel per turn.
// The full scan time is this interval multiplied by the number of physical PZEM modules.

const unsigned long PZEM_IDLE_READ_INTERVAL = 750;  // Idle scheduler interval used while the screen saver is active.
// Reduces CPU load and serial bus traffic while still keeping measurements updated.
// In idle mode, one channel is read every 200 ms instead of every 70 ms.
// This lowers refresh speed but improves stability and reduces power consumption.
// Measurements continue in idle mode, only at a lower temporal resolution.
// Benefit: lower load and better stability.
// Trade-off: fast load changes may be detected later or appear smoothed.

const unsigned long SHARED_VOLTAGE_INTERVAL = 3000;
// Voltage is treated as a shared reference and refreshed once per second.
// This is sufficient because mains voltage usually changes more slowly than current or power.

const unsigned long SHARED_FREQUENCY_INTERVAL = 10000;
// Frequency is treated as a shared reference and refreshed every 2 seconds.
// This is sufficient because mains frequency normally varies very little.

const unsigned long DISPLAY_REFRESH_INTERVAL = 250;  // TFT redraw interval. Decoupled from PZEM reads to reduce flicker.
// The display refreshes about four times per second using the latest available values.
// This interval does not control the PZEM read rate.
// It only controls how often the already captured values are redrawn.

int currentPZEMIndex = 0;  // Index of the next PZEM module to read.

unsigned long lastPZEMReadTime = 0;  // Timestamp of the last scheduler PZEM read.
bool pzemReadComplete = false;       // Set when the current PZEM read cycle has completed.

unsigned long measureInterval = 1000;  // Consolidated measurement capture/logging interval.
// PZEM channels are read continuously, but only one consolidated sample is saved per interval.

unsigned long lastMeasureTime = 0;  // Timestamp of the last consolidated measurement/logging cycle.
unsigned long lastDebugTime = 0;    // Reserved for throttling debug messages if Serial output is re-enabled.
unsigned long menuPressStart = 0;   // Start time used to detect a long menu/button press.
bool menuHeld = false;

unsigned long lastForcedWrite = 0;

// ---------- CALIBRATION ----------
// Simple field calibration for NILM: put the same resistive load on each channel,
// read the reference current with a clamp meter, adjust REF on the remote, and apply.
int calChannel = 0;
float calRefCurrent = 1.000f;
float lastCalRatio = 1.000f;
float* calFactors[3];
bool calSaveNextRequested = false;  // IR # key sets this; physical long OK also saves+next.

unsigned long lastCalMeasureTime = 0;
const unsigned long CAL_MEASURE_INTERVAL = 500;
float prevCalLiveValue = -1.0;

// ---------- HISTORY ----------
float histCurrent[7][HIST_SIZE];
float histPower[7][HIST_SIZE];
int histIndex = 0;

// ---------- BUTTONS ----------
Button btnUp = { BUTTON_UP_PIN, HIGH, 0 };
Button btnDown = { BUTTON_DOWN_PIN, HIGH, 0 };
Button btnMenu = { BUTTON_MENU_PIN, HIGH, 0 };

// ========== FALLBACK FOR RESTART WITHOUT NTP ==========
unsigned long systemStartTime = 0;  // Initialized in setup() and used by the restart fallback timer.
bool RESTART_FALLBACK_ACTIVATED = false;
const unsigned long MAX_TIME_WITHOUT_NTP = 24UL * 60UL * 60UL * 1000UL;  // Maximum uptime before fallback restart when NTP is unavailable.

// ---------- STATES ----------
enum SystemState {
  STATE_MEASURES,
  STATE_MENU,
  STATE_CAL_SOFT,
  STATE_SDMEM,
  STATE_REFRESH_CONFIG,
  STATE_WIFIINFO,
  STATE_PZEM_STATUS,
  STATE_SETTINGS,
  STATE_PZEM_ADDR,
  STATE_PZEM_ADDR_CONFIRM,
  STATE_SAVE_CALIB,
  STATE_RESET_CALIB,
  STATE_VER_CALIB,
  STATE_CUR_GRAPH,
  STATE_PWR_GRAPH,
  STATE_CUR_HIST,
  STATE_PWR_HIST,
  STATE_WEB_SERVER,
  STATE_IR_CAL_MENU,
  STATE_IR_GRAPH_MENU
};

enum MenuOption {
  MENU_EXIT,
  MENU_SDMEM,
  MENU_REFRESH_CONFIG,
  MENU_WIFIINFO,
  MENU_PZEM_STATUS,
  MENU_CALIBRATION_GENERAL,
  MENU_GRAPHS,
  MENU_WEB_SERVER,
  MENU_ITEMS
};

const char* menuItems[MENU_ITEMS] = {
  "1.EXIT",
  "2.SD Memory",
  "3.Refresh Rate",
  "4.WiFi Info",
  "5.PZEM Status",
  "6.General Calibration",
  "7.Graphs",
  "8.Web Server"
};

const char* calMenuItems[] = {
  "1.Program PZEM",
  "2.Cal I Simple",
  "3.Save Calib",
  "4.Reset Calib",
  "5.View Calib"
};
const int CAL_MENU_ITEMS = sizeof(calMenuItems) / sizeof(calMenuItems[0]);

const char* graphMenuItems[] = {
  "1.Current Graph",
  "2.Power Graph",
  "3.Current History",
  "4.Power History"
};
const int GRAPH_MENU_ITEMS = sizeof(graphMenuItems) / sizeof(graphMenuItems[0]);

SystemState currentState = STATE_MEASURES;
const int MENU_VISIBLE = 9;
int menuIndex = 0, menuTop = 0, addrIndex = 0;
int calSubIndex = 0, graphSubIndex = 0;
bool confirmWrite = true, redraw = true;

// ---------- NTP ----------
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600, 60000);
String prevTimeStr = "";


void readFastPZEMChannel(uint8_t idx, unsigned long now) {
  if (idx >= PZEM_COUNT) return;

  float i = pzem[idx]->current();
  float p = pzem[idx]->power();
  float pf = pzem[idx]->pf();

  bool validI = !isnan(i);
  bool validP = !isnan(p);
  bool validPF = !isnan(pf);

  if (validI && validP) {
    current[idx] = i * calI[idx];
    power[idx] = p * calP[idx];

    if (current[idx] < CURRENT_THRESHOLD) current[idx] = 0.0;
    if (power[idx] < POWER_THRESHOLD) power[idx] = 0.0;

    if (validPF) {
      pfArr[idx] = pf;
      if (pfArr[idx] < 0.0) pfArr[idx] = 0.0;
      if (pfArr[idx] > 1.0) pfArr[idx] = 1.0;
    } else if (current[idx] <= CURRENT_THRESHOLD || power[idx] <= POWER_THRESHOLD) {
      pfArr[idx] = 0.0;
    }

    voltage[idx] = avgVoltage > 100.0 ? avgVoltage : 230.0;
    frequencyArr[idx] = frequencyArr[0] > 45.0 ? frequencyArr[0] : 50.0;
    lastUpdateTime[idx] = now;
    readCount[idx]++;
    errorCount[idx] = 0;

    // Keep the original measurement path untouched: a valid PZEM read must always
    // update current/power and mark the meter as pending for SD logging.
    // The actual SD write rate is throttled later in writeSDDeferred(), not here.
    sdWritePending[idx] = true;
  } else {
    errorCount[idx]++;
    if (errorCount[idx] <= 3 || (errorCount[idx] % 20) == 0) {
      // SERIAL_DISABLED: Serial.printf("❌ M%d without stable response (I=%s P=%s PF=%s) err=%d\n",
      // SERIAL_DISABLED: idx,
      // SERIAL_DISABLED: validI ? "OK" : "NaN",
      // SERIAL_DISABLED: validP ? "OK" : "NaN",
      // SERIAL_DISABLED: validPF ? "OK" : "NaN",
      // SERIAL_DISABLED: errorCount[idx]);
    }
  }
}

void updateSharedVoltageFrequency(unsigned long now) {
  static unsigned long lastVoltageRead = 0;
  static unsigned long lastFrequencyRead = 0;

  bool updatedVoltage = false;
  bool updatedFrequency = false;

  if (now - lastVoltageRead >= SHARED_VOLTAGE_INTERVAL) {
    float vRef = pzem0.voltage();
    if (!isnan(vRef) && vRef > 50.0 && vRef < 300.0) {
      avgVoltage = vRef * calV[0];
      for (int i = 0; i < PZEM_COUNT; i++) {
        voltage[i] = avgVoltage;
      }
      updatedVoltage = true;
    }
    lastVoltageRead = now;
  }

  if (now - lastFrequencyRead >= SHARED_FREQUENCY_INTERVAL) {
    float fRef = pzem0.frequency();
    float validFreq = (!isnan(fRef) && fRef > 45.0 && fRef < 65.0) ? fRef : 50.0;
    for (int i = 0; i < PZEM_COUNT; i++) {
      frequencyArr[i] = validFreq;
    }
    updatedFrequency = true;
    lastFrequencyRead = now;
  }

  if (updatedVoltage || updatedFrequency) {
    lastUpdateTime[0] = now;
  }
}

// ========== CALCULATE AGGREGATE BASED ON MODE ==========
void calculateAggregate() {
  if (!aggregateEnabled) {
    // MODE 3: No aggregate at all - M6 doesn't exist
    voltage[6] = 0.0;
    current[6] = 0.0;
    power[6] = 0.0;
    frequencyArr[6] = 0.0;
    pfArr[6] = 0.0;
    return;
  }

  if (physicalAggregate) {
    // MODE 1: M0 is physical aggregate, M6 doesn't exist
    voltage[6] = 0.0;
    current[6] = 0.0;
    power[6] = 0.0;
    frequencyArr[6] = 0.0;
    pfArr[6] = 0.0;
    return;
  }

  // MODE 2: M6 = SUM M0+M1+M2+M3+M4+M5
  float sumI = 0, sumP = 0, sumV = 0;
  int validV = 0;

  for (int i = 0; i < 6; i++) {
    sumI += current[i];
    sumP += power[i];
    if (voltage[i] > 100.0 && voltage[i] < 300.0) {
      sumV += voltage[i];
      validV++;
    }
  }

  current[6] = sumI;
  power[6] = sumP;
  voltage[6] = validV > 0 ? sumV / validV : 230.0;
  frequencyArr[6] = 50.0;
  pfArr[6] = 1.0;

  if (fabs(power[6] - lastSDPower[6]) > POWER_THRESHOLD * 6) {
    sdWritePending[6] = true;
  }

  histCurrent[6][histIndex] = current[6];
  histPower[6][histIndex] = power[6];
  lastUpdateTime[6] = millis();

  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 5000) {
    lastDebug = millis();
    // SERIAL_DISABLED: Serial.printf("📊 M6 SUM: %.1f W (M0-M5)\n", power[6]);
  }
}

// ========== OPTIMIZED SD WRITING  ==========
// Single-file-descriptor logger: never keeps CSV files open permanently.

void createSessionCsvFileNames() {
#if ENABLE_SESSION_CSV_FILES
  unsigned long epoch = timeClient.getEpochTime();
  unsigned long sid;
  if (epoch > 1672531200UL) {
    sid = epoch % 1000000UL;
  } else {
    sid = (millis() / 1000UL) % 1000000UL;
  }

  for (uint8_t i = 0; i < LOG_FILE_COUNT; i++) {
    char name[16];
    // 8.3-compatible names: /M0xxxxxx.CSV ... /M5xxxxxx.CSV
    snprintf(name, sizeof(name), "/M%u%06lu.CSV", i, sid);
    fileName[i] = String(name);
    Serial.print("[CSV] session_file M");
    Serial.print(i);
    Serial.print("=");
    Serial.println(fileName[i]);
  }
#endif
}

void closeLogFiles() {
  // No persistent files are kept open in this version.
}

String archiveNameForMeter(uint8_t meter, uint8_t index) {
  char name[16];
  snprintf(name, sizeof(name), "/%u_%03u.CSV", meter, index);
  return String(name);
}

void rotateLogFile(uint8_t meter) {
  if (meter >= LOG_FILE_COUNT) return;

  String oldest = archiveNameForMeter(meter, LOG_ROTATE_KEEP);
  if (SD.exists(oldest)) SD.remove(oldest.c_str());

  for (int n = LOG_ROTATE_KEEP - 1; n >= 1; n--) {
    String from = archiveNameForMeter(meter, n);
    if (SD.exists(from)) {
      String to = archiveNameForMeter(meter, n + 1);
      if (SD.exists(to)) SD.remove(to.c_str());
      SD.rename(from.c_str(), to.c_str());
    }
  }

  if (SD.exists(fileName[meter])) {
    String first = archiveNameForMeter(meter, 1);
    if (SD.exists(first)) SD.remove(first.c_str());
    SD.rename(fileName[meter].c_str(), first.c_str());
  }
}

bool ensureLogFilePrepared(uint8_t meter) {
  if (!sdOK || meter >= LOG_FILE_COUNT) return false;

  unsigned long now = millis();

  // Rotation is intentionally checked only once per ROTATE_CHECK_INTERVAL.
  // This avoids opening each CSV in read mode on every single appended row.
  if (now - lastRotateCheck[meter] >= ROTATE_CHECK_INTERVAL) {
    lastRotateCheck[meter] = now;

    if (SD.exists(fileName[meter])) {
      File existing = SD.open(fileName[meter], FILE_READ);
      if (existing) {
        size_t sz = existing.size();
        existing.close();
        if (sz >= LOG_ROTATE_SIZE_BYTES) {
          Serial.print("[CSV] rotate M");
          Serial.println(meter);
          rotateLogFile(meter);
        }
      }
    }
  }

  if (!SD.exists(fileName[meter])) {
    File f = SD.open(fileName[meter], FILE_WRITE);
    if (!f) {
      Serial.print("[CSV] create_fail M");
      Serial.println(meter);
      return false;
    }
    f.println(CSV_HEADER);
    f.flush();
    f.close();
  }

  return true;
}

bool openLogFile(uint8_t meter) {
  // Compatibility wrapper used by older call sites. It does not keep the file open.
  return ensureLogFilePrepared(meter);
}

bool openLogFiles() {
  if (!sdOK) return false;
  bool ok = true;
  for (uint8_t i = 0; i < LOG_FILE_COUNT; i++) {
    if (!ensureLogFilePrepared(i)) ok = false;
    delay(1);
  }
  Serial.print("[CSV] prepared_logs=");
  Serial.println(ok ? "1" : "0");
  lastLogFlush = millis();
  return ok;
}

void flushLogFiles(bool force) {
  // No persistent files are kept open; every write is flushed/closed immediately.
  (void)force;
}

bool writeCSVRowToLog(uint8_t meter, long long timestamp) {
  if (!sdOK || meter >= LOG_FILE_COUNT) return false;
  if (!ensureLogFilePrepared(meter)) return false;

  // Always write the same 8 CSV fields declared in CSV_HEADER:
  // timestamp,VLN,A,W,f,PF,T,H
  // T and H are the ambient temperature and relative humidity measured by DHT11.
  // If the DHT11 has not returned a valid value yet, empty fields are written
  // instead of changing the number of CSV columns.
  char line[192];

  char tempField[16];
  char humField[16];

  if (!isnan(ambientTempC)) {
    snprintf(tempField, sizeof(tempField), "%.1f", ambientTempC);
  } else {
    tempField[0] = '\0';
  }

  if (!isnan(ambientHum)) {
    snprintf(humField, sizeof(humField), "%.1f", ambientHum);
  } else {
    humField[0] = '\0';
  }

  snprintf(line, sizeof(line), "%lld,%.2f,%.6f,%.3f,%.2f,%.3f,%s,%s",
           timestamp, voltage[meter], current[meter], power[meter],
           frequencyArr[meter], pfArr[meter], tempField, humField);

  File f = SD.open(fileName[meter], FILE_APPEND);
  if (!f) {
    Serial.print("[CSV] open_fail M");
    Serial.println(meter);
    return false;
  }

  size_t written = f.println(line);
  // close() commits the row; avoid explicit flush() on every NILM sample.
  f.close();

  if (written == 0) {
    Serial.print("[CSV] write_fail M");
    Serial.println(meter);
    return false;
  }

  lastSDValues[meter][0] = voltage[meter];
  lastSDCurrent[meter] = current[meter];
  lastSDPower[meter] = power[meter];
  lastSDFrequency[meter] = frequencyArr[meter];
  lastSDPF[meter] = pfArr[meter];
  sdWritePending[meter] = false;
  return true;
}

void writeSDDeferred() {
  if (!sdOK) return;

  unsigned long now = millis();
  static unsigned long lastForcedWriteLocal = 0;
  const unsigned long FORCE_WRITE_INTERVAL = 15000;

  // Periodic forced sample: mark all log files pending, but write only
  // one CSV row per loop call to keep RS485/PZEM responsive.
  if (now - lastForcedWriteLocal >= FORCE_WRITE_INTERVAL) {
    for (uint8_t i = 0; i < LOG_FILE_COUNT; i++) sdWritePending[i] = true;
    lastForcedWriteLocal = now;
  }

  if (now - lastSDWrite < SD_ROW_WRITE_GAP_MS) return;

  for (uint8_t n = 0; n < LOG_FILE_COUNT; n++) {
    uint8_t i = (sdWriteCursor + n) % LOG_FILE_COUNT;

    if (sdWritePending[i]) {
      // Measurement-safe throttling: PZEM readings are accepted immediately,
      // but CSV rows are appended at most once per meter every PER_METER_LOG_INTERVAL.
      // This preserves TFT/web live values while reducing SD stress.
      if (lastMeterLogWrite[i] != 0 && (now - lastMeterLogWrite[i]) < PER_METER_LOG_INTERVAL) {
        continue;
      }

      long long timestamp = (long long)timeClient.getEpochTime() * 1000LL + (millis() % 1000);
      if (writeCSVRowToLog(i, timestamp)) {
        lastMeterLogWrite[i] = now;
      }
      sdWriteCursor = (i + 1) % LOG_FILE_COUNT;
      lastSDWrite = now;
      break;
    }
  }
}



// ========== BUTTONS ==========
bool buttonFallingEdge(Button& b, unsigned long debounceMs) {
  bool state = digitalRead(b.pin);
  unsigned long now = millis();
  if (state != b.lastState && (now - b.lastChange) > debounceMs) {
    b.lastChange = now;
    b.lastState = state;
    if (state == LOW) return true;
  }
  return false;
}


// ================== LOAD CONFIGURATION ==================
bool loadConfig() {
  File f = SD.open("/config.json");
  if (!f) {
    Serial.println("[CFG] config.json NOT_FOUND");
    // Default: MODE 1 (M0 = physical aggregate)
    aggregateEnabled = true;
    physicalAggregate = true;
    aggregateName = "TOTAL REAL";
    modeName = "MODE: PHYSICAL AGG (M0 = SUM M1-M5)";
    return false;
  }

  DynamicJsonDocument doc(2048);
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) {
    Serial.print("[CFG] JSON_ERR=");
    Serial.println(err.c_str());

    // Default: MODE 1
    aggregateEnabled = true;
    physicalAggregate = true;
    aggregateName = "TOTAL REAL";
    modeName = "MODE: PHYSICAL AGG (M0 = SUM M1-M5)";
    return false;
  }

  // ---------- DEBUG: show JSON loaded ----------
  // SERIAL_DISABLED: Serial.println("\n=== CONFIG JSON LOADED ===");
  // SERIAL_DISABLED: serializeJsonPretty(doc, Serial);
  // SERIAL_DISABLED: Serial.println("\n===========================\n");

  // ---------- WiFi ----------
  JsonObject wifi = doc["wifi"];
  wifiSSID = wifi["ssid"] | "";
  wifiPASS = wifi["password"] | "";
  wifiHostname = wifi["hostname"] | "ESP32";
  wifiDHCP = wifi["dhcp"] | true;

  if (wifi["port"].is<const char*>()) {
    webPort = atoi(wifi["port"]);
  } else {
    webPort = wifi["port"] | 80;
  }

  if (!wifiDHCP) {
    ipLocal.fromString(wifi["ip"] | "0.0.0.0");
    ipGateway.fromString(wifi["gateway"] | "0.0.0.0");
    ipSubnet.fromString(wifi["subnet"] | "255.255.255.0");
    ipDNS.fromString(wifi["dns"] | "8.8.8.8");
  }

  // ---------- MQTT configuration loaded from config.json ----------
  // ---------- MQTT Configuration ----------
  JsonObject mqtt = doc["mqtt"];
  if (!mqtt.isNull()) {
    mqttEnabled = mqtt["enabled"] | false;
    mqttServer = mqtt["server"] | "";
    mqttPort = mqtt["port"] | 1883;
    mqttUser = mqtt["user"] | "";
    mqttPassword = mqtt["password"] | "";
    mqttTopic = mqtt["topic"] | "ompm/status";

    // SERIAL_DISABLED: Serial.println("✅ MQTT CONFIGURATION LOADED:");
    // SERIAL_DISABLED: Serial.printf("  enabled: %s\n", mqttEnabled ? "true" : "false");
    // SERIAL_DISABLED: Serial.printf("  server: %s:%d\n", mqttServer.c_str(), mqttPort);
    // SERIAL_DISABLED: Serial.printf("  topic: %s\n", mqttTopic.c_str());
  } else {
    // SERIAL_DISABLED: Serial.println("⚠️ No MQTT configuration found");
    mqttEnabled = false;
  }
  // ============================================================

  // ---------- Calibration ----------
  for (int i = 0; i < 6; i++) {
    calV[i] = doc["calibration"]["voltage"][i] | 1.0;
    calI[i] = doc["calibration"]["current"][i] | 1.0;
    calP[i] = doc["calibration"]["power"][i] | 1.0;
  }


  // Declare the aggregate JSON object only once in this function.
  JsonObject aggregate = doc["aggregate"];

  if (aggregate.isNull()) {
    // SERIAL_DISABLED: Serial.println("⚠️ No 'aggregate' section found - Using MODE 1 defaults");
    aggregateEnabled = true;
    physicalAggregate = true;
    aggregateName = "TOTAL REAL";
  } else {
    // Read existing configuration
    aggregateEnabled = aggregate["enabled"] | true;    // Default: true
    physicalAggregate = aggregate["physical"] | true;  // Default: true (MODE 1)
    aggregateName = aggregate["name"] | (physicalAggregate ? "TOTAL REAL" : "TOTAL SUM");

    // SERIAL_DISABLED: Serial.println(" AGGREGATE CONFIGURATION LOADED:");
    // SERIAL_DISABLED: Serial.printf("  enabled: %s\n", aggregateEnabled ? "true" : "false");
    // SERIAL_DISABLED: Serial.printf("  physical: %s\n", physicalAggregate ? "true" : "false");
    // SERIAL_DISABLED: Serial.printf("  name: %s\n", aggregateName.c_str());
  }

  // Safety fallback: if the aggregate mode is incomplete, default to Mode 1.
  if (aggregateEnabled && !aggregate.containsKey("physical")) {
    physicalAggregate = true;
  }

  // Generate the mode name shown on the display and web interface.
  if (!aggregateEnabled) {
    modeName = "MODE: 6 CHANNELS (NO AGGREGATE)";
  } else if (physicalAggregate) {
    modeName = "MODE: PHYSICAL AGG (M0 = SUM M1-M5)";
  } else {
    modeName = "MODE: 6 INDIV + M6 CALCULATED";
  }

  // SERIAL_DISABLED: Serial.println("\n=== FINAL AGGREGATE CONFIGURATION ===");
  // SERIAL_DISABLED: Serial.printf("Aggregate enabled: %s\n", aggregateEnabled ? "YES" : "NO");
  if (aggregateEnabled) {
    // SERIAL_DISABLED: Serial.printf("Physical aggregate: %s\n", physicalAggregate ? "YES (M0 = SUM M1-M5)" : "NO (M6 = SUM M0-M5)");
    // SERIAL_DISABLED: Serial.printf("Name: %s\n", aggregateName.c_str());
  } else {
    // SERIAL_DISABLED: Serial.println("No aggregate - 6 independent channels");
  }
  // SERIAL_DISABLED: Serial.println("=======================================\n");

  // SERIAL_DISABLED: Serial.println(" Config loaded OK");
  return true;
}

// ========== WIFI ==========
bool initWiFi() {
  Serial.println("[WIFI] START");
  Serial.print("[WIFI] SSID=");
  Serial.println(wifiSSID);
  Serial.print("[WIFI] DHCP=");
  Serial.println(wifiDHCP ? "1" : "0");
  Serial.flush();

  WiFi.disconnect(true);
  delay(250);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(wifiHostname.c_str());

  if (!wifiDHCP && wifiSSID.length() > 0) {
    if (!WiFi.config(ipLocal, ipGateway, ipSubnet, ipDNS)) {
      Serial.println("[WIFI] STATIC_IP_FAIL");
    }
  }

  Serial.println("[WIFI] BEGIN");
  Serial.flush();
  WiFi.begin(wifiSSID.c_str(), wifiPASS.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print('.');
    if ((attempts % 5) == 4) {
      Serial.print(" st=");
      Serial.print(WiFi.status());
    }
    Serial.flush();
    attempts++;
  }

  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WIFI] OK");
    Serial.print("[WIFI] IP=");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.print("[WIFI] FAIL st=");
  Serial.println(WiFi.status());
  return false;
}

String getFormattedTime() {
  static String lastTimeStr = "--:--";
  if (WiFi.status() != WL_CONNECTED) return lastTimeStr;

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return lastTimeStr;

  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 1000) {
    char buf[6];
    strftime(buf, sizeof(buf), "%H:%M", &timeinfo);
    lastTimeStr = String(buf);
    lastDisplayUpdate = millis();
  }

  return lastTimeStr;
}

// ========== SCREEN SAVER MANAGEMENT ==========
void checkScreenSaver() {
  unsigned long now = millis();

  // Optional debug checkpoint every 60 seconds.
  static unsigned long lastDebugScreen = 0;
  if (now - lastDebugScreen > 60000) {
    if (!screenSaverActive) {
      unsigned long remainingTime = (lastUserInteraction + SCREENSAVER_TIMEOUT - now) / 1000;
      // SERIAL_DISABLED: Serial.printf(" Time to screen saver: %lu seconds\n", remainingTime);
    }
    lastDebugScreen = now;
  }

  // If the screen is forced on (due to recent interaction)
  if (screenForcedOn) {
    if (now - screenOnTime >= SCREENSAVER_COOLDOWN) {
      screenForcedOn = false;
      // SERIAL_DISABLED: Serial.println(" Forced period finished");
    }
    return;
  }

  // Check whether the timeout passed without interaction
  if (!screenSaverActive && (now - lastUserInteraction >= SCREENSAVER_TIMEOUT)) {
    // Enable the screen saver.
    screenSaverActive = true;

    // First set the framebuffer to black, then cut the backlight
    // to prevent flickering just before going to sleep
    tft.fillScreen(ST77XX_BLACK);
    digitalWrite(TFT_BL_CTRL, HIGH);

    // Make sure it isn't redrawn and that everything is reset when you wake up
    needsDisplayUpdate = false;
    forceFullRedraw = true;
    lastDisplayRenderTime = now;

    // SERIAL_DISABLED: Serial.println(" SCREEN SAVER ENABLED - Backlight OFF");
  }
}

// ========== RESTART LOG ON SD ==========
void logRestartToSD() {
  if (!sdOK) return;

  File logFile = SD.open("/reboot.log", FILE_APPEND);
  if (!logFile) {
    // If it does not exist, create it with a header
    logFile = SD.open("/reboot.log", FILE_WRITE);
    if (logFile) {
      logFile.println("timestamp,date_time,reason,free_heap,uptime_sec");
      logFile.close();
      logFile = SD.open("/reboot.log", FILE_APPEND);
    }
  }

  if (logFile) {
    // Get timestamp and date/time
    unsigned long epoch = timeClient.getEpochTime();
    String dateTimeStr = "unknown";

    if (epoch > 1672531200) {  // If we have a valid time
      struct tm* tmInfo = localtime((time_t*)&epoch);
      char buffer[30];
      sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
              tmInfo->tm_year + 1900, tmInfo->tm_mon + 1, tmInfo->tm_mday,
              tmInfo->tm_hour, tmInfo->tm_min, tmInfo->tm_sec);
      dateTimeStr= String(buffer);
    }

    // Uptime in seconds
    unsigned long uptime = millis() / 1000;

    // Write line
    logFile.print(epoch);
    logFile.print(",");
    logFile.print(dateTimeStr);
    logFile.print(",SCHEDULED_RESTART");
    logFile.print(",");
    logFile.print(ESP.getFreeHeap());
    logFile.print(",");
    logFile.println(uptime);

    logFile.close();
    // SERIAL_DISABLED: Serial.println("✅ Restart logged in /reboot.log");
  }
}
// ========== USER INTERACTION HANDLING ==========
void registerUserInteraction(const char* source) {
  unsigned long now = millis();

  // 3-second filter
  if (!screenSaverActive && (now - lastUserInteraction < 250)) {
    return;
  }

  unsigned long inactiveTime = now - lastUserInteraction;
  lastUserInteraction = now;

  // SERIAL_DISABLED: Serial.printf("🖱️ Interaction [%s] (inactive: %lu ms)\n", source, inactiveTime);

  if (screenSaverActive) {
    // Disable the screen saver after user interaction.
    screenSaverActive = false;
    screenForcedOn = true;
    screenOnTime = now;
    digitalWrite(TFT_BL_CTRL, LOW);

    // LED steady on
    digitalWrite(LED_BUILTIN, HIGH);

    // SERIAL_DISABLED: Serial.println(" SCREEN SAVER DISABLED - Reactivating screen");

    // Force a full redraw after waking the display.
    forceFullRedraw = true;
    needsDisplayUpdate = true;
    lastDisplayRenderTime = 0;

    // Redraw the current screen immediately after wake-up.
    if (currentState == STATE_MEASURES) {
      tft.fillScreen(ST77XX_BLACK);  // Clear the screen before rebuilding the layout.
      initLayout();                  // Rebuild the static layout.
    } else {
      // Redraw according to current state
      switch (currentState) {
        case STATE_MENU:
          tft.fillScreen(ST77XX_BLACK);
          showMenu();
          break;
        case STATE_IR_CAL_MENU:
          tft.fillScreen(ST77XX_BLACK);
          showIRCalibrationMenu();
          break;
        case STATE_IR_GRAPH_MENU:
          tft.fillScreen(ST77XX_BLACK);
          showIRGraphsMenu();
          break;
        default:
          currentState = STATE_MEASURES;
          tft.fillScreen(ST77XX_BLACK);
          initLayout();
          break;
      }
    }
  } else {
    screenForcedOn = true;
    screenOnTime = now;
  }
}

// ================== DIAGNOSE CSV FILES ==================
void diagnoseCSV() {
  if (!sdOK) {
    Serial.println("[CSV] diag skipped: no SD");
    return;
  }

  // Do NOT scan CSV contents at boot. Large CSV files on FAT16 can stall the
  // ESP32 for a long time and starve the RS485/PZEM scheduler. Only report
  // file presence and size.
  Serial.println("[CSV] boot scan skipped");
  uint8_t maxMeter = LOG_FILE_COUNT;
  for (uint8_t i = 0; i < maxMeter; i++) {
    if (!SD.exists(fileName[i])) {
      ensureLogFilePrepared(i);
      continue;
    }
    uint32_t sz = 0;
    File f = SD.open(fileName[i], FILE_READ);
    if (f) {
      sz = (uint32_t)f.size();
      f.close();
    }
    Serial.print("[CSV] ");
    Serial.print(fileName[i]);
    Serial.print(" size=");
    Serial.println(sz);
    delay(1);
  }
}


// ================== TFT SCREEN - ADAPTATIVE ==================
void initLayout() {
  tft.setRotation(1);
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);


  // General framework
  tft.drawRect(0, 0, tft.width(), tft.height(), ST77XX_WHITE);

  // Header
  tft.fillRect(1, 1, tft.width() - 2, 14, ST77XX_BLACK);
  tft.drawFastHLine(1, 15, tft.width() - 2, ST77XX_BLUE);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(4, 4);
  tft.print("OMPM v68");

  tft.setCursor(104, 4);
  tft.print("--:--");

  // Column headers
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setCursor(4, 20);
  tft.print("CH");
  tft.setCursor(28, 20);
  tft.print("I(A)");
  tft.setCursor(74, 20);
  tft.print("P(W)");
  tft.setCursor(120, 20);
  tft.print("V(V)");

  // Visible rows: M0..M5
  for (int i = 0; i < 6; i++) {
    int y = 34 + i * 12;
    tft.drawFastHLine(2, y + 9, tft.width() - 4, ST77XX_BLACK);
    tft.setCursor(4, y);
    if (i == 0 && aggregateEnabled && physicalAggregate) {
      tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      tft.print("A0");
    } else {
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.print("M");
      tft.print(i);
    }
  }

  // Footer
  tft.drawFastHLine(1, 110, tft.width() - 2, ST77XX_BLUE);
  tft.setCursor(4, 116);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  if (aggregateEnabled && physicalAggregate) {
    tft.print("M0=SUM M1..M5");
  } else if (aggregateEnabled) {
    tft.print("M6 virtual sum");
  } else {
    tft.print("Independent ch.");
  }
}

// ================== UPDATE DISPLAY ==================
void updateDisplay() {
  if (screenSaverActive) {
    return;
  }

  if (forceFullRedraw) {
    initLayout();
    forceFullRedraw = false;
  }

  tft.setRotation(1);
  tft.setTextWrap(false);
  tft.setTextSize(1);

  // Hour
  String timeStr = getFormattedTime();
  tft.fillRect(104, 4, 50, 8, ST77XX_BLACK);
  tft.setCursor(104, 4);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.print(timeStr);

  // M0..M5
  for (int i = 0; i < 6; i++) {
    int y = 34 + i * 12;

    //  Channel tag
    tft.fillRect(4, y, 18, 8, ST77XX_BLACK);
    tft.setCursor(4, y);
    if (i == 0 && aggregateEnabled && physicalAggregate) {
      tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      tft.print("A0");
    } else {
      tft.setTextColor(voltage[i] > 100 ? ST77XX_WHITE : ST77XX_RED, ST77XX_BLACK);
      tft.print("M");
      tft.print(i);
    }

    // Current
    tft.fillRect(28, y, 38, 8, ST77XX_BLACK);
    tft.setCursor(28, y);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.print(current[i], 2);

    // Power
    tft.fillRect(74, y, 38, 8, ST77XX_BLACK);
    tft.setCursor(74, y);
    tft.print(power[i], 1);

    // Voltage
    tft.fillRect(120, y, 36, 8, ST77XX_BLACK);
    tft.setCursor(120, y);
    tft.print(voltage[i], 0);
  }

  // Quick status below
  tft.fillRect(4, 116, 150, 8, ST77XX_BLACK);
  tft.setCursor(4, 116);
  tft.setTextColor(sdOK ? ST77XX_GREEN : ST77XX_RED, ST77XX_BLACK);
  tft.print(sdOK ? "SD OK" : "NO SD");

  tft.setCursor(50, 116);
  tft.setTextColor((WiFi.status() == WL_CONNECTED) ? ST77XX_GREEN : ST77XX_RED, ST77XX_BLACK);
  tft.print((WiFi.status() == WL_CONNECTED) ? "WiFi" : "NoWi");

  tft.setCursor(96, 116);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  if (!isnan(ambientTempC)) {
    tft.print(ambientTempC, 0);
    tft.print("C");
  } else {
    tft.print("--C");
  }

  tft.setCursor(128, 116);
  if (!isnan(ambientHum)) {
    tft.print(ambientHum, 0);
    tft.print("%");
  } else {
    tft.print("--%");
  }
}


// ================== NEW LOG DRIVERS  ==================
void handleRebootLog() {
  if (!sdOK) {
    server.send(200, "text/html", "<html><body><h3>❌ SD not available</h3></body></html>");
    return;
  }

  if (!SD.exists("/reboot.log")) {
    server.send(200, "text/html", "<html><body><h3>📁 File not found reboot.log</h3></body></html>");
    return;
  }

  File file = SD.open("/reboot.log", FILE_READ);
  if (!file) {
    server.send(500, "text/plain", "Error opening file");
    return;
  }

  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Reboot Log - OMPM</title>";
  html += "<style>";
  html += "body { font-family: monospace; background: #2c3e50; color: #ecf0f1; padding: 20px; }";
  html += "h1 { color: #3498db; }";
  html += ".log { background: #34495e; padding: 15px; border-radius: 5px; }";
  html += ".btn { display: inline-block; padding: 10px 20px; background: #3498db; color: white; text-decoration: none; border-radius: 5px; margin: 10px 5px; }";
  html += ".btn-green { background: #2ecc71; }";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>🔄 Reboot Log</h1>";
  html += "<div class='log'><pre>";

  while (file.available()) {
    html += (char)file.read();
  }
  file.close();

  html += "</pre></div>";
  html += "<a href='/logs' class='btn'>📋 Back to Logs</a> ";
  html += "<a href='/download/reboot.log' class='btn btn-green'>📥 Download</a> ";
  html += "<a href='/clear/reboot.log' class='btn' style='background:#e74c3c;' onclick='return confirm(\"Delete file?\")'>🗑️ Delete</a>";
  html += "<br><a href='/'>🏠 Home</a>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleEventsLog() {
  if (!sdOK) {
    server.send(200, "text/html", "<html><body><h3>❌ SD not available</h3></body></html>");
    return;
  }

  if (!SD.exists("/events.log")) {
    server.send(200, "text/html", "<html><body><h3>📁 File not found events.log</h3></body></html>");
    return;
  }

  File file = SD.open("/events.log", FILE_READ);
  if (!file) {
    server.send(500, "text/plain", "Error opening file");
    return;
  }

  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Events Log - OMPM</title>";
  html += "<style>";
  html += "body { font-family: monospace; background: #2c3e50; color: #ecf0f1; padding: 20px; }";
  html += "h1 { color: #f39c12; }";
  html += ".log { background: #34495e; padding: 15px; border-radius: 5px; }";
  html += ".btn { display: inline-block; padding: 10px 20px; background: #3498db; color: white; text-decoration: none; border-radius: 5px; margin: 10px 5px; }";
  html += ".btn-green { background: #2ecc71; }";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>📋 Events Log</h1>";
  html += "<div class='log'><pre>";

  while (file.available()) {
    html += (char)file.read();
  }
  file.close();

  html += "</pre></div>";
  html += "<a href='/logs' class='btn'>📋 Back to Logs</a> ";
  html += "<a href='/download/events.log' class='btn btn-green'>📥 Download</a> ";
  html += "<a href='/clear/events.log' class='btn' style='background:#e74c3c;' onclick='return confirm(\"Delete file?\")'>🗑️ Delete</a>";
  html += "<br><a href='/'>🏠 Home</a>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleLogsIndex() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Logs - OMPM</title>";
  html += "<style>";
  html += "body { font-family: 'Segoe UI', Arial; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; padding: 20px; }";
  html += ".container { max-width: 800px; margin: 0 auto; background: white; padding: 30px; border-radius: 15px; box-shadow: 0 10px 30px rgba(0,0,0,0.2); }";
  html += "h1 { color: #2c3e50; border-bottom: 3px solid #3498db; padding-bottom: 10px; }";
  html += ".log-card { background: #f8f9fa; padding: 20px; margin: 15px 0; border-radius: 10px; border-left: 6px solid #3498db; }";
  html += ".log-card h3 { margin-top: 0; color: #2c3e50; }";
  html += ".btn { display: inline-block; padding: 10px 20px; margin: 5px; background: #3498db; color: white; text-decoration: none; border-radius: 5px; }";
  html += ".btn-small { padding: 5px 10px; font-size: 0.9em; }";
  html += ".btn-green { background: #2ecc71; }";
  html += ".btn-red { background: #e74c3c; }";
  html += ".status { color: #7f8c8d; font-size: 0.9em; }";
  html += "</style>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>📋 System Logs</h1>";

  // Reboot log
  html += "<div class='log-card'>";
  html += "<h3>🔄 Reboot Log</h3>";
  html += "<p class='status'>Register of restarts of the system</p>";
  if (SD.exists("/reboot.log")) {
    File f = SD.open("/reboot.log", FILE_READ);
    if (f) {
      int lines = 0;
      while (f.available()) {
        if (f.read() == '\n') lines++;
      }
      f.close();
      html += "<p><strong>Size:</strong> " + String(lines) + " lines</p>";
    }
    html += "<a href='/reboot.log' class='btn btn-small'>👁️ View</a> ";
    html += "<a href='/download/reboot.log' class='btn btn-small btn-green'>📥 Download</a>";
  } else {
    html += "<p><em>No data yet</em></p>";
  }
  html += "</div>";

  // Events log
  html += "<div class='log-card'>";
  html += "<h3>📊 Events Log</h3>";
  html += "<p class='status'>System events (startups, WiFi, etc.)</p>";
  if (SD.exists("/events.log")) {
    File f = SD.open("/events.log", FILE_READ);
    if (f) {
      int lines = 0;
      while (f.available()) {
        if (f.read() == '\n') lines++;
      }
      f.close();
      html += "<p><strong>Size:</strong> " + String(lines) + " lines</p>";
    }
    html += "<a href='/events.log' class='btn btn-small'>👁️ View</a> ";
    html += "<a href='/download/events.log' class='btn btn-small btn-green'>📥 Download</a>";
  } else {
    html += "<p><em>No data yet</em></p>";
  }
  html += "</div>";

  // SD diagnostic
  html += "<div class='log-card'>";
  html += "<h3>💾 SD Card Info</h3>";
  uint64_t totalBytes = SD.totalBytes();
  uint64_t usedBytes = SD.usedBytes();
  float totalMB = totalBytes / (1024.0 * 1024.0);
  float usedMB = usedBytes / (1024.0 * 1024.0);
  int usedPercent = totalBytes > 0 ? (usedBytes * 100) / totalBytes : 0;

  html += "<p><strong>Total:</strong> " + String(totalMB, 1) + " MB</p>";
  html += "<p><strong>Used:</strong> " + String(usedMB, 1) + " MB (" + String(usedPercent) + "%)</p>";
  html += "<p><strong>Free:</strong> " + String((totalBytes - usedBytes) / (1024.0 * 1024.0), 1) + " MB</p>";
  html += "</div>";

  html += "<br><a href='/' class='btn'>🏠 Back to Home</a>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

void handleClearLog() {
  String path = server.uri();
  path.replace("/clear", "");

  if (!sdOK) {
    server.send(200, "text/html", "<html><body><h3>❌ SD not available</h3><a href='/logs'>Back</a></body></html>");
    return;
  }

  if (SD.exists(path)) {
    SD.remove(path);
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta http-equiv='refresh' content='2;url=/logs'>";
    html += "</head><body>";
    html += "<h3>✅ File " + path + " erased</h3>";
    html += "<p>Redirecting...</p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  } else {
    server.send(404, "text/plain", "File not found");
  }
}

// ================== WEB SERVER - ROOT ==================
void handleRoot() {
  String html = FPSTR(INDEX_HTML);

  // Calculate statistics based on mode
  float totalPower = 0;
  int activeMeters = 0;
  float avgV = 0;
  int vCount = 0;

  if (physicalAggregate) {
    // MODE 1: M0 is physical aggregate (real sum of M1-M5) - NO M6
    totalPower = power[0];  // M0 already is the total sum
    // Count only M1-M5 as individual meters
    for (int i = 1; i < 6; i++) {
      if (voltage[i] > 100 && voltage[i] < 300) {
        activeMeters++;
        avgV += voltage[i];
        vCount++;
      }
    }
  } else {
    // MODE 2: All M0-M5 are individual, M6 is calculated sum
    for (int i = 0; i < 6; i++) {
      totalPower += power[i];
      if (voltage[i] > 100 && voltage[i] < 300) {
        activeMeters++;
        avgV += voltage[i];
        vCount++;
      }
    }
  }
  avgV = vCount > 0 ? avgV / vCount : 230;

  String powerColor = totalPower > 1000 ? "#e74c3c" : totalPower > 500 ? "#f39c12"
                                                                       : "#2ecc71";
  String voltageColor = avgV > 250 ? "#e74c3c" : avgV < 210 ? "#f39c12"
                                                            : "#2ecc71";
  String metersColor = "#2ecc71";  // Default green

  // ========== LIVE PANEL ADAPTATIVE ==========
  String meterStatus = "";

  // MODE BADGE
  String modeBadge = "";
  if (!aggregateEnabled) {
    modeBadge = "<span style='background:#6c757d; color:white; padding:3px 12px; border-radius:20px; margin-left:10px; font-weight:bold; font-size:0.9em;'>🔌 6 CHANNELS (NO AGG)</span>";
    metersColor = activeMeters == 6 ? "#2ecc71" : activeMeters == 0 ? "#e74c3c"
                                                                    : "#f39c12";
  } else if (physicalAggregate) {
    modeBadge = "<span style='background:#ffc107; color:#856404; padding:3px 12px; border-radius:20px; margin-left:10px; font-weight:bold; font-size:0.9em;'>⚡ PHYSICAL AGG (M0)</span>";
    metersColor = activeMeters == 5 ? "#2ecc71" : activeMeters == 0 ? "#e74c3c"
                                                                    : "#f39c12";
  } else {
    modeBadge = "<span style='background:#27ae60; color:white; padding:3px 12px; border-radius:20px; margin-left:10px; font-weight:bold; font-size:0.9em;'>🧮 M6 CALCULATED</span>";
    metersColor = activeMeters == 6 ? "#2ecc71" : activeMeters == 0 ? "#e74c3c"
                                                                    : "#f39c12";
  }

  // M0
  if (physicalAggregate) {
    meterStatus += "<li style='background:#fff3cd; border-left:6px solid #ffc107; margin-bottom:10px; padding:10px; border-radius:0 10px 10px 0;'>";
    meterStatus += "<div style='display:flex; align-items:center; flex-wrap:wrap;'>";
    meterStatus += "<strong style='color:#856404; font-size:1.2em;'><i class='fas fa-building'></i> M0 PHYSICAL AGGREGATE:</strong>";
    meterStatus += "<span style='background:#ffc107; color:#856404; padding:3px 15px; border-radius:25px; margin-left:15px; font-weight:bold;'>MAIN</span>";
    meterStatus += modeBadge;
    meterStatus += "</div>";
  } else {
    meterStatus += "<li style='border-left:6px solid #3498db; margin-bottom:10px; padding:10px; border-radius:0 10px 10px 0;'>";
    meterStatus += "<div style='display:flex; align-items:center; flex-wrap:wrap;'>";
    meterStatus += "<strong style='color:#2c3e50; font-size:1.2em;'><i class='fas fa-microchip'></i> M0 INDIVIDUAL:</strong>";
    meterStatus += modeBadge;
    meterStatus += "</div>";
  }

  meterStatus += "<br><span style='font-size:1.8em; font-weight:bold; color:#27ae60;'>" + String(power[0], 1) + " W</span>";
  meterStatus += "<span style='margin-left:20px; font-size:1.2em; color:#7f8c8d;'><i class='fas fa-bolt'></i> " + String(current[0], 3) + " A</span>";
  meterStatus += "<span style='margin-left:20px; font-size:1.2em; color:#7f8c8d;'><i class='fas fa-plug'></i> " + String(voltage[0], 1) + " V</span>";
  meterStatus += "</li>";

  // M1-M5
  String indivColors[5] = { "#3498db", "#2ecc71", "#e67e22", "#9b59b6", "#e74c3c" };
  for (int i = 1; i < 6; i++) {
    meterStatus += "<li style='border-left:6px solid " + indivColors[i - 1] + "; margin-bottom:8px; padding:8px; border-radius:0 8px 8px 0;'>";
    meterStatus += "<strong style='color:" + indivColors[i - 1] + ";'><i class='fas fa-microchip'></i> M" + String(i) + " INDIVIDUAL:</strong>";
    meterStatus += "<br><span style='font-size:1.4em; font-weight:bold; color:#2c3e50;'>" + String(power[i], 1) + " W</span>";
    meterStatus += "<span style='margin-left:15px; color:#7f8c8d;'><i class='fas fa-bolt'></i> " + String(current[i], 3) + " A</span>";
    meterStatus += "<span style='margin-left:15px; color:#7f8c8d;'><i class='fas fa-plug'></i> " + String(voltage[i], 1) + " V</span>";
    meterStatus += "</li>";
  }

  // M6 - ONLY IN MODE 2 (aggregateEnabled AND !physicalAggregate)
  if (aggregateEnabled && !physicalAggregate) {
    meterStatus += "<li style='border-top:3px solid #27ae60; border-left:6px solid #27ae60; background:#f0fff0; margin-top:15px; padding:15px; border-radius:0 10px 10px 0;'>";
    meterStatus += "<div style='display:flex; align-items:center; flex-wrap:wrap;'>";
    meterStatus += "<strong style='color:#27ae60; font-size:1.3em;'><i class='fas fa-calculator'></i> M6 " + aggregateName + ":</strong>";
    meterStatus += "<span style='background:#27ae60; color:white; padding:3px 15px; border-radius:25px; margin-left:15px; font-weight:bold;'>CALCULATED</span>";
    meterStatus += "</div>";
    meterStatus += "<br><span style='font-size:2.0em; font-weight:bold; color:#27ae60;'>" + String(power[6], 1) + " W</span>";
    meterStatus += "<span style='margin-left:20px; font-size:1.3em; color:#27ae60;'><i class='fas fa-bolt'></i> " + String(current[6], 3) + " A</span>";
    meterStatus += "<span style='margin-left:20px; font-size:1.3em; color:#27ae60;'><i class='fas fa-plug'></i> " + String(voltage[6], 1) + " V</span>";
    meterStatus += "<br><small style='color:#7f8c8d; display:block; margin-top:10px;'>Sum of M0 + M1 + M2 + M3 + M4 + M5</small>";
    meterStatus += "</li>";
  }


  String envBlock = "<li style='border-left:6px solid #16a085; background:#f0fffb; margin-bottom:10px; padding:10px; border-radius:0 10px 10px 0;'>";
  envBlock += "<strong style='color:#16a085;'><i class='fas fa-temperature-half'></i> ENVIRONMENT</strong><br>";
  envBlock += "<span style='font-size:1.2em; color:#2c3e50;'>🌡️ Temp: " + (isnan(ambientTempC) ? String("--") : String(ambientTempC, 1) + " °C") + "</span>";
  envBlock += "<span style='margin-left:20px; font-size:1.2em; color:#2c3e50;'>💧 Hum: " + (isnan(ambientHum) ? String("--") : String(ambientHum, 1) + " %") + "</span>";
  envBlock += "</li>";
  meterStatus = envBlock + meterStatus;

  // SD Info
  uint64_t totalBytes = SD.totalBytes();
  uint64_t usedBytes = SD.usedBytes();
  float totalMB = totalBytes / (1024.0 * 1024.0);
  float usedMB = usedBytes / (1024.0 * 1024.0);
  int usedPercent = totalBytes > 0 ? (usedBytes * 100) / totalBytes : 0;

  // Download/View links
  String downloadLinks = "";
  String viewLinks = "";
  for (int i = 0; i < 6; i++) {
    if (SD.exists(fileName[i])) {
      File f = SD.open(fileName[i], FILE_READ);
      float kb = f.size() / 1024.0;
      f.close();
      downloadLinks += "<li>M" + String(i) + " (" + String(kb, 1) + " KB) ";
      downloadLinks += "<a href='/download" + fileName[i] + "' class='btn-small'><i class='fas fa-file-csv'></i> CSV</a></li>";
      viewLinks += "<li>M" + String(i) + " <a href='/view" + fileName[i] + "' class='btn-small'><i class='fas fa-search'></i> View</a></li>";
    }
  }

  // M6 is calculated in RAM only. It is intentionally not logged/exported as 6.csv.

  // Meters display string
  String metersDisplay = "";
  if (!aggregateEnabled) {
    metersDisplay = String(activeMeters) + "/6";
  } else if (physicalAggregate) {
    metersDisplay = String(activeMeters) + "/5 + AGG";
  } else {
    metersDisplay = String(activeMeters) + "/6";
  }

  // Replace placeholders
  html.replace("__IP__", WiFi.localIP().toString());
  html.replace("__PORT__", String(webPort));
  html.replace("__OTA_PORT__", String(otaPort));
  html.replace("__TIME__", getFormattedTime());
  html.replace("__POWER__", String(totalPower, 1));
  html.replace("__POWER_COLOR__", powerColor);
  html.replace("__VOLTAGE__", String(avgV, 1));
  html.replace("__VOLTAGE_COLOR__", voltageColor);
  html.replace("__METERS__", metersDisplay);
  html.replace("__METERS_COLOR__", metersColor);
  html.replace("__DOWNLOAD_LINKS__", downloadLinks);
  html.replace("__VIEW_LINKS__", viewLinks);
  html.replace("__METER_STATUS__", meterStatus);
  html.replace("__SD_TOTAL__", String(totalMB, 1));
  html.replace("__SD_USED__", String(usedMB, 1));
  html.replace("__SD_FREE__", String((totalBytes - usedBytes) / (1024.0 * 1024.0), 1));
  html.replace("__SD_PERCENT__", String(usedPercent));
  html.replace("__FREE_HEAP__", String(ESP.getFreeHeap() / 1024));

  server.send(200, "text/html", html);
}


// ================== WEB SERVER - REALTIME ==================
void handleRealtime() {
  String html = FPSTR(REALTIME_HTML);
  String rows = "";

  // HEADER OF MODE
  String modeHeader = "<tr style='background:#34495e;'><td colspan='8' style='color:white; text-align:center; font-weight:bold; padding:10px;'>";
  if (!aggregateEnabled) {
    modeHeader += "🔌 MODE: 6 INDEPENDENT CHANNELS (NO AGGREGATE) 🔌";
  } else if (physicalAggregate) {
    modeHeader += "⚡ MODE: PHYSICAL AGGREGATE (M0 = REAL SUM M1-M5) | 5 INDIV + 1 AGG ⚡";
  } else {
    modeHeader += "🧮 MODE: 6 INDIVIDUAL + M6 CALCULATED 🧮";
  }
  modeHeader += "</td></tr>";
  rows += modeHeader;
  rows += "<tr style='background:#eefaf7;'><td colspan='8' style='text-align:center; font-weight:bold; color:#145a4a;'>";
  rows += "🌡️ Temp: " + (isnan(ambientTempC) ? String("--") : String(ambientTempC, 1) + " °C");
  rows += " &nbsp;&nbsp; 💧 Hum: " + (isnan(ambientHum) ? String("--") : String(ambientHum, 1) + " %");
  rows += "</td></tr>";

  // M0
  rows += "<tr>";
  if (physicalAggregate) {
    rows += "<td style='background:#fff3cd;'><strong style='color:#856404;'>M0 <span style='background:#ffc107; color:#856404; padding:2px 8px; border-radius:12px; margin-left:5px;'>PHYSICAL AGGREGATE</span></strong></td>";
  } else {
    rows += "<td><strong>M0 INDIVIDUAL</strong></td>";
  }

  if (voltage[0] > 100 && voltage[0] < 300) {
    if (power[0] > 1) rows += "<td><span style='color:#2ecc71;'>● Active</span></td>";
    else rows += "<td><span style='color:#f39c12;'>● Standby</span></td>";
  } else {
    rows += "<td><span style='color:#e74c3c;'>● Inactive</span></td>";
  }

  rows += "<td class='voltage'>" + String(voltage[0], 1) + "</td>";
  rows += "<td class='current'>" + String(current[0], 3) + "</td>";
  rows += "<td class='power'>" + String(power[0], 1) + "</td>";
  rows += "<td>" + String(frequencyArr[0], 1) + "</td>";
  rows += "<td>" + String(pfArr[0], 2) + "</td>";
  rows += "<td><a href='/download/0.csv'>📥 CSV</a></td>";
  rows += "</tr>";

  // M1-M5
  for (int i = 1; i < 6; i++) {
    rows += "<tr>";
    rows += "<td><strong>M" + String(i) + " INDIVIDUAL</strong></td>";

    if (voltage[i] > 100 && voltage[i] < 300) {
      if (power[i] > 1) rows += "<td><span style='color:#2ecc71;'>● Active</span></td>";
      else rows += "<td><span style='color:#f39c12;'>● Standby</span></td>";
    } else {
      rows += "<td><span style='color:#e74c3c;'>● Inactive</span></td>";
    }

    rows += "<td class='voltage'>" + String(voltage[i], 1) + "</td>";
    rows += "<td class='current'>" + String(current[i], 3) + "</td>";
    rows += "<td class='power'>" + String(power[i], 1) + "</td>";
    rows += "<td>" + String(frequencyArr[i], 1) + "</td>";
    rows += "<td>" + String(pfArr[i], 2) + "</td>";
    rows += "<td><a href='/download/" + String(i) + ".csv'>📥 CSV</a></td>";
    rows += "</tr>";
  }

  // M6 - ONLY IN MODE 2 (aggregateEnabled AND !physicalAggregate)
  if (aggregateEnabled && !physicalAggregate) {
    rows += "<tr style='background:#f0fff0; border-top:3px solid #27ae60;'>";
    rows += "<td><strong style='color:#27ae60;'>M6 <span style='background:#27ae60; color:white; padding:2px 8px; border-radius:12px; margin-left:5px;'>CALCULATED</span></strong></td>";
    rows += "<td><span style='background:#27ae60; color:white; padding:3px 8px; border-radius:12px;'>SUM M0-M5</span></td>";
    rows += "<td class='voltage'><strong>" + String(voltage[6], 1) + "</strong></td>";
    rows += "<td class='current'><strong>" + String(current[6], 3) + "</strong></td>";
    rows += "<td class='power'><strong>" + String(power[6], 1) + " W</strong></td>";
    rows += "<td>50.0</td>";
    rows += "<td>1.00</td>";
    rows += "<td><span style='color:#7f8c8d;'>RAM only</span></td>";
    rows += "</tr>";
  }

  // Totals by mode
  float totalPower = 0;
  int activeMeters = 0;
  float avgV = 0;
  int vCount = 0;

  if (physicalAggregate) {
    totalPower = power[0];
    for (int i = 1; i < 6; i++) {
      if (voltage[i] > 100 && voltage[i] < 300) {
        activeMeters++;
        avgV += voltage[i];
        vCount++;
      }
    }
  } else {
    for (int i = 0; i < 6; i++) {
      totalPower += power[i];
      if (voltage[i] > 100 && voltage[i] < 300) {
        activeMeters++;
        avgV += voltage[i];
        vCount++;
      }
    }
  }
  avgV = vCount > 0 ? avgV / vCount : 230;

  // Active meters display string
  String activeDisplay = "";
  if (!aggregateEnabled) {
    activeDisplay = String(activeMeters) + "/6";
  } else if (physicalAggregate) {
    activeDisplay = String(activeMeters) + "/5 + AGG";
  } else {
    activeDisplay = String(activeMeters) + "/6";
  }

  html.replace("__TIME__", getFormattedTime());
  html.replace("__TABLE_ROWS__", rows);
  html.replace("__TOTAL_POWER__", String(totalPower, 1));
  html.replace("__AVG_VOLTAGE__", String(avgV, 1));
  html.replace("__ACTIVE_METERS__", activeDisplay);

  server.send(200, "text/html", html);
}


// ================== GRAPHS ==================
void handleGraphs() {
  // SERIAL_DISABLED: Serial.println("📊 Serving graphs page");

  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>Graphs - OMPM</title>
  <script src='https://cdn.jsdelivr.net/npm/chart.js@3.9.1/dist/chart.min.js'></script>
  <style>
    body { margin: 0; padding: 20px; background: #2c3e50; font-family: 'Segoe UI', Arial; }
    .container { max-width: 1200px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 5px 15px rgba(0,0,0,0.2); }
    .chart-container { height: 500px; width: 100%; position: relative; margin: 20px 0; }
    .btn { padding: 10px 20px; margin: 5px; background: #3498db; color: white; border: none; border-radius: 5px; cursor: pointer; font-weight: bold; transition: all 0.3s; }
    .btn:hover { background: #2980b9; transform: scale(1.05); }
    .btn-small { padding: 5px 15px; font-size: 0.9em; }
    .btn.active { background: #27ae60; }
    .meter-selector { margin: 15px 0; padding: 15px; background: #f8f9fa; border-radius: 8px; }
    .meter-btn { padding: 8px 16px; margin: 3px; background: #6c757d; color: white; border: none; border-radius: 20px; cursor: pointer; font-weight: bold; transition: all 0.2s; }
    .meter-btn:hover { transform: scale(1.05); }
    .meter-btn.active { background: #28a745; }
    .stats-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 15px; margin: 20px 0; }
    .stat-card { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); padding: 15px; border-radius: 10px; color: white; text-align: center; box-shadow: 0 5px 15px rgba(0,0,0,0.1); }
    .stat-label { font-size: 0.85em; text-transform: uppercase; letter-spacing: 1px; opacity: 0.9; }
    .stat-value { font-size: 1.8em; font-weight: bold; margin-top: 5px; }
    .status { text-align: center; padding: 10px; margin-top: 10px; font-weight: bold; border-radius: 5px; }
    .footer { text-align: center; margin-top: 20px; color: #7f8c8d; font-size: 0.9em; }
    .range-selector { display: flex; flex-wrap: wrap; gap: 5px; justify-content: center; margin: 10px 0; }
    .mode-badge { display: inline-block; padding: 5px 15px; border-radius: 25px; font-weight: bold; margin-left: 10px; }
  </style>
</head>
<body>
  <div class='container'>
    <div style='display:flex; align-items:center; justify-content:space-between; flex-wrap:wrap; gap:10px;'>
      <div style='display:flex; align-items:center; flex-wrap:wrap; gap:10px;'>
        <h1 style='color:#2c3e50; margin:0;'>📈 Power Consumption</h1>
        <div id='modeBadge' style='background:#ffc107; color:#856404; padding:5px 15px; border-radius:25px; font-weight:bold;'>MODE</div>
      </div>
    </div>
    
    <div class='range-selector'>
      <button class='btn btn-small active' onclick='loadData(1)'>1 Hour</button>
      <button class='btn btn-small' onclick='loadData(6)'>6 Hours</button>
      <button class='btn btn-small' onclick='loadData(12)'>12 Hours</button>
      <button class='btn btn-small' onclick='loadData(24)'>24 Hours</button>
      <button class='btn btn-small' onclick='loadData(168)'>7 Days</button>
    </div>

    <div class='meter-selector' id='meterSelector'>
      <strong style='color:#2c3e50;'>🔌 Select Meters:</strong>
      <button class='meter-btn active' onclick='toggleMeter(0)'>M0</button>
      <button class='meter-btn active' onclick='toggleMeter(1)'>M1</button>
      <button class='meter-btn active' onclick='toggleMeter(2)'>M2</button>
      <button class='meter-btn active' onclick='toggleMeter(3)'>M3</button>
      <button class='meter-btn active' onclick='toggleMeter(4)'>M4</button>
      <button class='meter-btn active' onclick='toggleMeter(5)'>M5</button>
      <button class='meter-btn' id='m6btn' onclick='toggleMeter(6)'>M6</button>
    </div>

    

    <div class='chart-container'>
      <canvas id='powerChart'></canvas>
    </div>

    <div class='stats-grid' id='statsGrid'>
      <div class='stat-card'>
        <div class='stat-label'>Max Power</div>
        <div class='stat-value' id='maxPower'>0 W</div>
      </div>
      <div class='stat-card'>
        <div class='stat-label'>Min Power</div>
        <div class='stat-value' id='minPower'>0 W</div>
      </div>
      <div class='stat-card'>
        <div class='stat-label'>Average Power</div>
        <div class='stat-value' id='avgPower'>0 W</div>
      </div>
      <div class='stat-card'>
        <div class='stat-label'>Active Meters</div>
        <div class='stat-value' id='activeMeters'>0/7</div>
      </div>
    </div>

    <div id='status' class='status' style='background:#f8f9fa;'>✅ Ready</div>
    
    <div class='footer'>
      <i class='fas fa-bolt'></i> OMPM v68 - Real-time Power Monitoring
         <br>
      <span id='dataSource' style='font-size:0.7em; color:#95a5a6;'></span>
    </div>
  </div>

 <script>
    const colors = ['#3498db', '#2ecc71', '#e67e22', '#9b59b6', '#e74c3c', '#f1c40f', '#1abc9c'];
    let chart = null;
    let activeMeters = [0,1,2,3,4,5];
    let currentData = [];
    let currentHours = 1;
   
    let lastDataPoints = 0;
    let physicalMode = __PHYSICAL_MODE__;
    let aggregateEnabled = __AGGREGATE_ENABLED__;

    // Configure according to mode
    if (!aggregateEnabled) {
      document.getElementById('modeBadge').innerHTML = '🔌 6 CHANNELS';
      document.getElementById('modeBadge').style.background = '#6c757d';
      document.getElementById('modeBadge').style.color = 'white';
      document.getElementById('m6btn').style.display = 'none';
    } else if (physicalMode) {
      document.getElementById('modeBadge').innerHTML = '⚡ PHYSICAL AGG (M0)';
      document.getElementById('modeBadge').style.background = '#ffc107';
      document.getElementById('modeBadge').style.color = '#856404';
      document.getElementById('m6btn').style.display = 'none';
    } else {
      document.getElementById('modeBadge').innerHTML = '🧮 M6 CALCULATED';
      document.getElementById('modeBadge').style.background = '#27ae60';
      document.getElementById('modeBadge').style.color = 'white';
      document.getElementById('m6btn').style.display = 'inline-block';
      activeMeters.push(6);
    }

   

    function toggleMeter(meter) {
      const index = activeMeters.indexOf(meter);
      const btn = document.querySelector(`.meter-btn[onclick*="${meter}"]`);
      if (index === -1) {
        activeMeters.push(meter);
        btn.classList.add('active');
      } else {
        activeMeters.splice(index, 1);
        btn.classList.remove('active');
      }
      if (currentData.length > 0) {
        updateChart();
        updateStats();
      }
    }

    function loadData(hours) {
      currentHours = hours;
      
      document.querySelectorAll('.btn').forEach(btn => btn.classList.remove('active'));
      event.target.classList.add('active');
      
      const status = document.getElementById('status');
      status.innerHTML = '🔄 Loading data...';
      status.style.background = '#fff3cd';
      status.style.color = '#856404';
      
      const endTime = Math.floor(Date.now() / 1000);
      const startTime = endTime - (hours * 3600);
      
      fetch(`/graphdata?start=${startTime}&end=${endTime}`)
        .then(response => {
          if (!response.ok) throw new Error('Network error');
          return response.json();
        })
        .then(data => {
          console.log('📊 Data received:', data.length, 'points');
          currentData = data;
          lastDataPoints = data.length;
          
          
          
          updateChart();
          updateStats();
        })
        .catch(error => {
          status.innerHTML = `❌ Error: ${error.message}`;
          status.style.background = '#f8d7da';
          status.style.color = '#721c24';
          console.error('Error:', error);
        });
    }

   
    function updateChart() {
  const ctx = document.getElementById('powerChart').getContext('2d');
  
  if (chart) {
    chart.destroy();
    chart = null;
  }
  
  const datasets = [];
  
  activeMeters.sort().forEach(meter => {
    // Ensure timestamp is treated as a number
    const meterData = currentData
      .filter(d => d && d.meter === meter)
      .map(d => ({
        x: Number(d.timestamp),  // Force to number
        y: Number(d.power) || 0
      }))
      .sort((a, b) => a.x - b.x);
      
    if (meterData.length > 0) {
      const borderDash = (meter === 6 ? [5, 5] : []);
      
      datasets.push({
        label: meter === 6 ? `M6 ${physicalMode ? '' : '(SUM)'}` : `M${meter}`,
        data: meterData,
        borderColor: colors[meter] || '#95a5a6',
        backgroundColor: (colors[meter] || '#95a5a6') + '20',
        borderWidth: meter === 6 ? 3 : 2,
        borderDash: borderDash,
        pointRadius: 2,
        pointHoverRadius: 4,
        tension: 0.1
      });
    }
  });
  
  if (datasets.length === 0) {
    document.getElementById('status').innerHTML = '⚠️ No data for selected meters';
    return;
  }
  
  chart = new Chart(ctx, {
    type: 'line',
    data: { datasets },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      interaction: { mode: 'index', intersect: false },
      plugins: {
        tooltip: {
          callbacks: {
            label: function(context) {
              const date = new Date(context.raw.x * 1000);
              return `${context.dataset.label}: ${context.raw.y.toFixed(1)} W (${formatDate(date)})`;
            }
          }
        }
      },
      scales: {
        x: { 
          type: 'linear',
          title: { display: true, text: 'Time' },
          ticks: {
            callback: function(value) {
              const date = new Date(value * 1000);
              const hours = currentHours;
              
              if (hours <= 24) {
                return date.getHours().toString().padStart(2,'0') + ':' + 
                       date.getMinutes().toString().padStart(2,'0');
              } else if (hours <= 72) {
                const days = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'];
                return days[date.getDay()] + ' ' + 
                       date.getHours().toString().padStart(2,'0') + 'h';
              } else {
                return (date.getDate()) + '/' + (date.getMonth() + 1);
              }
            }
          }
        },
        y: { 
          beginAtZero: true, 
          title: { display: true, text: 'Power (W)' },
          grid: {
            color: 'rgba(0,0,0,0.1)'
          }
        }
      }
    }
  });
}

    function formatDate(date) {
      const hours = date.getHours().toString().padStart(2,'0');
      const minutes = date.getMinutes().toString().padStart(2,'0');
      const day = date.getDate().toString().padStart(2,'0');
      const month = (date.getMonth() + 1).toString().padStart(2,'0');
      
      if (currentHours <= 24) {
        return `${hours}:${minutes}`;
      } else if (currentHours <= 72) {
        const days = ['Sun','Mon','Tue','Wed','Thu','Fri','Sat'];
        return `${days[date.getDay()]} ${hours}:${minutes}`;
      } else {
        return `${day}/${month}`;
      }
    }

    function updateStats() {
      const selectedData = currentData.filter(d => activeMeters.includes(d.meter));
      const powers = selectedData.map(d => d.power);
      
      if (powers.length === 0) return;
      
      const maxPower = Math.max(...powers);
      const minPower = Math.min(...powers);
      const avgPower = powers.reduce((a,b) => a+b,0) / powers.length;
      
      document.getElementById('maxPower').innerHTML = maxPower.toFixed(1) + ' W';
      document.getElementById('minPower').innerHTML = minPower.toFixed(1) + ' W';
      document.getElementById('avgPower').innerHTML = avgPower.toFixed(1) + ' W';
      const totalMeterSlots = (!aggregateEnabled || physicalMode) ? 6 : 7;
      document.getElementById('activeMeters').innerHTML = activeMeters.length + '/' + totalMeterSlots;
      
       // Use the real operating mode from the loaded configuration.
        document.getElementById('maxPower').style.color = 'white';
        document.getElementById('minPower').style.color = 'white';
        document.getElementById('avgPower').style.color = 'white';
   
    }

    window.onload = () => {
      const fakeEvent = { target: document.querySelector('.btn') };
      loadData(1);
    };
  </script>

</body>
</html>
)rawliteral";

  html.replace("__PHYSICAL_MODE__", physicalAggregate ? "true" : "false");
  html.replace("__AGGREGATE_ENABLED__", aggregateEnabled ? "true" : "false");

  server.send(200, "text/html", html);
}
// ================== MEMORY MONITORING ==================
void logMemory(const char* location) {
  // SERIAL_DISABLED: Serial.printf("📊 Memory [%s]: Free=%d, Min=%d\n",
  // SERIAL_DISABLED: location,
  // SERIAL_DISABLED: ESP.getFreeHeap(),
  // SERIAL_DISABLED: ESP.getMinFreeHeap());
}


// ================== GET HISTORICAL DATA - STREAMING VERSION ==================
String getHistoricalDataRange(long startTimeSec, long endTimeSec) {
  flushLogFiles(true);
  // SERIAL_DISABLED: Serial.println("\n🔍 --- getHistoricalDataRange() ---");
  // SERIAL_DISABLED: Serial.printf("   Range: %ld -> %ld (%ld hours)\n",
  // SERIAL_DISABLED: startTimeSec, endTimeSec, (endTimeSec - startTimeSec) / 3600);

  // Calculate how many points we need based on the range
  int hours = (endTimeSec - startTimeSec) / 3600;
  if (hours < 1) hours = 1;

  // Calculate point spacing to avoid overcrowding the graph.
  int targetPointsPerMeter;
  if (hours <= 6) {
    targetPointsPerMeter = 200;  // 6h: one point every ~2 minutes
  } else if (hours <= 24) {
    targetPointsPerMeter = 200;  // 24h: one point every ~7 minutes
  } else if (hours <= 72) {
    targetPointsPerMeter = 150;  // 3 days: one point every ~30 minutes
  } else {
    targetPointsPerMeter = 100;  // >3 days: widely spaced
  }

  // SERIAL_DISABLED: Serial.printf("   Target: %d points per meter\n", targetPointsPerMeter);

  // Use String to build JSON manually (more efficient)
  String jsonOutput = "[";
  bool firstPoint = true;
  int totalPoints = 0;
  int maxMeter = LOG_FILE_COUNT;

  for (int meter = 0; meter < maxMeter; meter++) {

    String fname = fileName[meter];
    // SERIAL_DISABLED: Serial.printf("   📁 Reading %s... ", fname.c_str());

    if (SD.exists(fname)) {
      File file = SD.open(fname, FILE_READ);
      if (file) {
        // Skip header
        file.readStringUntil('\n');

        // Calculate spacing based on file size
        int fileSize = file.size();
        int approxLines = fileSize / 50;  // Approximation: 50 bytes per CSV line.
        int step = approxLines / targetPointsPerMeter;
        if (step < 1) step = 1;

        // SERIAL_DISABLED: Serial.printf("step=%d\n", step);

        int lineCount = 0;
        int pointsAdded = 0;

        while (file.available() && pointsAdded < targetPointsPerMeter) {
          String line = file.readStringUntil('\n');
          line.trim();

          // Only process the selected line according to the step
          if (lineCount % step == 0) {
            if (line.length() > 0) {
              int comma1 = line.indexOf(',');
              if (comma1 > 0) {
                String tsStr = line.substring(0, comma1);
                double timestampSec = tsStr.toDouble() / 1000.0;

                // FILTER BY TIME RANGE
                if (timestampSec >= startTimeSec && timestampSec <= endTimeSec) {
                  int comma2 = line.indexOf(',', comma1 + 1);
                  int comma3 = line.indexOf(',', comma2 + 1);
                  int comma4 = line.indexOf(',', comma3 + 1);

                  if (comma4 > 0) {
                    String vStr = line.substring(comma1 + 1, comma2);
                    String iStr = line.substring(comma2 + 1, comma3);
                    String pStr = line.substring(comma3 + 1, comma4);

                    float power = pStr.toFloat();

                    // Save only if there is significant power
                    if (power > 0.5) {
                      if (!firstPoint) {
                        jsonOutput += ",";
                      }
                      firstPoint = false;

                      // Build JSON manually (faster and more efficient)
                      jsonOutput += "{\"meter\":";
                      jsonOutput += String(meter);
                      jsonOutput += ",\"timestamp\":";
                      jsonOutput += String(timestampSec, 0);
                      jsonOutput += ",\"voltage\":";
                      jsonOutput += vStr;
                      jsonOutput += ",\"current\":";
                      jsonOutput += iStr;
                      jsonOutput += ",\"power\":";
                      jsonOutput += String(power, 1);
                      jsonOutput += "}";

                      totalPoints++;
                      pointsAdded++;
                    }
                  }
                }
              }
            }
          }
          lineCount++;

          // Release memory periodically
          if (lineCount % 100 == 0) {
            yield();  // Allow the watchdog to run
          }
        }
        file.close();
        // SERIAL_DISABLED: Serial.printf("%d points (step=%d)\n", pointsAdded, step);
      } else {
        // SERIAL_DISABLED: Serial.println("ERROR opening");
      }
    } else {
      // SERIAL_DISABLED: Serial.println("DOES NOT EXIST");
    }
  }

  jsonOutput += "]";

  // If there is no data, return an empty array
  if (totalPoints == 0) {
    // SERIAL_DISABLED: Serial.println("   ⚠️ No points with power were found");
    return "[]";
  }

  // SERIAL_DISABLED: Serial.printf("   📊 TOTAL: %d points\n", totalPoints);
  // SERIAL_DISABLED: Serial.printf("   📦 JSON size: %d bytes\n", jsonOutput.length());

  return jsonOutput;
}

// ================== HANDLE GRAPH DATA - FIXED VERSION ==================
void handleGraphData() {
  // SERIAL_DISABLED: Serial.println("\n📈 ========== HANDLE GRAPH DATA ==========");
  logMemory("Starting");

  if (!server.hasArg("start") || !server.hasArg("end")) {
    // SERIAL_DISABLED: Serial.println("❌ Missing parameters");
    server.send(400, "application/json", "[]");
    return;
  }

  long browserStartSec = server.arg("start").toInt();
  long browserEndSec = server.arg("end").toInt();

  if (browserStartSec <= 0 || browserEndSec <= 0 || browserStartSec >= browserEndSec) {
    // SERIAL_DISABLED: Serial.println("❌ Invalid parameters");
    server.send(400, "application/json", "[]");
    return;
  }

  // SERIAL_DISABLED: Serial.printf("📅 Requested range: %ld -> %ld (%ld hours)\n",
  // SERIAL_DISABLED: browserStartSec, browserEndSec, (browserEndSec - browserStartSec) / 3600);

  // Read data for the requested range
  // SERIAL_DISABLED: Serial.println("🔍 Reading data from SD...");
  String jsonData = getHistoricalDataRange(browserStartSec, browserEndSec);

  // real data

  // SERIAL_DISABLED: Serial.println("✅ Using REAL data from SD");

  logMemory("Before sending");

  // Configure headers
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");

  server.send(200, "application/json", jsonData);

  // SERIAL_DISABLED: Serial.printf("📤 Response sent: %d bytes\n", jsonData.length());
  logMemory("END");
}





void forceSDWrite() {
  if (!sdOK) {
    // SERIAL_DISABLED: Serial.println("ERROR: SD not available");
    return;
  }

  long long timestamp = (long long)timeClient.getEpochTime() * 1000LL + (millis() % 1000);
  uint8_t maxMeter = LOG_FILE_COUNT;
  for (uint8_t i = 0; i < maxMeter; i++) {
    sdWritePending[i] = true;
    writeCSVRowToLog(i, timestamp);
    delay(1);
  }
  flushLogFiles(true);
  lastSDWrite = millis();
  // SERIAL_DISABLED: Serial.println("✅ Forced write completed");
}

// ================== Other functions of the  TFT ==================
void showMenu() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(5, 0);
  tft.println("=== MENU ===");
  if (menuIndex < menuTop) menuTop = menuIndex;
  if (menuIndex >= menuTop + MENU_VISIBLE) menuTop = menuIndex - MENU_VISIBLE + 1;
  for (int i = 0; i < MENU_VISIBLE; i++) {
    int itemIdx = menuTop + i;
    if (itemIdx >= MENU_ITEMS) break;
    tft.setTextColor(itemIdx == menuIndex ? ST77XX_GREEN : ST77XX_WHITE);
    tft.setCursor(10, 15 + i * 10);
    tft.println(menuItems[itemIdx]);
  }
}

bool readDHT11Raw(int pin, float& tempC, float& hum) {
  uint8_t data[5] = { 0, 0, 0, 0, 0 };

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(20);
  digitalWrite(pin, HIGH);
  delayMicroseconds(40);
  pinMode(pin, INPUT_PULLUP);

  unsigned long t = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - t > 100) return false;
  }
  t = micros();
  while (digitalRead(pin) == LOW) {
    if (micros() - t > 100) return false;
  }
  t = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - t > 100) return false;
  }

  for (int i = 0; i < 40; i++) {
    t = micros();
    while (digitalRead(pin) == LOW) {
      if (micros() - t > 100) return false;
    }
    unsigned long startHigh = micros();
    while (digitalRead(pin) == HIGH) {
      if (micros() - startHigh > 120) break;
    }
    unsigned long highTime = micros() - startHigh;
    data[i / 8] <<= 1;
    if (highTime > 40) data[i / 8] |= 1;
  }

  uint8_t checksum = data[0] + data[1] + data[2] + data[3];
  if (checksum != data[4]) return false;

  hum = data[0];
  tempC = data[2];
  return true;
}

void updateDHT11() {
  unsigned long now = millis();

  // Read immediately at startup if no valid ambient value exists yet.
  // After the first valid read, respect DHT_READ_INTERVAL.
  if ((now - lastDHTRead < DHT_READ_INTERVAL) && !isnan(ambientTempC) && !isnan(ambientHum)) return;
  lastDHTRead = now;

  float t, h;
  if (readDHT11Raw(DHT_PIN, t, h)) {
    ambientTempC = t;
    ambientHum = h;
  }
  // If the DHT11 read fails, keep the last valid values instead of writing NaN
  // into the CSV after a transient sensor error.
}





void showWebServerIP() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(1);
  tft.setCursor(10, 10);
  tft.println("=== WEB SERVER ===");
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(10, 30);
    tft.println("Server ACTIVE!");
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(10, 50);
    tft.println("Access at:");
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(10, 65);
    tft.print("http://");
    tft.println(getIPWithPort());
    tft.setCursor(10, 80);
    tft.setTextColor(ST77XX_CYAN);
    tft.print("Port: ");
    tft.println(webPort);
    tft.setCursor(10, 95);
    tft.setTextColor(ST77XX_CYAN);
    tft.print("Mode: ");
    tft.println(modeName);
  } else {
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(10, 30);
    tft.println("WiFi ERROR");
    tft.setCursor(10, 50);
    tft.println("Not connected");
  }
  tft.setCursor(10, 115);
  tft.println("Press any button");
  tft.setCursor(10, 125);
  tft.println("to return to menu");
}

// ================== IP SUPPORT FUNCTIONS=================
String getIPWithPort() {
  if (WiFi.status() != WL_CONNECTED) return "No connection";
  return webPort != 80 ? WiFi.localIP().toString() + ":" + String(webPort) : WiFi.localIP().toString();
}

String getURLWithPort() {
  if (WiFi.status() != WL_CONNECTED) return "http://0.0.0.0";
  return webPort != 80 ? "http://" + WiFi.localIP().toString() + ":" + String(webPort) : "http://" + WiFi.localIP().toString();
}

void updateOTAProgress(int progress) {
  tft.fillRect(10, 100, 120, 10, ST77XX_BLACK);
  tft.setCursor(10, 100);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("Progress: ");
  tft.print(progress);
  tft.println("%");
}

void showMessage(const char* msg) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((tft.width() - w) / 2, (tft.height() - h) / 2);
  tft.println(msg);
  tft.setTextSize(1);
}

void showCurrentGraph() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(10, 10);
  tft.println("=== CURRENT GRAPH ===");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 30);
  tft.println("Real-time Current");
  for (int i = 0; i < 6; i++) {
    tft.setCursor(10, 50 + i * 10);
    tft.printf("M%d: %.3f A", i, current[i]);
  }
  if (!physicalAggregate) {
    tft.setCursor(10, 115);
    tft.setTextColor(ST77XX_GREEN);
    tft.printf("M6: %.3f A (SUM)", current[6]);
  }
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 140);
  tft.println("Press MENU to exit");
}

void showPowerGraph() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 10);
  tft.println("=== POWER GRAPH ===");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 30);
  tft.println("Real-time Power");
  for (int i = 0; i < 6; i++) {
    tft.setCursor(10, 50 + i * 10);
    tft.printf("M%d: %.1f W", i, power[i]);
  }
  if (!physicalAggregate) {
    tft.setCursor(10, 115);
    tft.setTextColor(ST77XX_GREEN);
    tft.printf("M6: %.1f W (SUM)", power[6]);
  }
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 140);
  tft.println("Press MENU to exit");
}

void showCurrentHist() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 10);
  tft.println("=== CURRENT HISTORY ===");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 30);
  tft.println("Feature in progress");
  tft.setCursor(10, 140);
  tft.println("Press MENU to exit");
}

void showPowerHist() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_ORANGE);
  tft.setCursor(10, 10);
  tft.println("=== POWER HISTORY ===");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 30);
  tft.println("Feature in progress");
  tft.setCursor(10, 140);
  tft.println("Press MENU to exit");
}

void showPzemAddrSel() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(5, 0);
  tft.println("=== PZEM: ADDR ===");
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(5, 20);
  tft.print("Addr #");
  tft.print(addrIndex + 1);
  tft.print(": 0x");
  tft.println(addr[addrIndex], HEX);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5, 45);
  tft.println("UP/DOWN: change addr");
  tft.setCursor(5, 57);
  tft.println("MENU: continue");
}

void showPzemAddrConfirm() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(5, 0);
  tft.println("CONFIRM PZEM");
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(5, 20);
  tft.print("Addr #");
  tft.print(addrIndex + 1);
  tft.print(": 0x");
  tft.println(addr[addrIndex], HEX);
  tft.setCursor(5, 45);
  tft.setTextColor(confirmWrite ? ST77XX_CYAN : ST77XX_WHITE);
  tft.println("WRITE");
  tft.setCursor(5, 57);
  tft.setTextColor(!confirmWrite ? ST77XX_CYAN : ST77XX_WHITE);
  tft.println("EXIT");
}

void showSDmem() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(5, 0);
  tft.println("=== SD MEMORY ===");
  uint64_t totalBytes = SD.totalBytes(), usedBytes = SD.usedBytes();
  float totalMB = totalBytes / (1024.0 * 1024.0), usedMB = usedBytes / (1024.0 * 1024.0);
  tft.setCursor(5, 18);
  tft.setTextColor(ST77XX_GREEN);
  tft.print("Total: ");
  tft.setTextColor(ST77XX_WHITE);
  tft.print(totalMB, 1);
  tft.println(" MB");
  tft.setCursor(5, 30);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("Used: ");
  tft.setTextColor(ST77XX_WHITE);
  tft.print(usedMB, 1);
  tft.print(" MB (");
  tft.print((usedMB / totalMB) * 100, 1);
  tft.println("%)");
}

void showWifiInfo() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(5, 0);
  tft.println("=== WIFI + TIME ===");
  tft.setCursor(5, 18);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("SSID: ");
  tft.setTextColor(ST77XX_WHITE);
  tft.println(WiFi.SSID());
  tft.setCursor(5, 30);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("IP: ");
  tft.setTextColor(ST77XX_WHITE);
  tft.println(getIPWithPort());
  tft.setCursor(5, 42);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("RSSI: ");
  tft.setTextColor(ST77XX_WHITE);
  tft.print(WiFi.RSSI());
  tft.println(" dBm");
  tft.setCursor(5, 54);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Mode: ");
  tft.setTextColor(ST77XX_WHITE);
  tft.println(modeName);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timeBuffer[9];
    snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    tft.setCursor(5, 66);
    tft.setTextColor(ST77XX_GREEN);
    tft.print("Time: ");
    tft.setTextColor(ST77XX_WHITE);
    tft.println(timeBuffer);
  } else {
    tft.setCursor(5, 66);
    tft.setTextColor(ST77XX_RED);
    tft.println("ERROR: No time");
  }
}

void showPzemStatus() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(5, 0);
  tft.setTextColor(ST77XX_ORANGE);
  tft.println("PZEM STATUS");

  // M0
  float v0 = pzem0.voltage();
  tft.setCursor(5, 20);
  tft.setTextColor(isnan(v0) ? ST77XX_RED : ST77XX_GREEN);
  tft.printf("M0:%s V:%.1f", isnan(v0) ? "ERR" : "OK", v0);

  // M1
  float v1 = pzem1.voltage();
  tft.setCursor(5, 30);
  tft.setTextColor(isnan(v1) ? ST77XX_RED : ST77XX_GREEN);
  tft.printf("M1:%s V:%.1f", isnan(v1) ? "ERR" : "OK", v1);

  // M2
  float v2 = pzem2.voltage();
  tft.setCursor(5, 40);
  tft.setTextColor(isnan(v2) ? ST77XX_RED : ST77XX_GREEN);
  tft.printf("M2:%s V:%.1f", isnan(v2) ? "ERR" : "OK", v2);

  // M3
  float v3 = pzem3.voltage();
  tft.setCursor(5, 50);
  tft.setTextColor(isnan(v3) ? ST77XX_RED : ST77XX_GREEN);
  tft.printf("M3:%s V:%.1f", isnan(v3) ? "ERR" : "OK", v3);

  // M4
  float v4 = pzem4.voltage();
  tft.setCursor(5, 60);
  tft.setTextColor(isnan(v4) ? ST77XX_RED : ST77XX_GREEN);
  tft.printf("M4:%s V:%.1f", isnan(v4) ? "ERR" : "OK", v4);

  // M5
  float v5 = pzem5.voltage();
  tft.setCursor(5, 70);
  tft.setTextColor(isnan(v5) ? ST77XX_RED : ST77XX_GREEN);
  tft.printf("M5:%s V:%.1f", isnan(v5) ? "ERR" : "OK", v5);

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5, 90);
  tft.println("Press any button");
  tft.setCursor(5, 100);
  tft.println("to return");
}

void showCalSoft() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);

  float ompmI = current[calChannel];
  float ompmP = power[calChannel];
  float ompmV = voltage[calChannel];

  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setCursor(5, 0);
  tft.println("=== CAL CURRENT ===");

  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(5, 16);
  tft.print("CH: M");
  tft.print(calChannel);
  tft.print("  calI=");
  tft.print(calI[calChannel], 3);

  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(5, 30);
  tft.print("OMPM: ");
  if (isnan(ompmI)) tft.print("nan");
  else tft.print(ompmI, 3);
  tft.println(" A");

  tft.setCursor(5, 42);
  tft.print("REF : ");
  tft.print(calRefCurrent, 3);
  tft.println(" A");

  tft.setCursor(5, 54);
  tft.print("P: ");
  tft.print(ompmP, 1);
  tft.print(" W  V:");
  tft.print(ompmV, 0);

  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.setCursor(5, 68);
  tft.print("Last ratio: ");
  tft.print(lastCalRatio, 4);

  tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  tft.setCursor(5, 84);
  tft.println("UP/DN: REF +/-0.010A");
  tft.setCursor(5, 96);
  tft.println("OK: apply factor");
  tft.setCursor(5, 108);
  tft.println("Long OK/#: save+next");

  prevCalLiveValue = -999999.0;
}

void handleMenuWithFlags(bool upPressed, bool downPressed, bool menuPressed) {
  if (upPressed) {
    menuIndex = (menuIndex + MENU_ITEMS - 1) % MENU_ITEMS;
    showMenu();
  }
  if (downPressed) {
    menuIndex = (menuIndex + 1) % MENU_ITEMS;
    showMenu();
  }
  if (menuPressed) {
    switch (menuIndex) {
      case MENU_EXIT:
        currentState = STATE_MEASURES;
        forceFullRedraw = true;
        needsDisplayUpdate = true;
        initLayout();
        break;
      case MENU_SDMEM:
        currentState = STATE_SDMEM;
        showSDmem();
        break;
      case MENU_REFRESH_CONFIG:
        currentState = STATE_REFRESH_CONFIG;
        showRefreshConfig();
        break;
      case MENU_WIFIINFO:
        currentState = STATE_WIFIINFO;
        showWifiInfo();
        break;
      case MENU_PZEM_STATUS:
        currentState = STATE_PZEM_STATUS;
        showPzemStatus();
        break;
      case MENU_CALIBRATION_GENERAL:
        currentState = STATE_IR_CAL_MENU;
        calSubIndex = 0;
        showIRCalibrationMenu();
        break;
      case MENU_GRAPHS:
        currentState = STATE_IR_GRAPH_MENU;
        graphSubIndex = 0;
        showIRGraphsMenu();
        break;
      case MENU_WEB_SERVER:
        currentState = STATE_WEB_SERVER;
        if (!serverStarted) initWebServer();
        showWebServerIP();
        redraw = true;
        break;
      default: showMenu(); break;
    }
  }
}

void showIRCalibrationMenu() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(5, 0);
  tft.println("=== CALIBRATION ===");

  for (int i = 0; i < CAL_MENU_ITEMS; i++) {
    tft.setTextColor(i == calSubIndex ? ST77XX_GREEN : ST77XX_WHITE);
    tft.setCursor(5, 18 + i * 12);
    tft.println(calMenuItems[i]);
  }

  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(5, 88);
  tft.println("0=Menu  *=Measures");
  tft.setCursor(5, 100);
  tft.println("UP/DOWN + OK");
}

void showIRGraphsMenu() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(5, 0);
  tft.println("=== GRAPHICS ===");

  for (int i = 0; i < GRAPH_MENU_ITEMS; i++) {
    tft.setTextColor(i == graphSubIndex ? ST77XX_GREEN : ST77XX_WHITE);
    tft.setCursor(5, 18 + i * 12);
    tft.println(graphMenuItems[i]);
  }

  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(5, 78);
  tft.println("0=Menu  *=Measures");
  tft.setCursor(5, 90);
  tft.println("UP/DOWN + OK");
}

void executeCalibrationSubmenu() {
  switch (calSubIndex) {
    case 0:
      currentState = STATE_PZEM_ADDR;
      showPzemAddrSel();
      break;
    case 1:
      currentState = STATE_CAL_SOFT;
      lastCalRatio = 1.000f;
      if (!isnan(current[calChannel]) && current[calChannel] > 0.020f) {
        calRefCurrent = current[calChannel];
      }
      showCalSoft();
      break;
    case 2:
      currentState = STATE_SAVE_CALIB;
      showSavingCalib();
      saveCalibration();
      showCalibSavedOK();
      delay(1200);
      currentState = STATE_MENU;
      redraw = true;
      break;
    case 3:
      currentState = STATE_RESET_CALIB;
      resetCalibrationDefaults();
      showMessage("RESET OK");
      delay(1200);
      currentState = STATE_MENU;
      redraw = true;
      break;
    case 4:
      currentState = STATE_VER_CALIB;
      showCurrentCalib();
      delay(3000);
      currentState = STATE_MENU;
      redraw = true;
      break;
    default:
      currentState = STATE_MENU;
      redraw = true;
      break;
  }
}

void executeGraphsSubmenu() {
  switch (graphSubIndex) {
    case 0:
      currentState = STATE_CUR_GRAPH;
      showCurrentGraph();
      break;
    case 1:
      currentState = STATE_PWR_GRAPH;
      showPowerGraph();
      break;
    case 2:
      currentState = STATE_CUR_HIST;
      showCurrentHist();
      break;
    case 3:
      currentState = STATE_PWR_HIST;
      showPowerHist();
      break;
    default:
      currentState = STATE_MENU;
      redraw = true;
      break;
  }
}

void handleCalibrationMenuWithFlags(bool upPressed, bool downPressed, bool menuPressed) {
  if (upPressed) {
    calSubIndex = (calSubIndex + CAL_MENU_ITEMS - 1) % CAL_MENU_ITEMS;
    showIRCalibrationMenu();
  }
  if (downPressed) {
    calSubIndex = (calSubIndex + 1) % CAL_MENU_ITEMS;
    showIRCalibrationMenu();
  }
  if (menuPressed) executeCalibrationSubmenu();
}

void applySimpleCurrentCalibration() {
  const float MIN_CAL_CURRENT = 0.020f;  // do not calibrate with near-zero current

  float measuredI = current[calChannel];       // already calibrated live value
  float oldCalI = calI[calChannel];
  float oldCalP = calP[calChannel];

  if (isnan(measuredI) || measuredI < MIN_CAL_CURRENT || calRefCurrent < MIN_CAL_CURRENT || oldCalI <= 0.0f) {
    tft.fillRect(5, 68, 150, 12, ST77XX_BLACK);
    tft.setCursor(5, 68);
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.println("ERR: current too low");
    return;
  }

  // current[] is already calibrated, so first recover the raw PZEM current.
  float rawI = measuredI / oldCalI;
  if (rawI < MIN_CAL_CURRENT) {
    tft.fillRect(5, 68, 150, 12, ST77XX_BLACK);
    tft.setCursor(5, 68);
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.println("ERR: raw current low");
    return;
  }

  float newCalI = calRefCurrent / rawI;
  if (newCalI < 0.100f) newCalI = 0.100f;
  if (newCalI > 10.000f) newCalI = 10.000f;

  lastCalRatio = newCalI / oldCalI;
  calI[calChannel] = newCalI;

  // With a resistive load and only a clamp-meter reference, apply the same ratio to power.
  // This keeps NILM current and power traces coherent without needing a wattmeter.
  calP[calChannel] = oldCalP * lastCalRatio;
  if (calP[calChannel] < 0.100f) calP[calChannel] = 0.100f;
  if (calP[calChannel] > 10.000f) calP[calChannel] = 10.000f;

  showCalSoft();
}

void saveCalAndNextChannel() {
  showSavingCalib();
  saveCalibration();
  showCalibSavedOK();
  delay(700);

  calChannel = (calChannel + 1) % 6;
  // Use the current measured value as the next starting reference when it is plausible.
  if (!isnan(current[calChannel]) && current[calChannel] > 0.020f) {
    calRefCurrent = current[calChannel];
  }
  showCalSoft();
}

void handleCalSoftWithFlags(bool upPressed, bool downPressed, bool menuPressed) {
  static unsigned long okHoldStart = 0;
  static bool okLongDone = false;

  // IR # key requests save+next. This is a reliable equivalent to long OK on remotes
  // which do not always send a normal repeat code.
  if (calSaveNextRequested) {
    calSaveNextRequested = false;
    saveCalAndNextChannel();
    return;
  }

  // Physical OK/MENU long press: save and move to next channel.
  if (digitalRead(BUTTON_MENU_PIN) == LOW) {
    if (okHoldStart == 0) okHoldStart = millis();
    if (!okLongDone && millis() - okHoldStart > 900) {
      okLongDone = true;
      saveCalAndNextChannel();
      return;
    }
  } else {
    okHoldStart = 0;
    okLongDone = false;
  }

  if (upPressed) {
    calRefCurrent += 0.010f;
    if (calRefCurrent > 100.000f) calRefCurrent = 100.000f;
    showCalSoft();
    return;
  }

  if (downPressed) {
    calRefCurrent -= 0.010f;
    if (calRefCurrent < 0.000f) calRefCurrent = 0.000f;
    showCalSoft();
    return;
  }

  // Short OK: apply calibration factor, but do not save yet.
  if (menuPressed && !okLongDone) {
    applySimpleCurrentCalibration();
    return;
  }

  unsigned long now = millis();
  if (now - lastCalMeasureTime >= CAL_MEASURE_INTERVAL) {
    lastCalMeasureTime = now;
    float newValue = current[calChannel];
    if (fabs(newValue - prevCalLiveValue) >= 0.005f) {
      showCalSoft();
      prevCalLiveValue = newValue;
    }
  }
}

void handleGraphsMenuWithFlags(bool upPressed, bool downPressed, bool menuPressed) {
  if (upPressed) {
    graphSubIndex = (graphSubIndex + GRAPH_MENU_ITEMS - 1) % GRAPH_MENU_ITEMS;
    showIRGraphsMenu();
  }
  if (downPressed) {
    graphSubIndex = (graphSubIndex + 1) % GRAPH_MENU_ITEMS;
    showIRGraphsMenu();
  }
  if (menuPressed) executeGraphsSubmenu();
}

void handleIRShortcutCode(uint32_t code) {
  auto goToMenu = [&]() {
    currentState = STATE_MENU;
    redraw = true;
  };

  auto goToMeasures = [&]() {
    currentState = STATE_MEASURES;
    forceFullRedraw = true;
    needsDisplayUpdate = true;
    redraw = true;
    initLayout();
  };

  // Global exit key: always returns to measurements.
  if (code == 0xFF6897) {  // *
    goToMeasures();
    return;
  }

  // Global Back key: always returns to the main menu.
  if (code == 0xFF9867) {  // 0
    goToMenu();
    return;
  }

  if (currentState == STATE_IR_CAL_MENU) {
    switch (code) {
      case 0xFFA25D:
        calSubIndex = 0;
        executeCalibrationSubmenu();
        return;  // 1
      case 0xFF629D:
        calSubIndex = 1;
        executeCalibrationSubmenu();
        return;  // 2
      case 0xFFE21D:
        calSubIndex = 2;
        executeCalibrationSubmenu();
        return;  // 3
      case 0xFF22DD:
        calSubIndex = 3;
        executeCalibrationSubmenu();
        return;  // 4
      case 0xFF02FD:
        calSubIndex = 4;
        executeCalibrationSubmenu();
        return;                                            
      case 0xFF38C7: executeCalibrationSubmenu(); return;  // OK
      default: return;                                     // UP/DOWN are processed as flags in the loop
    }
  }

  if (currentState == STATE_CAL_SOFT) {
    if (code == 0xFFB04F) {  // # = save calibration and move to next channel
      calSaveNextRequested = true;
      return;
    }
    // UP/DOWN/OK are converted to flags below by processIRRemote().
    return;
  }

  if (currentState == STATE_IR_GRAPH_MENU) {
    switch (code) {
      case 0xFFA25D:
        graphSubIndex = 0;
        executeGraphsSubmenu();
        return;  // 1
      case 0xFF629D:
        graphSubIndex = 1;
        executeGraphsSubmenu();
        return;  // 2
      case 0xFFE21D:
        graphSubIndex = 2;
        executeGraphsSubmenu();
        return;  // 3
      case 0xFF22DD:
        graphSubIndex = 3;
        executeGraphsSubmenu();
        return;                                       
      case 0xFF38C7: executeGraphsSubmenu(); return;  // OK
      default: return;                                // UP/DOWN are processed as flags in the loop
    }
  }

  switch (code) {
    case 0xFFA25D:  // 1
      goToMeasures();
      return;
    case 0xFF629D:  // 2
      currentState = STATE_SDMEM;
      showSDmem();
      return;
    case 0xFFE21D:  // 3
      currentState = STATE_REFRESH_CONFIG;
      showRefreshConfig();
      return;
    case 0xFF22DD:  // 4
      currentState = STATE_WIFIINFO;
      showWifiInfo();
      return;
    case 0xFF02FD:  // 5
      currentState = STATE_PZEM_STATUS;
      showPzemStatus();
      return;
    case 0xFFC23D:  // 6
      currentState = STATE_IR_CAL_MENU;
      calSubIndex = 0;
      showIRCalibrationMenu();
      return;
    case 0xFFE01F:  // 7
      currentState = STATE_IR_GRAPH_MENU;
      graphSubIndex = 0;
      showIRGraphsMenu();
      return;
    case 0xFFA857:  // 8
      currentState = STATE_WEB_SERVER;
      if (!serverStarted) initWebServer();
      showWebServerIP();
      redraw = true;
      return;
    default:
      return;
  }
}


void showRefreshConfig() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(5, 0);
  tft.println("=== REFRESH RATE ===");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(5, 20);
  tft.print("Current: ");
  tft.print(measureInterval);
  tft.println(" ms");
}

void showSavingCalib() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 40);
  tft.println("SAVING CALIB...");
}

void showCalibSavedOK() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(10, 40);
  tft.println("CALIBRATION SAVED");
}

void showCurrentCalib() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(5, 0);
  tft.println("CURRENT CALIBRATION");
  for (int i = 0; i < 6; i++) {
    tft.setCursor(5, 18 + i * 10);
    tft.print("M");
    tft.print(i);
    tft.print(" I:");
    tft.print(calI[i], 3);
    tft.print(" P:");
    tft.print(calP[i], 3);
  }
}


// ================== CALIBRATION ==================
void initCalFactors() {
  calFactors[0] = calV;
  calFactors[1] = calI;
  calFactors[2] = calP;
}
void resetCalibrationDefaults() {
  for (int i = 0; i < 6; i++) {
    calV[i] = 1;//0.9995;
    calI[i] = 1;//0.934;
    calP[i] = 1;//0.928;
  }
}

void saveCalibration() {
  File calibFile = SD.open("/calib.json", FILE_WRITE);
  if (!calibFile) return;
  DynamicJsonDocument doc(1024);
  JsonArray calV_json = doc.createNestedArray("calV");
  JsonArray calI_json = doc.createNestedArray("calI");
  JsonArray calP_json = doc.createNestedArray("calP");
  for (int i = 0; i < 6; i++) {
    calV_json.add(calV[i]);
    calI_json.add(calI[i]);
    calP_json.add(calP[i]);
  }
  doc["timestamp"] = timeClient.getFormattedTime();
  serializeJson(doc, calibFile);
  calibFile.close();
  // SERIAL_DISABLED: Serial.println("Calibration saved");
}

void loadCalibration() {
  File calibFile = SD.open("/calib.json");
  if (!calibFile) {
    Serial.println("[CAL] calib.json NOT_FOUND");
    return;
  }
  DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, calibFile);
  calibFile.close();
  if (err) {
    Serial.print("[CAL] JSON_ERR=");
    Serial.println(err.c_str());
    return;
  }
  JsonArray calV_json = doc["calV"], calI_json = doc["calI"], calP_json = doc["calP"];
  for (int i = 0; i < 6; i++) {
    calV[i] = calV_json[i];
    calI[i] = calI_json[i];
    calP[i] = calP_json[i];
  }
  Serial.println("[CAL] OK");
}

float readCalibrationValue(int type, int channel) {
  float value = 0.0;

  switch (channel) {
    case 0:
      if (type == 0) value = pzem0.voltage();
      else if (type == 1) value = pzem0.current();
      else if (type == 2) value = pzem0.power();
      break;
    case 1:
      if (type == 0) value = pzem1.voltage();
      else if (type == 1) value = pzem1.current();
      else if (type == 2) value = pzem1.power();
      break;
    case 2:
      if (type == 0) value = pzem2.voltage();
      else if (type == 1) value = pzem2.current();
      else if (type == 2) value = pzem2.power();
      break;
    case 3:
      if (type == 0) value = pzem3.voltage();
      else if (type == 1) value = pzem3.current();
      else if (type == 2) value = pzem3.power();
      break;
    case 4:
      if (type == 0) value = pzem4.voltage();
      else if (type == 1) value = pzem4.current();
      else if (type == 2) value = pzem4.power();
      break;
    case 5:
      if (type == 0) value = pzem5.voltage();
      else if (type == 1) value = pzem5.current();
      else if (type == 2) value = pzem5.power();
      break;
    default:
      return 0.0;
  }

  if (isnan(value)) return 0.0;

  switch (type) {
    case 0: return value * calV[channel];
    case 1: return value * calI[channel];
    case 2: return value * calP[channel];
    default: return 0.0;
  }
}

void updateCalSoftValue() {
  // Kept for compatibility with older calibration code paths.
  // Simple current calibration refresh is handled inside handleCalSoftWithFlags().
}

void verifyAddresses() {
  // SERIAL_DISABLED: Serial.println("\n🔍 Verifying PZEM addresses:");
  // SERIAL_DISABLED: Serial.println("═══════════════════════════════════════");

  uint8_t readAddr0 = pzem0.readAddress();
  // SERIAL_DISABLED: Serial.printf("M0: read 0x%02X - expected 0x%02X ", readAddr0, addr[0]);
  // SERIAL_DISABLED: Serial.println(readAddr0 == addr[0] ? "✅ OK" : "❌ INCORRECT");
  delay(200);

  uint8_t readAddr1 = pzem1.readAddress();
  // SERIAL_DISABLED: Serial.printf("M1: read 0x%02X - expected 0x%02X ", readAddr1, addr[1]);
  // SERIAL_DISABLED: Serial.println(readAddr1 == addr[1] ? "✅ OK" : "❌ INCORRECT");
  delay(200);

  uint8_t readAddr2 = pzem2.readAddress();
  // SERIAL_DISABLED: Serial.printf("M2: read 0x%02X - expected 0x%02X ", readAddr2, addr[2]);
  // SERIAL_DISABLED: Serial.println(readAddr2 == addr[2] ? "✅ OK" : "❌ INCORRECT");
  delay(200);

  uint8_t readAddr3 = pzem3.readAddress();
  // SERIAL_DISABLED: Serial.printf("M3: read 0x%02X - expected 0x%02X ", readAddr3, addr[3]);
  // SERIAL_DISABLED: Serial.println(readAddr3 == addr[3] ? "✅ OK" : "❌ INCORRECT");
  delay(200);

  uint8_t readAddr4 = pzem4.readAddress();
  // SERIAL_DISABLED: Serial.printf("M4: read 0x%02X - expected 0x%02X ", readAddr4, addr[4]);
  // SERIAL_DISABLED: Serial.println(readAddr4 == addr[4] ? "✅ OK" : "❌ INCORRECT");
  delay(200);

  uint8_t readAddr5 = pzem5.readAddress();
  // SERIAL_DISABLED: Serial.printf("M5: read 0x%02X - expected 0x%02X ", readAddr5, addr[5]);
  // SERIAL_DISABLED: Serial.println(readAddr5 == addr[5] ? "✅ OK" : "❌ INCORRECT");

  // SERIAL_DISABLED: Serial.println("═══════════════════════════════════════\n");
}

void handleExport() {
  String html = FPSTR(EXPORT_HTML);
  String meters = "<div style='display:flex; flex-wrap:wrap; gap:10px;'>";
  for (int i = 0; i < LOG_FILE_COUNT; i++) {
    if (SD.exists(fileName[i])) {
      File f = SD.open(fileName[i], FILE_READ);
      float kb = f.size() / 1024.0;
      f.close();
      meters += "<a href='/download" + fileName[i] + "' class='btn' style='padding:8px;";
      if (i == 6) meters += " background:#27ae60;";
      meters += "'>M" + String(i) + " (" + String(kb, 1) + "KB)</a>";
    }
  }
  meters += "</div>";
  html.replace("__METER_BUTTONS__", meters);
  server.send(200, "text/html", html);
}

void handleSDInfo() {
  String html = FPSTR(SDINFO_HTML);
  uint64_t totalBytes = SD.totalBytes(), usedBytes = SD.usedBytes();
  float totalMB = totalBytes / (1024.0 * 1024.0), usedMB = usedBytes / (1024.0 * 1024.0);
  int usedPercent = totalBytes > 0 ? (usedBytes * 100) / totalBytes : 0;
  String files = "<ul>";
  File root = SD.open("/");
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) files += "<li>" + String(file.name()) + " - " + String(file.size() / 1024.0, 1) + " KB</li>";
    file = root.openNextFile();
  }
  files += "</ul>";
  html.replace("__TOTAL__", String(totalMB, 1));
  html.replace("__USED__", String(usedMB, 1));
  html.replace("__FREE__", String((totalBytes - usedBytes) / (1024.0 * 1024.0), 1));
  html.replace("__PERCENT__", String(usedPercent));
  html.replace("__FILES__", files);
  server.send(200, "text/html", html);
}

void handleDownload() {
  flushLogFiles(true);
  String path = server.uri();
  path.replace("/download", "");
  if (SD.exists(path)) {
    File file = SD.open(path, FILE_READ);
    if (file) {
      server.sendHeader("Content-Type", "application/octet-stream");
      server.sendHeader("Content-Disposition", "attachment; filename=\"" + path.substring(1) + "\"");
      server.streamFile(file, "application/octet-stream");
      file.close();
    } else server.send(404, "text/plain", "Error opening file");
  } else server.send(404, "text/plain", "File not found");
}

void handleView() {
  flushLogFiles(true);
  String path = server.uri();
  path.replace("/view", "");
  if (SD.exists(path)) {
    File file = SD.open(path, FILE_READ);
    if (file) {
      String content = "<html><body><pre>";
      int lineCount = 0;
      while (file.available() && lineCount < 500) {
        content += (char)file.read();
        if (content.endsWith("\n")) lineCount++;
      }
      content += "</pre></body></html>";
      file.close();
      server.send(200, "text/html", content);
    } else server.send(404, "text/plain", "Error");
  } else server.send(404, "text/plain", "Not found");
}

void handleRestart() {
  String html = "<html><head><meta http-equiv='refresh' content='3;url=/'></head><body><h1>🔄 Restarting...</h1></body></html>";
  server.send(200, "text/html", html);
  delay(1000);
  ESP.restart();
}

void handleExportAll() {
  flushLogFiles(true);
  String csv = "timestamp,meter,VLN,A,W,f,PF,T,H\n";
  for (int meter = 0; meter < LOG_FILE_COUNT; meter++) {
    if (SD.exists(fileName[meter])) {
      File file = SD.open(fileName[meter], FILE_READ);
      if (file) {
        file.readStringUntil('\n');
        while (file.available()) {
          String line = file.readStringUntil('\n');
          if (line.length() > 0) {
            int comma = line.indexOf(',');
            if (comma > 0) csv += line.substring(0, comma) + "," + String(meter) + "," + line.substring(comma + 1) + "\n";
          }
        }
        file.close();
      }
    }
  }
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=ompm_all_data.csv");
  server.send(200, "text/csv", csv);
}

void handleExportCSV() {
  handleExportAll();
}
void handleConfig() {
  server.send(200, "text/html", "<html><body><h1>⚙️ Configuration</h1><p>In development</p><a href='/'>Back</a></body></html>");
}
void handleTest() {
  server.send(200, "text/html", "<html><body><h1>✅ Server OK</h1><p>IP: " + WiFi.localIP().toString() + "</p></body></html>");
}

// ========== SAFE SYSTEM RESTART WITH LOG ==========
void safeRestart() {
  // SERIAL_DISABLED: Serial.println("\n⚠️⚠️⚠️ STARTING SCHEDULED RESTART ⚠️⚠️⚠️");

  // 0. LOG THE RESTART TO SD (BEFORE SHUTTING ANYTHING DOWN!)
  logRestartToSD();

  // 1. Shut down network services properly
  if (serverStarted) {
    server.close();
    server.stop();
    // SERIAL_DISABLED: Serial.println("  - Web server stopped.");
  }

  if (otaEnabled) {
    ArduinoOTA.end();
    // SERIAL_DISABLED: Serial.println("  - OTA stopped.");
  }

  if (mqttEnabled && mqttClient.connected()) {
    mqttClient.disconnect();
    // SERIAL_DISABLED: Serial.println("  - MQTT disconnected.");
  }

  // 2. Display message on TFT
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(15, 50);
  tft.println("Scheduled restart");
  tft.setCursor(25, 65);
  tft.println("00:00 hours...");
  delay(2000);

  // 3. Force pending data write to SD
  if (sdOK) {
    // SERIAL_DISABLED: Serial.println("  - Forcing final write to SD...");
    forceSDWrite();
    delay(500);
  }

  // 4. Small pause to let operations complete
  delay(1000);
  logSystemEvent("SCHEDULED_RESTART");
  // SERIAL_DISABLED: Serial.println("✅ Safe restart completed. Goodbye!");
  // SERIAL_DISABLED: Serial.flush(); // Ensure the last message is sent

  // 5. Restart!
  ESP.restart();
}

void handleAdmin() {
  String html = FPSTR(ADMIN_HTML);
  html.replace("__HOSTNAME__", wifiHostname);
  html.replace("__IP__", WiFi.localIP().toString());
  html.replace("__OTA_PORT__", String(otaPort));
  server.send(200, "text/html", html);
}

void handleDiagnostics() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>"
                "<title>Diagnostics - OMPM</title><style>"
                "body{font-family:Arial,sans-serif;background:#f4f6f8;padding:20px;color:#2c3e50;}"
                ".card{max-width:900px;margin:0 auto;background:white;padding:24px;border-radius:12px;box-shadow:0 4px 14px rgba(0,0,0,.1);}"
                ".ok{color:#27ae60;font-weight:bold;}.warn{color:#e67e22;font-weight:bold;}.btn{display:inline-block;margin:6px 6px 0 0;padding:10px 16px;background:#3498db;color:white;text-decoration:none;border-radius:8px;}"
                "pre{background:#f8f9fa;padding:12px;border-radius:8px;overflow:auto;}</style></head><body><div class='card'>"
                "<h1>📋 Diagnostics</h1>";

  html += "<p><strong>Hostname:</strong> " + wifiHostname + "</p>";
  html += "<p><strong>IP:</strong> " + WiFi.localIP().toString() + "</p>";
  html += "<p><strong>Heap free:</strong> " + String(ESP.getFreeHeap()) + " bytes</p>";
  html += "<p><strong>Minimum heap:</strong> " + String(ESP.getMinFreeHeap()) + " bytes</p>";
  html += "<p><strong>WiFi:</strong> <span class='";
  html += (WiFi.status() == WL_CONNECTED) ? "ok'>Connected" : "warn'>Not connected";
  html += "</span></p>";
  html += "<p><strong>SD:</strong> <span class='";
  html += sdOK ? "ok'>Initialized" : "warn'>Not available";
  html += "</span></p><pre>";

  if (sdOK) {
    for (int i = 0; i < (physicalAggregate ? 6 : 7); i++) {
      html += "M" + String(i) + ": ";
      html += SD.exists(fileName[i]) ? "CSV present" : "CSV absent";
      html += "\n";
    }
  } else {
    html += "SD not available\n";
  }

  html += "</pre><a class='btn' href='/admin'>Back to Admin</a><a class='btn' href='/'>Home</a></div></body></html>";
  server.send(200, "text/html", html);
}

void handleOTAWeb() {
  String html = FPSTR(OTAWEB_HTML);
  html.replace("__HOSTNAME__", wifiHostname);
  html.replace("__IP__", WiFi.localIP().toString());
  html.replace("__HEAP__", String(ESP.getFreeHeap()));
  server.send(200, "text/html", html);
}

void handleOTAStatus() {
  String html = FPSTR(OTA_HTML);
  html.replace("__HOSTNAME__", wifiHostname);
  html.replace("__IP__", WiFi.localIP().toString());
  html.replace("__OTA_PORT__", String(otaPort));
  server.send(200, "text/html", html);
}

// ================== INIT WEB SERVER ==================
void initWebServer() {
  if (serverStarted) return;

  server.on("/", handleRoot);
  server.on("/realtime", handleRealtime);

  // NEW LOGS PATHS
  
  server.on("/logs", handleLogsIndex);
  server.on("/reboot.log", handleRebootLog);
  server.on("/events.log", handleEventsLog);
  server.on("/clear/reboot.log", handleClearLog);
  server.on("/clear/events.log", handleClearLog);


  server.on("/export", handleExport);
  server.on("/sdinfo", handleSDInfo);
  server.on("/graphs", handleGraphs);
  server.on("/graphdata", handleGraphData);
  server.on("/download", handleDownload);
  server.on("/view", handleView);
  server.on("/restart", handleRestart);
  server.on("/exportall", handleExportAll);
  server.on("/exportcsv", handleExportCSV);
  server.on("/config", handleConfig);
  server.on("/test", handleTest);
  server.on("/admin", handleAdmin);
  server.on("/diagnostics", handleDiagnostics);
  server.on("/ota", handleOTAWeb);
  server.on("/ota/update", handleOTAStatus);

  for (int i = 0; i < LOG_FILE_COUNT; i++) {
    String dl = "/download" + fileName[i];
    server.on(dl.c_str(), []() {
      handleDownload();
    });
    String vw = "/view" + fileName[i];
    server.on(vw.c_str(), []() {
      handleView();
    });
  }

  server.onNotFound([]() {
    server.send(404, "text/plain", "404 Not Found");
  });
  Serial.println("[WEB] BEGIN");
  server.begin();
  serverStarted = true;
  Serial.println("[WEB] OK");
}



// ================== OTA ==================
void initOTA() {
  if (!otaEnabled) return;

  ArduinoOTA.setPort(otaPort);
  ArduinoOTA.setHostname(wifiHostname.c_str());

  ArduinoOTA.onStart([]() {
    otaUpdating = true;
    // SERIAL_DISABLED: Serial.println("🚀 OTA started");
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(10, 40);
    tft.println("UPDATING...");
  });

  ArduinoOTA.onEnd([]() {
    otaUpdating = false;
    // SERIAL_DISABLED: Serial.println("✅ OTA completed");
  });

  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    otaUpdating = true;
    int perc = (p * 100) / t;
    if (perc % 10 == 0 || millis() - otaLastProgress > 500) {
      // SERIAL_DISABLED: Serial.printf("Progress: %u%%\r", perc);
      otaLastProgress = millis();
      updateOTAProgress(perc);
    }
  });

  ArduinoOTA.onError([](ota_error_t e) {
    otaUpdating = false;
    // SERIAL_DISABLED: Serial.printf("❌ OTA Error: %u\n", e);
  });

  ArduinoOTA.begin();
  // SERIAL_DISABLED: Serial.println("✅ OTA ready - Port: " + String(otaPort) + " - Host: " + wifiHostname);
}




// ==================  MQTT FUNCTIONS==================
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  // SERIAL_DISABLED: Serial.print("📨 MQTT message received [");
  // SERIAL_DISABLED: Serial.print(topic);
  // SERIAL_DISABLED: Serial.print("]: ");

  // Convert payload to a string
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  // SERIAL_DISABLED: Serial.println(message);

  // Procesar comandos simples
  if (String(topic) == "ompm/command") {
    if (message == "restart") {
      // SERIAL_DISABLED: Serial.println("🔄 Restarting by MQTT command");
      ESP.restart();
    } else if (message == "status") {
      publishMqttStatus();
    }
  }
}

void publishMqttStatus() {
  if (!mqttEnabled || WiFi.status() != WL_CONNECTED) return;

  // Verify MQTT connection
  if (!mqttClient.connected()) {
    String clientId = "OMPM-" + String(random(0xffff), HEX);

    // SERIAL_DISABLED: Serial.print("🔄 Connecting to MQTT... ");
    bool connected;

    if (mqttUser.length() > 0) {
      connected = mqttClient.connect(clientId.c_str(), mqttUser.c_str(), mqttPassword.c_str());
    } else {
      connected = mqttClient.connect(clientId.c_str());
    }

    if (connected) {
      // SERIAL_DISABLED: Serial.println("✅ Connected");
      // Suscribirse a comandos
      mqttClient.subscribe("ompm/command");
    } else {
      // SERIAL_DISABLED: Serial.println("❌ Failed");
      return;
    }
  }

  // SERIAL_DISABLED: Serial.println("📤 Publishing MQTT status...");

  // Create JSON with the data
  DynamicJsonDocument doc(1536);  // Enough for all data

  doc["timestamp"] = timeClient.getEpochTime();
  doc["mode"] = modeName;
  doc["free_heap"] = ESP.getFreeHeap();

  // WiFi information
  JsonObject wifiInfo = doc.createNestedObject("wifi");
  wifiInfo["rssi"] = WiFi.RSSI();
  wifiInfo["ip"] = WiFi.localIP().toString();

  // Meter data.
  JsonArray meters = doc.createNestedArray("meters");
  int numMeters = physicalAggregate ? 6 : 7;

  for (int i = 0; i < numMeters; i++) {
    JsonObject meter = meters.createNestedObject();
    meter["index"] = i;
    meter["voltage"] = voltage[i];
    meter["current"] = current[i];
    meter["power"] = power[i];
    meter["frequency"] = frequencyArr[i];
    meter["pf"] = pfArr[i];
  }

  // Totales
  float totalPower = 0;
  if (physicalAggregate) {
    totalPower = power[0];
  } else {
    for (int i = 0; i < 6; i++) totalPower += power[i];
  }
  doc["total_power"] = totalPower;

  String payload;
  serializeJson(doc, payload);

  if (mqttClient.publish(mqttTopic.c_str(), payload.c_str())) {
    // SERIAL_DISABLED: Serial.printf("✅ Published to %s (%d bytes)\n", mqttTopic.c_str(), payload.length());
  } else {
    // SERIAL_DISABLED: Serial.println("❌ Publish failed");
  }
}

// ========== SYSTEM EVENT LOGGING ==========
void logSystemEvent(const char* eventName) {
  if (!sdOK) return;

  File logFile = SD.open("/events.log", FILE_APPEND);
  if (!logFile) {
    logFile = SD.open("/events.log", FILE_WRITE);
    if (logFile) {
      logFile.println("timestamp,date_hour,eventName,heap");
      logFile.close();
      logFile = SD.open("/events.log", FILE_APPEND);
    }
  }

  if (logFile) {
    unsigned long epoch = timeClient.getEpochTime();
    String dateTimeStr = "unknown";

    if (epoch > 1672531200) {
      struct tm* tmInfo = localtime((time_t*)&epoch);
      char buffer[30];
      sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
              tmInfo->tm_year + 1900, tmInfo->tm_mon + 1, tmInfo->tm_mday,
              tmInfo->tm_hour, tmInfo->tm_min, tmInfo->tm_sec);
      dateTimeStr= String(buffer);
    }

    logFile.print(epoch);
    logFile.print(",");
    logFile.print(dateTimeStr);
    logFile.print(",");
    logFile.print(eventName);
    logFile.print(",");
    logFile.println(ESP.getFreeHeap());

    logFile.close();
  }
}


const char* irKeyName(uint32_t code) {
  switch (code) {
    case 0xFFA25D: return "1";
    case 0xFF629D: return "2";
    case 0xFFE21D: return "3";
    case 0xFF22DD: return "4";
    case 0xFF02FD: return "5";
    case 0xFFC23D: return "6";
    case 0xFFE01F: return "7";
    case 0xFFA857: return "8";
    case 0xFF906F: return "9";
    case 0xFF6897: return "*";
    case 0xFF9867: return "0";
    case 0xFFB04F: return "#";
    case 0xFF18E7: return "UP";
    case 0xFF10EF: return "LEFT";
    case 0xFF38C7: return "OK";
    case 0xFF5AA5: return "RIGHT";
    case 0xFF4AB5: return "DOWN";
    default: return "?";
  }
}

bool processIRRemote(bool& upPressed, bool& downPressed, bool& menuPressed) {
  if (!irrecv.decode(&irResults)) return false;

  uint32_t code = (uint32_t)irResults.value;
  irrecv.resume();

  if (code == 0 || code == 0xFFFFFFFF) return false;

  unsigned long now = millis();
  if (code == lastIRCode && (now - lastIRTime) < IR_DEBOUNCE_MS) return false;
  lastIRCode = code;
  lastIRTime = now;

  registerUserInteraction("IR");
  SystemState stateBeforeIR = currentState;
  handleIRShortcutCode(code);

  bool okConsumedBySubmenu = (code == 0xFF38C7) && (stateBeforeIR == STATE_IR_CAL_MENU || stateBeforeIR == STATE_IR_GRAPH_MENU);

  switch (code) {
    case 0xFF18E7: upPressed = true; break;    // UP
    case 0xFF4AB5: downPressed = true; break;  // DOWN
    case 0xFF38C7:                             // OK
      if (!okConsumedBySubmenu) menuPressed = true;
      break;
    case 0xFF5AA5:  // RIGHT
    case 0xFF10EF:  // LEFT
      menuPressed = true;
      break;
    default:
      break;
  }

  // SERIAL_DISABLED: Serial.print("IR 0x");
  // SERIAL_DISABLED: Serial.print(code, HEX);
  // SERIAL_DISABLED: Serial.print(" -> ");
  // SERIAL_DISABLED: Serial.println(irKeyName(code));
  return true;
}

void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== OMPM V68 BOOT ===");
  esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.print("[RESET] reason=");
  Serial.println((int)resetReason);
  delay(5);

  // ========== PIN INITIALIZATION ==========
  Serial.println("[BOOT] PINS");
  delay(5);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MENU_PIN, INPUT_PULLUP);
  pinMode(IR_PIN, INPUT);
  irrecv.enableIRIn();
  Serial.println("[BOOT] IR_OK");

  // ========== CONFIGURAR LED INTERNO ==========
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // LED on at startup

  pinMode(TFT_BL_CTRL, OUTPUT);
  digitalWrite(TFT_BL_CTRL, LOW);

  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  deselectSPIDevices();

  // ========== SD INITIALIZATION FIRST ==========
  // SD is initialized before TFT to avoid TFT/SPI state affecting the card.
  Serial.println("[BOOT] SD_FIRST");
  sdOK = beginSDWithRetries();

  // ========== TFT SPI INITIALIZATION ==========
  // Do NOT call global SPI.begin(14, 19, 13): GPIO19 is SD MISO.
  // TFT does not need MISO, so use a separate HSPI object with MISO=-1.
  Serial.println("[BOOT] SPI_TFT_HSPI");
  delay(5);
  deselectSPIDevices();
  spiTFT.begin(14, -1, 13, TFT_CS);
  delay(20);
  deselectSPIDevices();

  // ========== TFT INITIALIZATION ==========
  // SERIAL_DISABLED: Serial.println("Resetting TFT...");
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, LOW);
  delay(100);
  digitalWrite(TFT_RST, HIGH);
  delay(100);

  Serial.println("[BOOT] TFT");
  delay(5);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  Serial.println("[BOOT] TFT_OK");
  delay(5);

  // Initial visual test.
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 40);
  tft.println("TFT OK");
  tft.setTextSize(1);
  tft.setCursor(10, 70);
  tft.println("Esperando...");
  delay(500);
  bootStep("TFT_READY");

  // SERIAL_DISABLED: Serial.println("✅ TFT test completed");

  systemStartTime = millis();  //  Save start time

  // ========== SERIAL2 INITIALIZATION FOR PZEM ==========
  bootStep("PZEM_UART");
  Serial.println("[BOOT] PZEM_UART");
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("[BOOT] PZEM_UART_OK");

  // Do not open/create CSV or append logs before WiFi/NTP.
  // At boot the SD is used only to read config.json/calib.json.
  // Measurement CSV files are created later with fresh per-boot names.

  // ========== CONFIGURATION LOAD ==========
  bootStep("LOAD_CONFIG");
  Serial.println("[CFG] LOAD");
  bool cfgOK = loadConfig();
  Serial.print("[CFG] OK=");
  Serial.println(cfgOK ? "1" : "0");
  Serial.print("[CFG] SSID=");
  Serial.println(wifiSSID);
  Serial.print("[CFG] HOST=");
  Serial.println(wifiHostname);
  Serial.print("[CFG] DHCP=");
  Serial.println(wifiDHCP ? "1" : "0");

  // SERIAL_DISABLED: Serial.println("\n🔍 VERIFYING MODE AFTER LOAD:");
  // SERIAL_DISABLED: Serial.printf("  aggregateEnabled: %s\n", aggregateEnabled ? "true" : "false");
  // SERIAL_DISABLED: Serial.printf("  physicalAggregate: %s\n", physicalAggregate ? "true" : "false");
  // SERIAL_DISABLED: Serial.printf("  modeName: %s\n", modeName.c_str());
  // SERIAL_DISABLED: Serial.println("================================\n");

  // ========== WIFI INITIALIZATION ==========
  bootStep("WIFI");
  if (cfgOK && wifiSSID.length() > 0) {
    if (initWiFi()) {
      logSystemEvent("WIFI_CONNECTED");
    }
  } else {
    Serial.println("[WIFI] NO_CFG -> AP");
    WiFi.mode(WIFI_AP);
    bool apOK = WiFi.softAP(DEBUG_AP_SSID, DEBUG_AP_PASS);
    Serial.print("[AP] OK=");
    Serial.println(apOK ? "1" : "0");
    Serial.print("[AP] SSID=");
    Serial.println(DEBUG_AP_SSID);
    Serial.print("[AP] IP=");
    Serial.println(WiFi.softAPIP());
  }

  // ========== WELCOME SCREEN ==========
  bootStep("WELCOME");
  delay(250);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(35, 30);
  tft.println("OMPM v68");
  tft.setTextSize(1);
  tft.setCursor(25, 60);
  tft.println("Energy Monitor");
  tft.setCursor(35, 80);
  tft.println("v60 DUAL MODE");

  // Show the active hardware mode.
  tft.setTextColor(sdOK ? ST77XX_GREEN : ST77XX_RED);
  tft.setCursor(10, 100);
  tft.println(sdOK ? "SD READY" : "NO SD - AP MODE");


  delay(2000);

  // NTP INITIALIZATION
  bootStep("NTP");
  if (WiFi.status() == WL_CONNECTED) {
    // SERIAL_DISABLED: Serial.println("Initializing NTP...");
    timeClient.begin();
    timeClient.setTimeOffset(0);  // <-- previously you had 3600
    timeClient.setUpdateInterval(3600000);

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();

    for (int i = 0; i < 3; i++) {
      if (timeClient.update()) {
        // SERIAL_DISABLED: Serial.printf("NTP time UTC: %s\n", timeClient.getFormattedTime().c_str());
        break;
      } else {
        // SERIAL_DISABLED: Serial.println("NTP attempt failed, retrying...");
        delay(2000);
      }
    }
  }

  // ========== SESSION CSV FILES ==========
  // Only now, after config/WiFi/NTP, create fresh CSV filenames for this boot.
  // This avoids touching old/corrupt 0.csv..5.csv during startup and prevents a
  // damaged active log from blocking the next boot. The logging cadence remains
  // unchanged for NILM use.
  bootStep("CSV_SESSION");
  if (sdOK) {
    createSessionCsvFileNames();
    openLogFiles();
    logSystemEvent("Starting _system");
    String resetEvent = String("RESET_REASON_") + String((int)resetReason);
    logSystemEvent(resetEvent.c_str());
  }

  // ========== MQTT INITIALIZATION ==========
  bootStep("MQTT");
  if (mqttEnabled && WiFi.status() == WL_CONNECTED) {
    mqttClient.setServer(mqttServer.c_str(), mqttPort);
    mqttClient.setCallback(onMqttMessage);
    mqttClient.setBufferSize(2048);
    // SERIAL_DISABLED: Serial.printf("✅ MQTT configured: %s:%d\n", mqttServer.c_str(), mqttPort);
  }

  // ========== CALIBRATION ==========
  bootStep("CALIBRATION");
  loadCalibration();
  initCalFactors();

  // ========== WEB SERVER AND OTA ==========
  bootStep("WEB_SERVER");
  // Start web server also in AP mode so diagnostics remain available without SD/WiFi config.
  initWebServer();
  if (WiFi.status() == WL_CONNECTED && otaEnabled) initOTA();

  // ========== PZEM ADDRESS DIAGNOSTIC ==========
  // Recovery build: DO NOT query PZEM modules during setup.
  // If the RS485 bus or one PZEM is stuck, boot-time readAddress()/voltage()/current()
  // can starve the ESP32 task watchdog and create an endless boot loop.
  bootStep("PZEM_ADDR_SKIP");
  logSystemEvent("PZEM_ADDR_SKIP");
  delay(50);

  // ========== ARRAY INITIALIZATION ==========
  for (int i = 0; i < 7; i++) {
    voltage[i] = 0.0;
    current[i] = 0.0;
    power[i] = 0.0;
    frequencyArr[i] = 50.0;
    pfArr[i] = 1.0;
  }

  // ========== FINAL SETUP STEPS ==========
  bootStep("FINAL");
  lastPZEMReadTime = millis() - PZEM_READ_INTERVAL;
  currentPZEMIndex = 0;
  avgVoltage = 230.0;

  // ========== START SCREEN SAVER ==========
  screenSaverActive = false;
  screenForcedOn = false;
  lastUserInteraction = millis();

  // Force initial layout
  forceFullRedraw = true;
  needsDisplayUpdate = true;
  initLayout();

  // SERIAL_DISABLED: Serial.println("\n✅ System ready!");
  // SERIAL_DISABLED: Serial.printf("Free memory: %d bytes\n", ESP.getFreeHeap());
  // Do not scan/read existing CSV files at the end of setup.
  // Existing/corrupt logs must not be able to prevent a clean boot.
  // SERIAL_DISABLED: Serial.println("=== SETUP COMPLETED ===\n");
}

void loop() {
  unsigned long now = millis();

  static unsigned long lastHeartbeat = 0;
  if (now - lastHeartbeat > 5000) {
    Serial.println();
    Serial.println("[HB] alive");
    Serial.print("[HB] heap=");
    Serial.println(ESP.getFreeHeap());
    Serial.print("[HB] wifi=");
    Serial.println(WiFi.status());
    Serial.print("[HB] sd=");
    Serial.println(sdOK ? "1" : "0");
    lastHeartbeat = now;
  }

  // SD recovery: retry if the card was not ready at boot.
  static unsigned long lastSdRetry = 0;
  if (ENABLE_AUTO_SD_RETRY && !sdOK && now - lastSdRetry > 15000) {
    lastSdRetry = now;
    Serial.println("[SD] RETRY_SAFE");
    sdOK = beginSDWithRetries();
    Serial.print("[SD] RETRY_OK=");
    Serial.println(sdOK ? "1" : "0");
    if (sdOK) {
      openLogFiles();
      Serial.println("[CFG] RELOAD_AFTER_SD");
      bool cfgOK2 = loadConfig();
      Serial.print("[CFG] RELOAD_OK=");
      Serial.println(cfgOK2 ? "1" : "0");
      if (cfgOK2 && wifiSSID.length() > 0 && WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] RETRY_AFTER_SD");
        initWiFi();
      }
    }
  }

  // ========== INTERNAL LED MANAGEMENT ==========
  static unsigned long lastLedBlink = 0;
  static bool ledState = false;

  if (screenSaverActive) {
    // Slow blink when the screen saver is active (500ms ON, 500ms OFF)
    if (now - lastLedBlink >= 500) {
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
      lastLedBlink = now;
    }
  } else {
    // LED steady on when the screen is active
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // ========== START SCREEN SAVER ==========
  checkScreenSaver();
  updateDHT11();

  // ========== FAST AND STABLE NILM ACQUISITION (ROTATING RS485) ==========
  updateSharedVoltageFrequency(now);

  unsigned long readInterval = screenSaverActive ? PZEM_IDLE_READ_INTERVAL : PZEM_READ_INTERVAL;
  if (now - lastPZEMReadTime >= readInterval) {
    lastPZEMReadTime = now;

    readFastPZEMChannel(currentPZEMIndex, now);
    currentPZEMIndex = (currentPZEMIndex + 1) % PZEM_COUNT;

    calculateAggregate();
    if (!screenSaverActive && currentState == STATE_MEASURES) {
      needsDisplayUpdate = true;
    }
  }

  // ========== UPDATE DISPLAY ==========
  // Only update if:
  // 1. The screen saver is NOT active
  // 2. We are in STATE_MEASURES
  // 3. needsDisplayUpdate is true
  if (!screenSaverActive && currentState == STATE_MEASURES && needsDisplayUpdate && (now - lastDisplayRenderTime >= DISPLAY_REFRESH_INTERVAL)) {
    updateDisplay();
    needsDisplayUpdate = false;
    lastDisplayRenderTime = now;
  }

  // Watchdog disabled in this diagnostic version to avoid restarts due to low memory during tests




  // ========== WEB SERVER ==========
  if (serverStarted) {
    static unsigned long lastWeb = 0;
    if (millis() - lastWeb > 10) {
      server.handleClient();
      lastWeb = millis();
    }
  }

  // ========== OTA BACKGROUND ==========
  if (otaEnabled) ArduinoOTA.handle();

  // ========== BUTTONS ==========
  bool upPressed = buttonFallingEdge(btnUp);
  bool downPressed = buttonFallingEdge(btnDown);
  bool menuPressed = buttonFallingEdge(btnMenu);
  processIRRemote(upPressed, downPressed, menuPressed);

  // Register interaction if any button is pressed
  if (upPressed || downPressed || menuPressed) {
    registerUserInteraction("BUTTON");
  }

  // ========== SIMPLE CURRENT CALIBRATION SCREEN ==========
  if (currentState == STATE_CAL_SOFT) {
    if (redraw) {
      showCalSoft();
      redraw = false;
    }
    handleCalSoftWithFlags(upPressed, downPressed, menuPressed);
    return;
  }

  // ========== MENU SHORTCUT HANDLING ==========
  if (menuPressed) {
    if (currentState != STATE_MEASURES && currentState != STATE_MENU && currentState != STATE_IR_CAL_MENU && currentState != STATE_IR_GRAPH_MENU) {
      // SERIAL_DISABLED: Serial.printf("← Back to menu from state %d\n", currentState);
      currentState = STATE_MENU;
      redraw = true;
      return;
    }
  }

  // ========== NAVIGATION STATES ==========
  if (currentState == STATE_WEB_SERVER) {
    if (upPressed || downPressed || menuPressed) {
      currentState = STATE_MENU;
      redraw = true;
    }
    return;
  }

  if (currentState == STATE_IR_CAL_MENU) {
    if (redraw) {
      showIRCalibrationMenu();
      redraw = false;
    }
    handleCalibrationMenuWithFlags(upPressed, downPressed, menuPressed);
    return;
  }


  if (currentState == STATE_IR_GRAPH_MENU) {
    if (redraw) {
      showIRGraphsMenu();
      redraw = false;
    }
    handleGraphsMenuWithFlags(upPressed, downPressed, menuPressed);
    return;
  }

  if (currentState == STATE_MEASURES && menuPressed) {
    currentState = STATE_MENU;
    menuIndex = 0;
    redraw = true;
    return;
  }

  if (currentState == STATE_MENU) {
    if (redraw) {
      showMenu();
      redraw = false;
    }
    handleMenuWithFlags(upPressed, downPressed, menuPressed);
    return;
  }

  // ========== GRAPH STATES ==========
  if (currentState == STATE_CUR_GRAPH || currentState == STATE_PWR_GRAPH || currentState == STATE_CUR_HIST || currentState == STATE_PWR_HIST) {
    return;
  }

  // ========== SCHEDULED RESTART CHECK WITH FALLBACK ==========
  if (DAILY_RESTART_ENABLED && !RESTART_PENDING) {
    if (now - LAST_CHECK_RESTART > RESTART_CHECK_INTERVAL) {
      LAST_CHECK_RESTART = now;

      bool targetTimeReached = false;

      // ATTEMPT 1: Use NTP if available
      if (WiFi.status() == WL_CONNECTED && timeClient.isTimeSet()) {
        time_t rawTime = timeClient.getEpochTime();
        struct tm* localTimeInfo = localtime(&rawTime);

        if (localTimeInfo != nullptr) {
          int currentHour = localTimeInfo->tm_hour;
          int currentMinute = localTimeInfo->tm_min;

          if (currentHour == RESTART_HOUR && currentMinute == RESTART_MINUTE) {
            targetTimeReached = true;
            // SERIAL_DISABLED: Serial.printf("⏰ NTP time reached: %02d:%02d\n", currentHour, currentMinute);
          }
        }
      }
      // ATTEMPT 2: Fallback using `millis()` if NTP is unavailable but enough time has elapsed
      else {
        // Calculate how much time has passed since startup
        unsigned long elapsedTime = now - systemStartTime;

        // If more than 24 hours have passed and NTP is not available, force restart
        // (we assume it’s roughly the same time)
        if (elapsedTime >= MAX_TIME_WITHOUT_NTP && !RESTART_FALLBACK_ACTIVATED) {
          // SERIAL_DISABLED: Serial.println("⚠️ 24 hours without NTP - Forcing restart by fallback");
          targetTimeReached = true;
          RESTART_FALLBACK_ACTIVATED = true;  // Ensure the fallback restart is triggered only once.

        }

        // We can also try to restart at an approximate time
        // For example, if multiples of 24 hours have passed since startup
        if (!RESTART_FALLBACK_ACTIVATED && elapsedTime > 12UL * 60UL * 60UL * 1000UL) {
          // A 30-minute window around the target time
          unsigned long HoursSinceStart = elapsedTime / (60UL * 60UL * 1000UL);
          unsigned long cycleRemainder = elapsedTime % (24UL * 60UL * 60UL * 1000UL);

              // Trigger only during the first 30 minutes of a new 24-hour cycle.
          if (cycleRemainder < 30UL * 60UL * 1000UL) {
            // SERIAL_DISABLED: Serial.println("⚠️ Approximate fallback restart (window 30min)");
            targetTimeReached = true;
            RESTART_FALLBACK_ACTIVATED = true;
          }
        }
      }

      // If the condition is met (exact time or fallback), restart
      if (targetTimeReached) {
        RESTART_PENDING = true;
        safeRestart();
        return;
      }
    }
  }

  // ========== SD WRITE ==========
  if (sdOK) writeSDDeferred();

  // ========== REFRESH DISPLAY  ==========
  if (!screenSaverActive && currentState == STATE_MEASURES && needsDisplayUpdate && (now - lastDisplayRenderTime >= DISPLAY_REFRESH_INTERVAL)) {
    updateDisplay();
    needsDisplayUpdate = false;
    lastDisplayRenderTime = now;
  }

  // ========== MQTT ==========
  if (mqttEnabled && WiFi.status() == WL_CONNECTED) {
    mqttClient.loop();
    if (millis() - lastMqttPublish > MQTT_INTERVAL) {
      publishMqttStatus();
      lastMqttPublish = millis();
    }
  }

  yield();
  delay(5);
}
