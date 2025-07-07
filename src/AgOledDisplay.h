#ifndef _AG_OLED_DISPLAY_H_
#define _AG_OLED_DISPLAY_H_

#include "AgConfigure.h"
#include "AgValue.h"
#include "AirGradient.h"
#include "Main/PrintLog.h"
#include <Arduino.h>

class OledDisplay : public PrintLog {
private:
  Configuration &config;
  AirGradient *ag;
  bool isBegin = false;
  void *u8g2 = NULL;
  Measurements &value;
  bool isDisplayOff = false;

  typedef struct {
    int width;
    int height;
    unsigned char *icon;
  } xbm_icon_t;

  void showTempHum(bool hasStatus);
  void setCentralText(int y, String text);
  void setCentralText(int y, const char *text);
  void showIcon(int x, int y, xbm_icon_t *icon);

public:
  OledDisplay(Configuration &config, Measurements &value, Stream &log);
  ~OledDisplay();

  enum DashboardStatus {
    DashBoardStatusNone,
    DashBoardStatusWiFiIssue,
    DashBoardStatusServerIssue,
    DashBoardStatusAddToDashboard,
    DashBoardStatusDeviceId,
    DashBoardStatusOfflineMode,
  };

  void setAirGradient(AirGradient *ag);
  bool begin(void);
  void end(void);
  void setText(String &line1, String &line2, String &line3);
  void setText(const char *line1, const char *line2, const char *line3);
  void setText(String &line1, String &line2, String &line3, String &line4);
  void setText(const char *line1, const char *line2, const char *line3,
               const char *line4);
  void showDashboard(void);
  void showDashboard(DashboardStatus status);
  void setBrightness(int percent);
#ifdef ESP32
  void showFirmwareUpdateVersion(String version);
  void showFirmwareUpdateProgress(int percent);
  void showFirmwareUpdateSuccess(int count);
  void showFirmwareUpdateFailed(void);
  void showFirmwareUpdateSkipped(void);
  void showFirmwareUpdateUpToDate(void);
#else

#endif
  void showRebooting(void);
};

#endif /** _AG_OLED_DISPLAY_H_ */
