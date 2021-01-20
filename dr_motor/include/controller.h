#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <inttypes.h>
#include <stddef.h>
#include <vector>

#include <mbed.h>

class Controller {
  public:
    Controller(CAN&, const uint32_t);
    struct {int8_t x; int8_t y; int8_t z; int8_t rz;} axes = {};
    std::vector<bool> buttons;
    void setButtonEventListener(Callback<void(size_t, bool)>);
  private:
    CAN& can;
    const uint32_t canId;
    Callback<void(size_t, bool)> buttonCallback;
    void recieveData();
    void parse(const uint8_t*, const size_t);
};

#endif