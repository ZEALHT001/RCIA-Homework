#include "gpio.h"
#include "main.h"


namespace LightControl {

class Light {
private:
  GPIO_TypeDef *GPIOport;
  uint16_t GPIO_Pin;

public:
  Light(GPIO_TypeDef *port, uint16_t Pin) : GPIOport(port), GPIO_Pin(Pin) {}

protected:
  void open_light() const {
    HAL_GPIO_WritePin(GPIOport, GPIO_Pin, GPIO_PIN_SET);
  }
  void close_light() const {
    HAL_GPIO_WritePin(GPIOport, GPIO_Pin, GPIO_PIN_RESET);
  }

public:
  void contrl_light(uint8_t state) const {
    if (state == 1)
      open_light();
    else
      close_light();
  }
};
} // namespace LightControl
