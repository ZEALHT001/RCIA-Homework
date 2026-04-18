namespace LightControl
{

  class Light
  {
    public:
    struct GPIO
    {
      GPIO_TypeDef* GPIOx;
      uint16_t GPIO_Pin;
    } ;
  protected:
  

    void open_light(GPIO*GPIO)
    {
      HAL_GPIO_WritePin(GPIO->GPIOx, GPIO->GPIO_Pin, GPIO_PIN_SET);
    }
    void close_light(GPIO*GPIO)
    {
      HAL_GPIO_WritePin(GPIO->GPIOx, GPIO->GPIO_Pin, GPIO_PIN_RESET);
    }

  public:
    void contrl_light(GPIO*GPIO,uint8_t state)
    {
      if (state==1) open_light(GPIO);
      else close_light(GPIO);
    }
    
  };
}

