#include <Arduino.h>
#ifndef __BUTTON_H
#define __BUTTON_H


class Button
{
  private:
    enum BUTTON_STATE {BUTTON_STABLE, BUTTON_UNSTABLE};
    BUTTON_STATE state = BUTTON_STABLE;

    int8_t buttonPin = -1;
    int8_t buttonPos = 1;

    //assume active LOW
    uint8_t buttonPositon = HIGH;
    uint8_t tempButtonPos = HIGH;

    uint32_t lastBounceTime = 0;
    uint32_t debouncePeriod = 10; //in ms

  public:
    Button(uint8_t pin, uint32_t db = 10);
    void Init(bool usePullup = true);
    bool CheckButtonPress(void);


};

#endif
