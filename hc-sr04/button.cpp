#include "button.h"


Button::Button(uint8_t pin, uint32_t db)
{
  buttonPin = pin;
  debouncePeriod = db;

}

void Button::Init(bool usePullup = true) {
  if (usePullup) pinMode(buttonPin, INPUT_PULLUP);
  else pinMode(buttonPin, INPUT_PULLUP);
}

bool Button::CheckButtonPress(void) {
  tempButtonPos = buttonPos;
  buttonPos = digitalRead(buttonPin);

  switch (state) {
    case BUTTON_STABLE:
      if (buttonPos == tempButtonPos) {
        return false;
      }
      //change in button
      if (buttonPos != tempButtonPos) {
        state = BUTTON_UNSTABLE;
        lastBounceTime = millis();
        return false;
      }
      break;
    case BUTTON_UNSTABLE:
      //timer expires
      if (millis() >= lastBounceTime + debouncePeriod) {
        if (buttonPos == 0 && tempButtonPos == 0) {

          state = BUTTON_STABLE;

          return true;
        }
        else {
          state = BUTTON_STABLE;
          return false;
        }
      } else if (buttonPos != tempButtonPos) { //change in button
        lastBounceTime = millis();
        return false;
      }

      break;
  }


}





/* int8_t buttonPin = -1;

    enum BUTTON_STATE {BUTTON_STABLE, BUTTON_UNSTABLE};
    BUTTON_STATE state = BUTTON_STABLE;

    //assume active LOW
    uint8_t buttonPositon = HIGH;
    uint8_t tempButtonPos = HIGH;

    uint32_t lastBounceTime = 0;
    uint32_t debouncePeriod = 10; //in ms */
