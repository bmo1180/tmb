/*!
 * @version v1.0.0
 *              | v1.0.0 -> Tactile library with additional states.
 * 
 * @note    This is a library written in C++ using Arduino Core to implement
 *          button press event efficeintly.
 *  
 */

//===== INCLUDES SECTION =====================================================//
#include "Tactile.h"
//============================================================================//

//===== CLASS SECTION ========================================================//
namespace IO {

    Tactile::Tactile(uint8_t pin) {
        mPin = pin;
    }

    void Tactile::loop(void) {
        uint8_t currentState = digitalRead(mPin);
        mState = TACTILE_STATE_NONE;

        // Press Event
        if(currentState == LOW && mPreviousState == HIGH) {
            mPressTime = millis();
            mNextPressingEvent = 0;
        }

        const uint32_t buttonTime = millis() - mPressTime;

        // Pressing
        if(currentState == LOW && buttonTime > TACTILE_PRESSING_TIME) {
            if(millis() > mNextPressingEvent) {
                mState = TACTILE_STATE_PRESSING;
                mNextPressingEvent = millis() + 500;
            }
        }

        // Release Event
        if(currentState == HIGH && mPreviousState == LOW) {
            if(buttonTime > TACTILE_OVER_PRESS_TIME) {
                mState = TACTILE_STATE_OVER;
            } else if(buttonTime > TACITLE_LONG_PRESS_TIME) {
                mState = TACTILE_STATE_LONG;
            } else if(buttonTime > TACTILE_MIN_PRESS_TIME) {
                mState = TACTILE_STATE_SHORT;
            }
        }

        mPreviousState = currentState;
    }

    void Tactile::start(void) {
        pinMode(mPin, INPUT_PULLUP);
    }

    uint8_t Tactile::getState(void) {
        return mState;
    }

}
//============================================================================//
