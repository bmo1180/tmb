/*!
 * @version v1.0.0
 *              | v1.0.0 -> Tactile library with additional states.
 * 
 * @note    This is a library written in C++ using Arduino Core to implement
 *          button press event efficeintly.
 *  
 */
#ifndef __TACTILE_H__
#define __TACTILE_H__

//===== INCLUDES SECTIONS ====================================================//
#include <Arduino.h>
//============================================================================//

//===== DEFINITIONS SECTION ==================================================//
namespace IO {
    
    /*!
        Time duration of various press events
    */
    #define TACTILE_PRESSING_TIME       300
    #define TACTILE_MIN_PRESS_TIME      50
    #define TACITLE_LONG_PRESS_TIME     1000
    #define TACTILE_OVER_PRESS_TIME     3000

    /*!
        Tactile State Flags
    */
    enum TactileStateFlag {
        TACTILE_STATE_NONE,
        TACTILE_STATE_PRESSING,
        TACTILE_STATE_SHORT,
        TACTILE_STATE_LONG,
        TACTILE_STATE_OVER
    };

    /*!
     * @brief A class for handling tactile button efficienlty.
     */
    class Tactile {
        public:
            /*!
             * @brief Constructor of the class
             * @param [uint8_t] - Pin of the device to attach for polling
             */
            Tactile(uint8_t pin);

            /*!
             * @brief Polling operation for button press events.
             * @param None
             * @return None
             */
            void loop(void);

            /*!
             * @brief Initializes the button pin.
             * @param None
             * @return None
             */
            void start(void);

            /*!
             * @brief Gets the current state of the button.
             * @param None
             * @return [TactileStateFlag] - All relative flags based on pressed events
             */
            uint8_t getState(void);

        private:
            uint8_t mPin;
            uint8_t mPreviousState = HIGH;
            uint32_t mPressTime = 0;
            uint8_t mState = TACTILE_STATE_NONE;
            uint32_t mNextPressingEvent = 0;
    };

}
//============================================================================//

#endif
