#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

RS485Class _RS485(RS485_SERIAL_PORT, RS485_DEFAULT_TX_PIN, RS485_DEFAULT_DE_PIN, RS485_DEFAULT_RE_PIN);
ModbusRTUClientClass _ModbusRTUClient(_RS485);

/** AI Process control state machine **/
enum {
    PROCESS_IDLE = 0,
    PROCESS_MARVIN = 1,
    PROCESS_RECORDING = 2,
};
enum {
    LED_OFF = 0,
    LED_ON = 1,
    LED_BLINK_FAST = 2,
    LED_BLINK_SLOW = 3,
};
long process_state = PROCESS_IDLE;
long led_state = LED_OFF;

/** Master state machine **/
enum {
    GET_STATE = 0,
    SET_STATE = 1,
    CHECK_STATE = 2,
};
long master_state = GET_STATE;
unsigned long get_state_timer = 0;
#define SLAVE_NUMBER 2
int state_update_slave_id;

void updateSlave(int slave_id)
{
    Serial.printf("Update slave %d\n", slave_id);
    // Retry always if the write fails
    if (!_ModbusRTUClient.holdingRegisterWrite(slave_id, 0x00, process_state)) {
        delay(10);
        _ModbusRTUClient.holdingRegisterWrite(slave_id, 0x00, process_state);
    }
    delay(10);
    if (!_ModbusRTUClient.holdingRegisterWrite(slave_id, 0x01, led_state)) {
        delay(10);
        _ModbusRTUClient.holdingRegisterWrite(slave_id, 0x01, led_state);
    }
    delay(10);
    if (!_ModbusRTUClient.coilWrite(slave_id, 0x00, 1)) {
        delay(10);
        _ModbusRTUClient.coilWrite(slave_id, 0x00, 1);
    }
    delay(10);
    if (!_ModbusRTUClient.coilWrite(slave_id, 0x00, 0)) {
        delay(10);
        _ModbusRTUClient.coilWrite(slave_id, 0x00, 0);
    }
    delay(10);
}

void printUpdateFrom(int slave_id) {
    Serial.printf("Update from %d: state:%s, led:%s\n",
        slave_id,
        process_state == PROCESS_IDLE ? "IDLE" : process_state == PROCESS_MARVIN ? "MARVIN" : process_state == PROCESS_RECORDING ? "REC" : "ERROR",
        led_state == LED_OFF ? "OFF" : led_state == LED_ON ? "ON" : led_state == LED_BLINK_FAST ? "FAST" : led_state == LED_BLINK_SLOW ? "SLOW" : "ERROR");
    
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Modbus RTU Master");
    _ModbusRTUClient.begin(115200, SERIAL_8N1);

    // Reset all slaves two times
    for (int i = 1; i < (SLAVE_NUMBER + 1); i++)
        updateSlave(i);
}

void loop() {
    bool all_states_ok = true;
    long _process_state, _led_state;
    switch (master_state)
    {
    case GET_STATE:
        for (int i = 1; i < (SLAVE_NUMBER + 1); i++)
        {
            _process_state = _ModbusRTUClient.inputRegisterRead(i, 0x00);
            delay(10);
            _led_state = _ModbusRTUClient.inputRegisterRead(i, 0x01);
            delay(10);
            // Retry if the read fails
            if (_process_state == -1)
                _process_state = _ModbusRTUClient.inputRegisterRead(i, 0x00);
            delay(10);
            if (_led_state == -1)
                _led_state = _ModbusRTUClient.inputRegisterRead(i, 0x01);
            delay(10);

            Serial.printf("Slave %d: state:%s, led:%s\n",
                i,
                _process_state == PROCESS_IDLE ? "IDLE" : _process_state == PROCESS_MARVIN ? "MARVIN" : _process_state == PROCESS_RECORDING ? "REC" : "ERROR",
                _led_state == LED_OFF ? "OFF" : _led_state == LED_ON ? "ON" : _led_state == LED_BLINK_FAST ? "FAST" : _led_state == LED_BLINK_SLOW ? "SLOW" : "ERROR");
            Serial.printf("Master: state:%s, led:%s\n",
                process_state == PROCESS_IDLE ? "IDLE" : process_state == PROCESS_MARVIN ? "MARVIN" : process_state == PROCESS_RECORDING ? "REC" : "ERROR",
                led_state == LED_OFF ? "OFF" : led_state == LED_ON ? "ON" : led_state == LED_BLINK_FAST ? "FAST" : led_state == LED_BLINK_SLOW ? "SLOW" : "ERROR");

            if (_process_state != -1 && _led_state != -1) {
                if (process_state == PROCESS_MARVIN && _process_state == PROCESS_IDLE) {
                    process_state = PROCESS_IDLE;
                    if (led_state == LED_ON && _led_state == LED_BLINK_FAST) {
                        led_state = LED_BLINK_FAST;
                    } else {
                        led_state = LED_OFF;
                    }
                    printUpdateFrom(i);
                    master_state = SET_STATE;
                    break;
                }
                if (process_state == PROCESS_IDLE && _process_state == PROCESS_MARVIN) {
                    process_state = PROCESS_MARVIN;
                    led_state = LED_ON;
                    printUpdateFrom(i);
                    master_state = SET_STATE;
                    break;
                }
            }
        }
        break;
    case SET_STATE:
        // Write the process state and led state to all slaves
        Serial.printf("Update started\n");
        for (int i = 1; i < (SLAVE_NUMBER + 1); i++)
        {
            updateSlave(i);
        }
        Serial.printf("Update end\n");
        master_state = CHECK_STATE;
        break;
    case CHECK_STATE:
        Serial.printf("Check started\n");
        for (int i = 1; i < (SLAVE_NUMBER + 1); i++)
        {
            _process_state = _ModbusRTUClient.inputRegisterRead(i, 0x00);
            delay(10);
            _led_state = _ModbusRTUClient.inputRegisterRead(i, 0x01);
            delay(10);
            // Retry if the read fails
            if (_process_state == -1)
                _process_state = _ModbusRTUClient.inputRegisterRead(i, 0x00);
            delay(10);
            if (_led_state == -1)
                _led_state = _ModbusRTUClient.inputRegisterRead(i, 0x01);
            delay(10);

            if ((_process_state != process_state || _led_state != led_state) && (_process_state != -1 && _led_state != -1)) {
                all_states_ok = false;
                updateSlave(i);
            }
        }
        if (all_states_ok) {
            Serial.printf("Check OK\n");
            master_state = GET_STATE;
            get_state_timer = millis();
            break;
        }
        break;
    default:
        break;
    }
}

// Use second core to blink the LED
unsigned long led_timer = 0;
void led(void);

void setup1()
{
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop1()
{
    led();
}

void led(void) {
    switch (led_state) {
    case LED_OFF:
        digitalWrite(LED_BUILTIN, LOW);
        break;
    case LED_ON:
        digitalWrite(LED_BUILTIN, HIGH);
        break;
    case LED_BLINK_FAST:
        if (millis() - led_timer > 300) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            led_timer = millis();
        }
        break;
    case LED_BLINK_SLOW:
        if (millis() - led_timer > 1000) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            led_timer = millis();
        }
        break;
    }
}