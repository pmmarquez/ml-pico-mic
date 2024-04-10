#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW    5
#define EI_CLASSIFIER_SLICE_SIZE (EI_CLASSIFIER_RAW_SAMPLE_COUNT / EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)

#include <jarvis_inferencing.h>
#include <PDM.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

RS485Class _RS485(RS485_SERIAL_PORT, RS485_DEFAULT_TX_PIN, RS485_DEFAULT_DE_PIN, RS485_DEFAULT_RE_PIN);
ModbusRTUServerClass _ModbusRTUServer(_RS485);

/** Audio buffers, pointers and selectors */
typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

static inference_t inference;
static volatile bool record_ready = false;
// static signed short *sampleBuffer;
static signed short sampleBuffer[2048];
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);

/** Process control state machine **/
enum {
    PROCESS_IDLE = 0,
    PROCESS_MARVIN = 1,
    PROCESS_RECORDING = 2,
};
long process_state = PROCESS_IDLE;
unsigned long process_timer = 0;
void process(ei_impulse_result_t result);

/** Led control state machine **/
enum {
    LED_OFF = 0,
    LED_ON = 1,
    LED_BLINK_FAST = 2,
    LED_BLINK_SLOW = 3,
};
long led_last_state = LED_OFF;
long led_state = LED_OFF;
unsigned long led_timer = 0;
void led(void);

/**
 * @brief      Arduino setup function
 */
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    // while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf(" ms.\n");
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));

    run_classifier_init();
    if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
        ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }
}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */
void loop()
{
    bool m = microphone_inference_record();
    if (!m) {
        ei_printf("ERR: Failed to record audio...\n");
        return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR res = run_classifier_continuous(&signal, &result, debug_nn);
    if (res != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }

    if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)) {
        // print inference return code
        ei_printf("run_classifier returned: %d\r\n", res);
        process(result);
        print_inference_result(result);
        print_results = 0;
    }
}

#define SLAVE_ID 2

void setup1()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial2.setTX(PIN_SERIAL2_TX);
    Serial2.setRX(PIN_SERIAL2_RX);
    _ModbusRTUServer.begin(SLAVE_ID, 115200, SERIAL_8N1);
    _ModbusRTUServer.configureCoils(0x00, 1);
    _ModbusRTUServer.configureInputRegisters(0x00, 2);
    _ModbusRTUServer.configureHoldingRegisters(0x00, 2);
}

void loop1()
{
    int update_state_coil;
    long _process_state, _led_state;
    _ModbusRTUServer.poll();
    update_state_coil = _ModbusRTUServer.coilRead(0x00);
    _process_state = _ModbusRTUServer.holdingRegisterRead(0x00);
    _led_state = _ModbusRTUServer.holdingRegisterRead(0x01);
    if (update_state_coil == 1) {
        process_state = _process_state;
        led_state = _led_state;
    }
    _ModbusRTUServer.inputRegisterWrite(0x00, process_state);
    _ModbusRTUServer.inputRegisterWrite(0x01, led_state);
    led();
}

// process control state machine
void process(ei_impulse_result_t result) {
    switch (process_state) {
    case PROCESS_IDLE:
        if (result.classification[2].value > 0.6) { // marvin
            process_state = PROCESS_MARVIN;
            led_last_state = led_state;
            led_state = LED_ON;
            process_timer = millis();
        }
        break;
    case PROCESS_MARVIN:
        if (result.classification[3].value > 0.3 && led_last_state == LED_BLINK_FAST) { // off
            process_state = PROCESS_IDLE;
            led_state = LED_OFF;
            break;
        }
        if (result.classification[4].value > 0.4 && led_last_state == LED_OFF) { // on
            process_state = PROCESS_IDLE;
            led_state = LED_BLINK_FAST;
            break;
        }
        // if (result.classification[2].value > 0.5) { // forward
        //     process_state = PROCESS_RECORDING;
        //     set_led(LED_BLINK_SLOW);
        //     process_timer = millis();
        //     break;
        // }
        if (millis() - process_timer > 4000) {
            process_state = PROCESS_IDLE;
            led_state = led_last_state;
        }
        break;
    case PROCESS_RECORDING:
        if (millis() - process_timer > 5000) {
            process_state = PROCESS_IDLE;
            led_state = LED_OFF;
        }
        break;
    }
}

// led control function
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

/**
 * @brief      PDM buffer full callback
 *             Copy audio data to app buffers
 */
static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if ((inference.buf_ready == 0) && (record_ready == true)) {
        for(int i = 0; i < bytesRead>>1; i++) {
            inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

            if (inference.buf_count >= inference.n_samples) {
                inference.buf_select ^= 1;
                inference.buf_count = 0;
                inference.buf_ready = 1;
                break;
            }
        }
    }
}

/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);

    PDM.setBufferSize(2048);
    delay(250);

    // initialize PDM with:
    // - one channel (mono mode)
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("ERR: Failed to start PDM!");
        return false;
    }

    // optionally set the gain, defaults to 24
    // Note: values >=52 not supported
    PDM.setGain(50);

    record_ready = true;

    return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
    bool ret = true;

    if (inference.buf_ready == 1) {
        ei_printf(
            "Error sample buffer overrun. Decrease the number of slices per model window "
            "EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW is currently set to %d\n", EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
        ret = false;
    }

    while (inference.buf_ready == 0) {
        delay(1);
    }

    inference.buf_ready = 0;

    return ret;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    PDM.end();
    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
    record_ready = false;
}

void print_inference_result(ei_impulse_result_t result) {

    // Print how long it took to perform inference
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);

    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }

    ei_printf(" process: %s\r\n", process_state == PROCESS_IDLE ? "IDLE" : process_state == PROCESS_MARVIN ? "MARVIN" : "RECORDING");
    ei_printf(" led: %s\r\n", led_state == LED_OFF ? "OFF" : led_state == LED_ON ? "ON" : led_state == LED_BLINK_FAST ? "BLINK_FAST" : "BLINK_SLOW");

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
