#include "../../../furi/core/core_defines.h"

#include <furi.h>
#include <gui/gui.h>
#include <gui/view_port.h>
#include <input/input.h>
#include <gui/elements.h>
#include <furi_hal.h>
#include <furi_hal_i2c_config.h>
#include <furi_hal_resources.h>
#include <furi_hal_serial.h>
#include <stm32wbxx_ll_gpio.h>
#include <math.h>
#include <string.h>
#include <notification/notification_messages.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif
#include "drivers/i2c_bus.h"
#include "drivers/lsm303.h"
#include "drivers/gxhtc3c.h"
#include "drivers/kiisu_light_adc.h"

#define UART_BAUDRATE 115200
#define UART_RX_BUFFER_SIZE 64
#define DATA_COMMAND "data\n"

// Widget types
typedef enum {
    WidgetTypeAccelerometer,
    WidgetTypeMagnetometer,
    WidgetTypeTemperature,
    WidgetTypeHumidity,
    WidgetTypeLight,
    WidgetTypeCount
} WidgetType;

// App states
typedef enum {
    AppStateWidgetMenu,
    AppStateWidgetDetail,
    AppStateCalibrating
} AppState;

// Calibration mode (which sensor is being calibrated within the shared flow)
typedef enum {
    CalibModeMagnetometer = 0,
    CalibModeAccelerometer = 1,
} CalibMode;

// Widget structure
typedef struct {
    WidgetType type;
    char* title;
    bool enabled;
    bool ok;
    void* data;
    void (*draw_summary)(Canvas* canvas, int x, int y, int width, int height, void* data);
    void (*draw_detail)(Canvas* canvas, void* data);
} Widget;

// App structure
typedef struct {
    Gui* gui;
    ViewPort* vp;
    FuriMessageQueue* input_queue;
    I2cBus i2c;
    Lsm303 lsm;
    Gxhtc3c th;

    // Sensor data
    Lsm303Sample lsm_sample;
    Gxhtc3cSample th_sample;
    KiisuLightAdcSample light_sample;

    // UI state
    AppState state;
    int selected_widget;
    Widget widgets[WidgetTypeCount];

    // Polling timing
    uint32_t last_lsm_poll_ms;
    uint32_t last_th_poll_ms;
    uint32_t last_light_poll_ms;

    // Little Kiisu physics state (position in pixels, velocity px/s, angle rad, angular velocity rad/s)
    // Legacy single-sprite fields (unused now)
    float img_x;
    float img_y;
    float img_vx;
    float img_vy;
    float img_angle;
    float img_av;
    // New multi-sprite physics sim for accelerometer detail page 0
    int accel_detail_page; // 0 = gravity sim, 1 = raw data, 2 = level tool
    int sim_count; // number of sprites (5)
    int sim_w, sim_h; // sprite width/height in pixels
    float sim_r; // collision radius (circle approx)
    float sim_x[5], sim_y[5];
    float sim_vx[5], sim_vy[5];
    float sim_ang[5], sim_av[5];
    uint32_t last_physics_tick; // last tick for sim integration

    // Homescreen kitty (inside accelerometer box on widget menu)
    bool home_has_kitty; // enable a single kitty on homescreen
    float home_x, home_y; // center position in pixels
    float home_vx, home_vy; // velocity px/s
    float home_ang, home_av; // angle rad, angular velocity rad/s
    float home_r; // collision radius
    uint32_t home_last_tick; // last tick for homescreen kitty integration

    // Axis mapping hard-locked to +Z rotation (no fields needed)

    // Calibration data
    float mag_min_x, mag_max_x;
    float mag_min_y, mag_max_y;
    float mag_min_z, mag_max_z;
    // Computed calibration constants (from last completed calibration)
    bool mag_calibrated; // true if calibration constants below are valid
    float mag_offs[3]; // hard-iron offsets [uT]
    float mag_scale[3]; // soft-iron diagonal scale factors (unitless)
    bool mag_hw_offsets_applied; // if true, offsets are already applied in HW registers
    // Simple IIR smoothing for magnetometer (helps noise/LPF equivalence)
    float mag_filt[3];
    float mag_iir_alpha; // 0..1, fraction of new sample
    // Leveling (flat surface) baseline
    bool leveled;
    float level_ax, level_ay, level_az; // captured accel when flat
    uint32_t calibration_start_time;
    uint8_t calib_step; // 0=flat wait, 1=flat avg, 2=fig8 wait, 3=fig8 run, 4=compute, 5=complete
    uint32_t calib_duration_ms; // duration for current timed phase
    CalibMode calib_mode; // which sensor the flow is acting upon

    // Orientation: rotation of sensor frame to device frame (0, 90, 180, 270 degrees)
    uint8_t orientation_rot; // 0..3 multiples of 90°
    // PCB mounting quirk: swap X and Y from the sensor
    bool sensor_swap_xy;

    // Magnetometer detail view paging: 0 = Compass, 1 = Raw values
    int mag_detail_page;

    // Compass heading data
    float heading_deg; // instantaneous heading (0..360, 0 = North)
    float heading_deg_smooth; // smoothed heading
    float head_vec_x; // smoothing vector x = cos(heading)
    float head_vec_y; // smoothing vector y = sin(heading)
    // Heading configuration
    float mag_declination_deg; // local magnetic declination (deg), +East; adds to heading
    float heading_tilt_limit_deg; // do not update heading smoothing when tilt exceeds this
    // Compass snap state (cardinal lock)
    bool compass_snapped; // true when within snap threshold
    int compass_snap_idx; // 0=N,1=E,2=S,3=W when snapped
    // Orientation details for sim responsiveness
    float acc_pitch; // radians
    float acc_roll; // radians
    float yaw_rate_dps; // yaw rate deg/s (from heading change)
    float pitch_rate_dps; // pitch rate deg/s
    float roll_rate_dps; // roll rate deg/s
    float last_yaw_deg; // for rate calc
    float last_pitch; // radians
    float last_roll; // radians
    uint32_t last_orient_tick; // ms tick for rates

    // Notification
    NotificationApp* notification;

    // Level averaging during calibration step 2
    float level_sum_ax, level_sum_ay, level_sum_az;
    uint32_t level_samples;

    // UART communication
    FuriHalSerialHandle* uart_handle;
    uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
    size_t uart_rx_index;
} App;

static void am_apply(const App* app, float x, float y, float z, float* xo, float* yo, float* zo);
static void uart_rx_callback(FuriHalSerialHandle* handle, FuriHalSerialRxEvent event, void* ctx);
static void send_sensor_data(App* app);

// Main draw callback
static void draw_cb(Canvas* canvas, void* ctx) {
    UNUSED(canvas);
    UNUSED(ctx);
}

// Input callback
static void input_cb(InputEvent* e, void* ctx) {
    App* app = ctx;
    furi_message_queue_put(app->input_queue, e, 0);
}

// Apply mapping: given raw (x,y,z) -> mapped (xo,yo,zo)
static void am_apply(const App* app, float x, float y, float z, float* xo, float* yo, float* zo) {
    UNUSED(app);
    // Fixed +Z rotation mapping: X' = +Y, Y' = -X, Z' = +Z
    *xo = y;
    *yo = -x;
    *zo = z;
}

// UART receive callback - processes incoming commands
static void uart_rx_callback(FuriHalSerialHandle* handle, FuriHalSerialRxEvent event, void* ctx) {
    App* app = (App*)ctx;
    UNUSED(handle);
    
    if(event & FuriHalSerialRxEventData) {
        uint8_t data = furi_hal_serial_async_rx(app->uart_handle);
        
        // Add received byte to buffer
        if(app->uart_rx_index < UART_RX_BUFFER_SIZE - 1) {
            app->uart_rx_buffer[app->uart_rx_index] = data;
            app->uart_rx_index++;
            
            // Check for newline (end of command)
            if(data == '\n') {
                app->uart_rx_buffer[app->uart_rx_index - 1] = '\0'; // Replace \n with null terminator
                
                // Process command - check if it's "data"
                if(strcmp((char*)app->uart_rx_buffer, "data") == 0) {
                    send_sensor_data(app);
                }
                
                // Reset buffer
                app->uart_rx_index = 0;
                memset(app->uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
            }
        } else {
            // Buffer overflow, reset
            app->uart_rx_index = 0;
            memset(app->uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
        }
    }
}

// Send sensor data over UART in key-value format - one line at a time to save memory
static void send_sensor_data(App* app) {
    if(!app || !app->uart_handle) {
        FURI_LOG_E("UART", "Cannot send data - invalid app or UART handle");
        return;
    }
    
    char line[64];
    FURI_LOG_I("UART", "Sending sensor data line by line");
    
    // Send accelerometer data
    snprintf(line, sizeof(line), "accel_x=%.3f\n", (double)app->lsm_sample.ax);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    snprintf(line, sizeof(line), "accel_y=%.3f\n", (double)app->lsm_sample.ay);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    snprintf(line, sizeof(line), "accel_z=%.3f\n", (double)app->lsm_sample.az);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    snprintf(line, sizeof(line), "accel_ok=%d\n", app->lsm_sample.ok ? 1 : 0);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    // Send magnetometer data
    snprintf(line, sizeof(line), "mag_x=%.3f\n", (double)app->lsm_sample.mx);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    snprintf(line, sizeof(line), "mag_y=%.3f\n", (double)app->lsm_sample.my);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    snprintf(line, sizeof(line), "mag_z=%.3f\n", (double)app->lsm_sample.mz);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    snprintf(line, sizeof(line), "mag_ok=%d\n", app->lsm_sample.ok ? 1 : 0);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    snprintf(line, sizeof(line), "heading=%.1f\n", (double)app->heading_deg_smooth);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    // Send temperature data
    snprintf(line, sizeof(line), "temperature=%.1f\n", (double)app->th_sample.temperature_c);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    snprintf(line, sizeof(line), "temp_ok=%d\n", app->th_sample.ok ? 1 : 0);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    // Send humidity data
    snprintf(line, sizeof(line), "humidity=%.1f\n", (double)app->th_sample.humidity_rh);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    snprintf(line, sizeof(line), "humidity_ok=%d\n", app->th_sample.ok ? 1 : 0);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    // Send light sensor data
    snprintf(line, sizeof(line), "light_percent=%.1f\n", (double)app->light_sample.percent);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    snprintf(line, sizeof(line), "light_voltage=%.1f\n", (double)app->light_sample.voltage_mv);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    snprintf(line, sizeof(line), "light_ok=%d\n", app->light_sample.ok ? 1 : 0);
    furi_hal_serial_tx(app->uart_handle, (uint8_t*)line, strlen(line));
    
    FURI_LOG_I("UART", "Finished sending sensor data");
}

// App initialization
static App* app_alloc(void) {
    App* app = malloc(sizeof(App));

    // Initialize GUI
    app->gui = furi_record_open(RECORD_GUI);
    app->vp = view_port_alloc();
    view_port_draw_callback_set(app->vp, draw_cb, app);
    view_port_input_callback_set(app->vp, input_cb, app);
    gui_add_view_port(app->gui, app->vp, GuiLayerFullscreen);

    // Initialize input queue
    app->input_queue = furi_message_queue_alloc(8, sizeof(InputEvent));

    // Initialize sensors
    i2c_bus_init(&app->i2c, &furi_hal_i2c_handle_external, 50, false);
    lsm303_init(&app->lsm, &app->i2c, false);
    gxhtc3c_init(&app->th, &app->i2c, false);
    kiisu_light_adc_init();

    // Initialize sensor data
    app->lsm_sample.ok = false;
    app->th_sample.ok = false;
    app->light_sample.ok = false;
    app->light_sample.voltage_mv = 0.0f;
    app->light_sample.percent = 0.0f;

    // Initialize UI state
    app->state = AppStateWidgetMenu;
    app->selected_widget = 0;
    app->orientation_rot = 0;
    app->sensor_swap_xy = false; // use explicit mapping below instead of legacy swap
    app->mag_detail_page = 0;
    app->heading_deg = 0.0f;
    app->heading_deg_smooth = 0.0f;
    app->head_vec_x = 0.0f; // points North initially
    app->head_vec_y = 1.0f;
    app->mag_declination_deg = 0.0f;
    app->heading_tilt_limit_deg = 60.0f;
    app->compass_snapped = false;
    app->compass_snap_idx = -1;
    app->acc_pitch = 0.0f;
    app->acc_roll = 0.0f;
    app->yaw_rate_dps = 0.0f;
    app->pitch_rate_dps = 0.0f;
    app->roll_rate_dps = 0.0f;
    app->last_yaw_deg = 0.0f;
    app->last_pitch = 0.0f;
    app->last_roll = 0.0f;
    app->last_orient_tick = furi_get_tick();

    // Initialize calibration data
    app->mag_min_x = app->mag_min_y = app->mag_min_z = 9999;
    app->mag_max_x = app->mag_max_y = app->mag_max_z = -9999;
    app->mag_calibrated = false;
    app->mag_hw_offsets_applied = false;
    app->mag_offs[0] = app->mag_offs[1] = app->mag_offs[2] = 0.0f;
    app->mag_scale[0] = app->mag_scale[1] = app->mag_scale[2] = 1.0f;
    app->mag_filt[0] = app->mag_filt[1] = app->mag_filt[2] = 0.0f;
    app->mag_iir_alpha = 0.2f; // ~ light smoothing; adjust 0.1..0.3 based on responsiveness
    app->leveled = false;
    app->level_ax = app->level_ay = 0.0f;
    app->level_az = 1.0f;

    // Initialize polling timing
    app->last_lsm_poll_ms = 0;
    app->last_th_poll_ms = 0;
    app->last_light_poll_ms = 0;

    // Initialize Little Kiisu physics state (start centered in accel detail area)
    app->img_x = 64.0f; // legacy
    app->img_y = 32.0f;
    app->img_vx = 0.0f;
    app->img_vy = 0.0f;
    app->img_angle = 0.0f;
    app->img_av = 0.0f;
    app->accel_detail_page = 0;
    app->sim_count = 5;
    app->sim_w = 16; // kitty bitmap is 16x15 (from user snippet); height 15
    app->sim_h = 15;
    app->sim_r = 8.0f; // simple circle radius
    // Seed 5 sprites around a loose ring with varied velocities and spins
    for(int i = 0; i < 5; i++) {
        float ang = (float)i * 6.2831853f / 5.0f; // 2*pi/5
        float rx = 26.0f + 3.0f * (float)((i * 37) % 5); // slight radius variation
        float ry = 14.0f + 2.0f * (float)((i * 53) % 3);
        app->sim_x[i] = 64.0f + cosf(ang) * rx;
        app->sim_y[i] = 32.0f + sinf(ang) * ry;
        // varied initial velocity roughly tangential to the ring
        float tangx = -sinf(ang), tangy = cosf(ang);
        float vmag = 10.0f + (float)((i * 17) % 7);
        app->sim_vx[i] = tangx * (vmag * 0.2f);
        app->sim_vy[i] = tangy * (vmag * 0.2f);
        app->sim_ang[i] = ang * 0.3f;
        app->sim_av[i] = ((i % 2) ? 1.0f : -1.0f) * (0.6f + 0.1f * (float)((i * 29) % 3));
    }
    app->last_physics_tick = furi_get_tick();
    // Homescreen kitty
    app->home_has_kitty = true;
    // Accelerometer widget box: x=59,y=2,w=40,h=25; keep sprite fully inside
    app->home_r = 7.5f; // close to half of 16x15 sprite
    float box_x = 59.0f, box_y = 2.0f, box_w = 40.0f, box_h = 25.0f;
    app->home_x = box_x + box_w * 0.5f;
    app->home_y = box_y + box_h * 0.5f;
    app->home_vx = 0.0f;
    app->home_vy = 0.0f;
    app->home_ang = 0.0f;
    app->home_av = 0.0f;
    app->home_last_tick = furi_get_tick();
    // Axis mapping hard-locked via am_apply()

    // Notifications
    app->notification = furi_record_open(RECORD_NOTIFICATION);

    // Level averaging accumulators
    app->level_sum_ax = app->level_sum_ay = 0.0f;
    app->level_sum_az = 0.0f;
    app->level_samples = 0;

    // Initialize UART - use USART (was working before)
    app->uart_handle = furi_hal_serial_control_acquire(FuriHalSerialIdUsart);
    furi_check(app->uart_handle, "UART handle acquisition failed");
    furi_hal_serial_init(app->uart_handle, UART_BAUDRATE);
    app->uart_rx_index = 0;
    memset(app->uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
    furi_hal_serial_async_rx_start(app->uart_handle, uart_rx_callback, app, false);

    return app;
}

// App cleanup
static void app_free(App* app) {
    view_port_enabled_set(app->vp, false);
    gui_remove_view_port(app->gui, app->vp);
    view_port_free(app->vp);
    furi_message_queue_free(app->input_queue);
    
    // Cleanup UART
    if(app->uart_handle) {
        furi_hal_serial_async_rx_stop(app->uart_handle);
        furi_hal_serial_deinit(app->uart_handle);
        furi_hal_serial_control_release(app->uart_handle);
    }
    
    furi_record_close(RECORD_GUI);
    if(app->notification) furi_record_close(RECORD_NOTIFICATION);
    free(app);
}

// Main app function
int32_t esphome_connect_app(void* p) {
    UNUSED(p);
    App* app = app_alloc();
    const uint32_t poll_period_ms = 100;
    bool running = true;

    while(running) {
        InputEvent e;
        if(furi_message_queue_get(app->input_queue, &e, poll_period_ms) == FuriStatusOk) {

        }

        uint32_t now = furi_get_tick();

        // Poll sensors
        Lsm303Sample lsm_out;
        if(now - app->last_lsm_poll_ms >= 100) {
            if(lsm303_poll(&app->lsm, &lsm_out)) {
                app->lsm_sample = lsm_out;
                app->widgets[WidgetTypeAccelerometer].ok = lsm_out.ok;
                app->widgets[WidgetTypeMagnetometer].ok = lsm_out.ok;

                // Update compass data after polling
                if(lsm_out.ok) {
                    // During figure-eight active step, collect min/max
                    if(app->state == AppStateCalibrating && app->calib_step == 3) {
                        float mx = app->lsm_sample.mx;
                        float my = app->lsm_sample.my;
                        float mz = app->lsm_sample.mz;
                        if(mx < app->mag_min_x) app->mag_min_x = mx;
                        if(mx > app->mag_max_x) app->mag_max_x = mx;
                        if(my < app->mag_min_y) app->mag_min_y = my;
                        if(my > app->mag_max_y) app->mag_max_y = my;
                        if(mz < app->mag_min_z) app->mag_min_z = mz;
                        if(mz > app->mag_max_z) app->mag_max_z = mz;
                    }
                    // Update pitch/roll from accel
                    float axr = app->lsm_sample.ax;
                    float ayr = app->lsm_sample.ay;
                    float azr = app->lsm_sample.az;
                    if(app->sensor_swap_xy) {
                        float t = axr;
                        axr = ayr;
                        ayr = t;
                    }
                    float ax, ay, az;
                    am_apply(app, axr, ayr, azr, &ax, &ay, &az);
                    float roll = atan2f(ay, az == 0.0f ? 1e-6f : az);
                    float denom = sqrtf(ay * ay + az * az);
                    if(denom < 1e-6f) denom = 1e-6f;
                    float pitch = atan2f(-ax, denom);
                    // Rates
                    uint32_t tnow = now;
                    float dt_s = (tnow - app->last_orient_tick) / 1000.0f;
                    if(dt_s <= 0.0f) dt_s = 1e-3f;
                    // Yaw from heading
                    float yaw_deg = app->heading_deg_smooth;
                    float dyaw = yaw_deg - app->last_yaw_deg;
                    while(dyaw > 180.0f)
                        dyaw -= 360.0f;
                    while(dyaw < -180.0f)
                        dyaw += 360.0f;
                    app->yaw_rate_dps = dyaw / dt_s;
                    app->pitch_rate_dps =
                        (pitch - app->last_pitch) * (180.0f / (float)M_PI) / dt_s;
                    app->roll_rate_dps = (roll - app->last_roll) * (180.0f / (float)M_PI) / dt_s;
                    // Store
                    app->acc_pitch = pitch;
                    app->acc_roll = roll;
                    app->last_pitch = pitch;
                    app->last_roll = roll;
                    app->last_yaw_deg = yaw_deg;
                    app->last_orient_tick = tnow;
                }
            }
            app->last_lsm_poll_ms = now;
        }

        Gxhtc3cSample th_out;
        if(now - app->last_th_poll_ms >= 1000) {
            if(gxhtc3c_poll(&app->th, &th_out)) {
                app->th_sample = th_out;
                app->widgets[WidgetTypeTemperature].ok = th_out.ok;
                app->widgets[WidgetTypeHumidity].ok = th_out.ok;
            }
            app->last_th_poll_ms = now;
        }

        KiisuLightAdcSample light_out;
        if(now - app->last_light_poll_ms >= 500) {
            if(kiisu_light_adc_poll(&light_out)) {
                app->light_sample = light_out;
                app->widgets[WidgetTypeLight].ok = light_out.ok;
            }
            app->last_light_poll_ms = now;
        }

        view_port_update(app->vp);
    }

    app_free(app);
    return 0;
}
