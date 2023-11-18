void Temperature::PID_autotune(const celsius_t target, const heater_id_t heater_id, const int8_t ncycles, const bool set_result/*=false*/) {
    celsius_float_t current_temp = 0.0;
    int cycles = 0;
    bool heating = true;

    millis_t next_temp_ms = millis(), t1 = next_temp_ms, t2 = next_temp_ms;
    long t_high = 0, t_low = 0;

    raw_pid_t tune_pid = { 0, 0, 0 };
    celsius_float_t maxT = 0, minT = 10000;

    const bool isbed = (heater_id == H_BED),
           ischamber = (heater_id == H_CHAMBER);

    #if ENABLED(PIDTEMPCHAMBER)
      #define C_TERN(T,A,B) ((T) ? (A) : (B))
    #else
      #define C_TERN(T,A,B) (B)
    #endif
    #if ENABLED(PIDTEMPBED)
      #define B_TERN(T,A,B) ((T) ? (A) : (B))
    #else
      #define B_TERN(T,A,B) (B)
    #endif
    #define GHV(C,B,H) C_TERN(ischamber, C, B_TERN(isbed, B, H))
    #define SHV(V) C_TERN(ischamber, temp_chamber.soft_pwm_amount = V, B_TERN(isbed, temp_bed.soft_pwm_amount = V, temp_hotend[heater_id].soft_pwm_amount = V))
    #define ONHEATINGSTART() C_TERN(ischamber, printerEventLEDs.onChamberHeatingStart(), B_TERN(isbed, printerEventLEDs.onBedHeatingStart(), printerEventLEDs.onHotendHeatingStart()))
    #define ONHEATING(S,C,T) C_TERN(ischamber, printerEventLEDs.onChamberHeating(S,C,T), B_TERN(isbed, printerEventLEDs.onBedHeating(S,C,T), printerEventLEDs.onHotendHeating(S,C,T)))

    #define WATCH_PID DISABLED(NO_WATCH_PID_TUNING) && (ALL(WATCH_CHAMBER, PIDTEMPCHAMBER) || ALL(WATCH_BED, PIDTEMPBED) || ALL(WATCH_HOTENDS, PIDTEMP))

    #if WATCH_PID
      #if ALL(THERMAL_PROTECTION_CHAMBER, PIDTEMPCHAMBER)
        #define C_GTV(T,A,B) ((T) ? (A) : (B))
      #else
        #define C_GTV(T,A,B) (B)
      #endif
      #if ALL(THERMAL_PROTECTION_BED, PIDTEMPBED)
        #define B_GTV(T,A,B) ((T) ? (A) : (B))
      #else
        #define B_GTV(T,A,B) (B)
      #endif
      #define GTV(C,B,H) C_GTV(ischamber, C, B_GTV(isbed, B, H))
      const uint16_t watch_temp_period = GTV(WATCH_CHAMBER_TEMP_PERIOD, WATCH_BED_TEMP_PERIOD, WATCH_TEMP_PERIOD);
      const uint8_t watch_temp_increase = GTV(WATCH_CHAMBER_TEMP_INCREASE, WATCH_BED_TEMP_INCREASE, WATCH_TEMP_INCREASE);
      const celsius_float_t watch_temp_target = celsius_float_t(target - (watch_temp_increase + GTV(TEMP_CHAMBER_HYSTERESIS, TEMP_BED_HYSTERESIS, TEMP_HYSTERESIS) + 1));
      millis_t temp_change_ms = next_temp_ms + SEC_TO_MS(watch_temp_period);
      celsius_float_t next_watch_temp = 0.0;
      bool heated = false;
    #endif

    TERN_(HAS_FAN_LOGIC, fan_update_ms = next_temp_ms + fan_update_interval_ms);

    TERN_(EXTENSIBLE_UI, ExtUI::onPidTuning(ExtUI::result_t::PID_STARTED));
    TERN_(PROUI_PID_TUNE, dwinPidTuning(isbed ? PIDTEMPBED_START : PIDTEMP_START));

    if (target > GHV(CHAMBER_MAX_TARGET, BED_MAX_TARGET, temp_range[heater_id].maxtemp - (HOTEND_OVERSHOOT))) {
      SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_TEMP_TOO_HIGH);
      TERN_(EXTENSIBLE_UI, ExtUI::onPidTuning(ExtUI::result_t::PID_TEMP_TOO_HIGH));
      TERN_(PROUI_PID_TUNE, dwinPidTuning(PID_TEMP_TOO_HIGH));
      TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PID_TEMP_TOO_HIGH)));
      return;
    }

    SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_AUTOTUNE_START);

    disable_all_heaters();
    TERN_(AUTO_POWER_CONTROL, powerManager.power_on());

    long bias = GHV(MAX_CHAMBER_POWER, MAX_BED_POWER, PID_MAX) >> 1, d = bias;
    SHV(bias);

    #if ENABLED(PRINTER_EVENT_LEDS)
      const celsius_float_t start_temp = GHV(degChamber(), degBed(), degHotend(heater_id));
      LEDColor color = ONHEATINGSTART();
    #endif

    TERN_(TEMP_TUNING_MAINTAIN_FAN, adaptive_fan_slowing = false);

    LCD_MESSAGE(MSG_HEATING);

    // PID Tuning loop
    wait_for_heatup = true;
    while (wait_for_heatup) { // Can be interrupted with M108

      const millis_t ms = millis();

      if (updateTemperaturesIfReady()) { // temp sample ready

        // Get the current temperature and constrain it
        current_temp = GHV(degChamber(), degBed(), degHotend(heater_id));
        NOLESS(maxT, current_temp);
        NOMORE(minT, current_temp);

        #if ENABLED(PRINTER_EVENT_LEDS)
          ONHEATING(start_temp, current_temp, target);
        #endif

        TERN_(HAS_FAN_LOGIC, manage_extruder_fans(ms));

        if (heating && current_temp > target && ELAPSED(ms, t2 + 5000UL)) {
          heating = false;
          SHV((bias - d) >> 1);
          t1 = ms;
          t_high = t1 - t2;
          maxT = target;
        }

        if (!heating && current_temp < target && ELAPSED(ms, t1 + 5000UL)) {
          heating = true;
          t2 = ms;
          t_low = t2 - t1;
          if (cycles > 0) {
            const long max_pow = GHV(MAX_CHAMBER_POWER, MAX_BED_POWER, PID_MAX);
            bias += (d * (t_high - t_low)) / (t_low + t_high);
            LIMIT(bias, 20, max_pow - 20);
            d = (bias > max_pow >> 1) ? max_pow - 1 - bias : bias;

            SERIAL_ECHOPGM(STR_BIAS, bias, STR_D_COLON, d, STR_T_MIN, minT, STR_T_MAX, maxT);
            if (cycles > 2) {
              const float Ku = (4.0f * d) / (float(M_PI) * (maxT - minT) * 0.5f),
                          Tu = float(t_low + t_high) * 0.001f,
                          pf = (ischamber || isbed) ? 0.2f : 0.6f,
                          df = (ischamber || isbed) ? 1.0f / 3.0f : 1.0f / 8.0f;

              tune_pid.p = Ku * pf;
              tune_pid.i = tune_pid.p * 2.0f / Tu;
              tune_pid.d = tune_pid.p * Tu * df;

              SERIAL_ECHOLNPGM(STR_KU, Ku, STR_TU, Tu);
              if (ischamber || isbed)
                SERIAL_ECHOLNPGM(" No overshoot");
              else
                SERIAL_ECHOLNPGM(STR_CLASSIC_PID);
              SERIAL_ECHOLNPGM(STR_KP, tune_pid.p, STR_KI, tune_pid.i, STR_KD, tune_pid.d);
            }
          }
          SHV((bias + d) >> 1);
          TERN_(HAS_STATUS_MESSAGE, ui.status_printf(0, F(S_FMT " %i/%i"), GET_TEXT(MSG_PID_CYCLE), cycles, ncycles));
          cycles++;
          minT = target;
        }
      }

      // Did the temperature overshoot very far?
      #ifndef MAX_OVERSHOOT_PID_AUTOTUNE
        #define MAX_OVERSHOOT_PID_AUTOTUNE 30
      #endif
      if (current_temp > target + MAX_OVERSHOOT_PID_AUTOTUNE) {
        SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_TEMP_TOO_HIGH);
        TERN_(EXTENSIBLE_UI, ExtUI::onPidTuning(ExtUI::result_t::PID_TEMP_TOO_HIGH));
        TERN_(PROUI_PID_TUNE, dwinPidTuning(PID_TEMP_TOO_HIGH));
        TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PID_TEMP_TOO_HIGH)));
        break;
      }

      // Report heater states every 2 seconds
      if (ELAPSED(ms, next_temp_ms)) {
        #if HAS_TEMP_SENSOR
          print_heater_states(heater_id < 0 ? active_extruder : (int8_t)heater_id);
          SERIAL_EOL();
        #endif
        next_temp_ms = ms + 2000UL;

        // Make sure heating is actually working
        #if WATCH_PID
          if (ALL(WATCH_BED, WATCH_HOTENDS) || isbed == DISABLED(WATCH_HOTENDS) || ischamber == DISABLED(WATCH_HOTENDS)) {
            if (!heated) {                                            // If not yet reached target...
              if (current_temp > next_watch_temp) {                   // Over the watch temp?
                next_watch_temp = current_temp + watch_temp_increase; // - set the next temp to watch for
                temp_change_ms = ms + SEC_TO_MS(watch_temp_period);   // - move the expiration timer up
                if (current_temp > watch_temp_target) heated = true;  // - Flag if target temperature reached
              }
              else if (ELAPSED(ms, temp_change_ms))                   // Watch timer expired
                _TEMP_ERROR(heater_id, FPSTR(str_t_heating_failed), MSG_ERR_HEATING_FAILED, current_temp);
            }
            else if (current_temp < target - (MAX_OVERSHOOT_PID_AUTOTUNE)) // Heated, then temperature fell too far?
              _TEMP_ERROR(heater_id, FPSTR(str_t_thermal_runaway), MSG_ERR_THERMAL_RUNAWAY, current_temp);
          }
        #endif
      } // every 2 seconds

      // Timeout after MAX_CYCLE_TIME_PID_AUTOTUNE minutes since the last undershoot/overshoot cycle
      #ifndef MAX_CYCLE_TIME_PID_AUTOTUNE
        #define MAX_CYCLE_TIME_PID_AUTOTUNE 20L
      #endif
      if ((ms - _MIN(t1, t2)) > (MAX_CYCLE_TIME_PID_AUTOTUNE * 60L * 1000L)) {
        TERN_(DWIN_CREALITY_LCD, dwinPopupTemperature(0));
        TERN_(PROUI_PID_TUNE, dwinPidTuning(PID_TUNING_TIMEOUT));
        TERN_(EXTENSIBLE_UI, ExtUI::onPidTuning(ExtUI::result_t::PID_TUNING_TIMEOUT));
        TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PID_TIMEOUT)));
        SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_TIMEOUT);
        break;
      }

      if (cycles > ncycles && cycles > 2) {
        SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_AUTOTUNE_FINISHED);
        TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PID_AUTOTUNE_DONE)));

        #if ANY(PIDTEMPBED, PIDTEMPCHAMBER)
          FSTR_P const estring = GHV(F("chamber"), F("bed"), FPSTR(NUL_STR));
          say_default_(); SERIAL_ECHO(estring, F("Kp "), tune_pid.p);
          say_default_(); SERIAL_ECHO(estring, F("Ki "), tune_pid.i);
          say_default_(); SERIAL_ECHO(estring, F("Kd "), tune_pid.d);
        #else
          say_default_(); SERIAL_ECHOLNPGM("Kp ", tune_pid.p);
          say_default_(); SERIAL_ECHOLNPGM("Ki ", tune_pid.i);
          say_default_(); SERIAL_ECHOLNPGM("Kd ", tune_pid.d);
        #endif

        auto _set_hotend_pid = [](const uint8_t tool, const raw_pid_t &in_pid) {
          #if ENABLED(PIDTEMP)
            #if ENABLED(PID_PARAMS_PER_HOTEND)
              thermalManager.temp_hotend[tool].pid.set(in_pid);
            #else
              HOTEND_LOOP() thermalManager.temp_hotend[e].pid.set(in_pid);
            #endif
            updatePID();
          #endif
          UNUSED(tool); UNUSED(in_pid);
        };

        #if ENABLED(PIDTEMPBED)
          auto _set_bed_pid = [](const raw_pid_t &in_pid) {
            temp_bed.pid.set(in_pid);
          };
        #endif

        #if ENABLED(PIDTEMPCHAMBER)
          auto _set_chamber_pid = [](const raw_pid_t &in_pid) {
            temp_chamber.pid.set(in_pid);
          };
        #endif

        // Use the result? (As with "M303 U1")
        if (set_result)
          GHV(_set_chamber_pid(tune_pid), _set_bed_pid(tune_pid), _set_hotend_pid(heater_id, tune_pid));

        TERN_(PRINTER_EVENT_LEDS, printerEventLEDs.onPidTuningDone(color));

        TERN_(EXTENSIBLE_UI, ExtUI::onPidTuning(ExtUI::result_t::PID_DONE));
        TERN_(PROUI_PID_TUNE, dwinPidTuning(AUTOTUNE_DONE));

        goto EXIT_M303;
      }

      // Run HAL idle tasks
      hal.idletask();

      // Run UI update
      TERN(DWIN_CREALITY_LCD, dwinUpdate(), ui.update());
    }
    wait_for_heatup = false;

    disable_all_heaters();

    TERN_(PRINTER_EVENT_LEDS, printerEventLEDs.onPidTuningDone(color));

    TERN_(EXTENSIBLE_UI, ExtUI::onPidTuning(ExtUI::result_t::PID_DONE));
    TERN_(PROUI_PID_TUNE, dwinPidTuning(AUTOTUNE_DONE));

    EXIT_M303:
      TERN_(TEMP_TUNING_MAINTAIN_FAN, adaptive_fan_slowing = true);
      return;
  }