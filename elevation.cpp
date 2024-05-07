/*
 * elevation.c
 *
 *  Created on: Feb 26, 2017
 *      Author: jeff
 */
#ifdef FEATURE_ELEVATION_CONTROL
void service_rotation_elevation()
{
	static byte el_direction_change_flag = 0;
	static byte el_initial_slow_down_voltage = 0;

	if (el_state == INITIALIZE_NORMAL_UP)
	{
	  update_el_variable_outputs(normal_el_speed_voltage);
	  rotator(ACTIVATE,UP);
	  el_state = NORMAL_UP;
	}
	if (el_state == INITIALIZE_NORMAL_DOWN)
	{
	  update_el_variable_outputs(normal_el_speed_voltage);
	  rotator(ACTIVATE,DOWN);
	  el_state = NORMAL_DOWN;
	}
	if (el_state == INITIALIZE_SLOW_START_UP)
	{
	  update_el_variable_outputs(EL_SLOW_START_STARTING_PWM);
	  rotator(ACTIVATE,UP);
	  el_slowstart_start_time = millis();
	  el_last_step_time = 0;
	  el_slow_start_step = 0;
	  el_state = SLOW_START_UP;
	  #ifdef DEBUG_SERVICE_ROTATION
	  if (debug_mode) {Serial.println(F("service_rotation: INITIALIZE_SLOW_START_UP -> SLOW_START_UP"));}
	  #endif //DEBUG_SERVICE_ROTATION
	}

	if (el_state == INITIALIZE_SLOW_START_DOWN)
	{
	  update_el_variable_outputs(EL_SLOW_START_STARTING_PWM);
	  rotator(ACTIVATE,DOWN);
	  el_slowstart_start_time = millis();
	  el_last_step_time = 0;
	  el_slow_start_step = 0;
	  el_state = SLOW_START_DOWN;
	  #ifdef DEBUG_SERVICE_ROTATION
	  if (debug_mode) {Serial.println(F("service_rotation: INITIALIZE_SLOW_START_DOWN -> SLOW_START_DOWN"));}
	  #endif //DEBUG_SERVICE_ROTATION
	}

	if (el_state == INITIALIZE_TIMED_SLOW_DOWN_UP)
	{
	  el_direction_change_flag = 0;
	  el_timed_slow_down_start_time = millis();
	  el_last_step_time = millis();
	  el_slow_down_step = EL_SLOW_DOWN_STEPS-1;
	  el_state = TIMED_SLOW_DOWN_UP;
	}

	if (el_state == INITIALIZE_TIMED_SLOW_DOWN_DOWN)
	{
	  el_direction_change_flag = 0;
	  el_timed_slow_down_start_time = millis();
	  el_last_step_time = millis();
	  el_slow_down_step = EL_SLOW_DOWN_STEPS-1;
	  el_state = TIMED_SLOW_DOWN_DOWN;
	}

	if (el_state == INITIALIZE_DIR_CHANGE_TO_UP)
	{
	  el_direction_change_flag = 1;
	  el_timed_slow_down_start_time = millis();
	  el_last_step_time = millis();
	  el_slow_down_step = EL_SLOW_DOWN_STEPS-1;
	  el_state = TIMED_SLOW_DOWN_DOWN;
	}

	if (el_state == INITIALIZE_DIR_CHANGE_TO_DOWN)
	{
	  el_direction_change_flag = 1;
	  el_timed_slow_down_start_time = millis();
	  el_last_step_time = millis();
	  el_slow_down_step = EL_SLOW_DOWN_STEPS-1;
	  el_state = TIMED_SLOW_DOWN_UP;
	}

	// slow start-------------------------------------------------------------------------------------------------
	if ((el_state == SLOW_START_UP) || (el_state == SLOW_START_DOWN))
	{
	  if ((millis() - el_slowstart_start_time) >= EL_SLOW_START_UP_TIME)
	  {  // is it time to end slow start?
		#ifdef DEBUG_SERVICE_ROTATION
		if (debug_mode) {Serial.print(F("service_rotation: NORMAL_"));}
		#endif //DEBUG_SERVICE_ROTATION
		if (el_state == SLOW_START_UP)
		{
		  el_state = NORMAL_UP;
		  #ifdef DEBUG_SERVICE_ROTATION
		  if (debug_mode) {Serial.println(F("UP"));}
		  #endif //DEBUG_SERVICE_ROTATION
		} else
		{
		  el_state = NORMAL_DOWN;
		  #ifdef DEBUG_SERVICE_ROTATION
		  if (debug_mode) {Serial.println(F("DOWN"));}
		  #endif //DEBUG_SERVICE_ROTATION
		}
		update_el_variable_outputs(normal_el_speed_voltage);
	  } else
	  {  // it's not time to end slow start yet, but let's check if it's time to step up the speed voltage
		if (((millis() - el_last_step_time) > (EL_SLOW_START_UP_TIME/EL_SLOW_START_STEPS)) && (normal_el_speed_voltage > EL_SLOW_START_STARTING_PWM))
		{
		  #ifdef DEBUG_SERVICE_ROTATION
		  if (debug_mode) {
			Serial.print(F("service_rotation: step up: "));
			Serial.print(el_slow_start_step);
			Serial.print(F(" pwm: "));
			Serial.println((int)(EL_SLOW_START_STARTING_PWM+((normal_el_speed_voltage-EL_SLOW_START_STARTING_PWM)*((float)el_slow_start_step/(float)(EL_SLOW_START_STEPS-1)))));
		  }
		  #endif //DEBUG_SERVICE_ROTATION
		  update_el_variable_outputs((EL_SLOW_START_STARTING_PWM+((normal_el_speed_voltage-EL_SLOW_START_STARTING_PWM)*((float)el_slow_start_step/(float)(EL_SLOW_START_STEPS-1)))));
		  el_last_step_time = millis();
		  el_slow_start_step++;
		}
	  }
	} //((el_state == SLOW_START_UP) || (el_state == SLOW_START_DOWN))


	// timed slow down ------------------------------------------------------------------------------------------------------
	if (((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN)) && ((millis() - el_last_step_time) >= (TIMED_SLOW_DOWN_TIME/EL_SLOW_DOWN_STEPS)))
	{
	  #ifdef DEBUG_SERVICE_ROTATION
	  if (debug_mode)
	  {
		Serial.print(F("service_rotation: TIMED_SLOW_DOWN step down: "));
		Serial.print(el_slow_down_step);
		Serial.print(F(" pwm: "));
		Serial.println((int)(normal_el_speed_voltage*((float)el_slow_down_step/(float)EL_SLOW_DOWN_STEPS)));
	  }
	  #endif //DEBUG_SERVICE_ROTATION
	  update_el_variable_outputs((int)(normal_el_speed_voltage*((float)el_slow_down_step/(float)EL_SLOW_DOWN_STEPS)));
	  el_last_step_time = millis();
	  el_slow_down_step--;

	  if (el_slow_down_step == 0)
	  { // is it time to exit timed slow down?
		#ifdef DEBUG_SERVICE_ROTATION
		if (debug_mode) {Serial.println(F("service_rotation: TIMED_SLOW_DOWN->IDLE"));}
		#endif //DEBUG_SERVICE_ROTATION
		rotator(DEACTIVATE,UP);
		rotator(DEACTIVATE,DOWN);
		if (el_direction_change_flag)
		{
		  if (el_state == TIMED_SLOW_DOWN_UP)
		  {
			rotator(ACTIVATE,DOWN);
			if (el_slowstart_active) {el_state = INITIALIZE_SLOW_START_DOWN;} else {el_state = NORMAL_DOWN;};
			el_direction_change_flag = 0;
		  }
		  if (el_state == TIMED_SLOW_DOWN_DOWN)
		  {
			rotator(ACTIVATE,UP);
			if (el_slowstart_active) {el_state = INITIALIZE_SLOW_START_UP;} else {el_state = NORMAL_UP;};
			el_direction_change_flag = 0;
		  }
		} else
		{
		  el_state = IDLE;
		  el_request_queue_state = NONE;
		}
	  }
	}  //((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN))

	// slow down ---------------------------------------------------------------------------------------------------------------
	if ((el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN))
	{
	  // is it time to do another step down?
	  if (abs((target_elevation - elevation)/HEADING_MULTIPLIER) <= (((float)SLOW_DOWN_BEFORE_TARGET_EL*((float)el_slow_down_step/(float)EL_SLOW_DOWN_STEPS))))
	  {
		#ifdef DEBUG_SERVICE_ROTATION
		if (debug_mode)
		{
		  Serial.print(F("service_rotation: step down: "));
		  Serial.print(el_slow_down_step);
		  Serial.print(F(" pwm: "));
		  Serial.println((int)(EL_SLOW_DOWN_PWM_STOP+((el_initial_slow_down_voltage-EL_SLOW_DOWN_PWM_STOP)*((float)el_slow_down_step/(float)EL_SLOW_DOWN_STEPS))));
		}
		#endif //DEBUG_SERVICE_ROTATION
		update_el_variable_outputs((EL_SLOW_DOWN_PWM_STOP+((el_initial_slow_down_voltage-EL_SLOW_DOWN_PWM_STOP)*((float)el_slow_down_step/(float)EL_SLOW_DOWN_STEPS))));
		el_slow_down_step--;
	  }
	}  //((el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN))

	// normal -------------------------------------------------------------------------------------------------------------------
	// if slow down is enabled, see if we're ready to go into slowdown
	if (((el_state == NORMAL_UP) || (el_state == SLOW_START_UP) || (el_state == NORMAL_DOWN) || (el_state == SLOW_START_DOWN)) &&
	(el_request_queue_state == IN_PROGRESS_TO_TARGET) && el_slowdown_active && (abs((target_elevation - elevation)/HEADING_MULTIPLIER) <= SLOW_DOWN_BEFORE_TARGET_EL))
	{
	  #ifdef DEBUG_SERVICE_ROTATION
	  if (debug_mode) {Serial.print(F("service_rotation: SLOW_DOWN_"));}
	  #endif //DEBUG_SERVICE_ROTATION
	  el_slow_down_step = EL_SLOW_DOWN_STEPS-1;
	  if ((el_state == NORMAL_UP) || (el_state == SLOW_START_UP))
	  {
		el_state = SLOW_DOWN_UP;
		#ifdef DEBUG_SERVICE_ROTATION
		if (debug_mode) {Serial.println(F("UP"));}
		#endif //DEBUG_SERVICE_ROTATION
	  } else
	  {
		el_state = SLOW_DOWN_DOWN;
		#ifdef DEBUG_SERVICE_ROTATION
		if (debug_mode) {Serial.println(F("DOWN"));}
		#endif //DEBUG_SERVICE_ROTATION
	  }
	  if (EL_SLOW_DOWN_PWM_START < current_el_speed_voltage)
	  {
		update_el_variable_outputs(EL_SLOW_DOWN_PWM_START);
		el_initial_slow_down_voltage = EL_SLOW_DOWN_PWM_START;
	  } else
	  {
		el_initial_slow_down_voltage = current_el_speed_voltage;
	  }
	}

	// check rotation target --------------------------------------------------------------------------------------------------------
	if ((el_state != IDLE) && (el_request_queue_state == IN_PROGRESS_TO_TARGET) )
	{
	  if ((el_state == NORMAL_UP) || (el_state == SLOW_START_UP) || (el_state == SLOW_DOWN_UP))
	  {
		if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE*HEADING_MULTIPLIER)) || ((elevation > target_elevation) && ((elevation - target_elevation) < ((ELEVATION_TOLERANCE+5)*HEADING_MULTIPLIER))))
		{
		  delay(50);
		  read_elevation();
		  if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE*HEADING_MULTIPLIER)) || ((elevation > target_elevation) && ((elevation - target_elevation) < ((ELEVATION_TOLERANCE+5)*HEADING_MULTIPLIER))))
		  {
			rotator(DEACTIVATE,UP);
			rotator(DEACTIVATE,DOWN);
			el_state = IDLE;
			el_request_queue_state = NONE;
			#ifdef DEBUG_SERVICE_ROTATION
			if (debug_mode) {Serial.println(F("service_rotation: IDLE"));}
			#endif //DEBUG_SERVICE_ROTATION
		  }
		}
	  } else
	  {
		if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE*HEADING_MULTIPLIER)) ||
			((elevation < target_elevation) && ((target_elevation - elevation) < ((ELEVATION_TOLERANCE+5)*HEADING_MULTIPLIER))))
		{
		  delay(50);
		  read_elevation();
		  if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE*HEADING_MULTIPLIER)) ||
			  ((elevation < target_elevation) && ((target_elevation - elevation) < ((ELEVATION_TOLERANCE+5)*HEADING_MULTIPLIER))))
		  {
			rotator(DEACTIVATE,UP);
			rotator(DEACTIVATE,DOWN);
			el_state = IDLE;
			el_request_queue_state = NONE;
			#ifdef DEBUG_SERVICE_ROTATION
			if (debug_mode) {Serial.println(F("service_rotation: IDLE"));}
			#endif //DEBUG_SERVICE_ROTATION
		  }
		}
	  }
	}
} // service_rotator_elevation()
#endif //FEATURE_ELEVATION_CONTROL

