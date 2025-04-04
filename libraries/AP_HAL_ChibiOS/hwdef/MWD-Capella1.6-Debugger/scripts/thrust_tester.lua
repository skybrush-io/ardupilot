-- Configuration
-- PC13 PWM_TRIGGER_OUT OUTPUT HIGH GPIO(14)
-- PE12 PWM_TRIGGER_IN INPUT GPIO(15)
local INPUT_PIN = 15
local TRIGGER_PIN = 14
local motor1 = SRV_Channels:find_channel(33)

-- PWM sweep configuration
local START_PWM = 1050    -- Starting PWM value (μs)
local END_PWM = 1700     -- Ending PWM value (μs)
local PWM_INCREMENT = 50  -- PWM increment per step (μs)
local STEP_TIME = 2000   -- Time to hold each step (milliseconds)
local UPDATE_RATE = 1000 -- Update rate in milliseconds

-- State variables
local last_pin_state = false  -- Track last state
local motor_running = false   -- Track if motor is running
local current_pwm = START_PWM -- Current PWM value
local step_start_time = 0    -- When the current step started

gcs:send_text(0, "GPIO Motor Control Script Initialized")

-- Main update function
function update()
    local pin_state = gpio:read(INPUT_PIN)
    
    -- Only act if state has changed
    if pin_state ~= last_pin_state then
        if pin_state then  -- Pin changed to HIGH
            if not motor_running then  -- Only turn on if not already running
                if not arming:arm() then
                    gcs:send_text(0, "Failed to arm vehicle")
                    return update, UPDATE_RATE
                end
                step_start_time = millis()
                current_pwm = START_PWM
                SRV_Channels:set_output_pwm_chan_timeout(motor1, current_pwm, STEP_TIME+1000)
                motor_running = true
                gcs:send_text(0, string.format("Starting PWM: %d", current_pwm))
            end
        else  -- Pin changed to LOW
            if motor_running then  -- Only turn off if currently running
                SRV_Channels:set_output_pwm_chan_timeout(motor1, 1000, 100)
                motor_running = false
                gcs:send_text(0, "Motor turned OFF")
            end
        end
        last_pin_state = pin_state
    end
    
    -- Handle PWM stepping when motor is running
    if motor_running then
        -- Check if it's time for next step
        local elapsed_time = millis() - step_start_time
        if elapsed_time >= STEP_TIME then
            -- Time to move to next PWM value
            current_pwm = current_pwm + PWM_INCREMENT
            
            -- Check if we've reached or exceeded the end PWM
            if current_pwm > END_PWM then
                -- Test complete, turn off motor
                SRV_Channels:set_output_pwm_chan_timeout(motor1, 1000, 100)
                motor_running = false
                gcs:send_text(0, "Sweep complete - Motor turned OFF")
            else
                -- Start next step
                step_start_time = millis()
                SRV_Channels:set_output_pwm_chan_timeout(motor1, current_pwm, STEP_TIME+1000)
                gcs:send_text(0, string.format("PWM: %d", current_pwm))
            end
        end
    end
    
    return update, UPDATE_RATE
end

-- Register the script
return update()
