gcs:send_text(0, "Initializing fan control...")

local PARAM_TABLE_KEY = 152
assert(param:add_table(PARAM_TABLE_KEY, 'MWD_', 2), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'FAN_THRESH', 60), 'could not add FAN_THRESH')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'FAN_ENABLED', 0), 'could not add FAN_ENABLED')

local gpio_state = 0
local threshold = param:get("MWD_FAN_THRESH") or 60
local enabled = param:get("MWD_FAN_ENABLED") or 0
local off_threshold = threshold - 2

gpio:pinMode(13, 1)

function update ()
    if enabled ~= 1 then
        return update, 5000
    end

    local temp = ins:get_temperature(0)
    --gcs:send_text(0, string.format("Temp: %.1f°C (target: %.1f°C)", temp, off_threshold))
    
    if temp > threshold and gpio_state ~= 1 then
        gcs:send_text(0, string.format("Temp @ %.1f, exceeds threshold. Turning fan on.", temp, threshold))
        gpio:write(13, 1)
        gpio_state = 1
    elseif temp <= off_threshold and gpio_state == 1 then
        gcs:send_text(0, string.format("Temp @ %.1f, below off threshold. Turning fan off.", temp, off_threshold))
        gpio:write(13, 0)
        gpio_state = 0
    end

    return update, 10000
end

if enabled == 1 then
    gcs:send_text(0, "Fan control ready.")
end

return update, 100
