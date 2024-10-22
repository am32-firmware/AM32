--[[
   ArduPilot lua script to log debug messages from AM32 DroneCAN
   ESCs on the flight controller

   To install set SCR_ENABLE=1 and put this script in APM/SCRIPTS/ on
   the microSD of the flight controller then restart the flight
   controllr
--]]

local ESC_BASE = 30

local AM32_DEBUG = 100

local last_tstamp = {}
local ts_zero = uint32_t(0)
local reported_version_error = false

function log_AM32()
   for i = 0, 3 do
      local last_ts = last_tstamp[i] or ts_zero
      tstamp_us, msg = DroneCAN_get_FlexDebug(0, ESC_BASE+i, AM32_DEBUG, last_ts)
      if tstamp_us then
         local dt = (tstamp_us - last_ts):tofloat()*1.0e-6
         version, commutation_interval, num_commands, num_input, rx_errors, rxframe_error, rx_ecode, auto_advance_level = string.unpack("<BiHHHHiB", msg)
         if version ~= 1 then
            if not reported_version_error then
               reported_version_error = true
               gcs:send_text(0, string.format("AM32 debug version error %u", version))
            end
            return
         end
         logger:write('AMD1','Node,CI,NC,NI,RXerr,FrE,RXec,AAL','BiHHHHiB','#-------','--------',i,commutation_interval, num_commands, num_input, rx_errors, rxframe_error, rx_ecode, auto_advance_level)
         gcs:send_named_float(string.format('AM32_EC_%u',i), rx_ecode)
         -- gcs:send_named_float(string.format('AM32_NI_%u',i), num_input/dt)
         -- gcs:send_named_float(string.format('AM32_CI_%u',i), commutation_interval)
         last_tstamp[i] = tstamp_us
      end
   end
end

function update()
   log_AM32()
   return update, 5
end

gcs:send_text(0, "Loaded AM32_debug_log")

return update, 5


