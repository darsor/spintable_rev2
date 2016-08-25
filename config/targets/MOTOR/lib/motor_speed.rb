require 'cosmos/conversions/conversion'
module Cosmos
  class MotorSpeed < Conversion
    @@i = 0
    @@times = Array.new(10)
	@@ticks = Array.new(10)
    def call(value, packet, buffer)
	  @@times[@@i] = packet.read('SYSTIME', :CONVERTED)	#save the time
	  @@ticks[@@i] = packet.read('RAW_CNT', :RAW)			#save the count
	  speed = ( (@@ticks[@@i].to_f - @@ticks[@@i+1-@@ticks.size].to_f) / (@@times[@@i].to_f - @@times[@@i+1-@@times.size].to_f) ) * 0.0375
	  @@i = (@@i + 1) % 10								#increment i
	  return speed
    end
  end
end