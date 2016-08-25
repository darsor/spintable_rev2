require 'cosmos/conversions/conversion'
require 'cosmos/system/system.rb'
module Cosmos
  class ImuSystime < Conversion
	def initialize(imu)
      @imu = imu
    end
    def call(value, packet, buffer)
      return ((packet.read('TIMESTAMP', :RAW).to_f - System.telemetry.value('SENSORS', 'TIME', "IMUTIME#{@imu}").to_f) * 0.0000243) + System.telemetry.value('SENSORS', 'TIME', 'SYSTIME').to_f
    end
  end
end