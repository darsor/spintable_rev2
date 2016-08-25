require 'cosmos/conversions/conversion'
module Cosmos
  class ResetImu < Conversion
    def call(value, packet, buffer)
	  if value == 1
	    $IMU1_GYRO_SCALE = 245
		$IMU1_ACCEL_SCALE = 2
	  end
	  if value == 2
	    $IMU2_GYRO_SCALE = 245
		$IMU2_ACCEL_SCALE = 2
	  end
	  return value
    end
  end
end