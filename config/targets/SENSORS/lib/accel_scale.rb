require 'cosmos/conversions/conversion'
module Cosmos
  class AccelScale < Conversion
    def call(value, packet, buffer)
	  imu = packet.read("IMU", :RAW).to_f
	  value_f = value.to_f
	  if imu == 1.0
	    case value_f
		  when 0.0
		    $IMU1_ACCEL_SCALE = 2.0
		  when 1.0
		    $IMU1_ACCEL_SCALE = 4.0
		  when 2.0
		    $IMU1_ACCEL_SCALE = 8.0
		  when 3.0
		    $IMU1_ACCEL_SCALE = 16.0
		  else
		    $IMU1_ACCEL_SCALE = -1.0
		end
	  elsif imu == 2.0
	    case value_f
		  when 0.0
		    $IMU2_ACCEL_SCALE = 2.0
		  when 1.0
		    $IMU2_ACCEL_SCALE = 4.0
		  when 2.0
		    $IMU2_ACCEL_SCALE = 8.0
		  when 3.0
		    $IMU2_ACCEL_SCALE = 16.0
		  else
		    $IMU2_ACCEL_SCALE = -1.0
		end
	  end
	  return value
    end
  end
end