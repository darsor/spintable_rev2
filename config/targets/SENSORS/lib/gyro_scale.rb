require 'cosmos/conversions/conversion'
module Cosmos
  class GyroScale < Conversion
    def call(value, packet, buffer)
	  if packet.read("IMU", :RAW).to_f == 1.0
	    case value.to_f
		  when 0.0
		    $IMU1_GYRO_SCALE = 125.0
		  when 1.0
		    $IMU1_GYRO_SCALE = 245.0
		  when 2.0
		    $IMU1_GYRO_SCALE = 500.0
		  when 3.0
		    $IMU1_GYRO_SCALE = 1000.0
		  when 4.0
		    $IMU1_GYRO_SCALE = 2000.0
		end
	  else #elsif packet.read("IMU", :RAW).to_f == 2.0
	    case value.to_f
		  when 0.0
		    $IMU2_GYRO_SCALE = 125.0
		  when 1.0
		    $IMU2_GYRO_SCALE = 245.0
		  when 2.0
		    $IMU2_GYRO_SCALE = 500.0
		  when 3.0
		    $IMU2_GYRO_SCALE = 1000.0
		  when 4.0
		    $IMU2_GYRO_SCALE = 2000.0
		end
	  end
	  return value
    end
  end
end