require 'cosmos/conversions/conversion'

module Cosmos
  $IMU1_GYRO_SCALE = 245
  $IMU2_GYRO_SCALE = 245
  $IMU1_ACCEL_SCALE = 2
  $IMU2_ACCEL_SCALE = 2
  class AdcConversion < Conversion
	def initialize(bits, scale, type=0.0)
      @bits = bits
	  @scale = scale
	  @type = type
    end
    def call(value, packet, buffer)
	  res = 2**(@bits.to_f-1)
	  if value.to_f > 0.0
	    res -= 1
	  end
	  return case @type.to_f
	    when 0.0
	      value.to_f * (@scale.to_f / res.to_f)
		when 1.0
	      #value.to_f * ($IMU1_GYRO_SCALE.to_f / res.to_f)
		  $IMU1_GYRO_SCALE
		when 2.0
	      #value.to_f * ($IMU2_GYRO_SCALE.to_f / res.to_f)
		  $IMU2_GYRO_SCALE
		when 3.0
	      #value.to_f * ($IMU1_ACCEL_SCALE.to_f / res.to_f)
		  $IMU1_ACCEL_SCALE
		when 4.0
	      #value.to_f * ($IMU2_ACCEL_SCALE.to_f / res.to_f)
		  $IMU2_ACCEL_SCALE
		else -1.0
	  end
    end
  end
end