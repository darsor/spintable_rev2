require 'cosmos/conversions/conversion'
module Cosmos
  class Systime < Conversion
    def call(value, packet, buffer)
      return packet.read('SYSTIME_S', :RAW) + (packet.read('SYSTIME_U', :RAW)) / 1000000.0
    end
  end
end