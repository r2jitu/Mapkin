{{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build18 Motor Encoder Controller
//
// Author: Kwabena W. Agyeman
// Updated: 1/10/2011
// Designed For: P8X32A
// Version: 1.0
//
// Copyright (c) 2011 Kwabena W. Agyeman
// See end of file for terms of use.
//
// Update History:
//
// v1.0 - Original release - 1/10/2011.
//
// Runs two motors with two encoders.
//
// Nyamekye,
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}}

CON

  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

  _baudRateSpeed = 115_200

  _leftForward = 0
  _leftBackward = 1
  _rightForward = 2
  _rightBackward = 3

  _leftEncPower = 4 ' Place holder.
  _leftEncInput = 5

  _rightEncPower = 6 ' Place holder.
  _rightEncInput = 7
  
  _receiverPin = 31
  _transmitterPin = 30

OBJ

  hbd: "PWM2C_HBDEngine.spin"
  com: "RS232_COMEngine.spin"
  str: "ASCII0_STREngine.spin"

PUB demo | deltaA, deltaB

  ifnot( com.COMEngineStart(_receiverPin, _transmitterPin, _baudRateSpeed) and {
       } hbd.HBDEngineStart(_leftForward, _leftBackward, _rightForward, _rightBackward, 1000) )
    reboot

  ctra := constant((%01110 << 26) + _leftEncInput) ' Negative edge detection.
  ctrb := constant((%01110 << 26) + _rightEncInput) ' Negative edge detection.
  frqa := frqb := 1  
  phsa := phsb := deltaA := deltaB := 0
    
  repeat
    repeat
      result := com.receivedByte
      str.buildString(result)
    while(result <> 10)
    result := str.tokenizeString(str.trimString(str.builtString(true)))      

    ifnot(str.stringCompareCI(string("in"), result))
      hbd.leftDuty(str.decimalToInteger(str.tokenizeString(0)) * 10) ' Takes -100% to +100%.
      hbd.rightDuty(str.decimalToInteger(str.tokenizeString(0)) * 10) ' Takes -100% to +100%.

    elseifnot(str.stringCompareCI(string("out"), result))

      result := phsa
      result -= deltaA
      deltaA += result
    
      com.transmitString(1 + str.integerToDecimal(result, 10))
      com.transmitByte(" ")

      result := phsb
      result -= deltaB
      deltaB += result

      com.transmitString(1 + str.integerToDecimal(result, 10))
      com.transmitString(string(13, 10))
      
    else
      com.transmitString(string("IN <lft_duty> <rght_duty> [or] OUT (lft_delta) (rght_delta)", 13, 10))
      
{{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  TERMS OF USE: MIT License
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}}
