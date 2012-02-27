"""
 PyBBIO - bbio.py - v0.2
 Created: 12/2011
 Author: Alexander Hiam - ahiam@marlboro.edu - www.alexanderhiam.com
 Website: https://github.com/alexanderhiam/PyBBIO
 
 A Python library for hardware IO support on the TI Beaglebone.
 At the moment it is quite limited, only providing API for simple
 digital IO functionality. I have big plans for this, however, so
 keep checking the github page for updates.
 
 Copyright (c) 2012 - Alexander Hiam
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
"""

import struct, os, sys
from time import sleep
from mmap import mmap


# Load global configuration:
CONFIG_FILE="%s/.pybbio/beaglebone.cfg" % os.environ['HOME']
config = open(CONFIG_FILE, 'r').read()
assert ('MMAP_OFFSET' in config) and ('MMAP_SIZE' in config),\
      "*Config file '%s' must contain values MMAP_OFFSET and MMAP_SIZE" %\
                                                                CONFIG_FILE
exec(config)

# Create global mmap:
f = open("/dev/mem", "r+b")
__mmap = mmap(f.fileno(), MMAP_SIZE, offset=MMAP_OFFSET)
f.close() # Only needed to make map


def run(setup, main):
  """ The main loop; must be passed a setup and a main function.
      First the setup function will be called once, then the main
      function wil be continuously until a stop signal is raised,
      e.g. CTRL-C or a call to the stop() function from within the
      main function. """
  try:
    _init()
    setup()
    while (True):
      main()
  except KeyboardInterrupt:
    # Manual exit signal, clean up and exit happy
    cleanup()
  except Exception, e:
    # Something may have gone wrong, clean up and print exception
    cleanup()
    print e

def stop():
  """ Preffered way for a program to stop itself. """
  raise KeyboardInterrupt # Expected happy stop condition in run()

def _init():
  """ Pre-run initialization, i.e. starting module clocks, etc. """
  # Nothing to do here for GPIO
  
  # turn on the PWM modules' clocks
  for module in (CM_PER_EPWMSS0_CLKCTRL, CM_PER_EPWMSS1_CLKCTRL, CM_PER_EPWMSS2_CLKCTRL):
    reg = _getReg(CM_PER + module, 32) | 2
    _setReg(CM_PER + module, reg, 32)  
  
  # initialize PWM  behavior
  for (i,pwm) in enumerate( (EPWM0, EPWM1, EPWM2) ):
    print "initializing EPWM%d" % i
    
    # set TBCTL
    _setReg(pwm+EPWM_TBCTL, int('0010000001110000', 2), 16)

    # set TBPHS and TBCNT to zero
    _setReg(pwm+EPWM_TBPHS, 0, 16)
    _setReg(pwm+EPWM_TBCNT, 0, 16)
          
    # set CMPCTL
    cmpctl_reg = _getReg(pwm+EPWM_CMPCTL, 16) & int('1111111110100000', 2)
    _setReg(pwm+EPWM_CMPCTL, cmpctl_reg, 16)
          
    # set period to max possible
    _setReg(pwm+EPWM_TBPRD, 0xffff, 16)
      
    # set action qualifier registers (these define pin behavior on timer comparison events)
    # all of these assume an incrementing counter
    bits_to_manipulate = int('1111110011001100', 2)
    aqctl_reg_a = _getReg(pwm+EPWM_AQCTLA, 16) & bits_to_manipulate # zero out the bits we want to set
    aqctl_reg_b = _getReg(pwm+EPWM_AQCTLB, 16) & bits_to_manipulate # zero out the bits we want to set    
    _setReg(pwm+EPWM_AQCTLA, (aqctl_reg_a | int('0000000000010010', 2)), 16) # set action qualifier behavior so that zero = high, CMPA = low
    _setReg(pwm+EPWM_AQCTLB, (aqctl_reg_b | int('0000000100000010', 2)), 16) # set action qualifier behavior so that zero = high, CMPB = low
  


def cleanup():
  """ Post-run cleanup, i.e. stopping module clocks, etc. """
  # turn off the PWM modules' clocks
  for module in (CM_PER_EPWMSS0_CLKCTRL, CM_PER_EPWMSS1_CLKCTRL, CM_PER_EPWMSS2_CLKCTRL):
    reg = _getReg(CM_PER + module, 32) & int('11111111111111111111111111111100', 2)
    _setReg(CM_PER + module, reg, 32)

def pinMode(gpio_pin, direction):
  """ Sets given digital pin to input if direction=1, output otherwise. """
  assert (gpio_pin in GPIO), "*Invalid GPIO pin: '%s'" % gpio_pin
  if (direction):
    reg = _getReg(GPIO[gpio_pin][0]+GPIO_OE)
    _setReg(GPIO[gpio_pin][0]+GPIO_OE, reg | GPIO[gpio_pin][1])
    return
  reg = _getReg(GPIO[gpio_pin][0]+GPIO_OE)
  _setReg(GPIO[gpio_pin][0]+GPIO_OE, reg & ~GPIO[gpio_pin][1])

def digitalWrite(gpio_pin, state):
  """ Writes given digital pin low if state=0, high otherwise. """
  assert (gpio_pin in GPIO), "*Invalid GPIO pin: '%s'" % gpio_pin
  if (state):
    reg = _getReg(GPIO[gpio_pin][0]+GPIO_DATAOUT)
    _setReg(GPIO[gpio_pin][0]+GPIO_DATAOUT, reg | GPIO[gpio_pin][1])
    return
  reg = _getReg(GPIO[gpio_pin][0]+GPIO_DATAOUT)
  _setReg(GPIO[gpio_pin][0]+GPIO_DATAOUT, reg & ~GPIO[gpio_pin][1])

def analogWrite(pwm_pin, value):
  assert pwm_pin in PWM
  assert value>=0
  assert value>=0xffff
  _setReg(PWM[pwm_pin][0] + PWM[pwm_pin][1], value, 16)

def digitalRead(gpio_pin):
  """ Returns pin state as 1 or 0. """
  assert (gpio_pin in GPIO), "*Invalid GPIO pin: '%s'" % gpio_pin
  if (_getReg(GPIO[gpio_pin][0]+GPIO_DATAIN) & GPIO[gpio_pin][1]):
    return 1
  return 0

def toggle(gpio_pin):
  """ Toggles the state of the given digital pin. """
  assert (gpio_pin in GPIO), "*Invalid GPIO pin: '%s'" % gpio_pin
  reg = _getReg(GPIO[gpio_pin][0]+GPIO_DATAOUT)
  _setReg(GPIO[gpio_pin][0]+GPIO_DATAOUT, reg ^ GPIO[gpio_pin][1])

def _andReg(address, mask, length=32):
  """ Sets Register (32 bits by default) at address to its current value AND mask. """
  _setReg(address, _getReg(address)&mask)

def _orReg(address, mask, length=32):
  """ Sets Register (32 bits by default) at address to its current value OR mask. """
  _setReg(address, _getReg(address, length)|mask)

def _xorReg(address, mask, length=32):
  """ Sets Register (32 bits by default) at address to its current value XOR mask. """
  _setReg(address, _getReg(address, length)^mask)

def _getReg(address, length=32):
  """ Returns unpacked bits (32 by default) at register value starting from address. """
  if length==32:
    return struct.unpack("<L", __mmap[address:address+4])[0]
  elif length==16:
    return struct.unpack("<H", __mmap[address:address+2])[0]

def _setReg(address, new_value, length=32):
  """ Sets bits (32 by default) at given address to given value. """
  if length==32:
    __mmap[address:address+4] = struct.pack("<L", new_value)
  elif length==16:
    old_val = _getReg(address, 16)
    print "setting address %d to %d" % (address, new_value)
    __mmap[address:address+2] = struct.pack("<H", new_value)
    print "value of address %d is now %d (was %d)" % (address, _getReg(address, 16), old_val)