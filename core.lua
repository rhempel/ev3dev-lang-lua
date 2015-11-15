-- -----------------------------------------------------------------------------
-- core.lua - Lua binding for ev3dev peripherals
--
-- Copyright (c) 2015 Ralph Hempel <rhempel@hempeldesigngroup.com>
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
-- THE SOFTWARE.
-- -----------------------------------------------------------------------------
-- ~autogen autogen-header
-- Sections of the following code were auto-generated based on spec v0.9.3-pre, rev 2
-- ~autogen
-- -----------------------------------------------------------------------------

require 'lfs'

-- -----------------------------------------------------------------------------

require 'class'

-- -----------------------------------------------------------------------------
-- FileCache - a base class for reading and writing files that supports cached
--             file handles
--
-- TODO Take advantage of the __gc metatable field to handle the destructor

FileCache = class()

    function FileCache:init()
        self._cache = {}
    end

    function FileCache:file_handle(path, mode, reopen)
        reopen = reopen or false
        local f
	
        if nil == self._cache[path] then
           print( 'Open   ' .. path )
           f = io.open(path, mode)
           self._cache[path] = f
        elseif reopen then
           print( 'Reopen ' .. path )
           self._cache[path].close()
           f = io.open(path, mode)
           self._cache[path] = f
        else
           print( 'Return ' .. path )
           f = self._cache[path]
        end

        return f
    end

    function FileCache:read(path)
        local f, value

        f = self:file_handle(path, 'r')
        f:seek('set')
        value = f:read()

        if not value then
            f = self:file_handle(path, 'w+', true)
            value = f:read()
        end

        return value:match('^%s*(.-)%s*$')
    end

    function FileCache:write(path, value)
        local f, c

        f = self:file_handle(path, 'w')
        f:seek('set')
        c = f:write(value)

        if not c then
            f = self:file_handle(path, 'w+', true)
            f:write(value)
        end

        f:flush()
    end

-- -----------------------------------------------------------------------------
-- Device - a base class for manipulating ev3dev devices by reading and
--          writing their attribute files in the sys filesystem
--
-- TODO Take advantage of the __gc metatable field to handle the destructor

Device = class()

    Device.DEVICE_ROOT_PATH = '/sys/class'

    function Device:init(class_name, name, args)
        -- Spin through the Linux sysfs class for the device type and find
        -- a device that matches the provided name and attributes (if any).
        --
        -- Parameters:
        --
        --     class_name: class name of the device, a subdirectory of /sys/class.
        --         For example, 'tacho-motor'.
        --     name: pattern that device name should match.
        --         For example, 'sensor*' or 'motor*'. Default value: '*'.
        --     keyword arguments: used for matching the corresponding device
        --         attributes. For example, port_name='outA', or
        --         driver_name=['lego-ev3-us', 'lego-nxt-us']. When argument value
        --         is a list, then a match against any entry of the list is
        --         enough.
        --
        -- Example::
        --
        --     d = ev3dev.Device('tacho-motor', port_name='outA')
        --     s = ev3dev.Device('lego-sensor', driver_name=['lego-ev3-us', 'lego-nxt-us'])
        --
        -- When connected succesfully, the `connected` attribute is set to True.
    
        name = args['port_name'] or name or '*'

        print ( 'Calling init with ' .. (class_name or 'cn') .. ' ' .. (name or 'nm')  )
        local classpath = Device.DEVICE_ROOT_PATH .. '/' .. (class_name or 'blarg')

        -- Device:set('_attribute_cache', FileCache())
        self._attribute_cache = FileCache()
        print ( 'When creating a Device Instance, self is ', self )
        -- for k,v in pairs( self._ ) do print(k,v) end

        for file in lfs.dir( classpath ) do
            if file:match( name ) then
                self._path = classpath .. '/' .. file

                -- See if requested attributes match:
                -- TODO add this later
                if true then
                    self.connected = true
                    print( 'connected' )

                    -- TODO add index match clauses
                    return self
                end
            end
        end

        print( 'not connected' )
        self._path = ''
        self.connected = false
        return nil
    end
         
    function Device:_matches(attribute, pattern)
        -- TODO add _matches function
    end

    function Device:_get_attribute(attribute)
        print ( 'When getting attribute, self is ', self )
        -- for k,v in pairs( self._ ) do print(k,v) end

        return self._attribute_cache:read(self._path .. '/' .. attribute)
    end

    function Device:_set_attribute(attribute, value)
        return self._attribute_cache:write(self._path .. '/' .. attribute, value)
    end

    function Device:_get_attr_int(attribute)
        return self._attribute_cache:read(self._path .. '/' .. attribute) + 0
    end

    function Device:_set_attr_int(attribute, value)
        return self._attribute_cache:write(self._path .. '/' .. attribute, value)
    end

    function Device:_get_attr_string(attribute)
        return self._attribute_cache:read(self._path .. '/' .. attribute)
    end

    function Device:_set_attr_string(attribute, value)
        return self._attribute_cache:write(self._path .. '/' .. attribute, value)
    end

    function Device:_get_attr_line(attribute)
        return self._attribute_cache:read(self._path .. '/' .. attribute)
    end

    function Device:_get_attr_set(attribute)
        return self._attribute_cache:read(self._path .. '/' .. attribute)
    end

    function Device:_get_attr_from_set(attribute)
        return self._attribute_cache:read(self._path .. '/' .. attribute)
    end

-- ~autogen generic-class classes.motor>currentClass

Motor = Device:extend()
    
    -- The motor class provides a uniform interface for using motors with
    -- positional and directional feedback such as the EV3 and NXT motors.
    -- This feedback allows for precise control of the motors. This is the
    -- most common type of motor, so we just call it `motor`.

    Motor.SYSTEM_CLASS_NAME = 'tacho-motor'
    Motor.SYSTEM_DEVICE_NAME_CONVENTION = 'motor*'

    function Motor:init(port, name, args)
        name = name or Motor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, Motor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen

-- TODO add a gc metamethod binding
--  def __del__(self):
--      self.stop()

-- ~autogen generic-get-set classes.motor>currentClass

    Motor:set{ 
        command = { 
            -- Sends a command to the motor controller. See `commands` for a list of
            -- possible values.
            get = function(self) return 'command:write:only' end,
            set = function(self, value) return self:_set_attr_string('command', value) end
        },
        commands = { 
            -- Returns a list of commands that are supported by the motor
            -- controller. Possible values are `run-forever`, `run-to-abs-pos`, `run-to-rel-pos`,
            -- `run-timed`, `run-direct`, `stop` and `reset`. Not all commands may be supported.

            -- - `run-forever` will cause the motor to run until another command is sent.
            -- - `run-to-abs-pos` will run to an absolute position specified by `position_sp`
            --   and then stop using the command specified in `stop_command`.
            -- - `run-to-rel-pos` will run to a position relative to the current `position` value.
            --   The new position will be current `position` + `position_sp`. When the new
            --   position is reached, the motor will stop using the command specified by `stop_command`.
            -- - `run-timed` will run the motor for the amount of time specified in `time_sp`
            --   and then stop the motor using the command specified by `stop_command`.
            -- - `run-direct` will run the motor at the duty cycle specified by `duty_cycle_sp`.
            --   Unlike other run commands, changing `duty_cycle_sp` while running *will*
            --   take effect immediately.
            -- - `stop` will stop any of the run commands before they are complete using the
            --   command specified by `stop_command`.
            -- - `reset` will reset all of the motor parameter attributes to their default value.
            --   This will also have the effect of stopping the motor.
            get = function(self) return self:_get_attr_set('commands') end,
            set = function(self, value) return 'commands:read:only' end
        },
        count_per_rot = { 
            -- Returns the number of tacho counts in one rotation of the motor. Tacho counts
            -- are used by the position and speed attributes, so you can use this value
            -- to convert rotations or degrees to tacho counts. In the case of linear
            -- actuators, the units here will be counts per centimeter.
            get = function(self) return self:_get_attr_int('count_per_rot') end,
            set = function(self, value) return 'count_per_rot:read:only' end
        },
        driver_name = { 
            -- Returns the name of the driver that provides this tacho motor device.
            get = function(self) return self:_get_attr_string('driver_name') end,
            set = function(self, value) return 'driver_name:read:only' end
        },
        duty_cycle = { 
            -- Returns the current duty cycle of the motor. Units are percent. Values
            -- are -100 to 100.
            get = function(self) return self:_get_attr_int('duty_cycle') end,
            set = function(self, value) return 'duty_cycle:read:only' end
        },
        duty_cycle_sp = { 
            -- Writing sets the duty cycle setpoint. Reading returns the current value.
            -- Units are in percent. Valid values are -100 to 100. A negative value causes
            -- the motor to rotate in reverse. This value is only used when `speed_regulation`
            -- is off.
            get = function(self) return self:_get_attr_int('duty_cycle_sp') end,
            set = function(self, value) return self:_set_attr_int('duty_cycle_sp', value) end
        },
        encoder_polarity = { 
            -- Sets the polarity of the rotary encoder. This is an advanced feature to all
            -- use of motors that send inversed encoder signals to the EV3. This should
            -- be set correctly by the driver of a device. It You only need to change this
            -- value if you are using a unsupported device. Valid values are `normal` and
            -- `inversed`.
            get = function(self) return self:_get_attr_string('encoder_polarity') end,
            set = function(self, value) return self:_set_attr_string('encoder_polarity', value) end
        },
        polarity = { 
            -- Sets the polarity of the motor. With `normal` polarity, a positive duty
            -- cycle will cause the motor to rotate clockwise. With `inversed` polarity,
            -- a positive duty cycle will cause the motor to rotate counter-clockwise.
            -- Valid values are `normal` and `inversed`.
            get = function(self) return self:_get_attr_string('polarity') end,
            set = function(self, value) return self:_set_attr_string('polarity', value) end
        },
        port_name = { 
            -- Returns the name of the port that the motor is connected to.
            get = function(self) return self:_get_attr_string('port_name') end,
            set = function(self, value) return 'port_name:read:only' end
        },
        position = { 
            -- Returns the current position of the motor in pulses of the rotary
            -- encoder. When the motor rotates clockwise, the position will increase.
            -- Likewise, rotating counter-clockwise causes the position to decrease.
            -- Writing will set the position to that value.
            get = function(self) return self:_get_attr_int('position') end,
            set = function(self, value) return self:_set_attr_int('position', value) end
        },
        position_p = { 
            -- The proportional constant for the position PID.
            get = function(self) return self:_get_attr_int('hold_pid/Kp') end,
            set = function(self, value) return self:_set_attr_int('hold_pid/Kp', value) end
        },
        position_i = { 
            -- The integral constant for the position PID.
            get = function(self) return self:_get_attr_int('hold_pid/Ki') end,
            set = function(self, value) return self:_set_attr_int('hold_pid/Ki', value) end
        },
        position_d = { 
            -- The derivative constant for the position PID.
            get = function(self) return self:_get_attr_int('hold_pid/Kd') end,
            set = function(self, value) return self:_set_attr_int('hold_pid/Kd', value) end
        },
        position_sp = { 
            -- Writing specifies the target position for the `run-to-abs-pos` and `run-to-rel-pos`
            -- commands. Reading returns the current value. Units are in tacho counts. You
            -- can use the value returned by `counts_per_rot` to convert tacho counts to/from
            -- rotations or degrees.
            get = function(self) return self:_get_attr_int('position_sp') end,
            set = function(self, value) return self:_set_attr_int('position_sp', value) end
        },
        speed = { 
            -- Returns the current motor speed in tacho counts per second. Not, this is
            -- not necessarily degrees (although it is for LEGO motors). Use the `count_per_rot`
            -- attribute to convert this value to RPM or deg/sec.
            get = function(self) return self:_get_attr_int('speed') end,
            set = function(self, value) return 'speed:read:only' end
        },
        speed_sp = { 
            -- Writing sets the target speed in tacho counts per second used when `speed_regulation`
            -- is on. Reading returns the current value.  Use the `count_per_rot` attribute
            -- to convert RPM or deg/sec to tacho counts per second.
            get = function(self) return self:_get_attr_int('speed_sp') end,
            set = function(self, value) return self:_set_attr_int('speed_sp', value) end
        },
        ramp_up_sp = { 
            -- Writing sets the ramp up setpoint. Reading returns the current value. Units
            -- are in milliseconds. When set to a value > 0, the motor will ramp the power
            -- sent to the motor from 0 to 100% duty cycle over the span of this setpoint
            -- when starting the motor. If the maximum duty cycle is limited by `duty_cycle_sp`
            -- or speed regulation, the actual ramp time duration will be less than the setpoint.
            get = function(self) return self:_get_attr_int('ramp_up_sp') end,
            set = function(self, value) return self:_set_attr_int('ramp_up_sp', value) end
        },
        ramp_down_sp = { 
            -- Writing sets the ramp down setpoint. Reading returns the current value. Units
            -- are in milliseconds. When set to a value > 0, the motor will ramp the power
            -- sent to the motor from 100% duty cycle down to 0 over the span of this setpoint
            -- when stopping the motor. If the starting duty cycle is less than 100%, the
            -- ramp time duration will be less than the full span of the setpoint.
            get = function(self) return self:_get_attr_int('ramp_down_sp') end,
            set = function(self, value) return self:_set_attr_int('ramp_down_sp', value) end
        },
        speed_regulation_enabled = { 
            -- Turns speed regulation on or off. If speed regulation is on, the motor
            -- controller will vary the power supplied to the motor to try to maintain the
            -- speed specified in `speed_sp`. If speed regulation is off, the controller
            -- will use the power specified in `duty_cycle_sp`. Valid values are `on` and
            -- `off`.
            get = function(self) return self:_get_attr_string('speed_regulation') end,
            set = function(self, value) return self:_set_attr_string('speed_regulation', value) end
        },
        speed_regulation_p = { 
            -- The proportional constant for the speed regulation PID.
            get = function(self) return self:_get_attr_int('speed_pid/Kp') end,
            set = function(self, value) return self:_set_attr_int('speed_pid/Kp', value) end
        },
        speed_regulation_i = { 
            -- The integral constant for the speed regulation PID.
            get = function(self) return self:_get_attr_int('speed_pid/Ki') end,
            set = function(self, value) return self:_set_attr_int('speed_pid/Ki', value) end
        },
        speed_regulation_d = { 
            -- The derivative constant for the speed regulation PID.
            get = function(self) return self:_get_attr_int('speed_pid/Kd') end,
            set = function(self, value) return self:_set_attr_int('speed_pid/Kd', value) end
        },
        state = { 
            -- Reading returns a list of state flags. Possible flags are
            -- `running`, `ramping` `holding` and `stalled`.
            get = function(self) return self:_get_attr_set('state') end,
            set = function(self, value) return 'state:read:only' end
        },
        stop_command = { 
            -- Reading returns the current stop command. Writing sets the stop command.
            -- The value determines the motors behavior when `command` is set to `stop`.
            -- Also, it determines the motors behavior when a run command completes. See
            -- `stop_commands` for a list of possible values.
            get = function(self) return self:_get_attr_string('stop_command') end,
            set = function(self, value) return self:_set_attr_string('stop_command', value) end
        },
        stop_commands = { 
            -- Returns a list of stop modes supported by the motor controller.
            -- Possible values are `coast`, `brake` and `hold`. `coast` means that power will
            -- be removed from the motor and it will freely coast to a stop. `brake` means
            -- that power will be removed from the motor and a passive electrical load will
            -- be placed on the motor. This is usually done by shorting the motor terminals
            -- together. This load will absorb the energy from the rotation of the motors and
            -- cause the motor to stop more quickly than coasting. `hold` does not remove
            -- power from the motor. Instead it actively try to hold the motor at the current
            -- position. If an external force tries to turn the motor, the motor will 'push
            -- back' to maintain its position.
            get = function(self) return self:_get_attr_set('stop_commands') end,
            set = function(self, value) return 'stop_commands:read:only' end
        },
        time_sp = { 
            -- Writing specifies the amount of time the motor will run when using the
            -- `run-timed` command. Reading returns the current value. Units are in
            -- milliseconds.
            get = function(self) return self:_get_attr_int('time_sp') end,
            set = function(self, value) return self:_set_attr_int('time_sp', value) end
        },
    }
-- ~autogen
-- ~autogen generic-property-value classes.motor>currentClass

    -- Run the motor until another command is sent.
    COMMAND_RUN_FOREVER = 'run-forever'

    -- Run to an absolute position specified by `position_sp` and then
    -- stop using the command specified in `stop_command`.
    COMMAND_RUN_TO_ABS_POS = 'run-to-abs-pos'

    -- Run to a position relative to the current `position` value.
    -- The new position will be current `position` + `position_sp`.
    -- When the new position is reached, the motor will stop using
    -- the command specified by `stop_command`.
    COMMAND_RUN_TO_REL_POS = 'run-to-rel-pos'

    -- Run the motor for the amount of time specified in `time_sp`
    -- and then stop the motor using the command specified by `stop_command`.
    COMMAND_RUN_TIMED = 'run-timed'

    -- Run the motor at the duty cycle specified by `duty_cycle_sp`.
    -- Unlike other run commands, changing `duty_cycle_sp` while running *will*
    -- take effect immediately.
    COMMAND_RUN_DIRECT = 'run-direct'

    -- Stop any of the run commands before they are complete using the
    -- command specified by `stop_command`.
    COMMAND_STOP = 'stop'

    -- Reset all of the motor parameter attributes to their default value.
    -- This will also have the effect of stopping the motor.
    COMMAND_RESET = 'reset'

    -- Sets the normal polarity of the rotary encoder.
    ENCODER_POLARITY_NORMAL = 'normal'

    -- Sets the inversed polarity of the rotary encoder.
    ENCODER_POLARITY_INVERSED = 'inversed'

    -- With `normal` polarity, a positive duty cycle will
    -- cause the motor to rotate clockwise.
    POLARITY_NORMAL = 'normal'

    -- With `inversed` polarity, a positive duty cycle will
    -- cause the motor to rotate counter-clockwise.
    POLARITY_INVERSED = 'inversed'

    -- The motor controller will vary the power supplied to the motor
    -- to try to maintain the speed specified in `speed_sp`.
    SPEED_REGULATION_ON = 'on'

    -- The motor controller will use the power specified in `duty_cycle_sp`.
    SPEED_REGULATION_OFF = 'off'

    -- Power will be removed from the motor and it will freely coast to a stop.
    STOP_COMMAND_COAST = 'coast'

    -- Power will be removed from the motor and a passive electrical load will
    -- be placed on the motor. This is usually done by shorting the motor terminals
    -- together. This load will absorb the energy from the rotation of the motors and
    -- cause the motor to stop more quickly than coasting.
    STOP_COMMAND_BRAKE = 'brake'

    -- Does not remove power from the motor. Instead it actively try to hold the motor
    -- at the current position. If an external force tries to turn the motor, the motor
    -- will ``push back`` to maintain its position.
    STOP_COMMAND_HOLD = 'hold'

-- ~autogen
-- ~autogen motor_commands classes.motor>currentClass

    function Motor:run_forever(args)
        -- Run the motor until another command is sent.
        args = args or {}
        for k,v in pairs(args) do
            self[k] = v
        end
        self.command = 'run-forever'
    end

    function Motor:run_to_abs_pos(args)
        -- Run to an absolute position specified by `position_sp` and then
        -- stop using the command specified in `stop_command`.
        args = args or {}
        for k,v in pairs(args) do
            self[k] = v
        end
        self.command = 'run-to-abs-pos'
    end

    function Motor:run_to_rel_pos(args)
        -- Run to a position relative to the current `position` value.
        -- The new position will be current `position` + `position_sp`.
        -- When the new position is reached, the motor will stop using
        -- the command specified by `stop_command`.
        args = args or {}
        for k,v in pairs(args) do
            self[k] = v
        end
        self.command = 'run-to-rel-pos'
    end

    function Motor:run_timed(args)
        -- Run the motor for the amount of time specified in `time_sp`
        -- and then stop the motor using the command specified by `stop_command`.
        args = args or {}
        for k,v in pairs(args) do
            self[k] = v
        end
        self.command = 'run-timed'
    end

    function Motor:run_direct(args)
        -- Run the motor at the duty cycle specified by `duty_cycle_sp`.
        -- Unlike other run commands, changing `duty_cycle_sp` while running *will*
        -- take effect immediately.
        args = args or {}
        for k,v in pairs(args) do
            self[k] = v
        end
        self.command = 'run-direct'
    end

    function Motor:stop(args)
        -- Stop any of the run commands before they are complete using the
        -- command specified by `stop_command`.
        args = args or {}
        for k,v in pairs(args) do
            self[k] = v
        end
        self.command = 'stop'
    end

    function Motor:reset(args)
        -- Reset all of the motor parameter attributes to their default value.
        -- This will also have the effect of stopping the motor.
        args = args or {}
        for k,v in pairs(args) do
            self[k] = v
        end
        self.command = 'reset'
    end

-- ~autogen
-- ~autogen generic-class classes.largeMotor>currentClass

LargeMotor = Device:extend()
    
    -- EV3 large servo motor

    LargeMotor.SYSTEM_CLASS_NAME = Motor.SYSTEM_CLASS_NAME
    LargeMotor.SYSTEM_DEVICE_NAME_CONVENTION = Motor.SYSTEM_DEVICE_NAME_CONVENTION

    function LargeMotor:init(port, name, args)
        name = name or LargeMotor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, LargeMotor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-class classes.mediumMotor>currentClass

MediumMotor = Device:extend()
    
    -- EV3 medium servo motor

    MediumMotor.SYSTEM_CLASS_NAME = Motor.SYSTEM_CLASS_NAME
    MediumMotor.SYSTEM_DEVICE_NAME_CONVENTION = Motor.SYSTEM_DEVICE_NAME_CONVENTION

    function MediumMotor:init(port, name, args)
        name = name or MediumMotor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, MediumMotor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-class classes.dcMotor>currentClass

DcMotor = Device:extend()
    
    -- The DC motor class provides a uniform interface for using regular DC motors
    -- with no fancy controls or feedback. This includes LEGO MINDSTORMS RCX motors
    -- and LEGO Power Functions motors.

    DcMotor.SYSTEM_CLASS_NAME = 'dc-motor'
    DcMotor.SYSTEM_DEVICE_NAME_CONVENTION = 'motor*'

    function DcMotor:init(port, name, args)
        name = name or DcMotor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, DcMotor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen

-- TODO add a gc metamethod binding
--  def __del__(self):
--      self.stop()

-- ~autogen generic-get-set classes.dcMotor>currentClass

    DcMotor:set{ 
        command = { 
            -- Sets the command for the motor. Possible values are `run-forever`, `run-timed` and
            -- `stop`. Not all commands may be supported, so be sure to check the contents
            -- of the `commands` attribute.
            get = function(self) return 'command:write:only' end,
            set = function(self, value) return self:_set_attr_string('command', value) end
        },
        commands = { 
            -- Returns a list of commands supported by the motor
            -- controller.
            get = function(self) return self:_get_attr_set('commands') end,
            set = function(self, value) return 'commands:read:only' end
        },
        driver_name = { 
            -- Returns the name of the motor driver that loaded this device. See the list
            -- of [supported devices] for a list of drivers.
            get = function(self) return self:_get_attr_string('driver_name') end,
            set = function(self, value) return 'driver_name:read:only' end
        },
        duty_cycle = { 
            -- Shows the current duty cycle of the PWM signal sent to the motor. Values
            -- are -100 to 100 (-100% to 100%).
            get = function(self) return self:_get_attr_int('duty_cycle') end,
            set = function(self, value) return 'duty_cycle:read:only' end
        },
        duty_cycle_sp = { 
            -- Writing sets the duty cycle setpoint of the PWM signal sent to the motor.
            -- Valid values are -100 to 100 (-100% to 100%). Reading returns the current
            -- setpoint.
            get = function(self) return self:_get_attr_int('duty_cycle_sp') end,
            set = function(self, value) return self:_set_attr_int('duty_cycle_sp', value) end
        },
        polarity = { 
            -- Sets the polarity of the motor. Valid values are `normal` and `inversed`.
            get = function(self) return self:_get_attr_string('polarity') end,
            set = function(self, value) return self:_set_attr_string('polarity', value) end
        },
        port_name = { 
            -- Returns the name of the port that the motor is connected to.
            get = function(self) return self:_get_attr_string('port_name') end,
            set = function(self, value) return 'port_name:read:only' end
        },
        ramp_down_sp = { 
            -- Sets the time in milliseconds that it take the motor to ramp down from 100%
            -- to 0%. Valid values are 0 to 10000 (10 seconds). Default is 0.
            get = function(self) return self:_get_attr_int('ramp_down_sp') end,
            set = function(self, value) return self:_set_attr_int('ramp_down_sp', value) end
        },
        ramp_up_sp = { 
            -- Sets the time in milliseconds that it take the motor to up ramp from 0% to
            -- 100%. Valid values are 0 to 10000 (10 seconds). Default is 0.
            get = function(self) return self:_get_attr_int('ramp_up_sp') end,
            set = function(self, value) return self:_set_attr_int('ramp_up_sp', value) end
        },
        state = { 
            -- Gets a list of flags indicating the motor status. Possible
            -- flags are `running` and `ramping`. `running` indicates that the motor is
            -- powered. `ramping` indicates that the motor has not yet reached the
            -- `duty_cycle_sp`.
            get = function(self) return self:_get_attr_set('state') end,
            set = function(self, value) return 'state:read:only' end
        },
        stop_command = { 
            -- Sets the stop command that will be used when the motor stops. Read
            -- `stop_commands` to get the list of valid values.
            get = function(self) return 'stop_command:write:only' end,
            set = function(self, value) return self:_set_attr_string('stop_command', value) end
        },
        stop_commands = { 
            -- Gets a list of stop commands. Valid values are `coast`
            -- and `brake`.
            get = function(self) return self:_get_attr_set('stop_commands') end,
            set = function(self, value) return 'stop_commands:read:only' end
        },
        time_sp = { 
            -- Writing specifies the amount of time the motor will run when using the
            -- `run-timed` command. Reading returns the current value. Units are in
            -- milliseconds.
            get = function(self) return self:_get_attr_int('time_sp') end,
            set = function(self, value) return self:_set_attr_int('time_sp', value) end
        },
    }
-- ~autogen
-- ~autogen generic-property-value classes.dcMotor>currentClass

    -- Run the motor until another command is sent.
    COMMAND_RUN_FOREVER = 'run-forever'

    -- Run the motor for the amount of time specified in `time_sp`
    -- and then stop the motor using the command specified by `stop_command`.
    COMMAND_RUN_TIMED = 'run-timed'

    -- Run the motor at the duty cycle specified by `duty_cycle_sp`.
    -- Unlike other run commands, changing `duty_cycle_sp` while running *will*
    -- take effect immediately.
    COMMAND_RUN_DIRECT = 'run-direct'

    -- Stop any of the run commands before they are complete using the
    -- command specified by `stop_command`.
    COMMAND_STOP = 'stop'

    -- With `normal` polarity, a positive duty cycle will
    -- cause the motor to rotate clockwise.
    POLARITY_NORMAL = 'normal'

    -- With `inversed` polarity, a positive duty cycle will
    -- cause the motor to rotate counter-clockwise.
    POLARITY_INVERSED = 'inversed'

    -- Power will be removed from the motor and it will freely coast to a stop.
    STOP_COMMAND_COAST = 'coast'

    -- Power will be removed from the motor and a passive electrical load will
    -- be placed on the motor. This is usually done by shorting the motor terminals
    -- together. This load will absorb the energy from the rotation of the motors and
    -- cause the motor to stop more quickly than coasting.
    STOP_COMMAND_BRAKE = 'brake'

-- ~autogen
-- ~autogen motor_commands classes.dcMotor>currentClass

    function DcMotor:run_forever(args)
        -- Run the motor until another command is sent.
        args = args or {}
        for k,v in pairs(args) do
            self[k] = v
        end
        self.command = 'run-forever'
    end

    function DcMotor:run_timed(args)
        -- Run the motor for the amount of time specified in `time_sp`
        -- and then stop the motor using the command specified by `stop_command`.
        args = args or {}
        for k,v in pairs(args) do
            self[k] = v
        end
        self.command = 'run-timed'
    end

    function DcMotor:run_direct(args)
        -- Run the motor at the duty cycle specified by `duty_cycle_sp`.
        -- Unlike other run commands, changing `duty_cycle_sp` while running *will*
        -- take effect immediately.
        args = args or {}
        for k,v in pairs(args) do
            self[k] = v
        end
        self.command = 'run-direct'
    end

    function DcMotor:stop(args)
        -- Stop any of the run commands before they are complete using the
        -- command specified by `stop_command`.
        args = args or {}
        for k,v in pairs(args) do
            self[k] = v
        end
        self.command = 'stop'
    end

-- ~autogen
-- ~autogen generic-class classes.servoMotor>currentClass

ServoMotor = Device:extend()
    
    -- The servo motor class provides a uniform interface for using hobby type
    -- servo motors.

    ServoMotor.SYSTEM_CLASS_NAME = 'servo-motor'
    ServoMotor.SYSTEM_DEVICE_NAME_CONVENTION = 'motor*'

    function ServoMotor:init(port, name, args)
        name = name or ServoMotor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, ServoMotor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen

-- TODO add a gc metamethod binding
--  def __del__(self):
--      self.float()

-- ~autogen generic-get-set classes.servoMotor>currentClass

    ServoMotor:set{ 
        command = { 
            -- Sets the command for the servo. Valid values are `run` and `float`. Setting
            -- to `run` will cause the servo to be driven to the position_sp set in the
            -- `position_sp` attribute. Setting to `float` will remove power from the motor.
            get = function(self) return 'command:write:only' end,
            set = function(self, value) return self:_set_attr_string('command', value) end
        },
        driver_name = { 
            -- Returns the name of the motor driver that loaded this device. See the list
            -- of [supported devices] for a list of drivers.
            get = function(self) return self:_get_attr_string('driver_name') end,
            set = function(self, value) return 'driver_name:read:only' end
        },
        max_pulse_sp = { 
            -- Used to set the pulse size in milliseconds for the signal that tells the
            -- servo to drive to the maximum (clockwise) position_sp. Default value is 2400.
            -- Valid values are 2300 to 2700. You must write to the position_sp attribute for
            -- changes to this attribute to take effect.
            get = function(self) return self:_get_attr_int('max_pulse_sp') end,
            set = function(self, value) return self:_set_attr_int('max_pulse_sp', value) end
        },
        mid_pulse_sp = { 
            -- Used to set the pulse size in milliseconds for the signal that tells the
            -- servo to drive to the mid position_sp. Default value is 1500. Valid
            -- values are 1300 to 1700. For example, on a 180 degree servo, this would be
            -- 90 degrees. On continuous rotation servo, this is the 'neutral' position_sp
            -- where the motor does not turn. You must write to the position_sp attribute for
            -- changes to this attribute to take effect.
            get = function(self) return self:_get_attr_int('mid_pulse_sp') end,
            set = function(self, value) return self:_set_attr_int('mid_pulse_sp', value) end
        },
        min_pulse_sp = { 
            -- Used to set the pulse size in milliseconds for the signal that tells the
            -- servo to drive to the miniumum (counter-clockwise) position_sp. Default value
            -- is 600. Valid values are 300 to 700. You must write to the position_sp
            -- attribute for changes to this attribute to take effect.
            get = function(self) return self:_get_attr_int('min_pulse_sp') end,
            set = function(self, value) return self:_set_attr_int('min_pulse_sp', value) end
        },
        polarity = { 
            -- Sets the polarity of the servo. Valid values are `normal` and `inversed`.
            -- Setting the value to `inversed` will cause the position_sp value to be
            -- inversed. i.e `-100` will correspond to `max_pulse_sp`, and `100` will
            -- correspond to `min_pulse_sp`.
            get = function(self) return self:_get_attr_string('polarity') end,
            set = function(self, value) return self:_set_attr_string('polarity', value) end
        },
        port_name = { 
            -- Returns the name of the port that the motor is connected to.
            get = function(self) return self:_get_attr_string('port_name') end,
            set = function(self, value) return 'port_name:read:only' end
        },
        position_sp = { 
            -- Reading returns the current position_sp of the servo. Writing instructs the
            -- servo to move to the specified position_sp. Units are percent. Valid values
            -- are -100 to 100 (-100% to 100%) where `-100` corresponds to `min_pulse_sp`,
            -- `0` corresponds to `mid_pulse_sp` and `100` corresponds to `max_pulse_sp`.
            get = function(self) return self:_get_attr_int('position_sp') end,
            set = function(self, value) return self:_set_attr_int('position_sp', value) end
        },
        rate_sp = { 
            -- Sets the rate_sp at which the servo travels from 0 to 100.0% (half of the full
            -- range of the servo). Units are in milliseconds. Example: Setting the rate_sp
            -- to 1000 means that it will take a 180 degree servo 2 second to move from 0
            -- to 180 degrees. Note: Some servo controllers may not support this in which
            -- case reading and writing will fail with `-EOPNOTSUPP`. In continuous rotation
            -- servos, this value will affect the rate_sp at which the speed ramps up or down.
            get = function(self) return self:_get_attr_int('rate_sp') end,
            set = function(self, value) return self:_set_attr_int('rate_sp', value) end
        },
        state = { 
            -- Returns a list of flags indicating the state of the servo.
            -- Possible values are:
            -- * `running`: Indicates that the motor is powered.
            get = function(self) return self:_get_attr_set('state') end,
            set = function(self, value) return 'state:read:only' end
        },
    }
-- ~autogen
-- ~autogen generic-property-value classes.servoMotor>currentClass

    -- Drive servo to the position set in the `position_sp` attribute.
    COMMAND_RUN = 'run'

    -- Remove power from the motor.
    COMMAND_FLOAT = 'float'

    -- With `normal` polarity, a positive duty cycle will
    -- cause the motor to rotate clockwise.
    POLARITY_NORMAL = 'normal'

    -- With `inversed` polarity, a positive duty cycle will
    -- cause the motor to rotate counter-clockwise.
    POLARITY_INVERSED = 'inversed'

-- ~autogen
-- ~autogen generic-class classes.sensor>currentClass

Sensor = Device:extend()
    
    -- The sensor class provides a uniform interface for using most of the
    -- sensors available for the EV3. The various underlying device drivers will
    -- create a `lego-sensor` device for interacting with the sensors.

    -- Sensors are primarily controlled by setting the `mode` and monitored by
    -- reading the `value<N>` attributes. Values can be converted to floating point
    -- if needed by `value<N>` / 10.0 ^ `decimals`.

    -- Since the name of the `sensor<N>` device node does not correspond to the port
    -- that a sensor is plugged in to, you must look at the `port_name` attribute if
    -- you need to know which port a sensor is plugged in to. However, if you don't
    -- have more than one sensor of each type, you can just look for a matching
    -- `driver_name`. Then it will not matter which port a sensor is plugged in to - your
    -- program will still work.

    Sensor.SYSTEM_CLASS_NAME = 'lego-sensor'
    Sensor.SYSTEM_DEVICE_NAME_CONVENTION = 'sensor*'

    function Sensor:init(port, name, args)
        name = name or Sensor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, Sensor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-get-set classes.sensor>currentClass

    Sensor:set{ 
        command = { 
            -- Sends a command to the sensor.
            get = function(self) return 'command:write:only' end,
            set = function(self, value) return self:_set_attr_string('command', value) end
        },
        commands = { 
            -- Returns a list of the valid commands for the sensor.
            -- Returns -EOPNOTSUPP if no commands are supported.
            get = function(self) return self:_get_attr_set('commands') end,
            set = function(self, value) return 'commands:read:only' end
        },
        decimals = { 
            -- Returns the number of decimal places for the values in the `value<N>`
            -- attributes of the current mode.
            get = function(self) return self:_get_attr_int('decimals') end,
            set = function(self, value) return 'decimals:read:only' end
        },
        driver_name = { 
            -- Returns the name of the sensor device/driver. See the list of [supported
            -- sensors] for a complete list of drivers.
            get = function(self) return self:_get_attr_string('driver_name') end,
            set = function(self, value) return 'driver_name:read:only' end
        },
        mode = { 
            -- Returns the current mode. Writing one of the values returned by `modes`
            -- sets the sensor to that mode.
            get = function(self) return self:_get_attr_string('mode') end,
            set = function(self, value) return self:_set_attr_string('mode', value) end
        },
        modes = { 
            -- Returns a list of the valid modes for the sensor.
            get = function(self) return self:_get_attr_set('modes') end,
            set = function(self, value) return 'modes:read:only' end
        },
        num_values = { 
            -- Returns the number of `value<N>` attributes that will return a valid value
            -- for the current mode.
            get = function(self) return self:_get_attr_int('num_values') end,
            set = function(self, value) return 'num_values:read:only' end
        },
        port_name = { 
            -- Returns the name of the port that the sensor is connected to, e.g. `ev3:in1`.
            -- I2C sensors also include the I2C address (decimal), e.g. `ev3:in1:i2c8`.
            get = function(self) return self:_get_attr_string('port_name') end,
            set = function(self, value) return 'port_name:read:only' end
        },
        units = { 
            -- Returns the units of the measured value for the current mode. May return
            -- empty string
            get = function(self) return self:_get_attr_string('units') end,
            set = function(self, value) return 'units:read:only' end
        },
    }
-- ~autogen

-- TODO add the value and binary data formating helpers
--  def value(self, n=0):
--      if isinstance(n, numbers.Integral):
--          n = '{0:d}'.format(n)
--      elif isinstance(n, numbers.Real):
--          n = '{0:.0f}'.format(n)

--      if isinstance(n, str):
--          return self.get_attr_int('value'+n)
--      else:
--          return 0

--  @property
--  def bin_data_format(self):
--      """
--      Returns the format of the values in `bin_data` for the current mode.
--      Possible values are:

--      - `u8`: Unsigned 8-bit integer (byte)
--      - `s8`: Signed 8-bit integer (sbyte)
--      - `u16`: Unsigned 16-bit integer (ushort)
--      - `s16`: Signed 16-bit integer (short)
--      - `s16_be`: Signed 16-bit integer, big endian
--      - `s32`: Signed 32-bit integer (int)
--      - `float`: IEEE 754 32-bit floating point (float)
--      """
--      return self.get_attr_string('bin_data_format')

--  def bin_data(self, fmt=None):
--      """
--      Returns the unscaled raw values in the `value<N>` attributes as raw byte
--      array. Use `bin_data_format`, `num_values` and the individual sensor
--      documentation to determine how to interpret the data.

--      Use `fmt` to unpack the raw bytes into a struct.

--      Example::

--          >>> from ev3dev import *
--          >>> ir = InfraredSensor()
--          >>> ir.value()
--          28
--          >>> ir.bin_data('<b')
--          (28,)
--      """

--      if '_bin_data_size' not in self.__dict__:
--          self._bin_data_size = {
--                  "u8":     1,
--                  "s8":     1,
--                  "u16":    2,
--                  "s16":    2,
--                  "s16_be": 2,
--                  "s32":    4,
--                  "float":  4
--              }.get(self.bin_data_format, 1) * self.num_values

--      f = self._attribute_cache.file_handle(abspath(self._path + '/bin_data'), binary=True)
--      f.seek(0)
--      raw = bytearray(f.read(self._bin_data_size))

--      if fmt is None: return raw

--      return unpack(fmt, raw)

-- ~autogen generic-class classes.i2cSensor>currentClass

I2cSensor = Device:extend()
    
    -- A generic interface to control I2C-type EV3 sensors.

    I2cSensor.SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    I2cSensor.SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    function I2cSensor:init(port, name, args)
        name = name or I2cSensor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, I2cSensor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-get-set classes.i2cSensor>currentClass

    I2cSensor:set{ 
        fw_version = { 
            -- Returns the firmware version of the sensor if available. Currently only
            -- I2C/NXT sensors support this.
            get = function(self) return self:_get_attr_string('fw_version') end,
            set = function(self, value) return 'fw_version:read:only' end
        },
        poll_ms = { 
            -- Returns the polling period of the sensor in milliseconds. Writing sets the
            -- polling period. Setting to 0 disables polling. Minimum value is hard
            -- coded as 50 msec. Returns -EOPNOTSUPP if changing polling is not supported.
            -- Currently only I2C/NXT sensors support changing the polling period.
            get = function(self) return self:_get_attr_int('poll_ms') end,
            set = function(self, value) return self:_set_attr_int('poll_ms', value) end
        },
    }
-- ~autogen
-- ~autogen generic-class classes.colorSensor>currentClass

ColorSensor = Device:extend()
    
    -- LEGO EV3 color sensor.

    ColorSensor.SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    ColorSensor.SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    function ColorSensor:init(port, name, args)
        name = name or ColorSensor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, ColorSensor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-property-value classes.colorSensor>currentClass

    -- Reflected light. Red LED on.
    MODE_COL_REFLECT = 'COL-REFLECT'

    -- Ambient light. Red LEDs off.
    MODE_COL_AMBIENT = 'COL-AMBIENT'

    -- Color. All LEDs rapidly cycling, appears white.
    MODE_COL_COLOR = 'COL-COLOR'

    -- Raw reflected. Red LED on
    MODE_REF_RAW = 'REF-RAW'

    -- Raw Color Components. All LEDs rapidly cycling, appears white.
    MODE_RGB_RAW = 'RGB-RAW'

-- ~autogen
-- ~autogen generic-class classes.ultrasonicSensor>currentClass

UltrasonicSensor = Device:extend()
    
    -- LEGO EV3 ultrasonic sensor.

    UltrasonicSensor.SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    UltrasonicSensor.SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    function UltrasonicSensor:init(port, name, args)
        name = name or UltrasonicSensor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, UltrasonicSensor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-property-value classes.ultrasonicSensor>currentClass

    -- Continuous measurement in centimeters.
    -- LEDs: On, steady
    MODE_US_DIST_CM = 'US-DIST-CM'

    -- Continuous measurement in inches.
    -- LEDs: On, steady
    MODE_US_DIST_IN = 'US-DIST-IN'

    -- Listen.  LEDs: On, blinking
    MODE_US_LISTEN = 'US-LISTEN'

    -- Single measurement in centimeters.
    -- LEDs: On momentarily when mode is set, then off
    MODE_US_SI_CM = 'US-SI-CM'

    -- Single measurement in inches.
    -- LEDs: On momentarily when mode is set, then off
    MODE_US_SI_IN = 'US-SI-IN'

-- ~autogen
-- ~autogen generic-class classes.gyroSensor>currentClass

GyroSensor = Device:extend()
    
    -- LEGO EV3 gyro sensor.

    GyroSensor.SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    GyroSensor.SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    function GyroSensor:init(port, name, args)
        name = name or GyroSensor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, GyroSensor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-property-value classes.gyroSensor>currentClass

    -- Angle
    MODE_GYRO_ANG = 'GYRO-ANG'

    -- Rotational speed
    MODE_GYRO_RATE = 'GYRO-RATE'

    -- Raw sensor value
    MODE_GYRO_FAS = 'GYRO-FAS'

    -- Angle and rotational speed
    MODE_GYRO_G_A = 'GYRO-G&A'

    -- Calibration ???
    MODE_GYRO_CAL = 'GYRO-CAL'

-- ~autogen
-- ~autogen generic-class classes.infraredSensor>currentClass

InfraredSensor = Device:extend()
    
    -- LEGO EV3 infrared sensor.

    InfraredSensor.SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    InfraredSensor.SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    function InfraredSensor:init(port, name, args)
        name = name or InfraredSensor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, InfraredSensor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-property-value classes.infraredSensor>currentClass

    -- Proximity
    MODE_IR_PROX = 'IR-PROX'

    -- IR Seeker
    MODE_IR_SEEK = 'IR-SEEK'

    -- IR Remote Control
    MODE_IR_REMOTE = 'IR-REMOTE'

    -- IR Remote Control. State of the buttons is coded in binary
    MODE_IR_REM_A = 'IR-REM-A'

    -- Calibration ???
    MODE_IR_CAL = 'IR-CAL'

-- ~autogen
-- ~autogen generic-class classes.soundSensor>currentClass

SoundSensor = Device:extend()
    
    -- LEGO NXT Sound Sensor

    SoundSensor.SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SoundSensor.SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    function SoundSensor:init(port, name, args)
        name = name or SoundSensor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, SoundSensor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-property-value classes.soundSensor>currentClass

    -- Sound pressure level. Flat weighting
    MODE_DB = 'DB'

    -- Sound pressure level. A weighting
    MODE_DBA = 'DBA'

-- ~autogen
-- ~autogen generic-class classes.lightSensor>currentClass

LightSensor = Device:extend()
    
    -- LEGO NXT Light Sensor

    LightSensor.SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    LightSensor.SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    function LightSensor:init(port, name, args)
        name = name or LightSensor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, LightSensor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-property-value classes.lightSensor>currentClass

    -- Reflected light. LED on
    MODE_REFLECT = 'REFLECT'

    -- Ambient light. LED off
    MODE_AMBIENT = 'AMBIENT'

-- ~autogen
-- ~autogen generic-class classes.touchSensor>currentClass

TouchSensor = Device:extend()
    
    -- Touch Sensor

    TouchSensor.SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    TouchSensor.SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    function TouchSensor:init(port, name, args)
        name = name or TouchSensor.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, TouchSensor.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-class classes.led>currentClass

Led = Device:extend()
    
    -- Any device controlled by the generic LED driver.
    -- See https://www.kernel.org/doc/Documentation/leds/leds-class.txt
    -- for more details.

    Led.SYSTEM_CLASS_NAME = 'leds'
    Led.SYSTEM_DEVICE_NAME_CONVENTION = '*'

    function Led:init(port, name, args)
        name = name or Led.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, Led.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-get-set classes.led>currentClass

    Led:set{ 
        max_brightness = { 
            -- Returns the maximum allowable brightness value.
            get = function(self) return self:_get_attr_int('max_brightness') end,
            set = function(self, value) return 'max_brightness:read:only' end
        },
        brightness = { 
            -- Sets the brightness level. Possible values are from 0 to `max_brightness`.
            get = function(self) return self:_get_attr_int('brightness') end,
            set = function(self, value) return self:_set_attr_int('brightness', value) end
        },
        triggers = { 
            -- Returns a list of available triggers.
            get = function(self) return self:_get_attr_set('trigger') end,
            set = function(self, value) return 'trigger:read:only' end
        },
        trigger = { 
            -- Sets the led trigger. A trigger
            -- is a kernel based source of led events. Triggers can either be simple or
            -- complex. A simple trigger isn't configurable and is designed to slot into
            -- existing subsystems with minimal additional code. Examples are the `ide-disk` and
            -- `nand-disk` triggers.

            -- Complex triggers whilst available to all LEDs have LED specific
            -- parameters and work on a per LED basis. The `timer` trigger is an example.
            -- The `timer` trigger will periodically change the LED brightness between
            -- 0 and the current brightness setting. The `on` and `off` time can
            -- be specified via `delay_{on,off}` attributes in milliseconds.
            -- You can change the brightness value of a LED independently of the timer
            -- trigger. However, if you set the brightness value to 0 it will
            -- also disable the `timer` trigger.
            get = function(self) return self:_get_attr_from_set('trigger') end,
            set = function(self, value) return self:_set_attr_string('trigger', value) end
        },
        delay_on = { 
            -- The `timer` trigger will periodically change the LED brightness between
            -- 0 and the current brightness setting. The `on` time can
            -- be specified via `delay_on` attribute in milliseconds.
            get = function(self) return self:_get_attr_int('delay_on') end,
            set = function(self, value) return self:_set_attr_int('delay_on', value) end
        },
        delay_off = { 
            -- The `timer` trigger will periodically change the LED brightness between
            -- 0 and the current brightness setting. The `off` time can
            -- be specified via `delay_off` attribute in milliseconds.
            get = function(self) return self:_get_attr_int('delay_off') end,
            set = function(self, value) return self:_set_attr_int('delay_off', value) end
        },
    }
-- ~autogen

-- TODO add the LED and Button hand generated code
--     @property
--     def brightness_pct(self):
--         """
--         Returns led brightness as a fraction of max_brightness
--         """
--         return float(self.brightness) / self.max_brightness
-- 
--     @brightness_pct.setter
--     def brightness_pct(self, value):
--         self.brightness = value * self.max_brightness
-- 
-- 
-- class ButtonBase(object):
--     """
--     Abstract button interface.
--     """
-- 
--     @staticmethod
--     def on_change(changed_buttons):
--         """
--         This handler is called by `process()` whenever state of any button has
--         changed since last `process()` call. `changed_buttons` is a list of
--         tuples of changed button names and their states.
--         """
--         pass
-- 
--     _state = set([])
-- 
--     def any(self):
--         """
--         Checks if any button is pressed.
--         """
--         return bool(self.buttons_pressed)
-- 
--     def check_buttons(self, buttons=[]):
--         """
--         Check if currently pressed buttons exactly match the given list.
--         """
--         return set(self.buttons_pressed) == set(buttons)
-- 
--     def process(self):
--         """
--         Check for currenly pressed buttons. If the new state differs from the
--         old state, call the appropriate button event handlers.
--         """
--         new_state = set(self.buttons_pressed)
--         old_state = self._state
--         self._state = new_state
-- 
--         state_diff = new_state.symmetric_difference(old_state)
--         for button in state_diff:
--             handler = getattr(self, 'on_' + button)
--             if handler is not None: handler(button in new_state)
-- 
--         if self.on_change is not None and state_diff:
--             self.on_change([(button, button in new_state) for button in state_diff])
-- 
--     @property
--     def buttons_pressed(self):
--         raise NotImplementedError()
-- 
-- 
-- class ButtonEVIO(ButtonBase):
-- 
--     """
--     Provides a generic button reading mechanism that works with event interface
--     and may be adapted to platform specific implementations.
-- 
--     This implementation depends on the availability of the EVIOCGKEY ioctl
--     to be able to read the button state buffer. See Linux kernel source
--     in /include/uapi/linux/input.h for details.
--     """
-- 
--     KEY_MAX = 0x2FF
--     KEY_BUF_LEN = int((KEY_MAX + 7) / 8)
--     EVIOCGKEY = (2 << (14 + 8 + 8) | KEY_BUF_LEN << (8 + 8) | ord('E') << 8 | 0x18)
-- 
--     _buttons = {}
-- 
--     def __init__(self):
--         self._file_cache = FileCache()
--         self._buffer_cache = {}
--         for b in self._buttons:
--             self._button_file(self._buttons[b]['name'])
--             self._button_buffer(self._buttons[b]['name'])
-- 
--     def _button_file(self, name):
--         return self._file_cache.file_handle(name)
-- 
--     def _button_buffer(self, name):
--         if name not in self._buffer_cache:
--             self._buffer_cache[name] = array.array('B', [0] * self.KEY_BUF_LEN)
--         return self._buffer_cache[name]
-- 
--     @property
--     def buttons_pressed(self):
--         """
--         Returns list of names of pressed buttons.
--         """
--         for b in self._buffer_cache:
--             fcntl.ioctl(self._button_file(b), self.EVIOCGKEY, self._buffer_cache[b])
-- 
--         pressed = []
--         for k, v in self._buttons.items():
--             buf = self._buffer_cache[v['name']]
--             bit = v['value']
--             if not bool(buf[int(bit / 8)] & 1 << bit % 8):
--                 pressed += [k]
--         return pressed

-- ~autogen remote-control classes.infraredSensor.remoteControl>currentClass
RemoteControl = ButtonBase:extend()
    
    --- EV3 Remote Controller

    RemoteControl._BUTTON_VALUES = {
            [0] = {  },
            [1] = { 'red_up' },
            [2] = { 'red_down' },
            [3] = { 'blue_up' },
            [4] = { 'blue_down' },
            [5] = { 'red_up', 'blue_up' },
            [6] = { 'red_up', 'blue_down' },
            [7] = { 'red_down', 'blue_up' },
            [8] = { 'red_down', 'blue_down' },
            [9] = { 'beacon' },
            [10] = { 'red_up', 'red_down' },
            [11] = { 'blue_up', 'blue_down' }
            }

    on_red_up = function() end
    on_red_down = function() end
    on_blue_up = function() end
    on_blue_down = function() end
    on_beacon = function() end

    RemoteControl:set{ 
        red_up = {
            -- Checks if `red_up` button is pressed.
            get = function(self) return 'red_up' in self.buttons_pressed button end,
            set = function(self, value) return 'red_up:read:only' end
        },
        red_down = {
            -- Checks if `red_down` button is pressed.
            get = function(self) return 'red_down' in self.buttons_pressed button end,
            set = function(self, value) return 'red_down:read:only' end
        },
        blue_up = {
            -- Checks if `blue_up` button is pressed.
            get = function(self) return 'blue_up' in self.buttons_pressed button end,
            set = function(self, value) return 'blue_up:read:only' end
        },
        blue_down = {
            -- Checks if `blue_down` button is pressed.
            get = function(self) return 'blue_down' in self.buttons_pressed button end,
            set = function(self, value) return 'blue_down:read:only' end
        },
        beacon = {
            -- Checks if `beacon` button is pressed.
            get = function(self) return 'beacon' in self.buttons_pressed button end,
            set = function(self, value) return 'beacon:read:only' end
        }
    }
-- ~autogen

-- TODO add the rest of the button API
--    def __init__(self, sensor=None, channel=1):
--        if sensor is None:
--            self._sensor = InfraredSensor()
--        else:
--            self._sensor = sensor
--
--        self._channel = max(1, min(4, channel)) - 1
--        self._state = set([])
--
--        if self._sensor.connected:
--            self._sensor.mode = 'IR-REMOTE'
--
--    @property
--    def connected(self):
--        return self._sensor.connected
--
--    @property
--    def buttons_pressed(self):
--        """
--        Returns list of currently pressed buttons.
--        """
--        return RemoteControl._BUTTON_VALUES.get(self._sensor.value(self._channel), [])
--

-- ~autogen generic-class classes.powerSupply>currentClass

PowerSupply = Device:extend()
    
    -- A generic interface to read data from the system's power_supply class.
    -- Uses the built-in legoev3-battery if none is specified.

    PowerSupply.SYSTEM_CLASS_NAME = 'power_supply'
    PowerSupply.SYSTEM_DEVICE_NAME_CONVENTION = '*'

    function PowerSupply:init(port, name, args)
        name = name or PowerSupply.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, PowerSupply.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-get-set classes.powerSupply>currentClass

    PowerSupply:set{ 
        measured_current = { 
            -- The measured current that the battery is supplying (in microamps)
            get = function(self) return self:_get_attr_int('current_now') end,
            set = function(self, value) return 'current_now:read:only' end
        },
        measured_voltage = { 
            -- The measured voltage that the battery is supplying (in microvolts)
            get = function(self) return self:_get_attr_int('voltage_now') end,
            set = function(self, value) return 'voltage_now:read:only' end
        },
        max_voltage = { 
            get = function(self) return self:_get_attr_int('voltage_max_design') end,
            set = function(self, value) return 'voltage_max_design:read:only' end
        },
        min_voltage = { 
            get = function(self) return self:_get_attr_int('voltage_min_design') end,
            set = function(self, value) return 'voltage_min_design:read:only' end
        },
        technology = { 
            get = function(self) return self:_get_attr_string('technology') end,
            set = function(self, value) return 'technology:read:only' end
        },
        type = { 
            get = function(self) return self:_get_attr_string('type') end,
            set = function(self, value) return 'type:read:only' end
        },
    }
-- ~autogen

-- TODO add the power supply scaling functions
--  @property
--  def measured_amps(self):
--      """
--      The measured current that the battery is supplying (in amps)
--      """
--      return self.measured_current / 1e6
--
--  @property
--  def measured_volts(self):
--      """
--      The measured voltage that the battery is supplying (in volts)
--      """
--     return self.measured_voltage / 1e6

-- ~autogen generic-class classes.legoPort>currentClass

LegoPort = Device:extend()
    
    -- The `lego-port` class provides an interface for working with input and
    -- output ports that are compatible with LEGO MINDSTORMS RCX/NXT/EV3, LEGO
    -- WeDo and LEGO Power Functions sensors and motors. Supported devices include
    -- the LEGO MINDSTORMS EV3 Intelligent Brick, the LEGO WeDo USB hub and
    -- various sensor multiplexers from 3rd party manufacturers.

    -- Some types of ports may have multiple modes of operation. For example, the
    -- input ports on the EV3 brick can communicate with sensors using UART, I2C
    -- or analog validate signals - but not all at the same time. Therefore there
    -- are multiple modes available to connect to the different types of sensors.

    -- In most cases, ports are able to automatically detect what type of sensor
    -- or motor is connected. In some cases though, this must be manually specified
    -- using the `mode` and `set_device` attributes. The `mode` attribute affects
    -- how the port communicates with the connected device. For example the input
    -- ports on the EV3 brick can communicate using UART, I2C or analog voltages,
    -- but not all at the same time, so the mode must be set to the one that is
    -- appropriate for the connected sensor. The `set_device` attribute is used to
    -- specify the exact type of sensor that is connected. Note: the mode must be
    -- correctly set before setting the sensor type.

    -- Ports can be found at `/sys/class/lego-port/port<N>` where `<N>` is
    -- incremented each time a new port is registered. Note: The number is not
    -- related to the actual port at all - use the `port_name` attribute to find
    -- a specific port.

    LegoPort.SYSTEM_CLASS_NAME = 'lego_port'
    LegoPort.SYSTEM_DEVICE_NAME_CONVENTION = '*'

    function LegoPort:init(port, name, args)
        name = name or LegoPort.SYSTEM_DEVICE_NAME_CONVENTION
        args = args or {}

        args['port_name'] = port

        Device.init(self, LegoPort.SYSTEM_CLASS_NAME, name, args)
    end
-- ~autogen
-- ~autogen generic-get-set classes.legoPort>currentClass

    LegoPort:set{ 
        driver_name = { 
            -- Returns the name of the driver that loaded this device. You can find the
            -- complete list of drivers in the [list of port drivers].
            get = function(self) return self:_get_attr_string('driver_name') end,
            set = function(self, value) return 'driver_name:read:only' end
        },
        modes = { 
            -- Returns a list of the available modes of the port.
            get = function(self) return self:_get_attr_set('modes') end,
            set = function(self, value) return 'modes:read:only' end
        },
        mode = { 
            -- Reading returns the currently selected mode. Writing sets the mode.
            -- Generally speaking when the mode changes any sensor or motor devices
            -- associated with the port will be removed new ones loaded, however this
            -- this will depend on the individual driver implementing this class.
            get = function(self) return self:_get_attr_string('mode') end,
            set = function(self, value) return self:_set_attr_string('mode', value) end
        },
        port_name = { 
            -- Returns the name of the port. See individual driver documentation for
            -- the name that will be returned.
            get = function(self) return self:_get_attr_string('port_name') end,
            set = function(self, value) return 'port_name:read:only' end
        },
        set_device = { 
            -- For modes that support it, writing the name of a driver will cause a new
            -- device to be registered for that driver and attached to this port. For
            -- example, since NXT/Analog sensors cannot be auto-detected, you must use
            -- this attribute to load the correct driver. Returns -EOPNOTSUPP if setting a
            -- device is not supported.
            get = function(self) return 'set_device:write:only' end,
            set = function(self, value) return self:_set_attr_string('set_device', value) end
        },
        status = { 
            -- In most cases, reading status will return the same value as `mode`. In
            -- cases where there is an `auto` mode additional values may be returned,
            -- such as `no-device` or `error`. See individual port driver documentation
            -- for the full list of possible values.
            get = function(self) return self:_get_attr_string('status') end,
            set = function(self, value) return 'status:read:only' end
        },
    }
-- ~autogen

-- TODO add framebuffer support
--class FbMem(object):
--
--    """The framebuffer memory object.
--
--    Made of:
--        - the framebuffer file descriptor
--        - the fix screen info struct
--        - the var screen info struct
--        - the mapped memory
--    """
--
--    # ------------------------------------------------------------------
--    # The code is adapted from
--    # https://github.com/LinkCareServices/cairotft/blob/master/cairotft/linuxfb.py
--    #
--    # The original code came with the following license:
--    # ------------------------------------------------------------------
--    # Copyright (c) 2012 Kurichan
--    #
--    # This program is free software. It comes without any warranty, to
--    # the extent permitted by applicable law. You can redistribute it
--    # and/or modify it under the terms of the Do What The Fuck You Want
--    # To Public License, Version 2, as published by Sam Hocevar. See
--    # http://sam.zoy.org/wtfpl/COPYING for more details.
--    # ------------------------------------------------------------------
--
--    __slots__ = ('fid', 'fix_info', 'var_info', 'mmap')
--
--    FBIOGET_VSCREENINFO = 0x4600
--    FBIOGET_FSCREENINFO = 0x4602
--
--    FB_VISUAL_MONO01 = 0
--    FB_VISUAL_MONO10 = 1
--
--    class FixScreenInfo(ctypes.Structure):
--
--        """The fb_fix_screeninfo from fb.h."""
--
--        _fields_ = [
--            ('id_name', ctypes.c_char * 16),
--            ('smem_start', ctypes.c_ulong),
--            ('smem_len', ctypes.c_uint32),
--            ('type', ctypes.c_uint32),
--            ('type_aux', ctypes.c_uint32),
--            ('visual', ctypes.c_uint32),
--            ('xpanstep', ctypes.c_uint16),
--            ('ypanstep', ctypes.c_uint16),
--            ('ywrapstep', ctypes.c_uint16),
--            ('line_length', ctypes.c_uint32),
--            ('mmio_start', ctypes.c_ulong),
--            ('mmio_len', ctypes.c_uint32),
--            ('accel', ctypes.c_uint32),
--            ('reserved', ctypes.c_uint16 * 3),
--        ]
--
--    class VarScreenInfo(ctypes.Structure):
--
--        class FbBitField(ctypes.Structure):
--
--            """The fb_bitfield struct from fb.h."""
--
--            _fields_ = [
--                ('offset', ctypes.c_uint32),
--                ('length', ctypes.c_uint32),
--                ('msb_right', ctypes.c_uint32),
--            ]
--
--        """The fb_var_screeninfo struct from fb.h."""
--
--        _fields_ = [
--            ('xres', ctypes.c_uint32),
--            ('yres', ctypes.c_uint32),
--            ('xres_virtual', ctypes.c_uint32),
--            ('yres_virtual', ctypes.c_uint32),
--            ('xoffset', ctypes.c_uint32),
--            ('yoffset', ctypes.c_uint32),
--
--            ('bits_per_pixel', ctypes.c_uint32),
--            ('grayscale', ctypes.c_uint32),
--
--            ('red', FbBitField),
--            ('green', FbBitField),
--            ('blue', FbBitField),
--            ('transp', FbBitField),
--        ]
--
--    def __init__(self, fbdev=None):
--        """Create the FbMem framebuffer memory object."""
--        fid = FbMem._open_fbdev(fbdev)
--        fix_info = FbMem._get_fix_info(fid)
--        fbmmap = FbMem._map_fb_memory(fid, fix_info)
--        self.fid = fid
--        self.fix_info = fix_info
--        self.var_info = FbMem._get_var_info(fid)
--        self.mmap = fbmmap
--
--    def __del__(self):
--        """Close the FbMem framebuffer memory object."""
--        self.mmap.close()
--        FbMem._close_fbdev(self.fid)
--
--    @staticmethod
--    def _open_fbdev(fbdev=None):
--        """Return the framebuffer file descriptor.
--
--        Try to use the FRAMEBUFFER
--        environment variable if fbdev is not given. Use '/dev/fb0' by
--        default.
--        """
--        dev = fbdev or os.getenv('FRAMEBUFFER', '/dev/fb0')
--        fbfid = os.open(dev, os.O_RDWR)
--        return fbfid
--
--    @staticmethod
--    def _close_fbdev(fbfid):
--        """Close the framebuffer file descriptor."""
--        os.close(fbfid)
--
--    @staticmethod
--    def _get_fix_info(fbfid):
--        """Return the fix screen info from the framebuffer file descriptor."""
--        fix_info = FbMem.FixScreenInfo()
--        fcntl.ioctl(fbfid, FbMem.FBIOGET_FSCREENINFO, fix_info)
--        return fix_info
--
--    @staticmethod
--    def _get_var_info(fbfid):
--        """Return the var screen info from the framebuffer file descriptor."""
--        var_info = FbMem.VarScreenInfo()
--        fcntl.ioctl(fbfid, FbMem.FBIOGET_VSCREENINFO, var_info)
--        return var_info
--
--    @staticmethod
--    def _map_fb_memory(fbfid, fix_info):
--        """Map the framebuffer memory."""
--        return mmap.mmap(
--            fbfid,
--            fix_info.smem_len,
--            mmap.MAP_SHARED,
--            mmap.PROT_READ | mmap.PROT_WRITE,
--            offset=0
--        )
--
--
--class Screen(FbMem):
--    """
--    A convenience wrapper for the FbMem class.
--    Provides drawing functions from the python imaging library (PIL).
--    """
--
--    def __init__(self):
--        FbMem.__init__(self)
--
--        self._img = Image.new(
--                self.var_info.bits_per_pixel == 1 and "1" or "RGB",
--                (self.fix_info.line_length * 8 / self.var_info.bits_per_pixel, self.yres),
--                "white")
--
--        self._draw = ImageDraw.Draw(self._img)
--
--    @property
--    def xres(self):
--        """
--        Horizontal screen resolution
--        """
--        return self.var_info.xres
--
--    @property
--    def yres(self):
--        """
--        Vertical screen resolution
--        """
--        return self.var_info.yres
--
--    @property
--    def shape(self):
--        """
--        Dimensions of the screen.
--        """
--        return (self.xres, self.yres)
--
--    @property
--    def draw(self):
--        """
--        Returns a handle to PIL.ImageDraw.Draw class associated with the screen.
--
--        Example::
--
--            screen.draw.rectangle((10,10,60,20), fill='black')
--        """
--        return self._draw
--
--    def clear(self):
--        """
--        Clears the screen
--        """
--        self._draw.rectangle(((0, 0), self.shape), fill="white")
--
--    def _color565(self, r, g, b):
--        """Convert red, green, blue components to a 16-bit 565 RGB value. Components
--        should be values 0 to 255.
--        """
--        return (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))
--
--    def _img_to_rgb565_bytes(self):
--        pixels = [self._color565(r, g, b) for (r, g, b) in self._img.getdata()]
--        return pack('H' * len(pixels), *pixels)
--
--    def update(self):
--        """
--        Applies pending changes to the screen.
--        Nothing will be drawn on the screen until this function is called.
--        """
--        if self.var_info.bits_per_pixel == 1:
--            self.mmap[:] = self._img.tobytes("raw", "1;IR")
--        elif self.var_info.bits_per_pixel == 16:
--            self.mmap[:] = self._img_to_rgb565_bytes()
--        else:
--            raise Exception("Not supported")
--
--
--class Sound:
--    """
--    Sound-related functions. The class has only static methods and is not
--    intended for instantiation. It can beep, play wav files, or convert text to
--    speech.
--
--    Note that all methods of the class spawn system processes and return
--    subprocess.Popen objects. The methods are asynchronous (they return
--    immediately after child process was spawned, without waiting for its
--    completion), but you can call wait() on the returned result.
--
--    Examples::
--
--        # Play 'bark.wav', return immediately:
--        Sound.play('bark.wav')
--
--        # Introduce yourself, wait for completion:
--        Sound.speak('Hello, I am Robot').wait()
--    """
--
--    @staticmethod
--    def beep(args=''):
--        """
--        Call beep command with the provided arguments (if any).
--        See `beep man page`_ and google 'linux beep music' for inspiration.
--
--        .. _`beep man page`: http://manpages.debian.org/cgi-bin/man.cgi?query=beep
--        """
--        with open(os.devnull, 'w') as n:
--            return Popen('/usr/bin/beep %s' % args, stdout=n, shell=True)
--
--    @staticmethod
--    def tone(*args):
--        """
--        tone(tone_sequence):
--
--        Play tone sequence. The tone_sequence parameter is a list of tuples,
--        where each tuple contains up to three numbers. The first number is
--        frequency in Hz, the second is duration in milliseconds, and the third
--        is delay in milliseconds between this and the next tone in the
--        sequence.
--
--        Here is a cheerful example::
--
--            Sound.tone([
--                (392, 350, 100), (392, 350, 100), (392, 350, 100), (311.1, 250, 100),
--                (466.2, 25, 100), (392, 350, 100), (311.1, 250, 100), (466.2, 25, 100),
--                (392, 700, 100), (587.32, 350, 100), (587.32, 350, 100),
--                (587.32, 350, 100), (622.26, 250, 100), (466.2, 25, 100),
--                (369.99, 350, 100), (311.1, 250, 100), (466.2, 25, 100), (392, 700, 100),
--                (784, 350, 100), (392, 250, 100), (392, 25, 100), (784, 350, 100),
--                (739.98, 250, 100), (698.46, 25, 100), (659.26, 25, 100),
--                (622.26, 25, 100), (659.26, 50, 400), (415.3, 25, 200), (554.36, 350, 100),
--                (523.25, 250, 100), (493.88, 25, 100), (466.16, 25, 100), (440, 25, 100),
--                (466.16, 50, 400), (311.13, 25, 200), (369.99, 350, 100),
--                (311.13, 250, 100), (392, 25, 100), (466.16, 350, 100), (392, 250, 100),
--                (466.16, 25, 100), (587.32, 700, 100), (784, 350, 100), (392, 250, 100),
--                (392, 25, 100), (784, 350, 100), (739.98, 250, 100), (698.46, 25, 100),
--                (659.26, 25, 100), (622.26, 25, 100), (659.26, 50, 400), (415.3, 25, 200),
--                (554.36, 350, 100), (523.25, 250, 100), (493.88, 25, 100),
--                (466.16, 25, 100), (440, 25, 100), (466.16, 50, 400), (311.13, 25, 200),
--                (392, 350, 100), (311.13, 250, 100), (466.16, 25, 100),
--                (392.00, 300, 150), (311.13, 250, 100), (466.16, 25, 100), (392, 700)
--                ]).wait()
--
--        tone(frequency, duration):
--
--        Play single tone of given frequency (Hz) and duration (milliseconds).
--        """
--        def play_tone_sequence(tone_sequence):
--            def beep_args(frequency=None, duration=None, delay=None):
--                args = ''
--                if frequency is not None: args += '-f %s ' % frequency
--                if duration  is not None: args += '-l %s ' % duration
--                if delay     is not None: args += '-D %s ' % delay
--
--                return args
--
--            return Sound.beep(' -n '.join([beep_args(*t) for t in tone_sequence]))
--
--        if len(args) == 1:
--            return play_tone_sequence(args[0])
--        elif len(args) == 2:
--            return play_tone_sequence([(args[0], args[1])])
--        else:
--            raise Exception("Unsupported number of parameters in Sound.tone()")
--
--    @staticmethod
--    def play(wav_file):
--        """
--        Play wav file.
--        """
--        with open(os.devnull, 'w') as n:
--            return Popen('/usr/bin/aplay -q "%s"' % wav_file, stdout=n, shell=True)
--
--    @staticmethod
--    def speak(text):
--        """
--        Speak the given text aloud.
--        """
--        with open(os.devnull, 'w') as n:
--            return Popen('/usr/bin/espeak -a 200 --stdout "%s" | /usr/bin/aplay -q' % text, stdout=n, shell=True)
