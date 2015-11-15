Lua language bindings for ev3dev
================================

This is a Lua library implementing unified interface for ev3dev_ devices.

The class library is based on Jon Stoler's class.lua_ work.

Basic motor drivers are done - still to do:

- Support for stopping motors when the object is garbage collected
- Support for matching attributes when creating object instances
- Support for value property and binary formatting to sensor devices
- LED and Button Support
- Power Supply scaling functions
- Framebuffer support

.. _ev3dev: http://ev3dev.org
.. _class.lua: https://github.com/jonstoler/class.lua


