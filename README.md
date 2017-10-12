IoT


# How to install platformio

http://platformio.org/platformio-ide

`pip instsall platformio`

Follow directions for install

Once installed, install 2.0 beta



# Using platformio

When the text editor is open, open project folder icon in the left, select the directory that contains `src/`

If that dun not work, in the file pane (where u normally see the tree of files) right click and  `add project folder`
, this should be the same folder as I mentioned above

`platformio.ini` contains config information for the project. `lib_deps=` contains libraries that the project depends on. This is dope bc if you dont have them, it automatically downloads them from http://platformio.org/lib (which also has nice examples)

Check mark compiles, arrow thing uploads to the hardware.  Theres serial tools to see serial output

the `platformio` CLI is also cool, `platformio device monitor` is a cool command to see serial output from the device.
