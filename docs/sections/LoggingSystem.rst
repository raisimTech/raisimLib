#############################
Logging System
#############################

RaiSim logging system has been inspired by ``glog`` library.
It can compactly represent multiple lines of logging.
It does not save messages to a file.
It only streams the message to the standard output (i.e., ``cout``)
For example, instead of writing

.. code-block:: c

    if(fail) {
        std::cout<<date<<FILENAME<<LINENUMBER<< ... << "failed here \n";
        exit(0);
    }

you can write

.. code-block:: c

    RSFATAL_IF(fail, "failed here")

The available logging macros are

- ``RSINFO(msg)``: Prints out the message in white.
- ``RSWARN(msg)``: Prints out the message in yellow.
- ``RSFATAL(msg)``: Prints out the message in red then execute the fatal function
- ``RSINFO_IF(con, msg)``: Prints out the message in white if the condition is true.
- ``RSWARN_IF(con, msg)``: Prints out the message in yellow if the condition is true.
- ``RSFATAL_IF(con, msg)``: Prints out the message in red and calls the fatal function if the condition is true.
- ``RSASSERT(con, msg)``: Prints out the message in red and calls the fatal function if the condition is false.
- ``RSRETURN_IF(con, msg)``: Prints out the message in white then call ``return`` if the condition is true.
- ``RSISNAN(val)``: Prints out "${val} is nan" then calls the fatal function if the value is nan (not a number).
- ``RSISNAN_MSG(val, msg)``: Prints out the message then calls the fatal function if the value is nan (not a number).

The fatal function is ``std::exit(1);`` by default.
You can override it as

.. code-block:: c

    raisim::RaiSimMsg::setFatalCallback([](){throw;});

The macros actually affect the performance.
For examples, all ``_IF`` macros will check the boolean and branching might occur.

If you do not want to lose performance from branching, there are following debug versions of the macros available

- ``DRSINFO(msg)``
- ``DRSWARN(msg)``
- ``DRSFATAL(msg)``
- ``DRSINFO_IF(con, msg)``
- ``DRSWARN_IF(con, msg)``
- ``DRSFATAL_IF(con, msg)``
- ``DRSASSERT(con, msg)``
- ``DRSRETURN_IF(con, msg)``
- ``DRSISNAN(val)``
- ``DRSISNAN_MSG(val, msg)``

These are ignored if the build type is not debug.

