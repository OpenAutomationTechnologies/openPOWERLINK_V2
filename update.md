Update information {#page_update}
==================

# Update to release 2.5 {#sect_v2_5}

## Update object dictionary initialization {#sect_v2_5_update_od_init}

In openPOWERLINK 2.5, the object dictionary creation has been moved from the
stack library to the application. Therefore, on any application that shall be
updated to openPOWERLINK 2.5 or higher the following changes need to be done:

* Download openPOWERLINK 2.5.x or higher.
* Copy the files from `apps/common/src/obdcreate` to your application.
* Add the file `obdcreate.c` to your project. It has to be compiled into your
  application.
* Include `obdcreate.h` in your main application.
* Add a call to obdcreate_initObd() in your main application and pass a
  pointer to a structure that will be filled with the init parameters.
* This structure is then handed over to the stack via oplk_create().
* Move the object dictionary definition for your device to your application.
  Typically, you therefore move the file `objdict.h` to an application folder
  and add this folder to your default include path of the compiler.
* If `objdict.h` includes other header files (e.g. sections of the POWERLINK
  specification included via external headers), those parts have to be copied
  into your new `objdict.h`.
* Recompile the updated stack library and then your application

### Example code {#sect_v2_5_update_od_example}

    tOplkError          ret;
    tOplkApiInitParam   initParam;

    // Fill the rest of the initParameters (similar to previous stack versions)

    // Initialize object dictionary
    ret = obdcreate_initObd(&initParam.obdInitParam);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "obdcreate_initObd() failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

## Update object dictionary definition {#sect_v2_5_update_od_def}

When updating to the new stack, the object dictionary definition needs to be
adapted. First, the generic parts of the object dictionary have been moved into
the `objdict.h` file itself. It is recommended that you just copy the generic
section (between `OBD_BEGIN_PART_GENERIC()` and the according `OBD_END_PART()`)
from the given examples to your own `objdict.h` file. Depending on the kind of
device, you should use either the section from `CiA401_CN` for a POWERLINK CN
or `CiA302-4_MN` for a POWERLINK MN.

Additionally, please note that the definition of a callback on object access
has been replaced by a user event flag. Instead of defining `NULL` or a
callback function, you have to set either `FALSE` if access to this object
shall not be forwarded to the user or `TRUE` if a user event shall be
generated. Object access that is handled inside the stack itself and previously
had a definition of a stack-internal callback function (e.g. mapping objects...)
does not require the flag to be set to `TRUE`. These cases are handled
internally.
