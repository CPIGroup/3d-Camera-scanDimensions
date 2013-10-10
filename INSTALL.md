## Compiling

#### Delete old files if exists:

    rm -Rf CMakeFiles
    rm CMakeCache.txt
    rm cmake_install.cmake
    rm Makefile
    rm pclBox
    rm boxDimenstions.json

#### Generate Makefile:

    cmake .

#### Config JSON

Add following line in config.h for JSON output, and pclBox will add current box dimension in to **boxDimenstions.json**.

    #define OUTPUTJSONFILE      "./boxDimenstions.json"

#### Compiling:

    make

## Running

#### Running:

    ./pclBox

#### Operate:
* Clean desk;
* Press *"b"* and wait until program save background (about 20 seconds);
* Put the box on desk.
