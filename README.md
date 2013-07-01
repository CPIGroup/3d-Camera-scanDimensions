## Compiling

#### Delete old files if exists:

```bash
rm -Rf CMakeFiles
rm CMakeCache.txt
rm cmake_install.cmake
rm Makefile
rm pclBox
```

#### Generate Makefile:

```bash
cmake .
```

#### Config JSON

add following line in config.h for JSON output, and pclBox will add current box dimension in to boxDimenstions.json.
```cpp
#define OUTPUTJSONFILE      "./boxDimenstions.json"
```


#### Compiling:

```bash
make
```

## Running

#### Running:

```bash
./pclBox
````

#### Operate:
* Clean desk;
* Press "b" and wait until program save background (about 20 seconds);
* Put the box on desk.
