# Python demo

Run as:
```
python3 demo.py
```

It will read lines for 10 seconds and will compute how many lines / second were read.

# C/C++ demo

Compile as:
```
g++ -o demo demo.cpp rs232.c
```

Run as:
```
./demo
```

It will read lines for 10 seconds and will compute how many lines / second were read.

# To do

* Why is the python rate almost exactly 40 Hz (as it should be) and the C rate more like 41.5 Hz? You should check that the INS time difference matches the CPU time difference.
* Should we be concerned about %CPU? It's high, peaking at about 50% for both processes. Consider testing with `nice` or `cpulimit`. Consider using the `python3` version, since it's simpler, more robust, easier to parse, etc., and since it seems to be taking no more %CPU than the `C` version.
