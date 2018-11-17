# path-following-algorithm

README

This repository contains a description of a path-following algorithm and an implementation of the algorithm in C++.  The C++ implementation is not stand-alone, it was developed for use in a game, but it does show how the algorithm can be coded. There are some “magic numbers” in the code. They don’t necessarily correspond to anything in particular, they were just set based on experimentation. That is, they made the objects controlled by the algorithm appear to move smoothly.

The algorithm description might be a bit confusing, because I used “T” for the name of the parametric function variable. I left it as “T” because I used “t” in the code. I try to make it clear in the description when I’m referring to “time” and when I’m referring to the parametric variable.

The “CST” prefix is just my initials.

The section of the code in CST_Path.cpp that runs the algorithm is in the CST_Path::UpdatePostition() method.

The CST_Motion structure was not originally in the CST_Path.h file. I copied it into the file to provide additional information.

The code uses Hungarian notation. Please forgive me.
