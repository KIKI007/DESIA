# DESIA sourse code
Code for SIGGRAPH AISA 2018 Submission
This code is only for designing interlocking voxelized structures.

## File organization
There are two folders of this project:

1. **\data**: the test input and output puzzle files.
2. **\src**: the sourse code of our program.

## Extra Library
1. Boost : <https://www.boost.org>

## Compiling on Mac
1. Install **brew** on mac (https://brew.sh)
2. Install **boost**: in terminal, typing **brew install boost**
3. Use **Clion** IDE to compile the project: Download Clion(<https://www.jetbrains.com/clion/>). Use its **open project** to open the project (a cmake file) in /DESIA
4. Or use **cmake** to compile the /DESIA/CMakeLists.txt

## Usage

1. Read a .puz file in \DESIA\input_puz
2. Choose the target number of interlocking parts by setting **Number of pieces**. the defaut is 7
3. The **automatic search** button will keep searching until find a valid interlocking puzzle.
4. The **Generate Next Part** button only generate one more part. The program provided multiple choices. Change **Choose Nth Children** could show other valid solution. If you don't like any of the design, click **Go back** button.
5. Output the .puz file using the button **Write .puz**


## Parameter
1. 4x4x4 Solid Cube model, the maximum part number should not exceed 9
2. 4x4x4 Hollowed Cube model, the maximum part number should not exceed 7