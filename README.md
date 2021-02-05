# DESIA sourse code
Code for SIGGRAPH AISA 2018 Submission
 Note that this code is used for desinging interlocking voxelized structures.


## System Requirement
This code has been only tested in macOS High Sierra

## File organization
There are two folders of this project:

1. \DESIA: the sourse code of our program
2. \Libigl: an older version of libigl, for rendering and UI.

The input mode is in \DESIA\input_puz, which is in .puz format

## Extra Library
1. Boost : <https://www.boost.org>
2. Libigl* : <https://github.com/libigl/libigl>
3. Eigen 3**: <http://eigen.tuxfamily.org/index.php?title=Main_Page>

*: libigl changed its rendering system recently. Please use the older version we provided to avoid compiling error
**: Eigen 3 is included in Libigl, just in case.

## Compiling
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

###Thank you for your reviewing
