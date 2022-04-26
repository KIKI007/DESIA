DESIA
=========

|travis| |docs|

.. |travis| image:: https://travis-ci.org/EPFL-LGG/TopoLite.svg?branch=master
    :target: https://travis-ci.org/github/EPFL-LGG/TopoLite
    :alt: Travis Build Status

.. |docs| image:: https://readthedocs.org/projects/topolite/badge/?version=latest
   :target: https://topolite.readthedocs.io/en/latest/?badge=latest
   :alt: Documentation Status

.. begin_brief_description

.. image:: https://github.com/KIKI007/DESIA/raw/master/resources/project_teaser.png
        :alt: Teaser
        :align: center

DESIA is an algorithm for generating voxelized interlocking assemblies.

Abstract
--------

Interlocking assemblies have a long history in the design of puzzles, furniture, architecture, and other complex geometric structures. The key defining property of interlocking assemblies is that all component parts are immobilized by their geometric arrangement, preventing the assembly from falling apart. Computer graphics research has recently contributed design tools that allow creating new interlocking assemblies. However, these tools focus on specific kinds of assemblies and explore only a limited space of interlocking configurations, which restricts their applicability for design.

In this paper, we propose a new general framework for designing interlocking assemblies. The core idea is to represent part relationships with a family of base Directional Blocking Graphs and leverage efficient graph analysis tools to compute an interlocking arrangement of parts. This avoids the exponential complexity of brute-force search. Our algorithm iteratively constructs the geometry of assembly components, taking advantage of all existing blocking relations when constructing successive parts. As a result, our approach supports a wider range of assembly forms compared to previous methods and provides significantly more design flexibility. We show that our framework facilitates efficient design of complex interlocking assemblies, including new solutions that cannot be achieved by state of the art approaches.

Please check our DESIA2018SigA_ paper for more technical details.

.. DESIA2018SigA_: https://lgg.epfl.ch/publications/2018/DESIA/index.php

GUI Interface
-------------

.. image:: https://github.com/KIKI007/DESIA/raw/master/resources/screenshot.png
   :alt: Screenshot of DESIA
   :align: center

.. end_brief_description

Compilation
-----------
Clone the repository, run CMake to generate Makefiles and the rest should just work automatically.

- **MacOS**:
Xcode >= 11.5 (C++ 17)

.. code-block:: bash

    $ brew install boost

    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_BUILD_TYPE=Release ..
    $ make -j 16

- **Ubuntu(Linux)**:
gcc >= 7.5.0 (C++ 17)

.. code-block:: bash

    $ sudo apt-get install libboost-all-dev

    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_BUILD_TYPE=Release ..
    $ make -j 16


Usage
-----------

1. Read a .puz file in `\DESIA\data\input_puz`
2. Choose the target number of interlocking parts by setting **Number of pieces**. the defaut is 7
3. The **automatic search** button will keep searching until find a valid interlocking puzzle.
4. The **Generate Next Part** button only generate one more part. The program provided multiple choices. Change **Choose Nth Children** could show other valid solution. If you don't like any of the design, click **Go back** button.
5. Output the .puz file using the button **Write .puz**


Parameter
-----------
1. 4x4x4 Solid Cube model, the maximum part number should not exceed 9
2. 4x4x4 Hollowed Cube model, the maximum part number should not exceed 7