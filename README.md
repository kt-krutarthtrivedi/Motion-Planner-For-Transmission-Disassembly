# Motion Planner For Transmission Disassembly

## About the project
Given an idealized representation of the transmission, in a state with the bearings and input shaft removed, but the mainshaft still inside the case, implement an RRT* motion planner to create a full 6 DOF collision-free path or trajectory that moves the mainshaft from its initial configuration to a configuration outside the transmission.

## About the environment

A simplified model of the SM-465 transmission is shown in the figure below. Included in this model is the case, with PTO ports and bearing bores, plus mainshaft and countershaft assemblies. Refer to the [OpenSCAD files](https://github.com/kt-krutarthtrivedi/Motion-Planner-For-Transmission-Disassembly/tree/main/media/SM-465%20Transmission) attached to this repository for the exact models of the constituent parts.


- A simplified model

<img width="511" alt="Screenshot 2023-05-27 at 5 16 04 PM" src="https://github.com/kt-krutarthtrivedi/Motion-Planner-For-Transmission-Disassembly/assets/134632027/3304d84b-fa9f-432b-96d1-f771b5ae36c8">


&nbsp;

- A Top View

This illustrates the alignment of the mainshaft in the case. Note that the large white gear on the left side is actually the input gear on the countershaft. Whereas, in this view, the input (from the engine) is on the left side, and output to the driveshaft and wheels is on the right.

<img width="499" alt="Screenshot 2023-05-27 at 5 17 39 PM" src="https://github.com/kt-krutarthtrivedi/Motion-Planner-For-Transmission-Disassembly/assets/134632027/6e41922e-7e4e-4751-a013-17b88436a8c3">

&nbsp;

- A side view

This view illustrates the placement of both the mainshaft (top), and countershaft (bottom) assemblies. Note the large windows on either side of the case, these are for PTO (Power Take Off) drive mounting.

<img width="499" alt="Screenshot 2023-05-27 at 5 20 11 PM" src="https://github.com/kt-krutarthtrivedi/Motion-Planner-For-Transmission-Disassembly/assets/134632027/3ba7294d-43bd-4e13-be76-688ed7c4a208">
&nbsp;

## RRT* Motion Planner

- The explored tree structure looks like the below image:

<img width="504" alt="Screenshot 2023-05-27 at 5 23 41 PM" src="https://github.com/kt-krutarthtrivedi/Motion-Planner-For-Transmission-Disassembly/assets/134632027/b8bf42cc-6183-4fac-8409-4a62325574bb">

&nbsp;

- Planned Path 

<img width="504" alt="Screenshot 2023-05-27 at 5 25 51 PM" src="https://github.com/kt-krutarthtrivedi/Motion-Planner-For-Transmission-Disassembly/assets/134632027/d7912a8c-e9e5-4d06-af3e-c5ba591bf501">

&nbsp;

## Demo


https://github.com/kt-krutarthtrivedi/Motion-Planner-For-Transmission-Disassembly/assets/134632027/e2ae5892-22ff-4714-94c6-a9edb69bb30d



## References

* [Steven M. LaValle. Planning Algorithms. Cambridge University Press, May 2006.
9780521862059.](http://lavalle.pl/planning/)






