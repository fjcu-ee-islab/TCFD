# TCFD
TCFD(Traffic Camera Fault Detection) is an algorithm to detect the anomaly of traffic CCTV camera. This repo provides code, library and header.
<img width="1127" height="332" alt="image" src="https://github.com/user-attachments/assets/af8432bf-38b5-4d69-94a2-f77a5e45fd67" />
## Repository requirements
Before running this program, make sure your system has the following tools installed:
- Operating System: Windows 7
- IDE: Microsoft Visual Studio 2008
- Programming Language: C++
- Library: OpenCV 2.0
## Dataset prepare
Dataset was uploaded at following link include TCFD Daytime, TCFD Nighttime and groundtruth.
https://filedn.com/lv1UQc8ijkEXzTgOaCNTCbQ/Reproducible%20Research/TCFD/TCFD%20Dataset/
## Compile "evLaneCAD"
Copy the project and source code from the "code\evLaneCAD" folder. Open the project to view the list of header files and source files. After configuring the required OpenCV include directories, library directories, and linking the necessary libraries, you can compile the project.
## Execute "evLaneCAD"
After successful compilation, the executable file "evLaneCAD.exe" can be found in the "01-Source Code\evLaneCAD\Release" directory. The system must be launched from the Command Prompt using the following syntax:
`evLaneCAD.exe [Parameter 1] [Parameter 2] [Parameter 3]`
### Parameter 1 – Input file name. For example, if the file name is "abc.avi", simply enter "abc".
### Parameter 2 – Specifies whether to display the processing video. The valid values are 0 or 1:
0: Do not display the processing video.
1: Display the processing video. The current detection status is indicated by a green, yellow, or red bounding box.
### Parameter 3 – Sensitivity of the camera anomaly detection. The valid range is 1–10:
1: Highest sensitivity
7: Default value
10: Lowest sensitivity

You may refer to the "evLaneCADTest.bat" file in the same directory for an example of the command.

The following describes the system interface using a spray-paint obstruction on the camera view as an example. Launch the system by executing "OnlinCADTest.bat", which runs the command:

`evLaneCADTest.exe LaneDefocusing11 1 7`

After startup, the system displays the processing window with three possible status indicators, as shown in following Figure :

Figure (a): A green bounding box indicates that detection is in progress and no abnormal event has been detected.
Figure (b): A yellow bounding box indicates a suspected abnormal event, and the system is verifying the condition.
Figure (c): A red bounding box with a cross indicates that an abnormal event has been confirmed.
<img width="1127" height="332" alt="image" src="https://github.com/user-attachments/assets/96a5c510-8ef4-46f9-9124-9e38e6300d27" />

In addition, an output file named "OutputAlarm.txt" is generated in the same directory. If an alarm is detected, the output file records the video name and the timestamp at which the alarm occurred, as shown in the example file "Example-OutputAlarm.txt". An example of the output format is shown in following Figure.
<img width="702" height="305" alt="image" src="https://github.com/user-attachments/assets/f3589c82-bddb-4a2b-b8e3-d68e46f121eb" />
