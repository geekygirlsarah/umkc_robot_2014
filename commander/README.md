This is the main program to run on the motherboard of wesley. The other program that needs to run is the logging subscriber program.

Code documentation exists in the doc subdirectory and can be updated by runnign the makedoc.sh bash file. Doxygen must be installed to update docs (sudo apt-get install doxygen)

Binaries to be run should be put in the bin subdirectory

The final structure for the commander should be set up as follows:

|Commander binary' \n
|./bin/ \n
||All of the binaries needed by commander \n
||All of their requried components such as pictures or other files \n
|./res/ \n
||The notifiy_id.txt document and other resources needed by commander \n
