# binaryConverter
This program creates an executable which is used to convert recorded test data (in binary format) into human readable .txt files. The file **convertfpbin** is used to convert a single file. The bash script **convertFiles** is used to convert an entire directory of binary files.

### convertfpbin

The executable is used as follows:
```
./convertfpbin <filename> <filetype>
```
	where: 	<filename> is the absolute path of file to convert
		<filetype> depends on the the type of messages to decode. One of these:
			- **vehicleState**: vehicle state messages (in body frame)
			- **camMessage**: raw feature point message from camera
			- **fpDatabase**: feature point estimate message

	or, to bring up more information
```
./convertfpbin -o
```
The output of this executable will be the respective decoded message, in .txt format. The contents of each decoded file are specific to the message type being decoded:

### vehicleState message
**vehicleState**: <timestamp> <pos.x> <pos.y> <pos.z> <quat.0> <quat.1> <quat.2> <quat.3>
	where:
		- <timestamp> 	: time of nav solution (time since epoch [us])
		- <pos.x/y/z> 	: position of vehicle ( FRD inertial frame [m])
		- <quat.0/1/2/3>: attitude of vehicle (quaternion representation with q0 scalar)

### camMessage message
**camMessage**: <timestamp1> <timestamp2> <numFPS> <fp1.u> <fp1.v> <fp2.u> <fp2.v> ... <fpN.u> <fpN.v>
	where:
		- <timestamp1>	: Jevois time first byte of frame grabbed (time since epoch [us])
		- <timestamp2>	: time message arrives at filter (time since epoch [us])
		- <numFPS> 	: number of features extracted from frame
		- <fpk.u> 	: horizontal pixel coordinate of kth feature (scaled), right positive
		- <fpk.v> 	: vertical pixel coordinate of kth feature (scaled), down positive

#### fpDatabase message
**fpDatabase**: TODO!!!

### convertFiles shell script
This bash script is run on an entire directory to decode files (of the same filetype - see above) to .txt format. Your directory should separate binary files into subdirectories, where all elements of that subdirectory are of the same type.

./convertFiles command requires TWO and ONLY TWO input arguments:
```
./converFiles <dirpath> <filetype>
```
	where: 	<dirpath> is the absolute path to directory
		<filetype> depends on the the type of messages to decode. One of these:
			- **vehicleState**: vehicle state messages (in body frame)
			- **camMessage**: raw feature point message from camera
			- **fpDatabase**: feature point estimate message

## Building
Navigate to **/binaryConverter/** directory and simply run the command:
```
make all
```
