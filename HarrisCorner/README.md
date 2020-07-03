# Harris corner algorithm with Jevois camera
This module takes frames from Jevois camera, and extracts feature-points (corners) from the image using the Harris corner detection algorithm. This module is to be run onboard the Jevois camera. The Jevois camera has a dedicated CPU which can be used to run machine vision modules. In this way, feature-points are sent directly from the camera to the host computer. To familiarise yourself with the Jevois system, refer [here](http://jevois.org/doc/UserStartLinux.html). We will be programming on the Jevois camera, creating our Harris corner module. To enable this, start with [this guide](http://jevois.org/doc/Programmer.html).

To create a new module, follow [this link](http://jevois.org/doc/ProgrammerSDK.html).

Once the new module is created (and named HarrisCorner), copy the contents of this folder into the following directory:
> /harriscorner/src/Modules/HarrisCorner/

Replace the contents of the newly created HarrisCorner.C with the contents of this HarrisCorner.C.

Change the contents of postinstall to reflect author's name and the desired camera resolution parameters for this module. The format for this document is:

> \<USBmode\> \<USBwidth\> \<USBheight\> \<USBfps\> \<CAMmode\> \<CAMwidth\> \<CAMheight\> \<CAMfps\> \<Vendor\> \<Module\>
> CamMode can be only one of: YUYV, BAYER, RGB565
> USBmode can be only one of: YUYV, GREY, MJPG, BAYER, RGB565, BGR24, NONE

By selecting different resolutions and frame rates of the Jevois camera, we can select different computer vision modules. The contents of postinstall specify which settings will trigger the HarrisCorner module. These values need to be reflected in videomappings.cfg in the /jevois/ directory (more information on how to set this up can be found [here](http://jevois.org/doc/VideoMapping.html)).

