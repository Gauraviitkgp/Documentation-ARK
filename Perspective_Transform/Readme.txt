Firstly, Detect the corner points of the part of image where u want to apply perspective transformation.

for that use 

g++  detect_points.cpp -o opencv `pkg-config --cflags --libs opencv`

and then

./opencv imagename.imagetype

Click on the 4 points which u want to get transformed. Click in an cyclic order starting from left-top, RT,RB,LB.. Ensure that LT has min x+y and RB has the max x+y.

Then run the python command

python transform.py --image example_01.png --coords "[(73, 239), (356, 117), (475, 265), (187, 443)]"

coordinates are randomly taken here as an eg.
