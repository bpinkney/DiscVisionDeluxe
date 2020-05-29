@ECHO OFF

cd ..
cd ..
cd src
g++ main.cpp dfisx.cpp daero.cpp dgyro.cpp dpropagate.cpp -o main
main hyzer -0.35442 pitch 0.53664 discmold 0 posx -0.619 posy 0.344 posz 1.941 velx 15.005 vely 6.389 velz 8.854 spinrate -72.00863 wobble 0 matlab 1 windx 0 windy 0 windz 0
pause
